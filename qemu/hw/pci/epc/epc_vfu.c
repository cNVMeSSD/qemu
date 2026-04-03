/*
 * epc_vfu.c — vfio-user socket and remote RC callback lifecycle
 *
 * This translation unit owns the vfio-user context lifecycle and every
 * libvfio-user callback that fires in response to remote Root Complex (RC)
 * activity.  Specifically it contains:
 *
 *   - qepc_vfu_log            : forwards libvfio-user log messages to QEMU's
 *                               logging infrastructure via qemu_epc_debug().
 *   - qepc_vfu_run            : QEMU fd-handler that drives the vfio-user
 *                               event loop (vfu_run_ctx) once a client is
 *                               attached.
 *   - qepc_vfu_attach_ctx     : QEMU fd-handler that accepts an incoming
 *                               vfio-user client connection and then
 *                               transitions the fd handler to qepc_vfu_run.
 *   - qepc_pci_cfg_access_bar : helper for BAR register accesses within the
 *                               PCI configuration space, applying standard PCI
 *                               BAR-sizing semantics.
 *   - qepc_pci_cfg_access     : libvfio-user callback for all PCI config-space
 *                               accesses; delegates BAR offsets to
 *                               qepc_pci_cfg_access_bar.
 *   - qepc_bar_access         : common helper for per-BAR region callbacks;
 *                               forwards accesses to guest memory via DMA.
 *   - QEPC_ACCESS_BAR(n)      : macro that stamps out a typed per-BAR wrapper
 *                               around qepc_bar_access; instantiated for
 *                               BARs 0–5.
 *   - qepc_bar_handlers[]     : dispatch table of the six per-BAR callbacks,
 *                               used in qepc_ctrl_handle_start().
 *
 * The only non-static symbol in this file — the public entry point called by
 * epc_regs.c — is:
 *
 *   - qepc_ctrl_handle_start  : called when the guest writes to
 *                               QEPC_CTRL_OFF_START; creates and realizes the
 *                               vfio-user context, wires up all callbacks, and
 *                               begins listening for client connections.
 *
 * All other symbols are static (file-private) and are only referenced as
 * function-pointer callbacks installed into the vfio-user context or into
 * QEMU's fd-handler table.
 *
 * DMA helpers called by qepc_ctrl_handle_start (qepc_dma_register and
 * qepc_dma_unregister) live in epc_mem.c and are declared in qemu_epc.h.
 */

#include "qemu_epc.h"

/* ---------------------------------------------------------------------------
 * Logging callback
 * ------------------------------------------------------------------------- */

/*
 * qepc_vfu_log
 *
 * Logging callback installed into the vfio-user context via vfu_setup_log()
 * during qepc_ctrl_handle_start().  All messages emitted by the library are
 * forwarded to QEMU's own logging infrastructure via qemu_epc_debug() so
 * that they appear alongside device debug output under a unified prefix.
 *
 * TODO: Map vfio-user log levels to QEMU log categories/levels if finer-
 *       grained filtering is desired (e.g. suppress LOG_DEBUG in production).
 */
static void qepc_vfu_log(vfu_ctx_t *vfu_ctx, int level, const char *msg)
{
    trace_qepc_vfu_log(level, msg);
}

/* ---------------------------------------------------------------------------
 * vfio-user event-loop fd handlers
 * ------------------------------------------------------------------------- */

/*
 * qepc_vfu_run
 *
 * Event loop callback used by QEMU's fd handler to drive the vfio-user
 * context.  Installed by qepc_vfu_attach_ctx() on the poll fd returned by
 * vfu_get_poll_fd() after a successful vfu_attach_ctx() call.  QEMU's main
 * loop invokes this function whenever the fd becomes readable.
 *
 * It repeatedly calls vfu_run_ctx() until:
 *   - the client disconnects (ENOTCONN), or
 *   - a fatal error occurs, or
 *   - vfu_run_ctx() returns 0.
 *
 * EINTR is ignored and retried.
 *
 * TODO: Consider integrating with QEMU's event loop more tightly to detect
 *       and handle backend errors in a more structured way.
 */
static void qepc_vfu_run(void *opaque)
{
    QEPCState *s = opaque;
    int err = -1;

    trace_qepc_vfu_run_start(s->vfu_fd);

    while (err != 0) {
        err = vfu_run_ctx(s->vfu);
        if (err < 0) {
            if (errno == EINTR) {
                trace_qepc_vfu_run_eintr();
                continue;
            } else if (errno == ENOTCONN) {
                trace_qepc_vfu_run_disconnect();
                break;
            } else {
                trace_qepc_vfu_run_failed(err, errno);
                break;
            }
        } else {
            trace_qepc_vfu_run_exit_loop();
        }
    }

    trace_qepc_vfu_run_leaving();
}

/*
 * qepc_vfu_attach_ctx
 *
 * fd handler invoked when there is a pending connection or attach event
 * on the vfio-user socket.  It:
 *   - Detaches any existing handler on the current poll fd,
 *   - Calls vfu_attach_ctx() (with basic EAGAIN/EWOULDBLOCK retry),
 *   - Retrieves the poll fd for the attached context,
 *   - Registers qepc_vfu_run as the new handler for that fd.
 *
 * TODO: Add robust error reporting back to the guest when attachment fails.
 */
static void qepc_vfu_attach_ctx(void *opaque)
{
    QEPCState *s = opaque;
    int err;

    trace_qepc_vfu_attach_start(s->vfu_fd);

    /*
     * If we already have a poll fd installed, remove its handler before
     * attempting to attach a new context. This prevents stale handlers
     * from firing.
     */
    if (s->vfu_fd >= 0) {
        trace_qepc_vfu_attach_remove_handler(s->vfu_fd);
        qemu_set_fd_handler(s->vfu_fd, NULL, NULL, NULL);
    }

retry:
    err = vfu_attach_ctx(s->vfu);
    if (err < 0) {
        if (err == EAGAIN || errno == EWOULDBLOCK) {
            trace_qepc_vfu_attach_retry();
            goto retry;
        }
        trace_qepc_vfu_attach_failed(err, errno);
        return;
    }

    s->vfu_fd = vfu_get_poll_fd(s->vfu);
    if (s->vfu_fd < 0) {
        trace_qepc_vfu_attach_poll_fd_failed();
        return;
    }

    trace_qepc_vfu_attach_done(s->vfu_fd);
    qemu_set_fd_handler(s->vfu_fd, qepc_vfu_run, NULL, s);
}

/* ---------------------------------------------------------------------------
 * PCI configuration-space access callbacks
 * ------------------------------------------------------------------------- */

/*
 * qepc_pci_cfg_access_bar
 *
 * Helper for qepc_pci_cfg_access that deals specifically with accesses
 * to PCI BAR registers in the config space.
 *
 * The function enforces BAR sizing semantics by masking the written
 * value with the complement of the BAR size (standard PCI BAR handling),
 * and updates the cached PCIe config space representation accordingly.
 *
 * The offset-to-BAR mapping is:
 *   0x10/0x14 → BAR0 low/high
 *   0x18/0x1c → BAR2 low/high
 *   0x20/0x24 → BAR4 low/high
 *
 * This aligns with how Linux treats 64-bit BARs as pairs.
 *
 * Returns the number of bytes handled (== count) on success.
 */
static ssize_t qepc_pci_cfg_access_bar(QEPCState *s, char *const buf,
                                       size_t count, loff_t offset,
                                       const bool is_write)
{
    uint32_t t1, t2;
    int bar_index;
    bool is_high_dword = false;

    trace_qepc_pci_cfg_access_bar(is_write ? "write" : "read", (uint64_t)offset, (uint64_t)count);

    /* BAR registers are always 32-bit wide. */
    assert(count == sizeof(uint32_t));

    /*
     * Map config-space offset to BAR index:
     *   0x10/0x14 -> BAR0 low/high
     *   0x18/0x1c -> BAR2 low/high
     *   0x20/0x24 -> BAR4 low/high
     *
     * This aligns with how Linux treats 64-bit BARs as pairs.
     */
    switch (offset) {
    case 0x10:
        bar_index = 0;
        is_high_dword = false;
        break;
    case 0x14:
        bar_index = 0;
        is_high_dword = true;
        break;
    case 0x18:
        bar_index = 2;
        is_high_dword = false;
        break;
    case 0x1c:
        bar_index = 2;
        is_high_dword = true;
        break;
    case 0x20:
        bar_index = 4;
        is_high_dword = false;
        break;
    case 0x24:
        bar_index = 4;
        is_high_dword = true;
        break;
    default:
        bar_index = -1;
        break;
    }

    if (is_write) {
        memcpy(&t1, buf, sizeof(t1));

        if (bar_index >= 0 && !is_high_dword) {
            /*
             * Low dword write: apply size-masking semantics expected by PCI,
             * i.e. emulate the standard BAR size probing behavior.
             */
            uint32_t size = (uint32_t)s->bars[bar_index].size;

            if (size) {
                t2 = (~size + 1) & t1;
            } else {
                /* If size is not programmed yet, just store what the host wrote. */
                t2 = t1;
            }

            memcpy(s->pcie_config + offset, &t2, sizeof(t2));
            trace_qepc_pci_cfg_bar_low_write(bar_index, t1, t2, size);
        } else if (bar_index >= 0 && is_high_dword) {
            /*
             * High dword write of a 64-bit BAR: the host owns this value.
             * We simply store and return it verbatim. Masking is not applied
             * to the high 32 bits.
             */
            memcpy(s->pcie_config + offset, &t1, sizeof(t1));
            trace_qepc_pci_cfg_bar_high_write(bar_index, t1);
        } else {
            /*
             * Non-BAR or unsupported BAR offset: just store directly.
             */
            memcpy(s->pcie_config + offset, buf, count);
            trace_qepc_pci_cfg_bar_generic_write((uint64_t)offset, t1);
        }
    } else {
        memcpy(&t1, s->pcie_config + offset, sizeof(t1));
        trace_qepc_pci_cfg_bar_read((uint64_t)offset, t1);
        memcpy(buf, s->pcie_config + offset, count);
    }

    return count;
}

/*
 * qepc_pci_cfg_access
 *
 * libvfio-user callback that services accesses to the virtual PCI
 * configuration space as seen from the vfio-user client (Root Complex side).
 *
 * Design:
 *   libvfio-user maintains its own internal copy of PCI config space via
 *   vfu_pci_get_config_space().  For standard capabilities (MSI, MSI-X, PCIe)
 *   that were registered with vfu_pci_add_capability(), libvfio-user intercepts
 *   writes internally before forwarding here — but only when
 *   VFU_CAP_FLAG_CALLBACK is NOT set.  Because we set VFU_REGION_FLAG_ALWAYS_CB
 *   on the config region, ALL accesses arrive here first.
 *
 *   The correct pattern is therefore:
 *     - Reads:  serve from libvfio-user's own config space mirror so the Host
 *               always sees the authoritative capability state (enable bits,
 *               table BIR/offset, etc.) that the library manages.
 *     - Writes: mirror into BOTH our pcie_config shadow (so the EP driver can
 *               read back what the Host wrote via the PCI_CFG BAR) AND into
 *               libvfio-user's config space (so the library can update its
 *               internal interrupt state machine when capability enable bits
 *               are toggled).
 *
 *   BAR register accesses (offsets 0x10–0x27) are still delegated to
 *   qepc_pci_cfg_access_bar() which enforces BAR sizing semantics.
 *
 * Returns the number of bytes handled, or 0 on error.
 */
static ssize_t qepc_pci_cfg_access(vfu_ctx_t *vfu_ctx, char *const buf,
                                   size_t count, loff_t offset,
                                   const bool is_write)
{
    QEPCState *s = vfu_get_private(vfu_ctx);

    trace_qepc_pci_cfg_access(is_write ? "write" : "read", (uint64_t)offset, (uint64_t)count);

    if (offset + count > PCIE_CONFIG_SPACE_SIZE) {
        return 0;
    }

    /* Route BAR register accesses through the BAR-specific helper. */
    if (offset >= 0x10 && offset < 0x28)
        return qepc_pci_cfg_access_bar(s, buf, count, offset, is_write);

    if (is_write) {
        /*
         * Mirror the write into our shadow buffer so the EP guest driver can
         * observe Host-side configuration via the PCI_CFG BAR.
         */
        memcpy(s->pcie_config + offset, buf, count);

        /*
         * Also write into libvfio-user's own config space mirror.  This is
         * critical for capability enable bits: when the Host sets the MSI
         * Enable bit (offset PCI_MSI_FLAGS within the MSI cap) or the MSI-X
         * Enable bit (PCI_MSIX_FLAGS within the MSI-X cap), libvfio-user must
         * see that write to arm the corresponding interrupt eventfds.
         * Without this, vfu_irq_trigger() will always fail because the library
         * believes the interrupt type is still disabled.
         */
        vfu_pci_config_space_t *vfu_cfg = vfu_pci_get_config_space(vfu_ctx);
        if (vfu_cfg) {
            memcpy((uint8_t *)vfu_cfg + offset, buf, count);
            trace_qepc_pci_cfg_access_mirror((uint64_t)offset, (uint64_t)count);
        }

        trace_qepc_pci_cfg_access_write((uint64_t)offset, (uint64_t)count);
    } else {
        /*
         * Reads are served from libvfio-user's authoritative config space
         * mirror so the Host always sees the library's view of capability
         * state (e.g. the current MSI-X Function Mask bit, PBA contents).
         * Fall back to our shadow buffer if the vfu mirror is unavailable.
         */
        vfu_pci_config_space_t *vfu_cfg = vfu_pci_get_config_space(vfu_ctx);
        if (vfu_cfg) {
            memcpy(buf, (uint8_t *)vfu_cfg + offset, count);
            trace_qepc_pci_cfg_access_read_mirror((uint64_t)offset, (uint64_t)count);
        } else {
            memcpy(buf, s->pcie_config + offset, count);
            trace_qepc_pci_cfg_access_read_shadow((uint64_t)offset, (uint64_t)count);
        }
    }

    return count;
}

/* ---------------------------------------------------------------------------
 * BAR region access callbacks
 * ------------------------------------------------------------------------- */

/*
 * qepc_bar_access
 *
 * Common helper for libvfio-user BAR region callbacks.  It:
 *   - Computes the system physical address corresponding to the given BAR
 *     and offset using the BAR configuration recorded in QEPCState::bars[],
 *   - Uses QEMU's pci_dma_read/pci_dma_write helpers to perform the actual
 *     transfer on behalf of the vfio-user client.
 *
 * This effectively lets the remote (vfio-user) side access guest-visible
 * memory via DMA semantics.
 *
 * TODO: Validate that 'barno' is within bounds and that the BAR has been
 *       properly configured before allowing access.
 */
/*
 * qepc_bar_access_log_hex - log up to 16 bytes of a buffer as hex.
 * Used by qepc_bar_access() for read/write debug tracing.
 */
static void qepc_bar_access_log_hex(const char *prefix, dma_addr_t addr,
                                    size_t count, const char *data)
{
    int log_len = count < 16 ? (int)count : 16;
    char hexbuf[64] = {0};
    for (int i = 0; i < log_len; ++i)
        snprintf(hexbuf + i * 3, sizeof(hexbuf) - i * 3, "%02x ",
                 (unsigned char)data[i]);
    trace_qepc_bar_access_log_hex(prefix, (uint64_t)addr, (uint64_t)count, hexbuf);
}

/*
 * qepc_bar_access
 *
 * Common helper for libvfio-user BAR region callbacks.  Called for every
 * Host (Root Complex side) read or write to any of the endpoint's BARs.
 *
 * Three cases are handled in priority order:
 *
 *  1. MSI-X table / PBA intercept (writes only)
 *     When the Host configures an MSI-X vector it writes the 16-byte table
 *     entry (msg_addr_lo, msg_addr_hi, msg_data, vector_ctrl) into the BAR
 *     that holds the MSI-X table.  libvfio-user needs to see these writes
 *     in its own config-space mirror so it can arm the correct eventfd.
 *     We detect this by checking whether the access overlaps
 *     [msix_table_off, msix_table_off + msix_table_size) in msix_table_bar,
 *     or [msix_pba_off, msix_pba_off + msix_pba_size) in msix_pba_bar.
 *     For matching writes we mirror into the vfu config space AND still
 *     forward to Guest RAM so the EP driver can read the table back.
 *
 *  2. Doorbell intercept (writes only)
 *     If doorbell_bar != 0xFF and the write hits [doorbell_offset,
 *     doorbell_offset+4), it is a doorbell ring.  We call
 *     qepc_handle_ctrl_irq() to inject an interrupt into the Guest instead
 *     of performing a DMA write.  The 32-bit value written is passed as the
 *     irq_num argument so the guest driver can encode a vector number.
 *
 *  3. Normal BAR DMA forwarding
 *     All other accesses are forwarded to Guest RAM via pci_dma_read /
 *     pci_dma_write using the physical address recorded in bars[barno].
 */
static ssize_t qepc_bar_access(vfu_ctx_t *vfu_ctx, uint8_t barno,
                               char *const buf, size_t count, loff_t offset,
                               const bool is_write)
{
    QEPCState *s = vfu_get_private(vfu_ctx);
    dma_addr_t phys_addr;

    trace_qepc_bar_access(barno, (uint64_t)offset, (uint64_t)count, is_write ? "write" : "read");

    if (barno >= PCI_NUM_REGIONS) {
        trace_qepc_bar_access_invalid_bar(barno);
        return -1;
    }

    if (!(s->bar_mask & (1U << barno))) {
        trace_qepc_bar_access_disabled(barno, s->bar_mask);
        return -1;
    }

    if (offset < 0 || (uint64_t)offset + count > s->bars[barno].size) {
        trace_qepc_bar_access_oob(barno, (uint64_t)offset, (uint64_t)count, s->bars[barno].size);
        return -1;
    }

    /* ------------------------------------------------------------------
     * Case 1: MSI-X table / PBA intercept.
     *
     * When the Host writes an MSI-X vector entry we must mirror that write
     * into libvfio-user's config space so the library can arm the eventfd
     * for that vector.  The MSI-X table is not part of config space in the
     * PCI sense, but libvfio-user tracks it separately via the capability
     * registration done in qepc_ctrl_handle_start().
     *
     * We also forward the write to Guest RAM so the EP driver can read the
     * table back when it needs msg_addr/msg_data to construct an outbound
     * MSI-X write (mirroring what cdns_pcie_ep_send_msix_irq() does).
     * ------------------------------------------------------------------ */
    if (is_write && s->msix_num > 0) {
        bool in_table = (barno == s->msix_table_bar) &&
                        ((uint64_t)offset >= s->msix_table_off) &&
                        ((uint64_t)offset + count <=
                         s->msix_table_off + s->msix_table_size);
        bool in_pba   = (barno == s->msix_pba_bar) &&
                        ((uint64_t)offset >= s->msix_pba_off) &&
                        ((uint64_t)offset + count <=
                         s->msix_pba_off + s->msix_pba_size);

        if (in_table || in_pba) {
            /*
             * Mirror the write into libvfio-user's config space mirror at
             * the correct offset within the MSI-X capability structure.
             * libvfio-user lays out the MSI-X table immediately after the
             * 12-byte msixcap header in its internal representation; we find
             * the cap offset via vfu_pci_find_capability() then add the
             * within-BAR offset relative to the table/PBA base.
             */
            vfu_pci_config_space_t *vfu_cfg = vfu_pci_get_config_space(vfu_ctx);
            if (vfu_cfg) {
                size_t cap_off = vfu_pci_find_capability(vfu_ctx, false,
                                                         PCI_CAP_ID_MSIX);
                if (cap_off) {
                    /*
                     * The MSI-X table and PBA are BAR-resident, not config-
                     * space-resident in hardware. libvfio-user stores them
                     * contiguously after the cap header in its mirror.
                     * Table entries start at cap_off + PCI_CAP_MSIX_SIZEOF.
                     */
                    size_t mirror_base = cap_off + PCI_CAP_MSIX_SIZEOF;
                    size_t within_region = in_table
                        ? (size_t)offset - s->msix_table_off
                        : s->msix_table_size + ((size_t)offset - s->msix_pba_off);
                    size_t mirror_off = mirror_base + within_region;
                    if (mirror_off + count <= PCIE_CONFIG_SPACE_SIZE) {
                        memcpy((uint8_t *)vfu_cfg + mirror_off, buf, count);
                        trace_qepc_bar_access_msix_mirror(in_table ? "table" : "PBA", (uint64_t)mirror_off);
                    }
                }
            }
            /* Fall through: still DMA-write to Guest RAM so EP can read back */
        }
    }

    /* ------------------------------------------------------------------
     * Case 2: Doorbell intercept.
     *
     * A doorbell is a write-only register whose sole purpose is to signal
     * the Endpoint.  When the Host writes to it we must NOT forward the
     * write to Guest RAM; instead we inject a local interrupt using the
     * same delivery mechanism that the guest configured (INTx / MSI /
     * MSI-X).  The 32-bit written value is used as the interrupt vector
     * number, following the same 1-based convention as IRQ_NUM writes.
     * ------------------------------------------------------------------ */
    if (is_write &&
        s->doorbell_bar != 0xFF &&
        barno == s->doorbell_bar &&
        (uint64_t)offset == s->doorbell_offset &&
        count >= sizeof(uint32_t)) {

        uint32_t irq_val = 0;
        memcpy(&irq_val, buf, sizeof(irq_val));
        trace_qepc_bar_access_doorbell(barno, (uint64_t)offset, irq_val);
        /*
         * qepc_handle_ctrl_irq() is defined in epc_regs.c and declared in
         * qemu_epc.h.  It routes the interrupt via vfu_irq_trigger() using
         * s->irq_type (INTx / MSI / MSI-X) set by the guest driver.
         */
        qepc_handle_ctrl_irq(s, (int)irq_val);
        return count;
    }

    /* ------------------------------------------------------------------
     * Case 3: Normal DMA forwarding.
     * ------------------------------------------------------------------ */
    phys_addr = s->bars[barno].phys_addr + offset;
    trace_qepc_bar_access_dma_addr((uint64_t)phys_addr);

    if (is_write) {
        pci_dma_write(&s->dev, phys_addr, buf, count);
        qepc_bar_access_log_hex("DMA WRITE", phys_addr, count, buf);
    } else {
        pci_dma_read(&s->dev, phys_addr, buf, count);
        qepc_bar_access_log_hex("DMA READ", phys_addr, count, buf);
    }

    return count;
}

/*
 * QEPC_ACCESS_BAR
 *
 * Convenience macro that generates a thin wrapper around qepc_bar_access()
 * for a specific BAR number. These wrappers are then used as per-BAR
 * access callbacks when registering regions with libvfio-user.
 *
 * Six wrappers are instantiated below (BARs 0–5) and collected into the
 * qepc_bar_handlers[] dispatch table used in qepc_ctrl_handle_start().
 */
#define QEPC_ACCESS_BAR(barno)                                                 \
  static ssize_t qepc_bar_access_##barno(vfu_ctx_t *vfu_ctx, char *const buf,  \
                                         size_t count, loff_t offset,          \
                                         const bool is_write) {                \
    return qepc_bar_access(vfu_ctx, barno, buf, count, offset, is_write);      \
  }

QEPC_ACCESS_BAR(0)
QEPC_ACCESS_BAR(1)
QEPC_ACCESS_BAR(2)
QEPC_ACCESS_BAR(3)
QEPC_ACCESS_BAR(4)
QEPC_ACCESS_BAR(5)

/*
 * qepc_bar_handlers
 *
 * Dispatch table indexed by BAR number (0–5).  Each entry points to the
 * corresponding static qepc_bar_access_N() wrapper generated by
 * QEPC_ACCESS_BAR() above.  Used during qepc_ctrl_handle_start() to
 * register each enabled BAR with libvfio-user via vfu_setup_region().
 *
 * qepc_bar_access_handler_t is defined in qemu_epc.h — it is not
 * redefined here.
 */
static qepc_bar_access_handler_t qepc_bar_handlers[] = {
    qepc_bar_access_0,
    qepc_bar_access_1,
    qepc_bar_access_2,
    qepc_bar_access_3,
    qepc_bar_access_4,
    qepc_bar_access_5,
};

/* ---------------------------------------------------------------------------
 * Backend start — public entry point called from epc_regs.c
 * ------------------------------------------------------------------------- */

/* qepc_ctrl_handle_start
 *
 * Handles a write to the QEPC_CTRL_OFF_START register.
 *
 * This is the main initialization path for the vfio-user backend:
 *   - Creates the vfio-user context bound to sock_path,
 *   - Sets up logging, PCI type, config space region, BAR regions,
 *     IRQs, and DMA callbacks,
 *   - Realizes the vfio-user context and starts listening for client
 *     connections on the socket, installing qepc_vfu_attach_ctx as
 *     the fd handler.
 *
 * The 'val' argument is currently unused but reserved for future use
 * (e.g. flags or version negotiation).
 */
/*
 * qepc_ctrl_handle_start
 *
 * Handles a guest write to QEPC_CTRL_OFF_START.  This is the single
 * top-level initialization routine for the vfio-user backend; it must be
 * called only after the guest driver has fully programmed all capability
 * parameters via the control BAR registers.
 *
 * Initialization sequence:
 *
 *  1. Create the vfio-user context (non-blocking attach).
 *  2. Configure library logging.
 *  3. Initialize the PCI Express device type.
 *  4. Register the PCI config space region with ALWAYS_CB so all accesses
 *     route through qepc_pci_cfg_access().
 *  5. Register MSI capability via vfu_pci_add_capability() if msi_num > 0.
 *     This tells libvfio-user the layout of the MSI control/address/data
 *     registers so it can intercept Host writes to the Enable bit and arm
 *     the interrupt eventfd automatically.
 *  6. Register MSI-X capability via vfu_pci_add_capability() if msix_num > 0.
 *     This tells libvfio-user which BAR holds the table and PBA, and how
 *     many vectors are present.  Without this call the library is blind to
 *     Host writes to MSI-X vector entries, so vfu_irq_trigger() for
 *     VFU_DEV_MSIX_IRQ will always fail.
 *  7. Register enabled BAR regions.
 *  8. Set INTx, MSI, and MSI-X IRQ counts.
 *  9. Register DMA callbacks (epc_mem.c).
 * 10. Realize the context and start listening on the socket.
 */
int qepc_ctrl_handle_start(QEPCState *s, uint64_t val)
{
    int err;
    ssize_t cap_off;

    trace_qepc_ctrl_handle_start(val, s->sock_path ? s->sock_path : "<null>");

    s->vfu = vfu_create_ctx(VFU_TRANS_SOCK, s->sock_path,
                            LIBVFIO_USER_FLAG_ATTACH_NB, s, VFU_DEV_TYPE_PCI);
    if (!s->vfu) {
        trace_qepc_ctrl_handle_start_vfu_create_failed(errno);
        return -1;
    }

    /* Route libvfio-user logs to our qepc_vfu_log helper. */
    vfu_setup_log(s->vfu, qepc_vfu_log, LOG_DEBUG);

    err = vfu_pci_init(s->vfu, VFU_PCI_TYPE_EXPRESS, PCI_HEADER_TYPE_NORMAL, 0);
    if (err) {
        trace_qepc_ctrl_handle_start_pci_init_failed(err, errno);
        return -1;
    }

    /*
     * Copy the PCI header fields that the EP driver has already written into
     * pcie_config (via BAR 1 / cfg_base) into libvfio-user's own config space
     * mirror.
     *
     * Background: write_header() and the BAR configuration sequence in the
     * Linux qemu-epc driver write vendor/device ID, class codes, etc. directly
     * into s->pcie_config[] through the pci_cfg_mr RAM-backed MemoryRegion.
     * Those writes never pass through qepc_pci_cfg_access(), so libvfio-user's
     * mirror (vfu_pci_get_config_space()) is never updated with the basic
     * header fields.
     *
     * The RC (client QEMU vfio-user-pci) reads config space through
     * qepc_pci_cfg_access(), which serves reads from the vfu mirror.  Without
     * this copy the vendor ID at offset 0x00 reads back as 0x0000, which
     * Linux PCI enumeration treats as "slot empty" and the device never
     * appears in the guest's dmesg.
     *
     * We only copy the standard Type-0 header (64 bytes, offsets 0x00–0x3f).
     * Capability structures are managed by vfu_pci_add_capability() below and
     * must not be overwritten here.  BAR registers (0x10–0x27) are handled
     * by qepc_pci_cfg_access_bar() and are also left to the RC to assign.
     */
    {
        vfu_pci_config_space_t *vfu_cfg = vfu_pci_get_config_space(s->vfu);
        if (vfu_cfg) {
            /*
             * Copy only the non-BAR, non-capability-list header fields:
             *   0x00–0x0f : vendor/device ID, command, status, revision,
             *               class code, cache line, latency, header type, BIST
             *   0x28–0x3f : cardbus CIS, subsystem IDs, ROM BAR, cap pointer,
             *               interrupt line/pin, min_gnt/max_lat
             * Skip 0x10–0x27 (BARs) — the RC probes and assigns those itself.
             */
            memcpy((uint8_t *)vfu_cfg + 0x00, s->pcie_config + 0x00, 0x10);
            memcpy((uint8_t *)vfu_cfg + 0x28, s->pcie_config + 0x28, 0x18);
            trace_qepc_ctrl_handle_start_header_copied(*(uint16_t *)(s->pcie_config + 0x00), *(uint16_t *)(s->pcie_config + 0x02), *(uint16_t *)(s->pcie_config + 0x0a));
        } else {
            trace_qepc_ctrl_handle_start_header_null();
        }
    }

    trace_qepc_ctrl_handle_start_cfg_setup();
    err = vfu_setup_region(s->vfu, VFU_PCI_DEV_CFG_REGION_IDX,
                           PCIE_CONFIG_SPACE_SIZE, &qepc_pci_cfg_access,
                           VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB,
                           NULL, 0, -1, 0);
    if (err) {
        trace_qepc_ctrl_handle_start_cfg_failed(err, errno);
        return -1;
    }
    trace_qepc_ctrl_handle_start_cfg_done();

    /* ------------------------------------------------------------------
     * Step 5: MSI capability registration.
     *
     * vfu_pci_add_capability() writes a struct msicap into the library's
     * config space mirror and links it into the capability list.  Once
     * registered the library intercepts Host writes to the MSI control
     * word and arms/disarms the interrupt eventfd.  Without this call the
     * Host cannot enable MSI and vfu_irq_trigger(VFU_DEV_MSI_IRQ) fails.
     *
     * We only register if the guest driver has configured msi_num > 0 by
     * writing QEPC_CTRL_OFF_MSI_CFG before START.
     * ------------------------------------------------------------------ */
    if (s->msi_num > 0) {
        struct msicap msi_cap = { 0 };
        msi_cap.hdr.id = PCI_CAP_ID_MSI;
        /*
         * mc.c64 = 1: advertise 64-bit message address support.
         * mc.mmc encodes log2(msi_num) as the "multiple message capable"
         * field (3 bits, value 0-5 for 1/2/4/8/16/32 vectors).
         */
        msi_cap.mc.c64  = 1;
        msi_cap.mc.mmc  = __builtin_ctz(s->msi_num); /* log2 of power-of-two */
        /* pvm: per-vector masking — leave 0 (not supported) for simplicity */

        cap_off = vfu_pci_add_capability(s->vfu, 0, 0, &msi_cap);
        if (cap_off < 0) {
            trace_qepc_ctrl_handle_start_msi_cap_failed(errno);
            return -1;
        }
        trace_qepc_ctrl_handle_start_msi_cap_done((uint64_t)cap_off, s->msi_num);
    } else {
        trace_qepc_ctrl_handle_start_msi_cap_skip();
    }

    /* ------------------------------------------------------------------
     * Step 6: MSI-X capability registration.
     *
     * struct msixcap tells libvfio-user:
     *   mxc.ts   : table size minus 1 (N-1 for N vectors).
     *   mtab.tbir: BAR index that holds the MSI-X vector table.
     *   mtab.to  : dword offset of the table within that BAR (>> 3).
     *   mpba.pbir: BAR index that holds the PBA.
     *   mpba.pbao: dword offset of the PBA within that BAR (>> 3).
     *
     * After this call the library:
     *   - Knows where the table is so it can intercept Host writes to
     *     individual vector entries and record msg_addr/msg_data/vector_ctrl.
     *   - Knows which eventfd corresponds to each vector index.
     *   - Will correctly arm/disarm eventfds when the Host toggles the
     *     MSI-X Enable bit in the capability control word.
     *
     * Without this call:
     *   - The Host's writes to the MSI-X table BAR region arrive in
     *     qepc_bar_access() as ordinary DMA writes and are forwarded to
     *     Guest RAM instead of being intercepted.
     *   - vfu_irq_trigger(VFU_DEV_MSIX_IRQ, n) always returns -1 / EINVAL
     *     because the library has no eventfd armed for any vector.
     * ------------------------------------------------------------------ */
    if (s->msix_num > 0) {
        struct msixcap msix_cap = { 0 };
        msix_cap.hdr.id  = PCI_CAP_ID_MSIX;
        msix_cap.mxc.ts  = s->msix_num - 1;   /* table size field = N-1 */
        msix_cap.mtab.tbir = s->msix_table_bar;
        msix_cap.mtab.to   = s->msix_table_off >> 3; /* qword offset */
        msix_cap.mpba.pbir = s->msix_pba_bar;
        msix_cap.mpba.pbao = s->msix_pba_off  >> 3;

        cap_off = vfu_pci_add_capability(s->vfu, 0, 0, &msix_cap);
        if (cap_off < 0) {
            trace_qepc_ctrl_handle_start_msix_cap_failed(errno);
            return -1;
        }
        trace_qepc_ctrl_handle_start_msix_cap_done((uint64_t)cap_off, s->msix_num, s->msix_table_bar, s->msix_table_off, s->msix_pba_bar, s->msix_pba_off);
    } else {
        trace_qepc_ctrl_handle_start_msix_cap_skip();
    }

    /* Step 7: Register enabled BAR regions. */
    trace_qepc_ctrl_handle_start_bar_setup(s->bar_mask);
    for (int i = 0; i < PCI_NUM_REGIONS; i++) {
        if (!(s->bar_mask & (1 << i))) {
            trace_qepc_ctrl_handle_start_bar_skip(i, s->bar_mask);
            continue;
        }

        trace_qepc_ctrl_handle_start_bar_info(i, s->bars[i].phys_addr, s->bars[i].size, s->bars[i].flags);

        err = vfu_setup_region(s->vfu, VFU_PCI_DEV_BAR0_REGION_IDX + i,
                               s->bars[i].size, qepc_bar_handlers[i],
                               VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB,
                               NULL, 0, -1, 0);
        if (err) {
            trace_qepc_ctrl_handle_start_bar_failed(i, err, errno);
            return -1;
        }
        trace_qepc_ctrl_handle_start_bar_done(i);
    }
    trace_qepc_ctrl_handle_start_bars_done();

    /* Step 8: Set IRQ counts. */
    err = vfu_setup_device_nr_irqs(s->vfu, VFU_DEV_INTX_IRQ, 1);
    if (err < 0) {
        trace_qepc_ctrl_handle_start_intx_failed(err, errno);
        return err;
    }

    {
        /*
         * Advertise the number of MSI vectors actually configured by the
         * guest driver (or a safe default of 1 if none were configured).
         * The count must match the mmc field in the MSI cap registered above.
         */
        uint32_t msi_count = s->msi_num > 0 ? s->msi_num : 1;
        err = vfu_setup_device_nr_irqs(s->vfu, VFU_DEV_MSI_IRQ, msi_count);
        if (err < 0) {
            trace_qepc_ctrl_handle_start_msi_irq_failed(err, errno);
            return err;
        }
        trace_qepc_ctrl_handle_start_msi_irq_done(msi_count);
    }

    {
        /*
         * Advertise the number of MSI-X vectors.  Must be non-zero and must
         * match mxc.ts+1 in the MSI-X cap; use the guest-configured value or
         * fall back to 1 so the path doesn't hard-fail if MSI-X is unused.
         */
        uint32_t msix_count = s->msix_num > 0 ? s->msix_num : 1;
        err = vfu_setup_device_nr_irqs(s->vfu, VFU_DEV_MSIX_IRQ, msix_count);
        if (err < 0) {
            trace_qepc_ctrl_handle_start_msix_irq_failed(err, errno);
            return err;
        }
        trace_qepc_ctrl_handle_start_msix_irq_done(msix_count);
    }

    /* Step 9: Register DMA callbacks (epc_mem.c). */
    err = vfu_setup_device_dma(s->vfu, qepc_dma_register, qepc_dma_unregister);
    if (err) {
        trace_qepc_ctrl_handle_start_dma_failed(err, errno);
        return -1;
    }

    /* Step 10: Realize and start listening. */
    err = vfu_realize_ctx(s->vfu);
    if (err) {
        trace_qepc_ctrl_handle_start_realize_failed(err, errno);
        return -1;
    }

    /*
     * Step 10a: Sync capability layout from libvfio-user's config space mirror
     * back into s->pcie_config so the Linux EP driver can find MSI/MSI-X
     * capabilities by reading from BAR 1 (cfg_base).
     *
     * Background:
     *   vfu_pci_add_capability() writes MSI and MSI-X capability structures
     *   into libvfio-user's own internal config space mirror and links them
     *   into the capability list.  As a side-effect it also sets the
     *   PCI_STATUS_CAP_LIST bit in the Status register of that mirror.
     *   However, the EP-side config space shadow (s->pcie_config) — which is
     *   exposed to the Linux driver as a plain RAM-backed MemoryRegion on
     *   BAR 1 — is never updated by vfu_pci_add_capability().
     *
     *   qemu_ep_find_capability() in the Linux driver reads PCI_STATUS and
     *   PCI_CAPABILITY_LIST directly from cfg_base (BAR 1), which maps to
     *   s->pcie_config.  Without this sync it sees PCI_STATUS_CAP_LIST=0,
     *   immediately returns 0 ("capability not found"), and every subsequent
     *   MSI raise call fails with "MSI capability not found".
     *
     * What we copy:
     *   - PCI_STATUS (0x06-0x07): carries PCI_STATUS_CAP_LIST (bit 4).
     *   - PCI_CAPABILITY_LIST (0x34): head pointer of the capability chain.
     *   - The full capability region (0x40-0xff): where libvfio-user places
     *     the MSI and MSI-X capability structures.
     *
     * What we deliberately skip:
     *   - BARs (0x10-0x27): owned by the RC; must not be overwritten.
     *   - The header fields already copied in step 4 above (0x00-0x0f,
     *     0x28-0x3f): already consistent; re-copying is harmless but
     *     unnecessary.
     */
    {
        vfu_pci_config_space_t *vfu_cfg = vfu_pci_get_config_space(s->vfu);
        if (vfu_cfg) {
            uint8_t *vfu_raw = (uint8_t *)vfu_cfg;

            /* PCI Status register — propagate PCI_STATUS_CAP_LIST */
            memcpy(s->pcie_config + PCI_STATUS,
                   vfu_raw + PCI_STATUS,
                   sizeof(uint16_t));

            /* Capability list head pointer */
            s->pcie_config[PCI_CAPABILITY_LIST] =
                vfu_raw[PCI_CAPABILITY_LIST];

            /*
             * Standard capability area: 0x40–0xff.
             * libvfio-user places MSI at the first free slot (typically 0x40)
             * and MSI-X immediately after.  Copy the full 192-byte window so
             * the Linux capability walker finds both chains intact.
             */
            memcpy(s->pcie_config + 0x40, vfu_raw + 0x40, 0xc0);

            trace_qepc_ctrl_handle_start_caps_synced(*(uint16_t *)(s->pcie_config + PCI_STATUS), s->pcie_config[PCI_CAPABILITY_LIST]);
        } else {
            trace_qepc_ctrl_handle_start_caps_null();
        }
    }

    s->vfu_fd = vfu_get_poll_fd(s->vfu);
    if (s->vfu_fd < 0) {
        trace_qepc_ctrl_handle_start_poll_fd_failed(s->vfu_fd, errno);
        return -1;
    }

    trace_qepc_ctrl_handle_start_listening(s->sock_path, s->vfu_fd);
    qemu_set_fd_handler(s->vfu_fd, qepc_vfu_attach_ctx, NULL, s);

    return 0;
}
