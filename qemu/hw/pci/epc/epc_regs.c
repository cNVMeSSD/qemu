/*
 * epc_regs.c — Guest MMIO handlers for the QEMU PCI Endpoint Controller
 *
 * This file implements the MMIO read/write handlers for two of the EPC's
 * guest-visible BAR regions:
 *
 *   1. The Control BAR (qepc_ctrl_mmio_read / qepc_ctrl_mmio_write):
 *      The guest driver uses this region to:
 *        - Start the vfio-user backend (QEPC_CTRL_OFF_START).
 *        - Query the outbound window aperture base and size
 *          (QEPC_CTRL_OFF_WIN_START / QEPC_CTRL_OFF_WIN_SIZE).
 *        - Configure outbound (OB) translation windows: select a window index
 *          via QEPC_CTRL_OFF_OB_IDX, then write its physical, PCI, and size
 *          registers before enabling it via QEPC_CTRL_OFF_OB_ENABLE.
 *        - Trigger interrupts to the remote endpoint via
 *          QEPC_CTRL_OFF_IRQ_TYPE and QEPC_CTRL_OFF_IRQ_NUM.
 *
 *   2. The BAR Configuration region (qepc_cfg_bar_read / qepc_cfg_bar_write):
 *      The guest driver uses this region to program per-BAR properties
 *      (physical address, size, flags, enable mask) that are reflected into
 *      both QEPCState::bars[] and the device's PCIe configuration space image
 *      used by the vfio-user backend.
 *
 * The two helpers qepc_handle_ctrl_irq() and qepc_handle_single_window_setup()
 * are defined here because they are called only from qepc_ctrl_mmio_write().
 * Forward declarations at the top of this file break the textual dependency
 * so that qepc_ctrl_mmio_write() can appear after its callees without needing
 * to reorder the logical grouping of read-then-write handlers.
 *
 * All functions in this file are part of the public API declared in
 * qemu_epc.h — none are file-static.
 */

#include "qemu_epc.h"

/* ---------------------------------------------------------------------------
 * Forward declarations
 *
 * qepc_ctrl_mmio_write() calls these two helpers, which are defined later in
 * this file. Declaring them here avoids reordering the functions and keeps the
 * read handler immediately before the write handler for readability.
 * ------------------------------------------------------------------------- */
void qepc_handle_ctrl_irq(QEPCState *s, int irq_num);
void qepc_handle_single_window_setup(QEPCState *s, uint32_t idx, bool enable);

/* ---------------------------------------------------------------------------
 * Control BAR — read handler
 * ------------------------------------------------------------------------- */

/*
 * qepc_ctrl_mmio_read
 *
 * MMIO read handler for the EPC control BAR.
 *
 * Handles reads from:
 *   - QEPC_CTRL_OFF_WIN_START : Lower/upper 32 bits of the OB window aperture
 *                               base address as mapped in the guest's physical
 *                               address space.
 *   - QEPC_CTRL_OFF_WIN_SIZE  : Lower/upper 32 bits of the total OB aperture
 *                               size (NUM_OB_WINDOW * OB_WINDOW_SIZE).
 *   - QEPC_CTRL_OFF_OB_MASK   : Bitmask of currently active outbound windows.
 *
 * All registers are naturally aligned 32- or 64-bit accesses. Accesses of
 * other sizes are logged but still serviced. Unrecognised offsets return 0.
 */
uint64_t qepc_ctrl_mmio_read(void *opaque, hwaddr addr, unsigned size) {
  QEPCState *s = opaque;

  /* Log every read for debugging purposes */
  trace_qepc_ctrl_read((uint64_t)addr, size);

  /*
   * All registers are naturally aligned and sized accesses are expected.
   * Log unaligned accesses for debugging, but do not enforce strict size.
   */
  if (size != sizeof(uint32_t) && size != sizeof(uint64_t)) {
    trace_qepc_ctrl_read_unaligned(size);
  }

  switch (addr) {
  case QEPC_CTRL_OFF_WIN_START:
    /* Return the lower 32 bits of the outbound window base address */
    return s->ob_window_mr.addr;
  case QEPC_CTRL_OFF_WIN_START + sizeof(uint32_t):
    /* Return the upper 32 bits of the outbound window base address */
    return s->ob_window_mr.addr >> 32;
  case QEPC_CTRL_OFF_WIN_SIZE:
    /*
     * Return the lower 32 bits of the total outbound aperture size.
     *
     * The EPC memory pool (pci_epc_mem_init) is initialised with this value
     * as its capacity.  Returning only OB_WINDOW_SIZE (one slot) meant that
     * pci_epc_mem_alloc_addr() would only ever hand out addresses in the
     * first window's physical range, which is correct — but we should
     * advertise the full aperture so that the pool can in principle service
     * many simultaneous allocations without exhaustion.
     */
    return ((uint64_t)NUM_OB_WINDOW * OB_WINDOW_SIZE) & 0xffffffff;
  case QEPC_CTRL_OFF_WIN_SIZE + sizeof(uint32_t):
    /* Return the upper 32 bits of the total outbound aperture size */
    return ((uint64_t)NUM_OB_WINDOW * OB_WINDOW_SIZE) >> 32;
  case QEPC_CTRL_OFF_OB_MASK:
    /* Return the bitmask of currently active outbound mappings */
    return s->ob_mask;
  case QEPC_CTRL_OFF_MSI_CFG:
    /* Return the number of MSI vectors configured by the guest driver */
    return s->msi_num;
  case QEPC_CTRL_OFF_MSIX_NUM:
    /* Return the number of MSI-X vectors configured by the guest driver */
    return s->msix_num;

  case QEPC_CTRL_OFF_DMA_STATUS:
    return s->dma_status;

  case QEPC_CTRL_OFF_DMA_SRC:
    return (uint32_t)s->dma_src;
  case QEPC_CTRL_OFF_DMA_SRC + sizeof(uint32_t):
    return s->dma_src >> 32;

  case QEPC_CTRL_OFF_DMA_DST:
    return (uint32_t)s->dma_dst;
  case QEPC_CTRL_OFF_DMA_DST + sizeof(uint32_t):
    return s->dma_dst >> 32;

  case QEPC_CTRL_OFF_DMA_LEN:
    return s->dma_len;

  default:
    /* For any unhandled offset, log and return 0 */
    trace_qepc_ctrl_read_unhandled((uint64_t)addr);
  }

  return 0;
}

/* ---------------------------------------------------------------------------
 * IRQ helper
 * ------------------------------------------------------------------------- */

/*
 * qepc_handle_ctrl_irq
 *
 * Handles a write to the QEPC_CTRL_OFF_IRQ_NUM control register. Translates
 * the requested interrupt number into a libvfio-user trigger call appropriate
 * for the IRQ type that was previously recorded via QEPC_CTRL_OFF_IRQ_TYPE.
 *
 * IRQ type semantics:
 *   - IRQ_TYPE_INTX : A single INTx line is always triggered at subindex 0;
 *                     irq_num is ignored.
 *   - IRQ_TYPE_MSI  : irq_num selects the MSI vector; the subindex passed to
 *                     vfu_irq_trigger() is (irq_num - 1) to convert the
 *                     1-based guest value to a 0-based libvfio-user index.
 *   - Other types   : Logged and silently dropped; MSI-X support is not yet
 *                     implemented.
 *
 * TODO: Add IRQ_TYPE_MSIX support once the device model exposes MSI-X tables
 *       to the vfio-user backend.
 */
void qepc_handle_ctrl_irq(QEPCState *s, int irq_num) {
  int ret;

  trace_qepc_handle_ctrl_irq(irq_num, s->irq_type);

  switch (s->irq_type) {
  case IRQ_TYPE_INTX:
    /*
     * INTx is a single shared line.  libvfio-user subindex 0 corresponds
     * to INTA#.  The irq_num argument is ignored for INTx.
     */
    ret = vfu_irq_trigger(s->vfu, 0);
    break;

  case IRQ_TYPE_MSI:
    /*
     * MSI: subindex is 0-based vector number.  The guest driver passes a
     * 1-based interrupt_num (matching the Linux pci_epc_raise_irq() convention
     * used by e.g. cdns_pcie_ep_send_msi_irq), so subtract 1 here.
     */
    ret = vfu_irq_trigger(s->vfu, irq_num - 1);
    break;

  case IRQ_TYPE_MSIX:
    /*
     * MSI-X: subindex is the 0-based vector table entry index.  Same
     * 1-based-to-0-based conversion as MSI.
     *
     * libvfio-user knows which eventfd to signal because the Host wrote the
     * MSI-X table (msg_addr / msg_data per entry) via the BAR region callback,
     * and vfu_pci_add_capability() registered the table layout so the library
     * can intercept those writes and arm the correct eventfd.
     */
    ret = vfu_irq_trigger(s->vfu, irq_num - 1);
    break;

  default:
    trace_qepc_handle_ctrl_irq_unsupported(s->irq_type);
    return;
  }

  if (ret < 0) {
    trace_qepc_handle_ctrl_irq_failed(ret, errno);
  }
}

/* ---------------------------------------------------------------------------
 * Outbound window helper
 * ------------------------------------------------------------------------- */

/*
 * qepc_handle_single_window_setup
 *
 * Enables or disables a single outbound (OB) translation window identified by
 * @idx. This function is called in response to a write to the
 * QEPC_CTRL_OFF_OB_ENABLE register after the guest has already programmed the
 * window's PHYS, PCI, and SIZE fields.
 *
 * Enable path (@enable == true):
 *   - Window 0 is reserved for MSI/MSI-X interrupts. It is marked active in
 *     ob_mask but does not require a host virtual address mapping.
 *   - For data windows (idx > 0) the function walks rc_mrs[] to find a Root
 *     Complex DMA region whose IOVA range covers the configured PCI address.
 *     When a match is found it initialises a RAM-pointer MemoryRegion backed
 *     by the host virtual address and inserts it as a subregion of the OB
 *     window aperture at offset (idx * OB_WINDOW_SIZE).
 *   - If the window was previously mapped (ob_fast_mapped[idx] == true) the
 *     old subregion is removed and unparented before the new one is installed,
 *     allowing the guest to reconfigure an active window without a full reset.
 *   - If no matching RC region is found the function logs a debug message and
 *     returns without updating ob_mask; the window remains inactive.
 *
 * Disable path (@enable == false):
 *   - If the window is currently mapped the subregion is removed and unparented.
 *   - The corresponding ob_mask bit is always cleared, regardless of whether
 *     a mapping existed, so the guest reliably sees the window as inactive.
 *
 * Size alignment:
 *   QEMU's RAM-pointer MemoryRegion requires the size to be at least one page
 *   (4 KiB). map_size is therefore rounded up to 4096 when the configured
 *   window size is smaller.
 *
 * NOTE: @idx must be in [0, NUM_OB_WINDOW). Values outside this range are
 * silently ignored.
 */
void qepc_handle_single_window_setup(QEPCState *s, uint32_t idx, bool enable) {
  if (idx >= NUM_OB_WINDOW)
    return;

  /* Handle DISABLE Command */
  if (!enable) {
    if (s->ob_fast_mapped[idx]) {
      trace_qepc_ob_window_unmap(idx);
      memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[idx]);
      object_unparent(OBJECT(&s->rc_local_mr[idx]));
      s->ob_fast_mapped[idx] = false;
    }
    /* Ensure mask bit is cleared so the guest sees the window is inactive */
    s->ob_mask &= ~(1U << idx);
    return;
  }

  /* Handle ENABLE Command */
  if (s->obs[idx].size == 0)
    return;

  /* MSI window: no DMA mapping required */
  if (idx == 0) {
    trace_qepc_ob_window_enable_msi(idx, s->obs[idx].pci);

    /* Ensure no stale fast mapping exists */
    if (s->ob_fast_mapped[idx]) {
      memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[idx]);
      object_unparent(OBJECT(&s->rc_local_mr[idx]));
      s->ob_fast_mapped[idx] = false;
    }

    /* Mark window active even without vaddr */
    s->ob_mask |= (1U << idx);
    return;
  }

  /*
   * QEMU RAM pointers and subregions require page alignment (typically 4KB).
   * Round up the size to the next 4 KiB boundary so that:
   *   a) the MemoryRegion is never smaller than one page, and
   *   b) a non-page-aligned size (e.g. 0xfa001) doesn't leave a partial
   *      page at the tail that QEMU's memory subsystem may reject.
   */
  uint64_t map_size = ROUND_UP(s->obs[idx].size, 4096);
  if (map_size == 0)
    map_size = 4096;

  for (int j = 0; j < SUPPORT_RC_NUM_MRS; j++) {
    /* Skip empty or unmapped Root Complex regions */
    if (s->rc_mrs[j].size == 0 || s->rc_mrs[j].vaddr == NULL)
      continue;

    /*
     * Verify that the target PCI address fits entirely within a registered
     * Host RC region. Both the start and end of the window must be covered.
     */
    if (s->obs[idx].pci >= s->rc_mrs[j].rc_phys &&
        (s->obs[idx].pci + s->obs[idx].size) <=
            (s->rc_mrs[j].rc_phys + s->rc_mrs[j].size)) {

      uint64_t off = s->obs[idx].pci - s->rc_mrs[j].rc_phys;
      void *host_vaddr = (uint8_t *)s->rc_mrs[j].vaddr + off;

      /*
       * The window offset within ob_window_mr must match the byte offset of
       * obs[idx].phys from the aperture base address (ob_window_mr.addr).
       * This is the physical address that pci_epc_mem_alloc_addr() handed to
       * the EP driver, so ioremap(obs[idx].phys) maps exactly this offset.
       *
       * Using idx * OB_WINDOW_SIZE was wrong: the EPC memory pool allocates
       * addresses sequentially from the aperture base without honouring
       * window-index boundaries, so window N's physical address is NOT
       * necessarily at base + N * OB_WINDOW_SIZE.
       */
      hwaddr window_offset = (hwaddr)(s->obs[idx].phys - s->ob_window_mr.addr);

      /*
       * Clean up any existing mapping if the guest is re-configuring an
       * active window. Remove the old subregion before installing the new one
       * to avoid duplicate subregion entries in the container.
       */
      if (s->ob_fast_mapped[idx]) {
        trace_qepc_ob_window_unmap(idx);
        memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[idx]);
        object_unparent(OBJECT(&s->rc_local_mr[idx]));
      }

      char mr_name[32];
      snprintf(mr_name, sizeof(mr_name), "epc-ob-%d", idx);

      /* Initialize the RAM pointer to the Host's virtual memory */
      memory_region_init_ram_ptr(&s->rc_local_mr[idx], OBJECT(s), mr_name,
                                 map_size, host_vaddr);

      /* Add as a subregion to the main Outbound Window BAR container */
      memory_region_add_subregion(&s->ob_window_mr, window_offset,
                                  &s->rc_local_mr[idx]);

      s->ob_fast_mapped[idx] = true;
      s->ob_mask |= (1U << idx);

      trace_qepc_ob_window_enabled(idx, (uint64_t)window_offset, s->obs[idx].pci);
      return;
    }
  }

  trace_qepc_ob_window_no_rc_region(s->obs[idx].pci);
}

/* ---------------------------------------------------------------------------
 * Software DMA engine helper
 * ------------------------------------------------------------------------- */

/*
 * qepc_handle_dma_start
 *
 * Executes a DMA MEMCPY transfer requested by the guest driver:
 *
 *   s->dma_src  — source guest PA (may be an outbound-window slot, which
 *                 pci_dma_read dispatches through ob_window_mr and therefore
 *                 reads from the RC's vaddr-backed RAM subregion)
 *   s->dma_dst  — destination guest PA (normally guest RAM)
 *   s->dma_len  — byte count
 *
 * Uses pci_dma_read / pci_dma_write so that the EPC device's DMA address
 * space (= system address space without IOMMU) resolves both addresses
 * correctly.  The transfer is synchronous; dma_status is set before return.
 *
 * Called from qepc_ctrl_mmio_write with the BQL held — no additional
 * locking is needed.
 */
void qepc_handle_dma_start(QEPCState *s)
{
    void *buf;
    MemTxResult ret;

    if (s->dma_len == 0) {
        s->dma_status = 1; /* zero-length transfer always succeeds */
        return;
    }

    buf = g_malloc(s->dma_len);
    if (!buf) {
        trace_qepc_dma_start_alloc_failed(s->dma_len);
        s->dma_status = (uint32_t)-1;
        return;
    }

    /* Read from source (OB window or guest RAM) */
    ret = pci_dma_read(&s->dev, (dma_addr_t)s->dma_src, buf,
                       (dma_addr_t)s->dma_len);
    if (ret != MEMTX_OK) {
        trace_qepc_dma_start_read_failed((uint32_t)ret, s->dma_src, s->dma_len);
        g_free(buf);
        s->dma_status = (uint32_t)-1;
        return;
    }

    /* Write to destination (guest RAM) */
    ret = pci_dma_write(&s->dev, (dma_addr_t)s->dma_dst, buf,
                        (dma_addr_t)s->dma_len);
    if (ret != MEMTX_OK) {
        trace_qepc_dma_start_write_failed((uint32_t)ret, s->dma_dst, s->dma_len);
        g_free(buf);
        s->dma_status = (uint32_t)-1;
        return;
    }

    g_free(buf);
    s->dma_status = 1; /* success */
    trace_qepc_dma_start_complete(s->dma_src, s->dma_dst, s->dma_len);
}

/* ---------------------------------------------------------------------------
 * Control BAR — write handler
 * ------------------------------------------------------------------------- */

/*
 * qepc_ctrl_mmio_write
 *
 * MMIO write handler for the EPC control BAR.
 *
 * Handles writes to:
 *   - QEPC_CTRL_OFF_START    : Initializes/starts the vfio-user backend.
 *                              Delegates to qepc_ctrl_handle_start().
 *   - QEPC_CTRL_OFF_IRQ_TYPE : Records the IRQ delivery type (INTx / MSI /
 *                              MSI-X) for subsequent QEPC_CTRL_OFF_IRQ_NUM
 *                              writes.
 *   - QEPC_CTRL_OFF_IRQ_NUM  : Triggers an interrupt to the remote endpoint
 *                              by calling qepc_handle_ctrl_irq().
 *   - QEPC_CTRL_OFF_OB_ENABLE: Enables or disables the outbound window
 *                              currently selected by ob_idx.
 *   - QEPC_CTRL_OFF_OB_IDX   : Selects which outbound window subsequent
 *                              OB_PHYS/OB_PCI/OB_SIZE/OB_ENABLE writes apply
 *                              to. Must be in [0, NUM_OB_WINDOW).
 *   - QEPC_CTRL_OFF_OB_PHYS  : Lower/upper 32 bits of the host physical
 *                              address for the selected outbound window.
 *   - QEPC_CTRL_OFF_OB_PCI   : Lower/upper 32 bits of the PCI/IOVA address
 *                              for the selected outbound window.
 *   - QEPC_CTRL_OFF_OB_SIZE  : Lower/upper 32 bits of the size of the
 *                              selected outbound window.
 *
 * All 64-bit fields are split across two naturally aligned 32-bit accesses
 * (low word at the base offset, high word at base + 4). The low and high
 * halves are merged using read-modify-write to avoid tearing.
 *
 * Unrecognized offsets are logged but ignored.
 */
void qepc_ctrl_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size) {
  QEPCState *s = opaque;
  uint64_t tmp;

  trace_qepc_ctrl_write((uint64_t)addr, size, val);

  if (size != sizeof(uint32_t) && size != sizeof(uint64_t)) {
    trace_qepc_ctrl_write_unaligned(size);
  }

  switch (addr) {
  case QEPC_CTRL_OFF_START:
    qepc_ctrl_handle_start(s, val);
    return;

  case QEPC_CTRL_OFF_IRQ_TYPE:
    trace_qepc_ctrl_write_irq_type(val);
    s->irq_type = val;
    break;

  case QEPC_CTRL_OFF_IRQ_NUM:
    trace_qepc_ctrl_write_irq_num(val);
    qepc_handle_ctrl_irq(s, val);
    break;

  case QEPC_CTRL_OFF_OB_ENABLE: {
    uint8_t before_mask = s->ob_mask;
    bool before_enabled = (before_mask & (1U << s->ob_idx)) != 0;
    trace_qepc_ctrl_write_ob_enable_before(s->ob_idx, (uint32_t)val,
                                           before_mask, before_enabled);

    if (val) {
      trace_qepc_ctrl_write_ob_enable_window(s->ob_idx,
                                             s->obs[s->ob_idx].phys,
                                             s->obs[s->ob_idx].pci,
                                             s->obs[s->ob_idx].size);
    } else {
      trace_qepc_ctrl_write_ob_disable_window(s->ob_idx);
    }

    qepc_handle_single_window_setup(s, s->ob_idx, val != 0);

    uint8_t after_mask = s->ob_mask;
    bool after_enabled = (after_mask & (1U << s->ob_idx)) != 0;
    trace_qepc_ctrl_write_ob_enable_after(s->ob_idx, after_mask, after_enabled);
  } break;
    break;
    break;

  case QEPC_CTRL_OFF_OB_IDX:
    if (val >= NUM_OB_WINDOW) {
      trace_qepc_ctrl_write_ob_idx_invalid(val);
      return;
    }
    s->ob_idx = val;
    trace_qepc_ctrl_write_ob_idx(s->ob_idx);
    break;

  /* --- Outbound window physical address --- */
  case QEPC_CTRL_OFF_OB_PHYS:
    tmp = s->obs[s->ob_idx].phys;
    tmp = (tmp & ~0xffffffffUL) | (val & 0xffffffff);
    s->obs[s->ob_idx].phys = tmp;
    break;
  case QEPC_CTRL_OFF_OB_PHYS + sizeof(uint32_t):
    tmp = s->obs[s->ob_idx].phys;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->obs[s->ob_idx].phys = tmp;
    trace_qepc_ctrl_write_ob_phys(s->ob_idx, tmp);
    break;

  /* --- Outbound window PCI address --- */
  case QEPC_CTRL_OFF_OB_PCI:
    tmp = s->obs[s->ob_idx].pci;
    tmp = (tmp & ~0xffffffffUL) | (val & 0xffffffff);
    s->obs[s->ob_idx].pci = tmp;
    break;
  case QEPC_CTRL_OFF_OB_PCI + sizeof(uint32_t):
    tmp = s->obs[s->ob_idx].pci;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->obs[s->ob_idx].pci = tmp;
    trace_qepc_ctrl_write_ob_pci(s->ob_idx, tmp);
    break;

  /* --- Outbound window size --- */
  case QEPC_CTRL_OFF_OB_SIZE:
    tmp = s->obs[s->ob_idx].size;
    tmp = (tmp & ~0xffffffffUL) | (val & 0xffffffff);
    s->obs[s->ob_idx].size = tmp;
    break;
  case QEPC_CTRL_OFF_OB_SIZE + sizeof(uint32_t):
    tmp = s->obs[s->ob_idx].size;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->obs[s->ob_idx].size = tmp;
    trace_qepc_ctrl_write_ob_size(s->ob_idx, tmp);
    break;

  /* -----------------------------------------------------------------------
   * MSI-X capability configuration registers.
   *
   * The guest driver writes these before QEPC_CTRL_OFF_START so that
   * qepc_ctrl_handle_start() can call vfu_pci_add_capability() with a fully
   * populated struct msixcap.  Writing QEPC_CTRL_OFF_MSIX_CFG with val=1
   * commits the parameters; the actual capability registration happens inside
   * qepc_ctrl_handle_start() when it is later called.
   * ----------------------------------------------------------------------- */

  case QEPC_CTRL_OFF_MSIX_NUM:
    if (val == 0 || val > 2048) {
      trace_qepc_ctrl_write_msix_num_invalid(val);
      return;
    }
    s->msix_num = (uint16_t)val;
    /*
     * MSI-X table size = num_vectors * 16 bytes (each entry is 4 × uint32_t).
     * PBA size = ceil(num_vectors / 8) bytes, rounded up to 8-byte alignment.
     */
    s->msix_table_size = s->msix_num * 16;
    s->msix_pba_size   = ((s->msix_num + 63) / 64) * 8;
    trace_qepc_ctrl_write_msix_num(s->msix_num, s->msix_table_size, s->msix_pba_size);
    break;

  case QEPC_CTRL_OFF_MSIX_TBL_BAR:
    if (val >= PCI_NUM_REGIONS) {
      trace_qepc_ctrl_write_msix_tbl_bar_invalid(val);
      return;
    }
    s->msix_table_bar = (uint8_t)val;
    trace_qepc_ctrl_write_msix_tbl_bar(s->msix_table_bar);
    break;

  case QEPC_CTRL_OFF_MSIX_TBL_OFF:
    s->msix_table_off = (uint32_t)val;
    trace_qepc_ctrl_write_msix_tbl_off(s->msix_table_off);
    break;

  case QEPC_CTRL_OFF_MSIX_PBA_BAR:
    if (val >= PCI_NUM_REGIONS) {
      trace_qepc_ctrl_write_msix_pba_bar_invalid(val);
      return;
    }
    s->msix_pba_bar = (uint8_t)val;
    trace_qepc_ctrl_write_msix_pba_bar(s->msix_pba_bar);
    break;

  case QEPC_CTRL_OFF_MSIX_PBA_OFF:
    s->msix_pba_off = (uint32_t)val;
    trace_qepc_ctrl_write_msix_pba_off(s->msix_pba_off);
    break;

  case QEPC_CTRL_OFF_MSIX_CFG:
    /*
     * Commit write: validate that all MSI-X parameters have been set.
     * The actual vfu_pci_add_capability() call happens in
     * qepc_ctrl_handle_start() which reads these fields from QEPCState.
     * Here we just log and sanity-check.
     */
    if (val != 1) {
      trace_qepc_ctrl_write_msix_cfg_ignore(val);
      break;
    }
    if (s->msix_num == 0) {
      trace_qepc_ctrl_write_msix_cfg_no_num();
      break;
    }
    trace_qepc_ctrl_write_msix_cfg_commit(s->msix_num, s->msix_table_bar, s->msix_table_off, s->msix_pba_bar, s->msix_pba_off);
    break;

  /* -----------------------------------------------------------------------
   * MSI capability configuration register.
   *
   * Write the desired number of MSI vectors (must be a power of two, 1-32).
   * As with MSI-X, the vfu_pci_add_capability() call itself is deferred to
   * qepc_ctrl_handle_start().
   * ----------------------------------------------------------------------- */

  case QEPC_CTRL_OFF_MSI_CFG:
    if (val == 0 || val > 32) {
      trace_qepc_ctrl_write_msi_cfg_invalid(val);
      return;
    }
    /* Enforce power-of-two requirement from the PCI MSI spec. */
    if (val & (val - 1)) {
      trace_qepc_ctrl_write_msi_cfg_not_pow2(val);
      return;
    }
    s->msi_num = (uint16_t)val;
    trace_qepc_ctrl_write_msi_cfg_commit(s->msi_num);
    break;

  /* -----------------------------------------------------------------------
   * Doorbell register configuration.
   *
   * The guest programs the BAR index and byte offset of the doorbell register
   * before calling START.  Any write from the Host (Root Complex side) to
   * that BAR offset will be intercepted in qepc_bar_access() and converted
   * into a local interrupt injection rather than a DMA write to Guest RAM.
   * ----------------------------------------------------------------------- */

  case QEPC_CTRL_OFF_DB_BAR:
    if (val >= PCI_NUM_REGIONS) {
      trace_qepc_ctrl_write_db_bar_invalid(val);
      return;
    }
    s->doorbell_bar = (uint8_t)val;
    trace_qepc_ctrl_write_db_bar(s->doorbell_bar);
    break;

  case QEPC_CTRL_OFF_DB_OFF:
    s->doorbell_offset = (uint32_t)val;
    trace_qepc_ctrl_write_db_off(s->doorbell_offset);
    break;

  /* -----------------------------------------------------------------------
   * Software DMA engine registers.
   *
   * The guest programs SRC, DST, LEN (each as one or two 32-bit writes),
   * then writes 1 to START.  The handler executes the copy synchronously
   * and writes the result into dma_status before returning, so the guest's
   * polling loop sees the completion immediately.
   * ----------------------------------------------------------------------- */

  case QEPC_CTRL_OFF_DMA_SRC:
    s->dma_src = (s->dma_src & ~(uint64_t)0xffffffff) | (uint32_t)val;
    break;
  case QEPC_CTRL_OFF_DMA_SRC + sizeof(uint32_t):
    s->dma_src = (s->dma_src & 0xffffffff) | (val << 32);
    break;

  case QEPC_CTRL_OFF_DMA_DST:
    s->dma_dst = (s->dma_dst & ~(uint64_t)0xffffffff) | (uint32_t)val;
    break;
  case QEPC_CTRL_OFF_DMA_DST + sizeof(uint32_t):
    s->dma_dst = (s->dma_dst & 0xffffffff) | (val << 32);
    break;

  case QEPC_CTRL_OFF_DMA_LEN:
    s->dma_len = (uint32_t)val;
    trace_qepc_ctrl_write_dma_len(s->dma_len);
    break;

  case QEPC_CTRL_OFF_DMA_STATUS:
    /* Read-only from guest perspective; writes are ignored. */
    trace_qepc_ctrl_write_dma_status_ignored();
    break;

  case QEPC_CTRL_OFF_DMA_START:
    if (val) {
      trace_qepc_ctrl_write_dma_start(s->dma_src, s->dma_dst, s->dma_len);
      s->dma_status = 0; /* mark busy / clear previous result */
      qepc_handle_dma_start(s);
    }
    break;

  default:
    trace_qepc_ctrl_write_invalid_addr((uint64_t)addr);
    break;
  }
}

/* ---------------------------------------------------------------------------
 * BAR configuration region — read handler
 * ------------------------------------------------------------------------- */

/*
 * qepc_cfg_bar_read
 *
 * MMIO read handler for the BAR configuration region. Currently only
 * exposes the bar_mask, which indicates which BARs are configured/enabled.
 *
 * TODO: Consider exposing additional per-BAR configuration state here
 *       (e.g. size, flags, physical address) for debugging or introspection.
 */
uint64_t qepc_cfg_bar_read(void *opaque, hwaddr addr, unsigned size) {
  QEPCState *s = opaque;

  switch (addr) {
  case QEPC_BAR_CFG_OFF_MASK:
    return s->bar_mask;
  default:
    break;
  }

  return 0;
}

/* ---------------------------------------------------------------------------
 * BAR configuration region — write handler
 * ------------------------------------------------------------------------- */

/*
 * qepc_cfg_bar_write
 *
 * MMIO write handler for the BAR configuration region.
 *
 * The BAR configuration region provides a simple programming model:
 *   - QEPC_BAR_CFG_OFF_MASK:      Bitmask of enabled BARs. This is an
 *                                  internal enable/disable view for the
 *                                  vfio-user side and does not directly modify
 *                                  the PCI config space BAR contents.
 *   - QEPC_BAR_CFG_OFF_NUMBER:    Selects the BAR index (0..PCI_NUM_REGIONS-1)
 *                                  that subsequent FLAGS / PHYS_ADDR / SIZE
 *                                  writes will apply to.
 *   - QEPC_BAR_CFG_OFF_FLAGS:     Stores attribute bits (memory/IO,
 *                                  prefetchable, 32/64-bit etc.) into
 *                                  bars[bar_no].flags and updates the
 *                                  corresponding low BAR dword in the PCIe
 *                                  configuration space image.
 *   - QEPC_BAR_CFG_OFF_PHYS_ADDR: Records the backing EPC-side physical
 *                                  address for this BAR in bars[bar_no].
 *                                  Does NOT overwrite the host-visible BAR
 *                                  address — the RC programs that during
 *                                  enumeration via config space accesses.
 *   - QEPC_BAR_CFG_OFF_SIZE:      Records the BAR size used for
 *                                  sizing/masking in bars[bar_no].size.
 *
 * Writes here are reflected into both QEPCState::bars[] and the device's
 * PCIe configuration space image (pcie_config[]), which is then used by the
 * guest and the vfio-user backend.
 *
 * TODO: Validate bar_no against the number of supported BARs before every
 *       write case, and return an error indicator to the guest on violation.
 */
void qepc_cfg_bar_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size) {
  QEPCState *s = opaque;
  uint8_t *ptr8;
  uint32_t lo;
  uint64_t tmp;

  trace_qepc_cfg_bar_write((uint64_t)addr, size, val);

  if (addr + size > QEPC_BAR_CFG_OFF_SIZE + 8) {
    /* TODO: Turn this into a hard error once the ABI is stable. */
    trace_qepc_cfg_bar_write_overrun((uint64_t)(addr + size));
  }

  switch (addr) {
  case QEPC_BAR_CFG_OFF_MASK:
    /*
     * BAR enable mask programmed by the guest driver. This is an internal
     * enable/disable view for the vfio-user side and does not directly
     * modify the PCI config space BAR contents.
     */
    s->bar_mask = val;
    trace_qepc_cfg_bar_write_mask(s->bar_mask);
    break;
  case QEPC_BAR_CFG_OFF_NUMBER:
    if (val >= PCI_NUM_REGIONS) {
      trace_qepc_cfg_bar_write_number_invalid(val);
      return;
    }
    s->bar_no = val;
    trace_qepc_cfg_bar_write_number(s->bar_no);
    break;
  case QEPC_BAR_CFG_OFF_FLAGS: {
    /*
     * Only modify the low 32-bit BAR dword attribute bits (memory/IO,
     * prefetchable, 32/64-bit etc). Do not touch the high dword here;
     * the host (RC) owns the BAR contents and will program them via
     * config space writes.
     */
    uint32_t attr = (uint32_t)val;
    uint8_t bar = s->bar_no;

    if (bar >= PCI_NUM_REGIONS) {
      trace_qepc_cfg_bar_write_flags_invalid(bar);
      return;
    }

    s->bars[bar].flags = attr;

    ptr8 = s->pcie_config + 0x10 + 4 * bar;
    memcpy(&lo, ptr8, sizeof(lo));

    /*
     * Update only the lowest nibble (attr bits) and preserve the address
     * bits that may already have been programmed by the RC. For now we also
     * force memory space (bit 0 cleared) and mark as 32-bit non-prefetchable
     * (bits 2:1 = 00b). 64-bit semantics will be enforced by how BAR pairs
     * are interpreted in qepc_pci_cfg_access_bar().
     */
    lo = (lo & ~0xF) | (attr & 0xF);
    memcpy(ptr8, &lo, sizeof(lo));

    trace_qepc_cfg_bar_write_flags(bar, lo, attr);
    break;
  }
  case QEPC_BAR_CFG_OFF_RSV:
    break;
  case QEPC_BAR_CFG_OFF_PHYS_ADDR: {
    /*
     * Program the backing physical address for this BAR on the EPC side.
     * Do not overwrite the host-visible BAR address here; the RC will
     * program the BAR registers during enumeration and resource assignment
     * via config space accesses.
     */
    uint8_t bar = s->bar_no;

    if (bar >= PCI_NUM_REGIONS) {
      trace_qepc_cfg_bar_write_phys_invalid(bar);
      return;
    }
    s->bars[bar].phys_addr = val;

    /*
     * For debugging only: read out the current low 32-bit BAR dword from
     * the config space image without modifying it, so that we don't trample
     * over the high dword or RC-programmed contents.
     */
    ptr8 = s->pcie_config + 0x10 + 4 * bar;
    memcpy(&tmp, ptr8, sizeof(uint64_t));
    trace_qepc_cfg_bar_write_phys(bar, val, tmp & 0xffffffffUL);
    break;
  }
  case QEPC_BAR_CFG_OFF_SIZE: {
    uint8_t bar = s->bar_no;

    if (bar >= PCI_NUM_REGIONS) {
      trace_qepc_cfg_bar_write_size_invalid(bar);
      return;
    }
    if (val == 0) {
      trace_qepc_cfg_bar_write_size_zero(bar);
      return;
    }
    s->bars[bar].size = val;
    trace_qepc_cfg_bar_write_size(bar, val);
    break;
  }
  }
}
