/*
 * QEMU PCI Endpoint Controller (EPC) device
 *
 * This device exposes a PCI endpoint controller that is backed by a vfio-user
 * device. The EPC has:
 *   - A control BAR for starting/stopping and configuring outbound windows,
 *   - A BAR to expose PCIe config space to the remote (vfio-user) side,
 *   - A BAR to configure EPC-side BARs (physical address, size, flags),
 *   - A BAR that represents a set of outbound (OB) windows.
 *
 * The vfio-user client sees the PCI config space and BARs via libvfio-user
 * callbacks. QEMU maps the BARs and outbound windows into its own address
 * space and uses DMA helpers to forward accesses.
 *
 * Expected guest driver programming sequence (high level):
 *   1. Program BAR configuration:
 *      - Select a BAR index via QEPC_BAR_CFG_OFF_NUMBER.
 *      - Write QEPC_BAR_CFG_OFF_SIZE with the desired size.
 *      - Write QEPC_BAR_CFG_OFF_FLAGS to configure BAR properties.
 *      - Write QEPC_BAR_CFG_OFF_PHYS_ADDR with the system physical address.
 *      - Set QEPC_BAR_CFG_OFF_MASK bits to enable the BAR.
 *   2. Start the vfio-user backend:
 *      - Write QEPC_CTRL_OFF_START to kick off backend initialization.
 *   3. Configure outbound windows:
 *      - Select a window via QEPC_CTRL_OFF_OB_IDX.
 *      - Program QEPC_CTRL_OFF_OB_PHYS with the system physical address.
 *      - Program QEPC_CTRL_OFF_OB_PCI with the PCI/IOVA address.
 *      - Program QEPC_CTRL_OFF_OB_SIZE with the window size.
 *      - Update QEPC_CTRL_OFF_OB_MASK to enable the window.
 *   4. Trigger interrupts to the remote endpoint as needed via
 *      QEPC_CTRL_OFF_IRQ_TYPE and QEPC_CTRL_OFF_IRQ_NUM.
 *
 * TODO: Document the detailed register layout and any alignment/size
 *       requirements in an external specification.
 */

#include "qemu/osdep.h"
#include "qom/object.h"

#include "qemu/log.h"
#include "qemu/memalign.h"

#include "qapi/error.h"
#include "qapi/qapi-visit-sockets.h"

#include "hw/pci/msi.h"
#include "hw/pci/pci_device.h"
#include "hw/remote/vfio-user-obj.h"

#include "libvfio-user.h"

/* Uncomment to enable verbose debug logging for this device. */
#define DEBUG_QEMU_EPC
#ifdef DEBUG_QEMU_EPC
/* Lightweight debug logging helper, compiled out when DEBUG_QEMU_EPC is unset.
 */
#define qemu_epc_debug(fmt, ...) qemu_log("qemu_epc: " fmt "\n", ##__VA_ARGS__)
#else
#define qemu_epc_debug(...)                                                    \
  do {                                                                         \
  } while (0)
#endif

/* Number of outbound windows the controller exposes. */
#define NUM_OB_WINDOW 5
/* Size of each outbound window (guest-visible aperture size). */
#define OB_WINDOW_SIZE 0x40000000ULL

/*
 * Maximum number of registered root complex (RC) memory regions.
 * These correspond to DMA regions registered by the vfio-user client.
 */
#define SUPPORT_RC_NUM_MRS 5

/*
 * QEPCState
 *
 * Per-device state for the QEMU EPC implementation. This holds:
 *  - The underlying QEMU PCI device,
 *  - The vfio-user context (vfu_ctx_t) and associated poll fd,
 *  - BAR configuration programmed via the BAR configuration MMIO region,
 *  - Outbound window configuration and masks,
 *  - Registered RC DMA regions (used to back outbound mappings),
 *  - The various MemoryRegion objects that represent MMIO regions and the
 *    outbound window aperture.
 */

struct QEPCState {
  /*< private >*/
  /* Underlying QEMU PCI device (must be first for PCIDeviceClass). */
  PCIDevice dev;

  /* libvfio-user context and its pollable file descriptor. */
  vfu_ctx_t *vfu;
  int vfu_fd;

  /*
   * Path to the Unix domain socket on which this EPC listens for
   * vfio-user client connections. Set via the 'path' object property.
   */
  const char *sock_path;

  /*< public >*/
  /* MMIO regions exposed to the guest. */
  MemoryRegion ctrl_mr, pci_cfg_mr, bar_cfg_mr;
  /* Outbound window aperture BAR. */
  MemoryRegion ob_window_mr;

  QemuThread thread;

  /*
   * Per-BAR configuration as programmed via the BAR configuration MMIO
   * region. These values are used both to program the guest-visible
   * PCI config space and to back vfio-user BAR accesses.
   */
  struct {
    uint64_t phys_addr; /* System physical address backing this BAR. */
    uint64_t size;      /* Size of the BAR aperture in bytes. */
    uint8_t flags; /* Implementation-defined flags (e.g., IO/MEM, prefetch). */
  } bars[6];

  /*
   * Bitmask of enabled/configured BARs (bit N corresponds to BAR N).
   * bar_no is the current BAR selected for programming through the
   * BAR configuration MMIO interface.
   */
  uint8_t bar_mask;
  uint8_t bar_no;

  /* Backing store for the device's PCIe configuration space. */
  uint8_t *pcie_config;
  /* Interrupt type (INTx/MSI/MSI-X), currently only INTx is used. */
  uint8_t irq_type;

  /*
   * Outbound window definitions:
   *  - phys: system physical address backing this outbound window,
   *  - pci:  PCI address as seen by the vfio-user client,
   *  - size: size of the window in bytes.
   */
  struct {
    uint64_t phys;
    uint64_t pci;
    uint64_t size;
  } obs[NUM_OB_WINDOW];
  /*
   * Bitmask of enabled outbound windows (bit N corresponds to window N).
   * ob_idx is the index of the outbound window currently being programmed.
   */
  uint8_t ob_mask;
  uint8_t ob_idx;

  /*
   * Root complex (RC) DMA mappings registered by the vfio-user client.
   * rc_phys is the client's IOVA/RC physical address,
   * vaddr is the local virtual address backing the region in this process.
   *
   * vaddr may be NULL if the client registered the region without mapping it
   * into this process; in that case, accesses are not supported.
   * size is the region length.
   */
  struct rc_mr {
    uint64_t rc_phys;
    void *vaddr;
    uint64_t size;
  } rc_mrs[SUPPORT_RC_NUM_MRS];
  /* Number of valid entries in rc_mrs[] (also next free index). */
  int rc_mr_idx;

  MemoryRegion rc_local_mr[NUM_OB_WINDOW];

  /*
   * Tracks whether an outbound window is currently mapped via a fast-path
   * local MemoryRegion (rc_local_mr[i]). When false, accesses must go
   * through the slow-path helpers qepc_rc_dma_read/qepc_rc_dma_write.
   */
  bool ob_fast_mapped[NUM_OB_WINDOW];

  /*
   * MSI data
   */
  uint32_t msi_vectors; /* Number of MSI vectors */
  uint64_t msi_addr;    /* MSI message address */
  uint32_t msi_data;    /* MSI message data */
  uint32_t msi_vector;  /* Currently mapped MSI vector */
};

/* QOM type name for this device. */
#define TYPE_QEMU_EPC "qemu-epc"
OBJECT_DECLARE_SIMPLE_TYPE(QEPCState, QEMU_EPC)

#define QEPC_REVISION 0x00

#define QEPC_BAR_CFG_SIZE 1024

/*
 * BAR indices exposed to the guest:
 *   QEPC_BAR_CTRL       - Control and status registers for the EPC.
 *   QEPC_BAR_PCI_CFG    - MMIO view of PCIe config space (backed by
 *                         pcie_config).
 *   QEPC_BAR_BAR_CFG    - MMIO interface to configure EPC BARs.
 *   QEPC_BAR_OB_WINDOWS - Outbound window aperture.
 */
enum {
  QEPC_BAR_CTRL = 0,
  QEPC_BAR_PCI_CFG = 1,
  QEPC_BAR_BAR_CFG = 2,
  QEPC_BAR_OB_WINDOWS = 3,
};

/* IRQ types supported by the EPC device */
enum qepc_irq_type {
  IRQ_TYPE_UNKNOWN = 0, /* Unknown interrupt, ignore */
  IRQ_TYPE_INTX = 1,    /* Legacy INTx interrupt */
  IRQ_TYPE_MSI = 2,     /* MSI interrupt */
  IRQ_TYPE_MSIX = 3,    /* MSI-X interrupt */
};

/*
 * Control BAR register offsets.
 *
 * These registers are accessed via qepc_ctrl_mmio_read/qepc_ctrl_mmio_write
 * and are used by the guest driver to:
 *  - start the vfio-user backend,
 *  - configure and enable outbound windows,
 *  - trigger interrupts.
 */
enum {
  /* Write: kick off vfio-user backend initialization */
  QEPC_CTRL_OFF_START = 0x00,

  /* Endpoint outbound window configuration */
  QEPC_CTRL_OFF_WIN_START = 0x08, /* Window start physical address (uint64_t) */
  QEPC_CTRL_OFF_WIN_SIZE = 0x10,  /* Window size in bytes (uint64_t) */

  /* Interrupt control (INTx / MSI / MSI-X) */
  QEPC_CTRL_OFF_IRQ_TYPE = 0x18, /* Interrupt type selector (uint32_t) */
  QEPC_CTRL_OFF_IRQ_NUM = 0x1c,  /* IRQ / vector number (uint32_t) */

  /* Outbound address mapping */
  QEPC_CTRL_OFF_OB_MASK =
      0x20,                    /* Active outbound mappings bitmask (uint32_t) */
  QEPC_CTRL_OFF_OB_IDX = 0x24, /* Current outbound mapping index (uint32_t) */
  QEPC_CTRL_OFF_OB_PHYS =
      0x28, /* Outbound mapping physical address (uint64_t) */
  QEPC_CTRL_OFF_OB_PCI = 0x30,  /* Outbound mapping PCI address (uint64_t) */
  QEPC_CTRL_OFF_OB_SIZE = 0x38, /* Outbound mapping size (uint64_t) */

  /* MSI (single-message) support */
  QEPC_CTRL_OFF_MSI_CTRL = 0x40, /* MSI enable + vector count (uint64_t) */
  QEPC_CTRL_OFF_MSI_ADDR = 0x48, /* MSI message address (uint64_t) */
  QEPC_CTRL_OFF_MSI_DATA = 0x50, /* MSI message data (uint32_t, padded) */

  /* MSI-X support */
  QEPC_CTRL_OFF_MSIX_CTRL = 0x58, /* MSI-X enable + function mask (uint64_t) */
  QEPC_CTRL_OFF_MSIX_COUNT =
      0x60, /* Number of MSI-X vectors supported (uint64_t) */
  QEPC_CTRL_OFF_MSIX_TABLE =
      0x68, /* Offset to MSI-X table MMIO region (uint64_t) */
  QEPC_CTRL_OFF_MSIX_PBA =
      0x70, /* Offset to MSI-X PBA MMIO region (uint64_t) */

  /* End of control register space */
  QEPC_CTRL_SIZE = QEPC_CTRL_OFF_MSIX_PBA + sizeof(uint64_t),
};

/*
 * qepc_ctrl_mmio_read
 *
 * MMIO read handler for the EPC control BAR.
 *
 * This exposes:
 *   - The base address and size of the outbound window aperture,
 *   - The current outbound window enable mask,
 *   - The number of MSI interrupts supported (for get_msi()).
 *
 * Unhandled offsets return 0 and log a debug message.
 */
static uint64_t qepc_ctrl_mmio_read(void *opaque, hwaddr addr, unsigned size) {
  QEPCState *s = opaque;

  /* Log every read for debugging purposes */
  qemu_epc_debug("CTRL read: addr 0x%lx, size 0x%x", addr, size);

  /*
   * All registers are naturally aligned and sized accesses are expected.
   * Log unaligned accesses for debugging, but do not enforce strict size.
   */
  if (size != sizeof(uint32_t) && size != sizeof(uint64_t)) {
    qemu_epc_debug("%s: unaligned access size %u", __func__, size);
  }

  switch (addr) {
  case QEPC_CTRL_OFF_WIN_START:
    /* Return the lower 32 bits of the outbound window base address */
    return s->ob_window_mr.addr;
  case QEPC_CTRL_OFF_WIN_START + sizeof(uint32_t):
    /* Return the upper 32 bits of the outbound window base address */
    return s->ob_window_mr.addr >> 32;
  case QEPC_CTRL_OFF_WIN_SIZE:
    /* Return the lower 32 bits of the outbound window size */
    return OB_WINDOW_SIZE & 0xffffffff;
  case QEPC_CTRL_OFF_WIN_SIZE + sizeof(uint32_t):
    /* Return the upper 32 bits of the outbound window size */
    return OB_WINDOW_SIZE >> 32;
  case QEPC_CTRL_OFF_OB_MASK:
    /* Return the bitmask of currently active outbound mappings */
    return s->ob_mask;

  /* --- MSI support for get_msi() --- */
  case QEPC_CTRL_OFF_MSI_CTRL:
    /*
     * Return the number of MSI vectors supported.
     * The kernel driver calls pci_epc_get_msi(), which reads this register.
     * This does not include the enable bit; only the count is returned.
     */
    return s->msi_vectors;

  default:
    /* For any unhandled offset, log and return 0 */
    qemu_epc_debug("unhandled read: off %ld", addr);
  }

  return 0;
}

/*
 * qepc_pci_cfg_access_bar
 *
 * Helper for qepc_pci_cfg_access that deals specifically with accesses
 * to PCI BAR registers in the config space.
 *
 * The function enforces BAR sizing semantics by masking the written
 * value with the complement of the BAR size (standard PCI BAR handling),
 * and updates the cached PCIe config space representation accordingly.
 */
static ssize_t qepc_pci_cfg_access_bar(QEPCState *s, char *const buf,
                                       size_t count, loff_t offset,
                                       const bool is_write) {
  uint32_t t1, t2;
  int bar_index;
  bool is_high_dword = false;

  qemu_epc_debug("%s: %s: offset 0x%lx, size 0x%lx", __func__,
                 is_write ? "write" : "read", offset, count);

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
      qemu_epc_debug("BAR%d low write: host=0x%x stored=0x%x size=0x%x",
                     bar_index, t1, t2, size);
    } else if (bar_index >= 0 && is_high_dword) {
      /*
       * High dword write of a 64-bit BAR: the host owns this value.
       * We simply store and return it verbatim. Masking is not applied
       * to the high 32 bits.
       */
      memcpy(s->pcie_config + offset, &t1, sizeof(t1));
      qemu_epc_debug("BAR%d high write: host=0x%x", bar_index, t1);
    } else {
      /*
       * Non-BAR or unsupported BAR offset: just store directly.
       */
      memcpy(s->pcie_config + offset, buf, count);
      qemu_epc_debug("generic BAR cfg write: off=0x%lx val=0x%x", offset, t1);
    }
  } else {
    memcpy(&t1, s->pcie_config + offset, sizeof(t1));
    qemu_epc_debug("cfg BAR read: off 0x%lx: value 0x%x", offset, t1);
    memcpy(buf, s->pcie_config + offset, count);
  }

  return count;
}

/*
 * qepc_pci_cfg_access
 *
 * libvfio-user callback that services accesses to the virtual PCI
 * configuration space as seen from the vfio-user client.
 *
 * - Bounds-checks all accesses against the config space size.
 * - Delegates BAR register accesses (0x10–0x27) to qepc_pci_cfg_access_bar
 *   in order to apply BAR sizing and masking semantics.
 * - For all other offsets, performs a straightforward read/write into the
 *   cached pcie_config buffer.
 *
 * Returns the number of bytes handled (0 on error).
 */
static ssize_t qepc_pci_cfg_access(vfu_ctx_t *vfu_ctx, char *const buf,
                                   size_t count, loff_t offset,
                                   const bool is_write) {
  QEPCState *s = vfu_get_private(vfu_ctx);

  qemu_epc_debug("%s: %s: offset 0x%lx, size 0x%lx", __func__,
                 is_write ? "write" : "read", offset, count);

  if (offset + count > PCIE_CONFIG_SPACE_SIZE) {
    return 0;
  }
  /* Route BAR register accesses through the BAR-specific helper. */
  if (offset >= 0x10 && offset < 0x28)
    return qepc_pci_cfg_access_bar(s, buf, count, offset, is_write);

  if (is_write) {
    memcpy(s->pcie_config + offset, buf, count);
    qemu_epc_debug("cfg access: write: count %ld", count);
  } else {
    qemu_epc_debug("cfg access: read: count %ld", count);
    memcpy(buf, s->pcie_config + offset, count);
  }

  return count;
}

/*
 * qepc_vfu_run
 *
 * Event loop callback used by QEMU's fd handler to drive the vfio-user
 * context. It repeatedly calls vfu_run_ctx() until:
 *   - the client disconnects (ENOTCONN), or
 *   - a fatal error occurs, or
 *   - vfu_run_ctx() returns 0.
 *
 * EINTR is ignored and retried.
 * TODO: Consider integrating with QEMU's event loop more tightly to detect
 *       and handle backend errors in a more structured way.
 */
static void qepc_vfu_run(void *opaque) {
  QEPCState *s = opaque;
  int err = -1;

  qemu_epc_debug("starting vfu main loop (fd=%d)", s->vfu_fd);

  while (err != 0) {
    err = vfu_run_ctx(s->vfu);
    if (err < 0) {
      if (errno == EINTR) {
        qemu_epc_debug("%s: vfu_run_ctx interrupted (EINTR), retrying",
                       __func__);
        continue;
      } else if (errno == ENOTCONN) {
        qemu_epc_debug("%s: vfu_run_ctx: client disconnected (ENOTCONN)",
                       __func__);
        break;
      } else {
        qemu_epc_debug("%s: vfu_run_ctx failed: err=%d errno=%d", __func__, err,
                       errno);
        break;
      }
    } else {
      qemu_epc_debug("%s: vfu_run_ctx returned 0, exiting loop", __func__);
    }
  }

  qemu_epc_debug("%s: leaving vfu main loop", __func__);
}

/*
 * qepc_vfu_attach_ctx
 *
 * fd handler invoked when there is a pending connection or attach event
 * on the vfio-user socket. It:
 *   - Detaches any existing handler on the current poll fd,
 *   - Calls vfu_attach_ctx() (with basic EAGAIN/EWOULDBLOCK retry),
 *   - Retrieves the poll fd for the attached context,
 *   - Registers qepc_vfu_run as the new handler for that fd.
 *
 * TODO: Add robust error reporting back to the guest when attachment fails.
 */
static void qepc_vfu_attach_ctx(void *opaque) {
  QEPCState *s = opaque;
  int err;

  qemu_epc_debug("%s: attach vfu client requested (fd=%d)", __func__,
                 s->vfu_fd);

  /*
   * If we already have a poll fd installed, remove its handler before
   * attempting to attach a new context. This prevents stale handlers
   * from firing.
   */
  if (s->vfu_fd >= 0) {
    qemu_epc_debug("%s: removing previous fd handler (fd=%d)", __func__,
                   s->vfu_fd);
    qemu_set_fd_handler(s->vfu_fd, NULL, NULL, NULL);
  }

retry:
  err = vfu_attach_ctx(s->vfu);
  if (err < 0) {
    if (err == EAGAIN || errno == EWOULDBLOCK) {
      qemu_epc_debug("%s: vfu_attach_ctx returned EAGAIN/EWOULDBLOCK, retrying",
                     __func__);
      goto retry;
    }
    qemu_epc_debug("%s: vfu_attach_ctx failed: %d (errno=%d)", __func__, err,
                   errno);
    return;
  }

  s->vfu_fd = vfu_get_poll_fd(s->vfu);
  if (s->vfu_fd < 0) {
    qemu_epc_debug("%s: vfu_get_poll_fd failed", __func__);
    return;
  }

  qemu_epc_debug("%s: vfu client attached, poll fd=%d", __func__, s->vfu_fd);
  qemu_set_fd_handler(s->vfu_fd, qepc_vfu_run, NULL, s);
}

/*
 * qepc_vfu_log
 *
 * Logging callback for libvfio-user. Currently just forwards the message
 * to QEMU's logging infrastructure using qemu_epc_debug().
 *
 * TODO: Map vfio-user log levels to QEMU log categories/levels if finer
 *       grained control is desired.
 */
static void qepc_vfu_log(vfu_ctx_t *vfu_ctx, int level, const char *msg) {
  qemu_epc_debug("vfu: %d: %s", level, msg);
}

/*
 * qepc_bar_access
 *
 * Common helper for libvfio-user BAR region callbacks. It:
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
static ssize_t qepc_bar_access(vfu_ctx_t *vfu_ctx, uint8_t barno,
                               char *const buf, size_t count, loff_t offset,
                               const bool is_write) {

  QEPCState *s = vfu_get_private(vfu_ctx);
  dma_addr_t phys_addr;
  qemu_epc_debug("%s: bar=%u offset=0x%lx size=0x%zx %s", __func__, barno,
                 (unsigned long)offset, count, is_write ? "write" : "read");

  if (barno >= PCI_NUM_REGIONS) {
    qemu_epc_debug("%s: invalid BAR %u", __func__, barno);
    return -1;
  }

  if (!(s->bar_mask & (1U << barno))) {
    qemu_epc_debug("%s: access to disabled BAR %u (bar_mask=0x%x)", __func__,
                   barno, s->bar_mask);
    return -1;
  }

  /*
   * Ensure that accesses stay within the configured BAR size. We currently
   * reject out-of-bounds accesses to avoid undefined behaviour.
   */
  if (offset < 0 || (uint64_t)offset + count > s->bars[barno].size) {
    qemu_epc_debug(
        "%s: BAR %u out-of-bounds (off=0x%lx, size=0x%zx, bar_size=0x%lx)",
        __func__, barno, (unsigned long)offset, count,
        (unsigned long)s->bars[barno].size);
    return -1;
  }

  phys_addr = s->bars[barno].phys_addr + offset;
  qemu_epc_debug("%s: resolved DMA addr=0x%lx", __func__,
                 (unsigned long)phys_addr);

  if (is_write) {
    pci_dma_write(&s->dev, phys_addr, buf, count);
  } else {
    pci_dma_read(&s->dev, phys_addr, buf, count);
  }
  return count;
}

/*
 * QEPC_ACCESS_BAR
 *
 * Convenience macro that generates a thin wrapper around qepc_bar_access()
 * for a specific BAR number. These wrappers are then used as per-BAR
 * access callbacks when registering regions with libvfio-user.
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

/* Signature for bar access callbacks used with libvfio-user. */
typedef ssize_t (*qepc_bar_access_handler_t)(vfu_ctx_t *vfu_ctx,
                                             char *const buf, size_t count,
                                             loff_t offset,
                                             const bool is_write);

static qepc_bar_access_handler_t qepc_bar_handlers[] = {
    qepc_bar_access_0, qepc_bar_access_1, qepc_bar_access_2,
    qepc_bar_access_3, qepc_bar_access_4, qepc_bar_access_5};

/*
 * qepc_dma_register
 *
 * libvfio-user DMA registration callback. The vfio-user client informs
 * us about an IOVA range (info->iova) backed by a host virtual address
 * (info->vaddr). We record these in rc_mrs[] and later use them to back
 * outbound mappings.
 *
 * TODO: Implement proper lifetime management for rc_mrs entries and
 *       use a more efficient lookup structure than a simple linear array.
 */
static void qepc_dma_register(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info) {
  QEPCState *s = vfu_get_private(vfu_ctx);

  qemu_epc_debug(
      "%s: register iova=[0x%lx..0x%lx) len=0x%lx vaddr=%p prot=0x%x", __func__,
      (uint64_t)info->iova.iov_base,
      (uint64_t)info->iova.iov_base + info->iova.iov_len,
      (uint64_t)info->iova.iov_len, info->vaddr, info->prot);

  if (!info->vaddr) {
    qemu_epc_debug("%s: shared memory is required: vaddr is NULL", __func__);
    return;
  }

  if (s->rc_mr_idx == SUPPORT_RC_NUM_MRS) {
    /* TODO: Grow this table dynamically or fail the DMA registration. */
    qemu_epc_debug("%s: unsupported: rc_mrs table full (%d entries, max=%d)",
                   __func__, s->rc_mr_idx, SUPPORT_RC_NUM_MRS);
    return;
  }

  s->rc_mrs[s->rc_mr_idx] = (struct rc_mr){
      .rc_phys = (uint64_t)info->iova.iov_base,
      .size = info->iova.iov_len,
      .vaddr = info->vaddr,
  };
  qemu_epc_debug("%s: stored rc_mrs[%d]: rc_phys=0x%lx size=0x%lx vaddr=%p",
                 __func__, s->rc_mr_idx,
                 (unsigned long)s->rc_mrs[s->rc_mr_idx].rc_phys,
                 (unsigned long)s->rc_mrs[s->rc_mr_idx].size,
                 s->rc_mrs[s->rc_mr_idx].vaddr);

  s->rc_mr_idx++;
}

/*
 * qepc_dma_unregister
 *
 * libvfio-user DMA unregistration callback. The vfio-user client removes
 * a previously registered DMA region.
 *
 * Currently this only logs the unregistration event and does not remove
 * entries from rc_mrs[].
 *
 * TODO: Match the unregistered region against rc_mrs[] and reclaim the slot
 *       so that it can be reused for future DMA registrations.
 */
static void qepc_dma_unregister(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info) {
  QEPCState *s = vfu_get_private(vfu_ctx);

  qemu_epc_debug("%s: unregister iova=[0x%lx..0x%lx) len=0x%lx vaddr=%p",
                 __func__, (uint64_t)info->iova.iov_base,
                 (uint64_t)info->iova.iov_base + info->iova.iov_len,
                 (uint64_t)info->iova.iov_len, info->vaddr);

  /*
   * Match the unregistered region against rc_mrs[] and reclaim the slot.
   * Also unmap any OB fast-path local MemoryRegion that referenced this rc_mr.
   */
  for (int i = 0; i < s->rc_mr_idx; i++) {
    if (s->rc_mrs[i].rc_phys == (uint64_t)info->iova.iov_base &&
        s->rc_mrs[i].size == info->iova.iov_len &&
        s->rc_mrs[i].vaddr == info->vaddr) {

      qemu_epc_debug(
          "%s: found matching rc_mrs[%d], rc_phys=0x%lx size=0x%lx vaddr=%p",
          __func__, i, (unsigned long)s->rc_mrs[i].rc_phys,
          (unsigned long)s->rc_mrs[i].size, s->rc_mrs[i].vaddr);

      /* Unmap any OB windows that were mapped using this rc_mr. */
      for (int w = 0; w < NUM_OB_WINDOW; w++) {
        if (!s->ob_fast_mapped[w]) {
          continue;
        }

        uint64_t pci = s->obs[w].pci;
        uint64_t sz = s->obs[w].size;

        if (pci >= s->rc_mrs[i].rc_phys &&
            (pci + sz) <= (s->rc_mrs[i].rc_phys + s->rc_mrs[i].size)) {
          qemu_epc_debug("%s: removing fast mapping for ob window %d "
                         "(pci=0x%lx size=0x%lx)",
                         __func__, w, (unsigned long)pci, (unsigned long)sz);

          /* Remove the per-window subregion from the OB aperture */
          memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[w]);

          s->ob_fast_mapped[w] = false;
          s->ob_mask &= ~(1 << w);
        }
      }

      /* Remove the rc_mr entry by shifting subsequent entries down. */
      for (int k = i; k + 1 < s->rc_mr_idx; k++) {
        s->rc_mrs[k] = s->rc_mrs[k + 1];
      }

      /* Clear the now-unused last slot */
      s->rc_mr_idx--;
      if (s->rc_mr_idx >= 0) {
        s->rc_mrs[s->rc_mr_idx].rc_phys = 0;
        s->rc_mrs[s->rc_mr_idx].size = 0;
        s->rc_mrs[s->rc_mr_idx].vaddr = NULL;
      }

      qemu_epc_debug("%s: rc_mrs compacted, new rc_mr_idx=%d", __func__,
                     s->rc_mr_idx);
      return;
    }
  }

  qemu_epc_debug("%s: no matching rc_mr found for unregister", __func__);
}

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
static int qepc_ctrl_handle_start(QEPCState *s, uint64_t val) {
  int err;

  qemu_epc_debug("%s: start requested (val=0x%lx, sock_path=%s)", __func__,
                 (unsigned long)val, s->sock_path ? s->sock_path : "<null>");

  s->vfu = vfu_create_ctx(VFU_TRANS_SOCK, s->sock_path,
                          LIBVFIO_USER_FLAG_ATTACH_NB, s, VFU_DEV_TYPE_PCI);
  if (!s->vfu) {
    qemu_epc_debug("%s: failed at vfu_create_ctx (errno=%d)", __func__, errno);
    return -1;
  }

  /* Route libvfio-user logs to our qepc_vfu_log helper. */
  vfu_setup_log(s->vfu, qepc_vfu_log, LOG_DEBUG);

  err = vfu_pci_init(s->vfu, VFU_PCI_TYPE_EXPRESS, PCI_HEADER_TYPE_NORMAL, 0);
  if (err) {
    qemu_epc_debug("%s: failed at vfu_pci_init (err=%d errno=%d)", __func__,
                   err, errno);
    return -1;
  }

  qemu_epc_debug("%s: setting up PCI config space region", __func__);
  err = vfu_setup_region(s->vfu, VFU_PCI_DEV_CFG_REGION_IDX,
                         PCIE_CONFIG_SPACE_SIZE, &qepc_pci_cfg_access,
                         VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB, NULL,
                         0, -1, 0);
  if (err) {
    qemu_epc_debug("%s: failed at vfu_setup_region (cfg) err=%d errno=%d",
                   __func__, err, errno);
    return -1;
  }
  qemu_epc_debug("%s: PCI config space region setup complete", __func__);

  // setup bars
  qemu_epc_debug("%s: setting up BAR regions (bar_mask=0x%x)", __func__,
                 s->bar_mask);
  for (int i = 0; i < PCI_NUM_REGIONS; i++) {
    /* Skip BARs that aren't enabled in the bar_mask (configured via BAR_CFG).
     */
    if (!(s->bar_mask & (1 << i))) {
      qemu_epc_debug("%s: skipping disabled BAR %d (not set in bar_mask=0x%x)",
                     __func__, i, s->bar_mask);
      continue;
    }

    qemu_epc_debug("%s: setup bar %d: phys=0x%lx size=0x%lx flags=0x%x",
                   __func__, i, (unsigned long)s->bars[i].phys_addr,
                   (unsigned long)s->bars[i].size, s->bars[i].flags);

    /* Expose each enabled BAR to the vfio-user client. */
    err = vfu_setup_region(s->vfu, VFU_PCI_DEV_BAR0_REGION_IDX + i,
                           s->bars[i].size, qepc_bar_handlers[i],
                           VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB, NULL,
                           0, -1, 0);
    if (err) {
      qemu_epc_debug(
          "%s: failed at vfu_setup_region for bar %d (err=%d errno=%d)",
          __func__, i, err, errno);
      return -1;
    }
    qemu_epc_debug("%s: BAR %d region setup complete", __func__, i);
  }
  qemu_epc_debug("%s: all BAR regions setup complete", __func__);

  // setup irqs
  err = vfu_setup_device_nr_irqs(s->vfu, VFU_DEV_INTX_IRQ, 1);
  if (err < 0) {
    qemu_epc_debug("%s: failed to setup irq (err=%d errno=%d)", __func__, err,
                   errno);
    return err;
  }

  // Register MSI interrupts only if MSI has been enabled
  if (s->msi_vectors > 0) {
    err = vfu_setup_device_nr_irqs(s->vfu, VFU_DEV_MSI_IRQ, s->msi_vectors);
    if (err < 0) {
      qemu_epc_debug("%s: failed to setup MSI irqs (err=%d errno=%d)", __func__,
                     err, errno);
      return err;
    }
  }

  // setup for dma
  err = vfu_setup_device_dma(s->vfu, qepc_dma_register, qepc_dma_unregister);
  if (err) {
    qemu_epc_debug("%s: failed to setup dma (err=%d errno=%d)", __func__, err,
                   errno);
    return -1;
  }

  err = vfu_realize_ctx(s->vfu);
  if (err) {
    qemu_epc_debug("%s: failed at vfu_realize_ctx (err=%d errno=%d)", __func__,
                   err, errno);
    return -1;
  }

  s->vfu_fd = vfu_get_poll_fd(s->vfu);
  if (s->vfu_fd < 0) {
    qemu_epc_debug("%s: failed at vfu_get_poll_fd (ret=%d errno=%d)", __func__,
                   s->vfu_fd, errno);
    return -1;
  }

  qemu_epc_debug("%s: listening vfu connection from %s (fd=%d)", __func__,
                 s->sock_path, s->vfu_fd);
  qemu_set_fd_handler(s->vfu_fd, qepc_vfu_attach_ctx, NULL, s);

  return 0;
}

/*
 * qepc_handle_ctrl_irq
 *
 * Handles a write to the IRQ_NUM control register. Currently, the irq_num
 * parameter is ignored and a single INTx line (index 0) is triggered via
 * vfu_irq_trigger().
 *
 * TODO: Use irq_num to select between multiple interrupt lines or types
 *       once such functionality is implemented in the device model.
 */
static void qepc_handle_ctrl_irq(QEPCState *s, int irq_num) {
  int ret;

  qemu_epc_debug("%s: trigger IRQ (irq_num=%d type=%u)", __func__, irq_num,
                 s->irq_type);

  switch (s->irq_type) {
  case IRQ_TYPE_INTX:
    // INTx is still only a single line, subindex 0
    ret = vfu_irq_trigger(s->vfu, 0);
    break;
  case IRQ_TYPE_MSI:
    // Trigger the requested MSI vector
    if (irq_num >= 0 && irq_num <= s->msi_vectors) {
      ret = vfu_irq_trigger(s->vfu, irq_num - 1);
    } else {
      qemu_epc_debug("%s: invalid MSI vector %d", __func__, irq_num);
      return;
    }
    break;
  default:
    qemu_epc_debug("%s: unsupported IRQ type %u", __func__, s->irq_type);
    return;
  }

  if (ret < 0) {
    qemu_epc_debug("%s: vfu_irq_trigger failed (errno=%d)", __func__, errno);
  }
}

/*
 * qepc_handle_enable_disale_ob
 *
 * Handles writes to the outbound window enable/disable mask register.
 * Compares the new mask with the previous one and, for each window whose
 * state changed, attempts to map the corresponding RC DMA region into
 * the EPC outbound aperture.
 *
 * The current implementation:
 *   - Only supports enabling outbound windows; disabling logs a message
 *     but does not tear down mappings.
 *   - Uses rc_local_mr[0] for all windows, effectively allowing only one
 *     active local mapping at a time.
 *
 * TODO:
 *   - Implement proper disable logic to remove subregions and free
 *     associated resources.
 *   - Use a distinct rc_local_mr[] entry per outbound window instead of
 *     always using index 0.
 *   - Add bounds checking on ob_idx and validate obs[] entries.
 */
static void qepc_handle_enable_disable_ob(QEPCState *s, uint64_t val) {
  uint8_t prev = s->ob_mask;

  /*
   * Only a limited number of outbound windows are supported. The guest
   * should never set bits outside NUM_OB_WINDOW, but if it does, we
   * ignore them and only act on the supported subset.
   */
  uint8_t new_mask = val & ((1U << NUM_OB_WINDOW) - 1);

  qemu_epc_debug("%s: change ob_mask: prev=0x%x new=0x%x", __func__, prev,
                 new_mask);

  for (int i = 0; i < NUM_OB_WINDOW; i++) {
    uint8_t bit = (1 << i);

    if ((prev & bit) == (new_mask & bit)) {
      continue;
    }

    if (prev & bit) {
      /*
       * Disable path: if the window was previously fast-mapped, remove the
       * local subregion to keep internal state consistent and free the slot.
       */
      qemu_epc_debug("%s: disabling ob window %d", __func__, i);

      /* Remove fast-path mapping if present */
      if (s->ob_fast_mapped[i]) {
        qemu_epc_debug("%s: unmapping fast local MR for ob window %d", __func__,
                       i);
        /* Remove subregion from the OB aperture if present */
        memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[i]);
        s->ob_fast_mapped[i] = false;
      }

      /* Clear the bit in the software mask */
      s->ob_mask &= ~bit;
      qemu_epc_debug("%s: ob window %d disabled, ob_mask=0x%x", __func__, i,
                     s->ob_mask);
      continue;
    } else {
      qemu_epc_debug(
          "%s: enabling ob window %d (phys=0x%lx pci=0x%lx size=0x%lx)",
          __func__, i, (unsigned long)s->obs[i].phys,
          (unsigned long)s->obs[i].pci, (unsigned long)s->obs[i].size);
    }

    /*
     * Sanity-check the programmed outbound window before trying to map it.
     */
    if (s->obs[i].size == 0) {
      qemu_epc_debug("%s: skip enabling ob window %d, size is 0", __func__, i);
      continue;
    }

    for (int j = 0; j < SUPPORT_RC_NUM_MRS; j++) {

      if (s->rc_mrs[j].size == 0 || s->rc_mrs[j].vaddr == NULL) {
        continue;
      }

      if (s->rc_mrs[j].rc_phys <= s->obs[i].pci &&
          (s->obs[i].pci + s->obs[i].size) <=
              s->rc_mrs[j].rc_phys + s->rc_mrs[j].size) {

        uint64_t off = s->obs[i].pci - s->rc_mrs[j].rc_phys;

        void *start = (uint8_t *)s->rc_mrs[j].vaddr + off;
        uint64_t size = s->obs[i].size;

        qemu_epc_debug("%s: mapping ob window %d using rc_mrs[%d]: "
                       "rc_phys=0x%lx off=0x%lx size=0x%lx start=%p",
                       __func__, i, j, (unsigned long)s->rc_mrs[j].rc_phys,
                       (unsigned long)off, (unsigned long)size, start);

        /* Ensure IOMMU address space is available before attempting to map.
         * We map inside the OB aperture MR itself (relative offset) rather
         * than inserting a mapping under the IOMMU root with an absolute
         * physical address.
         */
        AddressSpace *as = pci_device_iommu_address_space(&s->dev);
        if (!as) {
          qemu_epc_debug(
              "%s: no IOMMU address space available, cannot map ob window %d",
              __func__, i);
          goto done_check_next;
        }

        /*
         * Compute the per-window relative offset inside the OB aperture and
         * validate bounds against the total OB aperture to avoid overlapping
         * or out-of-range subregion additions which will trigger assertions.
         */
        hwaddr window_offset = (hwaddr)i * (hwaddr)OB_WINDOW_SIZE;
        uint64_t ob_aperture_size =
            (uint64_t)NUM_OB_WINDOW * (uint64_t)OB_WINDOW_SIZE;

        if ((uint64_t)window_offset + size > ob_aperture_size) {
          qemu_epc_debug("%s: calculated window_offset (0x%lx) + size (0x%lx) "
                         "out of OB aperture bounds (aperture=0x%lx), skip",
                         __func__, (unsigned long)window_offset,
                         (unsigned long)size, (unsigned long)ob_aperture_size);
          goto done_check_next;
        }

        /* If there's an existing fast mapping for this window, remove it so
         * we can re-map with the new parameters. This avoids skipping remap
         * when obs[] changed between enable operations.
         */
        if (s->ob_fast_mapped[i]) {
          qemu_epc_debug("%s: ob window %d already fast-mapped; removing old "
                         "mapping to remap",
                         __func__, i);
          memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[i]);
          s->ob_fast_mapped[i] = false;
        }

        /* Initialize a per-window local memory region and map it at the
         * correct relative offset within the OB aperture MR.
         */
        char mr_name[64];
        snprintf(mr_name, sizeof(mr_name), "qemu-epc/rc-local-%d", i);

        memory_region_init_ram_ptr(&s->rc_local_mr[i], OBJECT(s), mr_name, size,
                                   start);

        /* Add as a subregion of the OB aperture at the computed offset. */
        memory_region_add_subregion(&s->ob_window_mr, window_offset,
                                    &s->rc_local_mr[i]);

        /*
         * Mark this window as enabled in the software mask and mark the fast
         * mapping as present so subsequent enables won't re-map the same
         * region unless obs[] changes (we remove and remap above).
         */
        s->ob_fast_mapped[i] = true;
        s->ob_mask |= bit;
        qemu_epc_debug("%s: ob window %d enabled and mapped at "
                       "ob_window_base=0x%lx (window_offset=0x%lx)",
                       __func__, i, (unsigned long)s->ob_window_mr.addr,
                       (unsigned long)window_offset);
        goto done;
      }
    done_check_next:;
    }

    qemu_epc_debug(
        "%s: no matching RC MR for ob window %d (pci=0x%lx size=0x%lx)",
        __func__, i, (unsigned long)s->obs[i].pci,
        (unsigned long)s->obs[i].size);
  }
done:
  qemu_epc_debug("%s: final ob_mask=0x%x", __func__, s->ob_mask);
}

/*
 * qepc_ctrl_mmio_write
 *
 * MMIO write handler for the EPC control BAR.
 *
 * Handles writes to:
 *   - QEPC_CTRL_OFF_START    : Initializes/starts vfio-user backend
 *   - QEPC_CTRL_OFF_IRQ_TYPE : Records IRQ type (INTx/MSI/MSI-X)
 *   - QEPC_CTRL_OFF_IRQ_NUM  : Triggers an interrupt
 *   - QEPC_CTRL_OFF_OB_*     : Configures outbound windows
 *   - QEPC_CTRL_OFF_MSI_*    : Configures MSI
 *
 * Unrecognized offsets are logged but ignored.
 */
static void qepc_ctrl_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size) {
  QEPCState *s = opaque;
  uint64_t tmp;

  qemu_epc_debug("CTRL write: addr 0x%lx, size %u, val=0x%lx", addr, size,
                 (unsigned long)val);

  if (size != sizeof(uint32_t) && size != sizeof(uint64_t)) {
    qemu_epc_debug("%s: unaligned access size %u", __func__, size);
  }

  switch (addr) {
  case QEPC_CTRL_OFF_START:
    qepc_ctrl_handle_start(s, val);
    return;

  case QEPC_CTRL_OFF_IRQ_TYPE:
    qemu_epc_debug("%s: IRQ_TYPE write: val=0x%lx", __func__,
                   (unsigned long)val);
    s->irq_type = val;
    break;

  case QEPC_CTRL_OFF_IRQ_NUM:
    qemu_epc_debug("%s: IRQ_NUM write: val=%" PRIu64, __func__, val);
    qepc_handle_ctrl_irq(s, val);
    break;

  case QEPC_CTRL_OFF_OB_MASK:
    qepc_handle_enable_disable_ob(s, val);
    break;

  case QEPC_CTRL_OFF_OB_IDX:
    if (val >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: invalid ob_idx %" PRIu64, __func__, val);
      return;
    }
    s->ob_idx = val;
    qemu_epc_debug("%s: ob_idx set to %u", __func__, s->ob_idx);
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
    qemu_epc_debug("ob map phys: %d: 0x%lx", s->ob_idx, tmp);
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
    qemu_epc_debug("ob map pci: %d: 0x%lx", s->ob_idx, tmp);
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
    qemu_epc_debug("ob map size: %d: 0x%lx", s->ob_idx, tmp);
    break;

  /* --- MSI support --- */
  case QEPC_CTRL_OFF_MSI_CTRL:
    tmp = s->msi_vectors;
    tmp = (tmp & ~0xffffffffUL) | (val & 0xffffffff);
    s->msi_vectors = tmp;
    qemu_epc_debug("MSI_CTRL write: enable=%b vectors=%u", s->msi_vectors > 0,
                   s->msi_vectors);
    break;
  case QEPC_CTRL_OFF_MSI_CTRL + sizeof(uint32_t):
    tmp = s->msi_vectors;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->msi_vectors = tmp;
    qemu_epc_debug("MSI_CTRL high write: enable=%b vectors=%u",
                   s->msi_vectors > 0, s->msi_vectors);
    break;

  case QEPC_CTRL_OFF_MSI_ADDR:
    tmp = s->msi_addr;
    tmp = (tmp & ~0xffffffffUL) | (val & 0xffffffff);
    s->msi_addr = tmp;
    qemu_epc_debug("MSI_ADDR write: 0x%lx", s->msi_addr);
    break;
  case QEPC_CTRL_OFF_MSI_ADDR + sizeof(uint32_t):
    tmp = s->msi_addr;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->msi_addr = tmp;
    qemu_epc_debug("MSI_ADDR high write: 0x%lx", s->msi_addr);
    break;

  case QEPC_CTRL_OFF_MSI_DATA:
    s->msi_data = val & 0xffffffff;
    qemu_epc_debug("MSI_DATA write: 0x%x", s->msi_data);
    break;

  /* Placeholder for map_msi_irq; could use an offset past MSI_ADDR/DATA */
  case QEPC_CTRL_OFF_MSI_ADDR + 0x100:
    s->msi_vector = val;
    qemu_epc_debug("MSI mapped: phys=0x%lx vector=%u", s->msi_addr,
                   s->msi_vector);
    break;

  default:
    qemu_epc_debug("CTRL write: invalid address 0x%lx", addr);
    break;
  }
}
static const MemoryRegionOps qepc_ctrl_mmio_ops = {
    .read = qepc_ctrl_mmio_read,
    .write = qepc_ctrl_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

enum {
  QEPC_BAR_CFG_OFF_MASK = 0x00,
  QEPC_BAR_CFG_OFF_NUMBER = 0x01,
  QEPC_BAR_CFG_OFF_FLAGS = 0x02,
  QEPC_BAR_CFG_OFF_RSV = 0x04,
  QEPC_BAR_CFG_OFF_PHYS_ADDR = 0x08,
  QEPC_BAR_CFG_OFF_SIZE = 0x10,
};

/*
 * qepc_cfg_bar_read
 *
 * MMIO read handler for the BAR configuration region. Currently only
 * exposes the bar_mask, which indicates which BARs are configured/enabled.
 *
 * TODO: Consider exposing additional per-BAR configuration state here
 *       (e.g. size, flags, physical address) for debugging or introspection.
 */
static uint64_t qepc_cfg_bar_read(void *opaque, hwaddr addr, unsigned size) {
  QEPCState *s = opaque;

  switch (addr) {
  case QEPC_BAR_CFG_OFF_MASK:
    return s->bar_mask;
  default:
    break;
  }

  return 0;
}

/*
 * qepc_cfg_bar_write
 *
 * MMIO write handler for the BAR configuration region.
 *
 * The BAR configuration region provides a simple programming model:
 *   - QEPC_BAR_CFG_OFF_MASK:   bitmask of enabled BARs.
 *   - QEPC_BAR_CFG_OFF_NUMBER: selects the BAR index to configure.
 *   - QEPC_BAR_CFG_OFF_FLAGS:  stores flags into bars[bar_no].flags and
 *                              updates the corresponding BAR in PCI config.
 *   - QEPC_BAR_CFG_OFF_PHYS_ADDR: stores the backing system physical address
 *                                 and updates the PCI BAR address.
 *   - QEPC_BAR_CFG_OFF_SIZE:  records the BAR size used for sizing/masking.
 *
 * Writes here are reflected into both QEPCState::bars[] and the device's
 * PCIe configuration space image, which is then used by the guest and the
 * vfio-user backend.
 *
 * TODO: Validate bar_no against the number of supported BARs.
 */
static void qepc_cfg_bar_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size) {
  QEPCState *s = opaque;
  uint8_t *ptr8;
  uint32_t lo;
  uint64_t tmp;

  qemu_epc_debug("%s: write addr=0x%lx size=%u val=0x%lx", __func__,
                 (unsigned long)addr, size, (unsigned long)val);

  if (addr + size > QEPC_BAR_CFG_OFF_SIZE + 8) {
    /* TODO: Turn this into a hard error once the ABI is stable. */
    qemu_epc_debug("%s: overrun %ld", __func__, addr + size);
  }

  switch (addr) {
  case QEPC_BAR_CFG_OFF_MASK:
    /*
     * BAR enable mask programmed by the guest driver. This is an internal
     * enable/disable view for the vfio-user side and does not directly
     * modify the PCI config space BAR contents.
     */
    s->bar_mask = val;
    qemu_epc_debug("%s: bar_mask updated to 0x%x", __func__, s->bar_mask);
    break;
  case QEPC_BAR_CFG_OFF_NUMBER:
    if (val >= PCI_NUM_REGIONS) {
      qemu_epc_debug("%s: invalid BAR number %" PRIu64, __func__, val);
      return;
    }
    s->bar_no = val;
    qemu_epc_debug("%s: bar_no set to %u", __func__, s->bar_no);
    break;
  case QEPC_BAR_CFG_OFF_FLAGS: {
    /*
     * Only modify the low 32-bit BAR dword attribute bits (memory/io,
     * prefetchable, 32/64-bit etc). Do not touch the high dword here;
     * the host (RC) owns the BAR contents and will program them via
     * config space writes.
     */
    uint32_t attr = (uint32_t)val;
    uint8_t bar = s->bar_no;

    if (bar >= PCI_NUM_REGIONS) {
      qemu_epc_debug("%s: FLAGS write with invalid bar_no %u", __func__, bar);
      return;
    }

    s->bars[bar].flags = attr;

    ptr8 = s->pcie_config + 0x10 + 4 * bar;
    memcpy(&lo, ptr8, sizeof(lo));

    /*
     * Update only the lowest nibble (attr bits) and preserve the
     * address bits that may already have been programmed by the RC.
     * For now we also force memory space (bit 0 cleared) and mark
     * as 32-bit non-prefetchable (bit 2:1 = 00b). 64-bit semantics
     * will be enforced by how BAR pairs are interpreted in
     * qepc_pci_cfg_access_bar().
     */
    lo = (lo & ~0xF) | (attr & 0xF);
    memcpy(ptr8, &lo, sizeof(lo));

    qemu_epc_debug("%s: bar[%d] FLAGS updated lo=0x%x (flags 0x%x)", __func__,
                   bar, lo, attr);
    break;
  }
  case QEPC_BAR_CFG_OFF_RSV:
    break;
  case QEPC_BAR_CFG_OFF_PHYS_ADDR: {
    /*
     * Program the backing physical address for this BAR on the EPC side.
     * Do not overwrite the host-visible BAR address here; the RC will
     * program the BAR registers during enumeration and resource
     * assignment via config space accesses.
     */
    uint8_t bar = s->bar_no;

    if (bar >= PCI_NUM_REGIONS) {
      qemu_epc_debug("%s: PHYS_ADDR write with invalid bar_no %u", __func__,
                     bar);
      return;
    }
    s->bars[bar].phys_addr = val;

    /*
     * For debugging only, read out the current low 32-bit BAR dword
     * from the config space image without modifying it, so that we
     * don't trample over the high dword or RC-programmed contents.
     */
    ptr8 = s->pcie_config + 0x10 + 4 * bar;
    memcpy(&tmp, ptr8, sizeof(uint64_t));
    qemu_epc_debug("%s: bar[%d] PHYS_ADDR=0x%lx (cfg_lo=0x%lx)", __func__, bar,
                   (unsigned long)val, (unsigned long)(tmp & 0xffffffffUL));
    break;
  }
  case QEPC_BAR_CFG_OFF_SIZE: {
    uint8_t bar = s->bar_no;

    if (bar >= PCI_NUM_REGIONS) {
      qemu_epc_debug("%s: SIZE write with invalid bar_no %u", __func__, bar);
      return;
    }
    if (val == 0) {
      qemu_epc_debug("%s: ignoring zero-sized BAR %u", __func__, bar);
      return;
    }
    s->bars[bar].size = val;
    qemu_epc_debug("%s: bar[%d] SIZE=0x%lx", __func__, bar, (unsigned long)val);
    break;
  }
  }
}

/* MemoryRegionOps for the BAR configuration MMIO window. */
static const MemoryRegionOps qemu_epc_mmio_bar_cfg_ops = {
    .read = qepc_cfg_bar_read,
    .write = qepc_cfg_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * qepc_realize
 *
 * PCIDeviceClass::realize callback. This is invoked when the EPC device
 * is created in the QEMU machine. It:
 *   - Validates that the vfio-user socket path has been provided,
 *   - Initializes the MemoryRegion objects for:
 *       * the control BAR,
 *       * the PCI config-space backing store,
 *       * the BAR configuration MMIO region,
 *       * the outbound window aperture,
 *   - Registers all of the above as BARs on the PCI device.
 *
 * Note that this does not start the vfio-user backend yet; that is done
 * when the guest writes to the QEPC_CTRL_OFF_START register.
 */
static void qepc_realize(PCIDevice *pci_dev, Error **errp) {
  QEPCState *s = QEMU_EPC(pci_dev);

  qemu_epc_debug("realize");

  if (!s->sock_path) {
    error_setg(errp, "qemu-epc: sock_path should be set");
    return;
  }

  qemu_epc_debug(
      "initializing control BAR memory region: size=0x%lx (raw=0x%x)",
      (unsigned long)pow2ceil(QEPC_CTRL_SIZE), QEPC_CTRL_SIZE);
  memory_region_init_io(&s->ctrl_mr, OBJECT(s), &qepc_ctrl_mmio_ops, s,
                        "qemu-epc/ctrl", pow2ceil(QEPC_CTRL_SIZE));

  /*
   * Allocate page-aligned backing for PCIe config space so that it can be
   * safely mapped as RAM by KVM without triggering alignment errors.
   */
  qemu_epc_debug(
      "requesting UA-aligned PCI config space memory region: size=0x%lx ptr=%p",
      (unsigned long)PCIE_CONFIG_SPACE_SIZE, s->pcie_config);
  size_t page_size = qemu_real_host_page_size();
  s->pcie_config = qemu_memalign(page_size, PCIE_CONFIG_SPACE_SIZE);
  if (!s->pcie_config) {
    error_setg(errp, "qemu-epc: failed to allocate PCI config space");
    return;
  }
  memset(s->pcie_config, 0, PCIE_CONFIG_SPACE_SIZE);

  qemu_epc_debug(
      "initializing PCI config space memory region: size=0x%lx ptr=%p",
      (unsigned long)PCIE_CONFIG_SPACE_SIZE, s->pcie_config);
  memory_region_init_ram_ptr(&s->pci_cfg_mr, OBJECT(s), "qemu-epc/cfg-cfg",
                             PCIE_CONFIG_SPACE_SIZE, s->pcie_config);

  qemu_epc_debug("initializing BAR config memory region: size=0x%lx (raw=0x%x)",
                 (unsigned long)pow2ceil(QEPC_BAR_CFG_SIZE), QEPC_BAR_CFG_SIZE);
  memory_region_init_io(&s->bar_cfg_mr, OBJECT(s), &qemu_epc_mmio_bar_cfg_ops,
                        s, "qemu-epc/bar-cfg", pow2ceil(QEPC_BAR_CFG_SIZE));

  qemu_epc_debug("initializing outbound window memory region: size=0x%lx "
                 "(windows=%d window_size=0x%llx)",
                 (unsigned long)pow2ceil(NUM_OB_WINDOW * OB_WINDOW_SIZE),
                 NUM_OB_WINDOW, (unsigned long long)OB_WINDOW_SIZE);
  memory_region_init(&s->ob_window_mr, NULL, "qemu-epc/ob",
                     pow2ceil(NUM_OB_WINDOW * OB_WINDOW_SIZE));

  pci_register_bar(pci_dev, QEPC_BAR_CTRL, PCI_BASE_ADDRESS_MEM_TYPE_32,
                   &s->ctrl_mr);

  pci_register_bar(pci_dev, QEPC_BAR_PCI_CFG, PCI_BASE_ADDRESS_SPACE_MEMORY,
                   &s->pci_cfg_mr);

  pci_register_bar(pci_dev, QEPC_BAR_BAR_CFG, PCI_BASE_ADDRESS_SPACE_MEMORY,
                   &s->bar_cfg_mr);

  pci_register_bar(pci_dev, QEPC_BAR_OB_WINDOWS, PCI_BASE_ADDRESS_MEM_TYPE_64,
                   &s->ob_window_mr);

  /*
   * Implement a corresponding .exit callback that:
   *   - Tears down the vfio-user context if it was created,
   *   - Unregisters any rc_local_mr subregions from the OB aperture,
   *   - Closes s->vfu_fd and frees any dynamically allocated resources.
   */
}
static void qepc_exit(PCIDevice *pci_dev) {
  QEPCState *s = QEMU_EPC(pci_dev);

  qemu_epc_debug("%s: exit called", __func__);

  /* Remove any per-window fast mappings */
  for (int i = 0; i < NUM_OB_WINDOW; i++) {
    if (s->ob_fast_mapped[i]) {
      qemu_epc_debug("%s: removing rc_local_mr[%d] subregion", __func__, i);
      memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[i]);
      s->ob_fast_mapped[i] = false;
    }
  }

  /* Destroy vfu context if present */
  if (s->vfu) {
    qemu_epc_debug("%s: destroying vfu ctx", __func__);
    vfu_destroy_ctx(s->vfu);
    s->vfu = NULL;
  }

  /* Clear fd handler if one was installed */
  if (s->vfu_fd >= 0) {
    qemu_epc_debug("%s: clearing vfu fd handler (fd=%d)", __func__, s->vfu_fd);
    qemu_set_fd_handler(s->vfu_fd, NULL, NULL, NULL);
    s->vfu_fd = -1;
  }

  /* Free duplicated socket path if present */
  if (s->sock_path) {
    g_free((void *)s->sock_path);
    s->sock_path = NULL;
  }
}

/*
 * qepc_object_set_path
 *
 * Setter for the "path" QOM property. This property selects the Unix
 * domain socket path used by the vfio-user backend.
 *
 * The string is duplicated and stored in QEPCState::sock_path.
 *
 * TODO: Validate that 'str' is a valid and safe filesystem path, and
 *       optionally reject relative paths depending on deployment needs.
 */
static void qepc_object_set_path(Object *obj, const char *str, Error **errp) {
  QEPCState *s = QEMU_EPC(obj);

  qemu_epc_debug("socket path: %s", str);
  s->sock_path = g_strdup(str);
}

/*
 * qepc_class_init
 *
 * QOM class initialization function for the EPC device. It:
 *   - Registers the "path" property (used to configure the vfio-user socket),
 *   - Hooks up the PCIDeviceClass::realize callback,
 *   - Sets PCI identification fields (vendor, device, class),
 *   - Declares the device as a miscellaneous PCI device.
 *
 * TODO: Implement and hook up a proper 'exit' callback for cleanup of
 *       vfio-user contexts and memory regions.
 */
static void qepc_class_init(ObjectClass *klass, const void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);
  PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

  qemu_epc_debug("initialize class");

  object_class_property_add_str(klass, "sock-path", NULL, qepc_object_set_path);

  k->realize = qepc_realize;
  k->exit = qepc_exit;
  k->vendor_id = PCI_VENDOR_ID_REDHAT;
  k->device_id = PCI_DEVICE_ID_REDHAT_QEMU_EPC;
  k->revision = QEPC_REVISION;
  k->class_id = PCI_CLASS_OTHERS;

  dc->desc = "QEMU Endpoint Controller device";
  set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

/*
 * qepc_info
 *
 * Static type information for registering the EPC device with the QOM
 * type system. Declares this device as a PCI device that implements the
 * conventional PCI device interface.
 */
static const TypeInfo qepc_info = {
    .name = TYPE_QEMU_EPC,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(QEPCState),
    .class_init = qepc_class_init,
    .interfaces = (InterfaceInfo[]){{INTERFACE_CONVENTIONAL_PCI_DEVICE}, {}},
};

/*
 * qemu_epc_register_type
 *
 * Type registration entry point called at QEMU initialization time.
 */
static void qemu_epc_register_type(void) { type_register_static(&qepc_info); }

type_init(qemu_epc_register_type);
