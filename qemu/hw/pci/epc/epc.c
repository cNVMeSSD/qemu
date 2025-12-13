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
/* Lightweight debug logging helper, compiled out when DEBUG_QEMU_EPC is unset. */
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
    uint8_t flags;      /* Implementation-defined flags (e.g., IO/MEM, prefetch). */
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
   /* TODO: Remove the stale commented-out register definitions above once the
    *       enum-based layout is stable and documented externally. */
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
   * vaddr is the local virtual address backing the region in this process,
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
};

/* QOM type name for this device. */
#define TYPE_QEMU_EPC "qemu-epc"
OBJECT_DECLARE_SIMPLE_TYPE(QEPCState, QEMU_EPC)

#define QEPC_REVISION 0x00

#define QEPC_BAR_CFG_SIZE 1024

/*
 * BAR indices exposed to the guest:
 *   QEPC_BAR_CTRL       - Control and status registers for the EPC.
 *   QEPC_BAR_PCI_CFG    - MMIO view of PCIe config space (backed by pcie_config).
 *   QEPC_BAR_BAR_CFG    - MMIO interface to configure EPC BARs.
 *   QEPC_BAR_OB_WINDOWS - Outbound window aperture.
 */
enum {
  QEPC_BAR_CTRL = 0,
  QEPC_BAR_PCI_CFG = 1,
  QEPC_BAR_BAR_CFG = 2,
  QEPC_BAR_OB_WINDOWS = 3,
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
  /* Write: kick off vfio-user backend initialization. */
  QEPC_CTRL_OFF_START = 0x00,
  QEPC_CTRL_OFF_WIN_START = 0x08,
  QEPC_CTRL_OFF_WIN_SIZE = 0x10,
  QEPC_CTRL_OFF_IRQ_TYPE = 0x18,
  QEPC_CTRL_OFF_IRQ_NUM = 0x1c,
  QEPC_CTRL_OFF_OB_MASK = 0x20,
  QEPC_CTRL_OFF_OB_IDX = 0x24,
  QEPC_CTRL_OFF_OB_PHYS = 0x28,
  QEPC_CTRL_OFF_OB_PCI = 0x30,
  QEPC_CTRL_OFF_OB_SIZE = 0x38,

  QEPC_CTRL_SIZE = QEPC_CTRL_OFF_OB_SIZE + sizeof(uint64_t)
};

/*
#define QEPC_CTRL_OFF_START (0x00)
#define QEPC_CTRL_OFF_WIN_START (0x08)
#define QEPC_CTRL_OFF_WIN_SIZE (0x10)
#define QEPC_CTRL_OFF_IRQ_TYPE (0x18)
#define QEPC_CTRL_OFF_IRQ_NUM (0x1c)
#define QEPC_CTRL_OFF_OB_IDX (0x20)
#define QEPC_CTRL_OFF_OB_FLAG (0x24)
#define QEPC_CTRL_OFF_OB_PHYS (0x28)
#define QEPC_CTRL_OFF_OB_PCI (0x30)
#define QEPC_CTRL_OFF_OB_SIZE (0x38)
#define QEPC_CTRL_SIZE (QEPC_CTRL_OFF_OB_SIZE + sizeof(uint64_t))
*/

/*
 * qepc_ctrl_mmio_read
 *
 * MMIO read handler for the EPC control BAR.
 *
 * This exposes:
 *   - The base address and size of the outbound window aperture,
 *   - The current outbound window enable mask.
 *
 * Unhandled offsets currently return 0 and log a debug message.
 * TODO: Define and expose additional status registers (e.g. error codes,
 *       backend state) if needed by the guest driver.
 */
static uint64_t qepc_ctrl_mmio_read(void *opaque, hwaddr addr, unsigned size) {
  QEPCState *s = opaque;

  qemu_epc_debug("CTRL read: addr 0x%lx, size 0x%x", addr, size);

  /*
   * All registers in this MMIO region are naturally aligned and sized
   * accesses are expected. Don't enforce size, but it's better to only
   * allow aligned access.
   */
  if (size != sizeof(uint32_t) && size != sizeof(uint64_t)) {
    qemu_epc_debug("%s: unaligned access size %u", __func__, size);
  }

  switch (addr) {
  case QEPC_CTRL_OFF_WIN_START:
    return s->ob_window_mr.addr;
  case QEPC_CTRL_OFF_WIN_START + sizeof(uint32_t):
    return s->ob_window_mr.addr >> 32;
  case QEPC_CTRL_OFF_WIN_SIZE:
    return OB_WINDOW_SIZE & 0xffffffff;
  case QEPC_CTRL_OFF_WIN_SIZE + sizeof(uint32_t):
    return OB_WINDOW_SIZE >> 32;
  case QEPC_CTRL_OFF_OB_MASK:
    return s->ob_mask;
  default:;
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

  qemu_epc_debug("%s: %s: offset 0x%lx, size 0x%lx", __func__,
                 is_write ? "write" : "read", offset, count);

  /* BAR registers are always 32-bit wide. */
  assert(count == sizeof(uint32_t));

  if (is_write) {
    switch (offset) {
    case 0x10: // BAR 0
      memcpy(&t1, buf, sizeof(t1));
      /* Mask according to BAR0 size; emulate PCI BAR size probing. */
      t2 = (~(uint32_t)s->bars[0].size + 1) & t1;
      memcpy(s->pcie_config + offset, &t2, sizeof(t2));
      break;
    case 0x18: // BAR 2
      memcpy(&t1, buf, sizeof(t1));
      /* Mask according to BAR2 size; emulate PCI BAR size probing. */
      t2 = (~(uint32_t)s->bars[2].size + 1) & t1;
      memcpy(s->pcie_config + offset, &t2, sizeof(t2));
      break;
    case 0x20: // BAR 4
      memcpy(&t1, buf, sizeof(t1));
      /* Mask according to BAR4 size; emulate PCI BAR size probing. */
      t2 = (~(uint32_t)s->bars[4].size + 1) & t1;
      memcpy(s->pcie_config + offset, &t2, sizeof(t2));
      break;
    default:
      memcpy(s->pcie_config + offset, buf, count);
    }
  } else {
    memcpy(&t1, s->pcie_config + offset, sizeof(t1));
    qemu_epc_debug("off 0x%lx: value 0x%x", offset, t1);
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
static void *qepc_thread(void *opaque) {
  int err;
  kjkQEPCState *s = opaque;
kj
  qemu_epc_debug("start thread vfu thread");

  // TODO: Validate sock_path correctness before creating the context.
  s->vfu = vfu_create_ctx(VFU_TRANS_SOCK, s->sock_path,
                          LIBVFIO_USER_FLAG_ATTACH_NB, s, VFU_DEV_TYPE_PCI);
  if (!s->vfu) {
    return NULL;
  }

  err = vfu_pci_init(s->vfu, VFU_PCI_TYPE_EXPRESS, PCI_HEADER_TYPE_NORMAL, 0);
  if (err) {
    return NULL;
  }

  // Expose PCI config space to the vfio-user client.
  err = vfu_setup_region(s->vfu, VFU_PCI_DEV_CFG_REGION_IDX,
                         PCIE_CONFIG_SPACE_SIZE, &qepc_pci_cfg_access,
                         VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB, NULL,
                         0, -1, 0);
  if (err) {
    qemu_epc_debug("failed at vfu_setup_region");
    return NULL;
  }
  // setup bars
  // setup irqs
  // vfu_realize_ctx
  err = vfu_realize_ctx(s->vfu);
  if (err) {
    qemu_epc_debug("failed at vfu_realize_ctx");
    return NULL;
  }
  // vfu_get_poll_fd
  // qemu_set_fd_handler(pollfd, );

  return NULL;
}
*/

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
/* Forward declaration so qepc_vfu_attach_ctx can reference this callback. */
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
        qemu_epc_debug("%s: vfu_run_ctx failed: err=%d errno=%d",
                       __func__, err, errno);
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
  dma_addr_t addr;

  qemu_epc_debug("%s: bar=%u offset=0x%lx size=0x%zx %s", __func__, barno,
                 (unsigned long)offset, count,
                 is_write ? "write" : "read");

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
    qemu_epc_debug("%s: BAR %u out-of-bounds (off=0x%lx, size=0x%zx, bar_size=0x%lx)",
                   __func__, barno, (unsigned long)offset, count,
                   (unsigned long)s->bars[barno].size);
    return -1;
  }

  addr = s->bars[barno].phys_addr + offset;
  qemu_epc_debug("%s: resolved DMA addr=0x%lx", __func__, (unsigned long)addr);

  if (is_write) {
    pci_dma_write(&s->dev, addr, buf, count);
  } else {
    pci_dma_read(&s->dev, addr, buf, count);
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
      "%s: register iova=[0x%lx..0x%lx) len=0x%lx vaddr=%p prot=0x%x",
      __func__, (uint64_t)info->iova.iov_base,
      (uint64_t)info->iova.iov_base + info->iova.iov_len,
      (uint64_t)info->iova.iov_len, info->vaddr, info->prot);

  if (!info->vaddr) {
    /* TODO: Support non-host-mapped DMA regions if needed. */
    qemu_epc_debug("%s: unsupported: vaddr is NULL", __func__);
    return;
  }

  if (s->rc_mr_idx == SUPPORT_RC_NUM_MRS) {
    /* TODO: Grow this table dynamically or fail the DMA registration. */
    qemu_epc_debug(
        "%s: unsupported: rc_mrs table full (%d entries, max=%d)",
        __func__, s->rc_mr_idx, SUPPORT_RC_NUM_MRS);
    return;
  }

  s->rc_mrs[s->rc_mr_idx] = (struct rc_mr){
      .rc_phys = (uint64_t)info->iova.iov_base,
      .size = info->iova.iov_len,
      .vaddr = info->vaddr,
  };
  qemu_epc_debug(
      "%s: stored rc_mrs[%d]: rc_phys=0x%lx size=0x%lx vaddr=%p",
      __func__, s->rc_mr_idx, (unsigned long)s->rc_mrs[s->rc_mr_idx].rc_phys,
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

  qemu_epc_debug(
      "%s: unregister iova=[0x%lx..0x%lx) len=0x%lx vaddr=%p",
      __func__, (uint64_t)info->iova.iov_base,
      (uint64_t)info->iova.iov_base + info->iova.iov_len,
      (uint64_t)info->iova.iov_len, info->vaddr);

  /* TODO: remove from rc_mrs[] and log which slot was reclaimed. */
  (void)s;
}

/*
 * qepc_ctrl_handle_start
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

  err = vfu_setup_region(s->vfu, VFU_PCI_DEV_CFG_REGION_IDX,
                         PCIE_CONFIG_SPACE_SIZE, &qepc_pci_cfg_access,
                         VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB, NULL,
                         0, -1, 0);
  if (err) {
    qemu_epc_debug("%s: failed at vfu_setup_region (cfg) err=%d errno=%d",
                   __func__, err, errno);
    return -1;
  }

  // setup bars
  for (int i = 0; i < PCI_NUM_REGIONS; i++) {

    if (!(s->bar_mask & (1 << i))) {
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
  }

  // setup irqs
  /* TODO: Add support for MSI/MSI-X if needed by the endpoint. */
  err = vfu_setup_device_nr_irqs(s->vfu, VFU_DEV_INTX_IRQ, 1);
  if (err < 0) {
    qemu_epc_debug("%s: failed to setup irq (err=%d errno=%d)", __func__, err,
                   errno);
    return err;
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
    qemu_epc_debug("%s: failed at vfu_realize_ctx (err=%d errno=%d)",
                   __func__, err, errno);
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
  qemu_epc_debug("%s: trigger IRQ (irq_num=%d type=%u)", __func__, irq_num,
                 s->irq_type);
  vfu_irq_trigger(s->vfu, 0);
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
static void qepc_handle_enable_disale_ob(QEPCState *s, uint64_t val) {
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
      /* TODO: Implement outbound window disable/unmap path. */
      qemu_epc_debug("%s: disabling ob window %d is not supported yet",
                     __func__, i);
      /* For now, just clear the bit in the software mask. */
      s->ob_mask &= ~bit;
      continue;
    } else {
      qemu_epc_debug("%s: enabling ob window %d (phys=0x%lx pci=0x%lx size=0x%lx)",
                     __func__, i, (unsigned long)s->obs[i].phys,
                     (unsigned long)s->obs[i].pci,
                     (unsigned long)s->obs[i].size);
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

        // sprintf(s->rc_local_mr_name, "qemu-epc/rc-local-%d", 0);
        /* TODO: Use i as index instead of 0 once per-window mappings are supported. */
        qemu_epc_debug("%s: mapping ob window %d using rc_mrs[%d]: rc_phys=0x%lx off=0x%lx size=0x%lx start=%p",
                       __func__, i, j,
                       (unsigned long)s->rc_mrs[j].rc_phys,
                       (unsigned long)off, (unsigned long)size, start);

        memory_region_init_ram_ptr(&s->rc_local_mr[0], OBJECT(s),
                                   "qemu-epc/rc-local-0", size, start);

        AddressSpace *as = pci_device_iommu_address_space(&s->dev);

        memory_region_add_subregion(as->root, s->ob_window_mr.addr,
                                    &s->rc_local_mr[0]);

        /*
         * Mark this window as enabled in the software mask.
         * Note: hardware-visible behavior is still controlled by
         * the guest via the QEPC_CTRL_OFF_OB_MASK register.
         */
        s->ob_mask |= bit;
        qemu_epc_debug(
            "%s: ob window %d enabled and mapped at ob_window_base=0x%lx",
            __func__, i, (unsigned long)s->ob_window_mr.addr);
        goto done;
      }
    }

    qemu_epc_debug("%s: no matching RC MR for ob window %d (pci=0x%lx size=0x%lx)",
                   __func__, i, (unsigned long)s->obs[i].pci,
                   (unsigned long)s->obs[i].size);
  }
done:
  qemu_epc_debug("%s: final ob_mask=0x%x", __func__, s->ob_mask);
}

/*
 * qepc_ctrl_mmio_write
 *
 * MMIO write handler for the EPC control BAR. It interprets writes to:
 *
 *   QEPC_CTRL_OFF_START    - Initializes and starts the vfio-user backend.
 *   QEPC_CTRL_OFF_IRQ_TYPE - Records the interrupt type (currently unused).
 *   QEPC_CTRL_OFF_IRQ_NUM  - Triggers an interrupt via qepc_handle_ctrl_irq().
 *   QEPC_CTRL_OFF_OB_MASK  - Enables/disables outbound windows.
 *   QEPC_CTRL_OFF_OB_IDX   - Selects which outbound window is being configured.
 *   QEPC_CTRL_OFF_OB_PHYS  - Programs the system physical address for the
 *                            selected outbound window (low/high dwords).
 *   QEPC_CTRL_OFF_OB_PCI   - Programs the PCI address as seen by the client
 *                            for the selected outbound window (low/high).
 *   QEPC_CTRL_OFF_OB_SIZE  - Programs the size of the selected outbound window
 *                            (low/high).
 *
 * Unrecognized offsets are logged but otherwise ignored.
 *
 * TODO: Add validation of sizes, address alignment, and index bounds
 *       (ob_idx < NUM_OB_WINDOW).
 */
static void qepc_ctrl_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size) {
  QEPCState *s = opaque;
  uint64_t tmp;

  qemu_epc_debug("CTRL write: addr 0x%lx, size 0x%x, val=0x%lx", addr, size,
                 (unsigned long)val);

  /*
   * All registers in this MMIO region are naturally aligned and sized
   * accesses are expected. Don't enforce size, but it's better to only
   * allow aligned access.
   */
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
    qepc_handle_enable_disale_ob(s, val);
    break;
  case QEPC_CTRL_OFF_OB_IDX:
    if (val >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: invalid ob_idx %" PRIu64, __func__, val);
      return;
    }
    s->ob_idx = val;
    qemu_epc_debug("%s: ob_idx set to %u", __func__, s->ob_idx);
    break;
  case QEPC_CTRL_OFF_OB_PHYS:
    if (s->ob_idx >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: write to OB_PHYS with invalid ob_idx %u",
                     __func__, s->ob_idx);
      return;
    }
    tmp = s->obs[s->ob_idx].phys;
    tmp = (tmp & ~0xffffffffUL) | (val & 0xffffffff);
    s->obs[s->ob_idx].phys = tmp;
    break;
  case QEPC_CTRL_OFF_OB_PHYS + sizeof(uint32_t):
    if (s->ob_idx >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: write to OB_PHYS[hi] with invalid ob_idx %u",
                     __func__, s->ob_idx);
      return;
    }
    tmp = s->obs[s->ob_idx].phys;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->obs[s->ob_idx].phys = tmp;
    qemu_epc_debug("ob map phys: %d: 0x%lx", s->ob_idx, tmp);
    break;
  case QEPC_CTRL_OFF_OB_PCI:
    if (s->ob_idx >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: write to OB_PCI with invalid ob_idx %u",
                     __func__, s->ob_idx);
      return;
    }
    tmp = s->obs[s->ob_idx].pci;
    tmp = (tmp & ~0xffffffff) | (val & 0xffffffff);
    s->obs[s->ob_idx].pci = tmp;
    break;
  case QEPC_CTRL_OFF_OB_PCI + sizeof(uint32_t):
    if (s->ob_idx >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: write to OB_PCI[hi] with invalid ob_idx %u",
                     __func__, s->ob_idx);
      return;
    }
    tmp = s->obs[s->ob_idx].pci;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->obs[s->ob_idx].pci = tmp;
    qemu_epc_debug("ob map pci: %d: 0x%lx", s->ob_idx, tmp);
    break;
  case QEPC_CTRL_OFF_OB_SIZE:
    if (s->ob_idx >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: write to OB_SIZE with invalid ob_idx %u",
                     __func__, s->ob_idx);
      return;
    }
    tmp = s->obs[s->ob_idx].size;
    tmp = (tmp & ~0xffffffff) | (val & 0xffffffff);
    s->obs[s->ob_idx].size = tmp;
    break;
  case QEPC_CTRL_OFF_OB_SIZE + sizeof(uint32_t):
    if (s->ob_idx >= NUM_OB_WINDOW) {
      qemu_epc_debug("%s: write to OB_SIZE[hi] with invalid ob_idx %u",
                     __func__, s->ob_idx);
      return;
    }
    tmp = s->obs[s->ob_idx].size;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->obs[s->ob_idx].size = tmp;
    qemu_epc_debug("ob map size: %d: 0x%lx", s->ob_idx, tmp);
    break;
  default:
    qemu_epc_debug("CTRL write: invalid address 0x%lx", addr);
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
  void *ptr;
  uint64_t tmp;

  qemu_epc_debug("%s: write addr=0x%lx size=%u val=0x%lx", __func__,
                 (unsigned long)addr, size, (unsigned long)val);

  if (addr + size > QEPC_BAR_CFG_OFF_SIZE + 8) {
    /* TODO: Turn this into a hard error once the ABI is stable. */
    qemu_epc_debug("%s: overrun %ld", __func__, addr + size);
  }

  switch (addr) {
  case QEPC_BAR_CFG_OFF_MASK:
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
  case QEPC_BAR_CFG_OFF_FLAGS:
    if (s->bar_no >= PCI_NUM_REGIONS) {
      qemu_epc_debug("%s: FLAGS write with invalid bar_no %u", __func__,
                     s->bar_no);
      return;
    }
    s->bars[s->bar_no].flags = val;
    ptr = s->pcie_config + 0x10 + 4 * s->bar_no;
    memcpy(&tmp, ptr, sizeof(uint64_t));
    tmp = (val & 0xf) | (tmp & 0xfffffff0) | 0x4;
    memcpy(ptr, &tmp, sizeof(uint64_t));
    qemu_epc_debug("%s: bar[%d] 0x%lx (flags %lx)", __func__, s->bar_no, tmp,
                   val);
    break;
  case QEPC_BAR_CFG_OFF_RSV:
    break;
  case QEPC_BAR_CFG_OFF_PHYS_ADDR:
    if (s->bar_no >= PCI_NUM_REGIONS) {
      qemu_epc_debug("%s: PHYS_ADDR write with invalid bar_no %u", __func__,
                     s->bar_no);
      return;
    }
    s->bars[s->bar_no].phys_addr = val;
    ptr = s->pcie_config + 0x10 + 4 * s->bar_no;
    memcpy(&tmp, ptr, sizeof(uint64_t));
    tmp = (val & 0xfffffff0) | (tmp & 0xf);
    memcpy(ptr, &tmp, sizeof(uint64_t));
    qemu_epc_debug("%s: bar[%d] 0x%lx(addr %lx)", __func__, s->bar_no, tmp,
                   val);
    break;
  case QEPC_BAR_CFG_OFF_SIZE:
    if (s->bar_no >= PCI_NUM_REGIONS) {
      qemu_epc_debug("%s: SIZE write with invalid bar_no %u", __func__,
                     s->bar_no);
      return;
    }
    if (val == 0) {
      qemu_epc_debug("%s: ignoring zero-sized BAR %u", __func__, s->bar_no);
      return;
    }
    s->bars[s->bar_no].size = val;
    break;
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

  /*
   * Allocate page-aligned backing for PCIe config space so that it can be
   * safely mapped as RAM by KVM without triggering alignment errors.
   */
  size_t page_size = qemu_real_host_page_size();
  if (PCIE_CONFIG_SPACE_SIZE % page_size != 0) {
    qemu_epc_debug("%s: PCIE_CONFIG_SPACE_SIZE (0x%zx) is not a multiple of "
                   "host page size (0x%zx)", __func__,
                   (size_t)PCIE_CONFIG_SPACE_SIZE, page_size);
    error_setg(errp,
               "qemu-epc: PCIE_CONFIG_SPACE_SIZE is not page-size aligned");
    return;
  }

  s->pcie_config = qemu_memalign(page_size, PCIE_CONFIG_SPACE_SIZE);
  if (!s->pcie_config) {
    error_setg(errp, "qemu-epc: failed to allocate pcie_config");
    return;
  }
  memset(s->pcie_config, 0, PCIE_CONFIG_SPACE_SIZE);

  /*
   * Initialize vfio-user related state. The vfio-user context itself is
   * created lazily when the guest writes to QEPC_CTRL_OFF_START.
   */
  qemu_epc_debug("initializing ctrl_mr");
  memory_region_init_io(&s->ctrl_mr, OBJECT(s), &qepc_ctrl_mmio_ops, s,
                        "qemu-epc/ctrl", pow2ceil(QEPC_CTRL_SIZE));

  qemu_epc_debug("initializing pci_cfg_mr");
  memory_region_init_ram_ptr(&s->pci_cfg_mr, OBJECT(s), "qemu-epc/cfg-cfg",
                             PCIE_CONFIG_SPACE_SIZE, s->pcie_config);

  qemu_epc_debug("initializing bar_cfg_mr");
  memory_region_init_io(&s->bar_cfg_mr, OBJECT(s), &qemu_epc_mmio_bar_cfg_ops,
                        s, "qemu-epc/bar-cfg", pow2ceil(QEPC_BAR_CFG_SIZE));

  qemu_epc_debug("initializing ob_window_mr");
  memory_region_init(&s->ob_window_mr, NULL, "qemu-epc/ob",
                     pow2ceil(NUM_OB_WINDOW * OB_WINDOW_SIZE));
  qemu_epc_debug("registering BAR %d (ctrl)", QEPC_BAR_CTRL);
  pci_register_bar(pci_dev, QEPC_BAR_CTRL, PCI_BASE_ADDRESS_MEM_TYPE_32,
                   &s->ctrl_mr);
  qemu_epc_debug("registering BAR %d (pci_cfg)", QEPC_BAR_PCI_CFG);
  pci_register_bar(pci_dev, QEPC_BAR_PCI_CFG, PCI_BASE_ADDRESS_SPACE_MEMORY,
                   &s->pci_cfg_mr);
  qemu_epc_debug("registering BAR %d (bar_cfg)", QEPC_BAR_BAR_CFG);
  pci_register_bar(pci_dev, QEPC_BAR_BAR_CFG, PCI_BASE_ADDRESS_SPACE_MEMORY,
                   &s->bar_cfg_mr);

  qemu_epc_debug("registering BAR %d (ob_windows)", QEPC_BAR_OB_WINDOWS);
  pci_register_bar(pci_dev, QEPC_BAR_OB_WINDOWS, PCI_BASE_ADDRESS_MEM_TYPE_64,
                   &s->ob_window_mr);

  qemu_epc_debug("device memory realized and pinned");
  qemu_epc_debug(
      "%s: initial state: sock_path=%s bar_mask=0x%x ob_mask=0x%x rc_mr_idx=%d",
      __func__, s->sock_path ? s->sock_path : "<unset>", s->bar_mask,
      s->ob_mask, s->rc_mr_idx);
  /*
   * TODO: Implement a corresponding .exit callback that:
   *   - Tears down the vfio-user context if it was created,
   *   - Unregisters any rc_local_mr subregions from the IOMMU address space,
   *   - Closes s->vfu_fd and frees any dynamically allocated resources.
   */
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
static void qepc_object_set_socket(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp) {
  QEPCState *s = QEMU_EPC(obj);

  visit_type_SocketAddress(v, name, &s->socket, errp);
  if (s->socket->type != SOCKET_ADDRESS_TYPE_UNIX) {
    error_setg(errp, "qemu-epc: Unsupported socket type - %s",
               SocketAddressType_str(s->socket->type));

    s->socket = NULL;
    return;
  }
}
*/

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
  // k->exit = qepc_exit;
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
