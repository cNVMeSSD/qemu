/*
 * qemu_epc.h — Central shared header for the QEMU PCI Endpoint Controller
 *
 * This header is included by every .c file that makes up the EPC device:
 *   - epc_core.c   : QOM type registration and device lifecycle (realize/exit)
 *   - epc_regs.c   : MMIO register handlers (control BAR and BAR-cfg BAR)
 *   - epc_vfu.c    : libvfio-user context setup and event-loop callbacks
 *   - epc_mem.c    : DMA region registration/unregistration helpers
 *
 * It provides:
 *   - All required #includes
 *   - Debug logging macro
 *   - Shared constants and #defines
 *   - The QEPCState device-state struct
 *   - Enumerations for BAR indices, IRQ types, and register offsets
 *   - The qepc_bar_access_handler_t callback typedef
 *   - Cross-file function prototypes (only those called by name from a
 *     different translation unit)
 */

#ifndef QEMU_EPC_H
#define QEMU_EPC_H

/* =========================================================================
 * Includes
 * ========================================================================= */

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

/* =========================================================================
 * Debug logging
 * ========================================================================= */

/* Uncomment to enable verbose debug logging for this device. */
#define DEBUG_QEMU_EPC
#ifdef DEBUG_QEMU_EPC
#define qemu_epc_debug(fmt, ...) qemu_log("qemu_epc: " fmt "\n", ##__VA_ARGS__)
#else
#define qemu_epc_debug(...)                                                    \
  do {                                                                         \
  } while (0)
#endif

/* =========================================================================
 * Constants
 * ========================================================================= */

/*
 * Number of outbound windows the controller exposes.
 * Window 0 is reserved for MSI/MSI-X interrupts.
 * Windows 1..(NUM_OB_WINDOW-1) are available for data transfer.
 */
#define NUM_OB_WINDOW 32

/* Size of each outbound window (guest-visible aperture size). */
#define OB_WINDOW_SIZE 0x40000000ULL

/*
 * Maximum number of registered root complex (RC) memory regions.
 * These correspond to DMA regions registered by the vfio-user client.
 * Should be >= NUM_OB_WINDOW (currently 6: 1 for MSI, 5 for data).
 */
#define SUPPORT_RC_NUM_MRS 32

/* QOM type name for this device. */
#define TYPE_QEMU_EPC "qemu-epc"

#define QEPC_REVISION 0x00

#define QEPC_BAR_CFG_SIZE 1024

/* =========================================================================
 * Device state struct
 * ========================================================================= */

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
  /* Interrupt type (INTx/MSI/MSI-X) as last written by the guest driver. */
  uint8_t irq_type;

  /*
   * MSI-X table location, recorded at vfio-user start time so that
   * qepc_bar_access() can distinguish table writes (which must be forwarded
   * to libvfio-user's config-space mirror) from ordinary BAR data writes.
   *
   * msix_table_bar  : BAR index (0-5) that holds the MSI-X table.
   * msix_table_off  : byte offset of the table within that BAR.
   * msix_table_size : byte length of the table  (num_vectors * 16).
   * msix_pba_bar    : BAR index that holds the PBA.
   * msix_pba_off    : byte offset of the PBA within that BAR.
   * msix_pba_size   : byte length of the PBA  (ceil(num_vectors/8)).
   * msix_num        : number of MSI-X vectors configured by the guest driver.
   *
   * All fields are zero until the guest writes QEPC_CTRL_OFF_MSIX_CFG.
   */
  uint8_t msix_table_bar;
  uint32_t msix_table_off;
  uint32_t msix_table_size;
  uint8_t msix_pba_bar;
  uint32_t msix_pba_off;
  uint32_t msix_pba_size;
  uint16_t msix_num;

  /*
   * MSI capability location, populated when the guest writes
   * QEPC_CTRL_OFF_MSI_CFG.  Used to validate MSI enable/data reads from
   * pcie_config during qepc_handle_ctrl_irq().
   *
   * msi_num : number of MSI vectors (must be a power of two, 1-32).
   */
  uint16_t msi_num;

  /*
   * Doorbell register configuration.
   *
   * A "doorbell" is a write-only register in one of the endpoint BARs.
   * When the Host (Root Complex side) writes to this offset the device
   * model must inject an interrupt into the Guest rather than forwarding
   * the write to Guest RAM.
   *
   * doorbell_bar    : BAR index (0-5) that contains the doorbell register.
   *                   0xFF means "not configured".
   * doorbell_offset : byte offset within doorbell_bar of the register.
   *                   Any write to [doorbell_offset, doorbell_offset+3]
   *                   triggers the doorbell action.
   *
   * The doorbell interrupt is delivered using the same mechanism as
   * qepc_handle_ctrl_irq() — INTx, MSI, or MSI-X depending on irq_type —
   * using the value written as the vector/subindex selector.
   */
  uint8_t doorbell_bar;
  uint32_t doorbell_offset;

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
};

OBJECT_DECLARE_SIMPLE_TYPE(QEPCState, QEMU_EPC)

/* =========================================================================
 * BAR index enum
 * ========================================================================= */

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

/* =========================================================================
 * IRQ type enum
 * ========================================================================= */

/* IRQ types supported by the EPC device */
enum qepc_irq_type {
  IRQ_TYPE_UNKNOWN = 0, /* Unknown interrupt, ignore */
  IRQ_TYPE_INTX    = (1 << 0),    /* Legacy INTx interrupt */
  IRQ_TYPE_MSI     = (1 << 1),     /* MSI interrupt */
  IRQ_TYPE_MSIX    = (1 << 2),    /* MSI-X interrupt */
};

/* =========================================================================
 * Control BAR register offsets enum
 * ========================================================================= */

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
  QEPC_CTRL_OFF_OB_MASK = 0x20, /* RO: Outbound mappings bitmask (uint32_t) */
  QEPC_CTRL_OFF_OB_IDX = 0x24,  /* Current outbound mapping index (uint32_t) */
  QEPC_CTRL_OFF_OB_PHYS =
      0x28, /* Outbound mapping physical address (uint64_t) */
  QEPC_CTRL_OFF_OB_PCI = 0x30,  /* Outbound mapping PCI address (uint64_t) */
  QEPC_CTRL_OFF_OB_SIZE = 0x38, /* Outbound mapping size (uint64_t) */

  /* New command-based OB window registers */
  QEPC_CTRL_OFF_OB_ENABLE =
      0x40, /* WO: Write 1 to enable s->ob_idx, 0 to disable */

  /*
   * MSI-X capability configuration.
   *
   * The guest driver writes these registers before issuing QEPC_CTRL_OFF_START
   * so that qepc_ctrl_handle_start() can call vfu_pci_add_capability() with a
   * correctly populated struct msixcap.
   *
   * QEPC_CTRL_OFF_MSIX_CFG:     WO, write 1 to commit the MSI-X parameters
   *                              below and register the capability.
   * QEPC_CTRL_OFF_MSIX_NUM:     number of vectors (uint16_t, max 2048).
   * QEPC_CTRL_OFF_MSIX_TBL_BAR: BAR index holding the MSI-X table (uint8_t).
   * QEPC_CTRL_OFF_MSIX_TBL_OFF: byte offset of the table in that BAR
   * (uint32_t). QEPC_CTRL_OFF_MSIX_PBA_BAR: BAR index holding the PBA
   * (uint8_t). QEPC_CTRL_OFF_MSIX_PBA_OFF: byte offset of the PBA in that BAR
   * (uint32_t).
   */
  QEPC_CTRL_OFF_MSIX_CFG = 0x48,
  QEPC_CTRL_OFF_MSIX_NUM = 0x4c,
  QEPC_CTRL_OFF_MSIX_TBL_BAR = 0x50,
  QEPC_CTRL_OFF_MSIX_TBL_OFF = 0x54,
  QEPC_CTRL_OFF_MSIX_PBA_BAR = 0x58,
  QEPC_CTRL_OFF_MSIX_PBA_OFF = 0x5c,

  /*
   * MSI capability configuration.
   *
   * QEPC_CTRL_OFF_MSI_CFG: WO, write the number of MSI vectors (1-32,
   *                         must be a power of two) to register the MSI
   *                         capability with libvfio-user.
   */
  QEPC_CTRL_OFF_MSI_CFG = 0x60,

  /*
   * Doorbell register configuration.
   *
   * The guest programs these once before issuing START so that
   * qepc_bar_access() knows which BAR write to intercept.
   *
   * QEPC_CTRL_OFF_DB_BAR: BAR index (0-5) containing the doorbell (uint8_t).
   * QEPC_CTRL_OFF_DB_OFF: byte offset of the doorbell register (uint32_t).
   *                        Any 4-byte write to this offset is intercepted.
   */
  QEPC_CTRL_OFF_DB_BAR = 0x64,
  QEPC_CTRL_OFF_DB_OFF = 0x68,

  /* End of control register space */
  QEPC_CTRL_SIZE = QEPC_CTRL_OFF_DB_OFF + sizeof(uint32_t),
};

/* =========================================================================
 * BAR-CFG register offsets enum
 * ========================================================================= */

enum {
  QEPC_BAR_CFG_OFF_MASK = 0x00,
  QEPC_BAR_CFG_OFF_NUMBER = 0x01,
  QEPC_BAR_CFG_OFF_FLAGS = 0x02,
  QEPC_BAR_CFG_OFF_RSV = 0x04,
  QEPC_BAR_CFG_OFF_PHYS_ADDR = 0x08,
  QEPC_BAR_CFG_OFF_SIZE = 0x10,
};

/* =========================================================================
 * BAR access callback typedef
 * ========================================================================= */

/* Signature for bar access callbacks used with libvfio-user. */
typedef ssize_t (*qepc_bar_access_handler_t)(vfu_ctx_t *vfu_ctx,
                                             char *const buf, size_t count,
                                             loff_t offset,
                                             const bool is_write);

/* =========================================================================
 * Cross-file function prototypes
 *
 * Only functions that are called BY NAME from a DIFFERENT .c file appear
 * here.  Functions that are used exclusively as callbacks via function
 * pointers and never called by name from another translation unit are kept
 * static in their own file and intentionally omitted:
 *
 *   epc_vfu.c internals (static, never called by name from outside):
 *     - qepc_vfu_log         : libvfio-user log callback, passed as pointer
 *     - qepc_vfu_run         : fd handler, installed via qemu_set_fd_handler
 *     - qepc_vfu_attach_ctx  : fd handler, installed via qemu_set_fd_handler
 *     - qepc_bar_access      : BAR access helper, wrapped by QEPC_ACCESS_BAR
 *     - qepc_pci_cfg_access  : vfio-user region callback, passed as pointer
 *     - qepc_pci_cfg_access_bar : called only from qepc_pci_cfg_access (same
 *                                 file)
 * ========================================================================= */

/* -------------------------------------------------------------------------
 * From epc_regs.c
 *
 * These are referenced by the MemoryRegionOps tables defined in epc_core.c
 * and therefore must be visible across translation units.
 * ---------------------------------------------------------------------- */

/**
 * qepc_ctrl_mmio_read - MMIO read handler for the EPC control BAR.
 * @opaque: pointer to QEPCState
 * @addr:   register offset within the control BAR
 * @size:   access size in bytes (expected 4 or 8)
 *
 * Returns the register value at @addr, or 0 for unhandled offsets.
 */
uint64_t qepc_ctrl_mmio_read(void *opaque, hwaddr addr, unsigned size);

/**
 * qepc_ctrl_mmio_write - MMIO write handler for the EPC control BAR.
 * @opaque: pointer to QEPCState
 * @addr:   register offset within the control BAR
 * @val:    value to write
 * @size:   access size in bytes (expected 4 or 8)
 */
void qepc_ctrl_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size);

/**
 * qepc_cfg_bar_read - MMIO read handler for the BAR configuration region.
 * @opaque: pointer to QEPCState
 * @addr:   register offset within the BAR-cfg BAR
 * @size:   access size in bytes
 *
 * Returns the register value at @addr, or 0 for unhandled offsets.
 */
uint64_t qepc_cfg_bar_read(void *opaque, hwaddr addr, unsigned size);

/**
 * qepc_cfg_bar_write - MMIO write handler for the BAR configuration region.
 * @opaque: pointer to QEPCState
 * @addr:   register offset within the BAR-cfg BAR
 * @val:    value to write
 * @size:   access size in bytes
 */
void qepc_cfg_bar_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);

/**
 * qepc_handle_ctrl_irq - Trigger a device interrupt toward the vfio-user
 *                        client.
 * @s:       pointer to QEPCState
 * @irq_num: interrupt number / MSI vector (meaning depends on s->irq_type)
 *
 * Dispatches the interrupt via vfu_irq_trigger() using the IRQ type recorded
 * in s->irq_type.  Called from qepc_ctrl_mmio_write when
 * QEPC_CTRL_OFF_IRQ_NUM is written.
 */
void qepc_handle_ctrl_irq(QEPCState *s, int irq_num);

/**
 * qepc_handle_single_window_setup - Enable or disable a single outbound
 *                                   window.
 * @s:      pointer to QEPCState
 * @idx:    outbound window index (0..NUM_OB_WINDOW-1)
 * @enable: true to map the window, false to unmap it
 *
 * Looks up a registered RC DMA region that covers obs[idx].pci, creates (or
 * removes) a RAM-pointer MemoryRegion subregion within ob_window_mr, and
 * updates ob_mask accordingly.  Called from qepc_ctrl_mmio_write when
 * QEPC_CTRL_OFF_OB_ENABLE is written.
 */
void qepc_handle_single_window_setup(QEPCState *s, uint32_t idx, bool enable);

/* -------------------------------------------------------------------------
 * From epc_vfu.c
 *
 * Called by name from epc_regs.c (qepc_ctrl_mmio_write) to initialize the
 * libvfio-user backend when the guest writes to QEPC_CTRL_OFF_START.
 * ---------------------------------------------------------------------- */

/**
 * qepc_ctrl_handle_start - Initialize and start the vfio-user backend.
 * @s:   pointer to QEPCState
 * @val: value written to QEPC_CTRL_OFF_START (reserved for future flags)
 *
 * Creates the vfio-user context bound to s->sock_path, sets up PCI config
 * space, BAR regions, IRQ counts, and DMA callbacks, then realizes the
 * context and installs the attach fd handler.
 *
 * Returns 0 on success, -1 on error.
 */
int qepc_ctrl_handle_start(QEPCState *s, uint64_t val);

/* -------------------------------------------------------------------------
 * From epc_mem.c
 *
 * Passed as function pointers to vfu_setup_device_dma() inside epc_vfu.c.
 * They are declared here so that epc_vfu.c can reference them by name when
 * building the pointer values passed to libvfio-user.
 * ---------------------------------------------------------------------- */

/**
 * qepc_dma_register - libvfio-user DMA registration callback.
 * @vfu_ctx: the vfio-user context that owns the registration
 * @info:    descriptor of the IOVA range and its host virtual address
 *
 * Records the IOVA-to-vaddr mapping in QEPCState::rc_mrs[] so that
 * outbound window setup can locate a suitable host pointer.
 */
void qepc_dma_register(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info);

/**
 * qepc_dma_unregister - libvfio-user DMA unregistration callback.
 * @vfu_ctx: the vfio-user context that owns the registration
 * @info:    descriptor of the IOVA range being removed
 *
 * Removes the matching entry from QEPCState::rc_mrs[], unmaps any outbound
 * window fast-path MemoryRegion subregions that depended on it, and
 * compacts the rc_mrs[] array.
 */
void qepc_dma_unregister(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info);

#endif /* QEMU_EPC_H */
