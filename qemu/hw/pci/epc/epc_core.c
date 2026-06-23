/*
 * epc_core.c — QOM type registration and device lifecycle for the QEMU PCI
 *              Endpoint Controller (EPC) device.
 *
 * This file is responsible for:
 *   - The MemoryRegionOps dispatch tables that wire up the guest-visible MMIO
 *     regions to the register handlers defined in epc_regs.c.
 *   - The PCIDeviceClass::realize callback (qepc_realize), which allocates and
 *     initialises every MemoryRegion and registers them as PCI BARs.
 *   - The PCIDeviceClass::exit callback (qepc_exit), which tears down outbound
 *     window fast-path mappings, destroys the vfio-user context, and frees
 *     any other dynamically allocated resources.
 *   - The QOM object property setter (qepc_object_set_path) for the
 *     "sock-path" property.
 *   - The QOM class initialiser (qepc_class_init) and static TypeInfo
 *     (qepc_info).
 *   - The type_init() entry point that registers the device with QEMU's type
 *     system at startup.
 *
 * All functions in this file are file-private (static) and are invoked either
 * by the QOM / PCI infrastructure via function-pointer callbacks or directly
 * within this translation unit.  None of them need to be declared in the
 * shared header.
 */

#include "qemu_epc.h"

/* =========================================================================
 * MemoryRegionOps tables
 *
 * These tables reference qepc_ctrl_mmio_read / qepc_ctrl_mmio_write (defined
 * in epc_regs.c) and qepc_cfg_bar_read / qepc_cfg_bar_write (also defined in
 * epc_regs.c).  The prototypes are available via qemu_epc.h so no additional
 * declarations are needed here.
 * ========================================================================= */

/*
 * qepc_ctrl_mmio_ops — MemoryRegionOps for the EPC control BAR.
 *
 * Read/write handlers are implemented in epc_regs.c.  The control BAR exposes
 * the outbound window aperture parameters, the ob_mask, and the command
 * registers for starting the vfio-user backend, triggering interrupts, and
 * configuring individual outbound windows.
 */
static const MemoryRegionOps qepc_ctrl_mmio_ops = {
    .read = qepc_ctrl_mmio_read,
    .write = qepc_ctrl_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * qemu_epc_mmio_bar_cfg_ops — MemoryRegionOps for the BAR configuration MMIO
 * window.
 *
 * Read/write handlers are implemented in epc_regs.c.  This region lets the
 * guest driver configure per-BAR attributes (physical address, size, flags,
 * enable mask) before the vfio-user backend is started.
 */
static const MemoryRegionOps qemu_epc_mmio_bar_cfg_ops = {
    .read = qepc_cfg_bar_read,
    .write = qepc_cfg_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};



/* =========================================================================
 * PCIDeviceClass callbacks
 * ========================================================================= */

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
static void qepc_realize(PCIDevice *pci_dev, Error **errp)
{
    QEPCState *s = QEMU_EPC(pci_dev);

    if (!s->sock_path) {
        error_setg(errp, "qemu-epc: sock_path should be set");
        return;
    }

    /*
     * Initialize interrupt capability state to "not yet configured".
     * The guest driver programs these via the QEPC_CTRL_OFF_MSIX_CFG,
     * QEPC_CTRL_OFF_MSI_CFG, QEPC_CTRL_OFF_DB_BAR, and QEPC_CTRL_OFF_DB_OFF
     * control registers before issuing QEPC_CTRL_OFF_START.
     *
     * doorbell_bar uses 0xFF as a sentinel for "not configured"; any write
     * to qepc_bar_access() checks this before intercepting as a doorbell.
     */
    s->msix_num        = 0;
    s->msix_table_bar  = 0;
    s->msix_table_off  = 0;
    s->msix_table_size = 0;
    s->msix_pba_bar    = 0;
    s->msix_pba_off    = 0;
    s->msix_pba_size   = 0;
    s->msi_num         = 0;
    s->doorbell_bar    = 0xFF; /* sentinel: no doorbell configured */
    s->doorbell_offset = 0;

//    /*
//     * Add a 32-vector MSI capability to this PCI device so that server Linux
//     * can call pci_alloc_irq_vectors() on it.  This MSI is entirely local to
//     * the server machine — it is unrelated to the MSI/MSI-X capability that
//     * qepc_ctrl_handle_start() later registers with libvfio-user for the RC
//     * side.  When the RC rings the doorbell (writes to the configured BAR
//     * offset), qepc_bar_access() calls msi_notify(&s->dev, 0) to deliver
//     * this interrupt to the server kernel.
//     *
//     * offset=0   → let QEMU auto-place the capability in config space.
//     * nr_vectors=32 → maximum allowed by the PCI MSI spec; covers any number
//     *                 of doorbell channels the EPF driver may request.
//     * msi64bit=true → advertise 64-bit message address support.
//     * msi_per_vector_mask=true → allow per-vector masking.
//     */
//    if (msi_init(pci_dev, 0, 16, true, true, errp) < 0) {
//        return;
//    }
//    trace_qepc_realize_msi_init(16);
//
    trace_qepc_realize_ctrl_bar((uint64_t)pow2ceil(QEPC_CTRL_SIZE), QEPC_CTRL_SIZE);
    memory_region_init_io(&s->ctrl_mr, OBJECT(s), &qepc_ctrl_mmio_ops, s,
                          "qemu-epc/ctrl", pow2ceil(QEPC_CTRL_SIZE));

    /*
     * Allocate page-aligned backing for PCIe config space so that it can be
     * safely mapped as RAM by KVM without triggering alignment errors.
     */
    trace_qepc_realize_pci_config_alloc((uint64_t)PCIE_CONFIG_SPACE_SIZE, s->pcie_config);
    size_t page_size = qemu_real_host_page_size();
    s->pcie_config = qemu_memalign(page_size, PCIE_CONFIG_SPACE_SIZE);
    if (!s->pcie_config) {
        error_setg(errp, "qemu-epc: failed to allocate PCI config space");
        return;
    }
    memset(s->pcie_config, 0, PCIE_CONFIG_SPACE_SIZE);

    /*
     * MSI and MSI-X capabilities are NOT added here at realize time.
     * They are registered with libvfio-user during qepc_ctrl_handle_start()
     * (epc_vfu.c) once the guest driver has programmed the capability
     * parameters via QEPC_CTRL_OFF_MSIX_CFG and QEPC_CTRL_OFF_MSI_CFG.
     * Adding them here would be premature because s->msix_num and s->msi_num
     * are still zero at this point.
     */


    trace_qepc_realize_pci_config_init((uint64_t)PCIE_CONFIG_SPACE_SIZE, s->pcie_config);
    memory_region_init_ram_ptr(&s->pci_cfg_mr, OBJECT(s), "qemu-epc/cfg-cfg",
                               PCIE_CONFIG_SPACE_SIZE, s->pcie_config);

    trace_qepc_realize_bar_cfg((uint64_t)pow2ceil(QEPC_BAR_CFG_SIZE), QEPC_BAR_CFG_SIZE);
    memory_region_init_io(&s->bar_cfg_mr, OBJECT(s), &qemu_epc_mmio_bar_cfg_ops,
                          s, "qemu-epc/bar-cfg", pow2ceil(QEPC_BAR_CFG_SIZE));

    trace_qepc_realize_ob_window((uint64_t)pow2ceil(NUM_OB_WINDOW * OB_WINDOW_SIZE),
        NUM_OB_WINDOW, (uint64_t)OB_WINDOW_SIZE);
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


    // VFIO-User Setup

    /*
     * Implement a corresponding .exit callback that:
     *   - Tears down the vfio-user context if it was created,
     *   - Unregisters any rc_local_mr subregions from the OB aperture,
     *   - Closes s->vfu_fd and frees any dynamically allocated resources.
     */
}

static void qepc_exit(PCIDevice *pci_dev)
{
    QEPCState *s = QEMU_EPC(pci_dev);

    trace_qepc_exit();
//
//    /* Tear down the server-side doorbell MSI capability. */
//    msi_uninit(pci_dev);

    /* Remove any per-window fast mappings */
    for (int i = 0; i < NUM_OB_WINDOW; i++) {
        if (s->ob_fast_mapped[i]) {
            trace_qepc_exit_unmap_ob(i);
            memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[i]);
            s->ob_fast_mapped[i] = false;
        }
    }

    /* Destroy vfu context if present */
    if (s->vfu) {
        trace_qepc_exit_destroy_vfu();
        vfu_destroy_ctx(s->vfu);
        s->vfu = NULL;
    }

    /* Clear fd handler if one was installed */
    if (s->vfu_fd >= 0) {
        trace_qepc_exit_clear_fd(s->vfu_fd);
        qemu_set_fd_handler(s->vfu_fd, NULL, NULL, NULL);
        s->vfu_fd = -1;
    }

    /* Free duplicated socket path if present */
    if (s->sock_path) {
        g_free((void *)s->sock_path);
        s->sock_path = NULL;
    }
}

/* =========================================================================
 * QOM property setter
 * ========================================================================= */

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
static void qepc_object_set_path(Object *obj, const char *str, Error **errp)
{
    QEPCState *s = QEMU_EPC(obj);

    trace_qepc_set_path(str);
    s->sock_path = g_strdup(str);
}

/* =========================================================================
 * QOM class initialiser
 * ========================================================================= */

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
static void qepc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    trace_qepc_class_init();

    object_class_property_add_str(klass, "sock-path", NULL,
                                  qepc_object_set_path);

    k->realize = qepc_realize;
    k->exit = qepc_exit;
    k->vendor_id = PCI_VENDOR_ID_REDHAT;
    k->device_id = PCI_DEVICE_ID_REDHAT_QEMU_EPC;
    k->revision = QEPC_REVISION;
    k->class_id = PCI_CLASS_OTHERS;

    dc->desc = "QEMU Endpoint Controller device";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

/* =========================================================================
 * QOM type registration
 * ========================================================================= */

/*
 * qepc_info
 *
 * Static type information for registering the EPC device with the QOM
 * type system. Declares this device as a PCI device that implements the
 * conventional PCI device interface.
 */
static const TypeInfo qepc_info = {
    .name          = TYPE_QEMU_EPC,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(QEPCState),
    .class_init    = qepc_class_init,
    .interfaces    = (InterfaceInfo[]){{INTERFACE_CONVENTIONAL_PCI_DEVICE}, {}},
};

/*
 * qemu_epc_register_type
 *
 * Type registration entry point called at QEMU initialization time.
 */
static void qemu_epc_register_type(void) { type_register_static(&qepc_info); }

type_init(qemu_epc_register_type);
