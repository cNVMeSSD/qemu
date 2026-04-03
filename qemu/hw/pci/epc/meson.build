# =============================================================================
# QEMU PCI-Express Endpoint Controller (EPC) — build rules
#
# The original monolithic implementation (epc.c) has been refactored into
# the four translation units listed below. epc.c is retained as a reference
# but is excluded from the build.
# =============================================================================

system_ss.add(when: 'CONFIG_PCI_EPC', if_true: files(
    'epc_core.c',   # QOM lifecycle: realize, exit, class/type init, properties
    'epc_regs.c',   # Guest MMIO handlers: ctrl BAR and BAR-cfg read/write
    'epc_vfu.c',    # vfio-user socket and remote RC callback lifecycle
    'epc_mem.c',    # DMA helpers: RC memory-region register/unregister
))
