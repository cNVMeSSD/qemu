/*
 * epc_mem.c — DMA region registration and outbound window translation
 *             for the QEMU PCI Endpoint Controller (EPC) device.
 *
 * This file implements the two libvfio-user DMA callbacks:
 *
 *   qepc_dma_register()   — called when the vfio-user client maps a new DMA
 *                            region into the IOVA space.
 *   qepc_dma_unregister() — called when the vfio-user client unmaps a
 *                            previously registered DMA region.
 *
 * DMA region lifecycle
 * --------------------
 * When the remote Root Complex (RC) driver registers a DMA region it supplies:
 *   - An IOVA range  (info->iova.iov_base .. iov_base + iov_len).
 *   - A host virtual address (info->vaddr) that QEMU can use to access the
 *     backing memory directly without going through a second copy.
 *
 * qepc_dma_register() records each such mapping in the QEPCState::rc_mrs[]
 * table (up to SUPPORT_RC_NUM_MRS entries). The table is later consulted by
 * qepc_handle_single_window_setup() (epc_regs.c) when the guest EP driver
 * enables an outbound window: the window's programmed PCI/IOVA address is
 * looked up in rc_mrs[] so that a host-virtual-address-backed RAM pointer
 * MemoryRegion can be installed in the OB aperture, giving the EP guest
 * zero-copy access to RC memory.
 *
 * qepc_dma_unregister() removes the corresponding entry from rc_mrs[] by
 * shifting subsequent entries down to keep the table compact.  Before
 * reclaiming the slot it also tears down any outbound window fast-path
 * MemoryRegions that referenced the departing rc_mr entry, so that the guest
 * cannot continue to access memory that the RC has unmapped.
 *
 * Known limitations
 * -----------------
 *  - rc_mrs[] is a fixed-size linear array.  If more than SUPPORT_RC_NUM_MRS
 *    regions are registered simultaneously the excess registrations are
 *    silently dropped.  A future implementation should either grow the table
 *    dynamically or propagate a proper error to the vfio-user client.
 *
 *  - The unregister path walks the entire rc_mrs[] array with O(n) cost.
 *    For the current maximum table size this is acceptable, but a hash table
 *    keyed on (iova_base, len, vaddr) would be more scalable.
 *
 *  - Only outbound windows that are fully contained within the departing DMA
 *    region are torn down.  Partially overlapping windows (which should not
 *    occur given correct guest driver behaviour) are left mapped.
 */

#include "qemu_epc.h"

/*
 * qepc_dma_register
 *
 * libvfio-user DMA registration callback. Called by libvfio-user when the
 * remote vfio-user client (the Root Complex driver) maps a new IOVA region
 * and provides the corresponding host virtual address that QEMU can use for
 * direct memory access.
 *
 * The registration is recorded in QEPCState::rc_mrs[] at the next free slot
 * (rc_mr_idx). The stored entry holds:
 *   - rc_phys : the IOVA base address (info->iova.iov_base), used as the key
 *               when matching outbound window PCI addresses.
 *   - size    : the length of the IOVA range (info->iova.iov_len).
 *   - vaddr   : the host virtual address supplied by libvfio-user, used to
 *               initialise RAM-pointer MemoryRegions for OB windows.
 *
 * Registrations without a valid host virtual address (info->vaddr == NULL)
 * are rejected because they cannot back a RAM-pointer MemoryRegion; shared
 * memory is required for the fast-path OB window mapping to work.
 *
 * If the rc_mrs[] table is already full (rc_mr_idx == SUPPORT_RC_NUM_MRS)
 * the registration is dropped with a debug log message.
 *
 * TODO: Grow the rc_mrs[] table dynamically or propagate a proper error
 *       to the vfio-user client when the table is full, rather than silently
 *       dropping the registration.
 */
void qepc_dma_register(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info) {
  QEPCState *s = vfu_get_private(vfu_ctx);

  qemu_epc_debug(
      "%s: register iova=[0x%lx..0x%lx) len=0x%lx vaddr=%p prot=0x%x",
      __func__, (uint64_t)info->iova.iov_base,
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
      .size    = info->iova.iov_len,
      .vaddr   = info->vaddr,
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
 * libvfio-user DMA unregistration callback. Called by libvfio-user when the
 * remote vfio-user client (the Root Complex driver) unmaps a previously
 * registered IOVA region.
 *
 * This function performs three steps:
 *
 *   1. Locate the matching rc_mrs[] entry by comparing rc_phys, size, and
 *      vaddr against the values in info. If no match is found the function
 *      logs a debug message and returns — this should not happen under
 *      correct guest driver behaviour.
 *
 *   2. Tear down any outbound window fast-path mappings that reference the
 *      departing DMA region. An OB window is considered to reference the
 *      region if its programmed PCI address range [obs[w].pci,
 *      obs[w].pci + obs[w].size) is fully contained within the region's
 *      IOVA range [rc_phys, rc_phys + size). For each such window:
 *        - The per-window MemoryRegion subregion is removed from the OB
 *          aperture container (ob_window_mr).
 *        - ob_fast_mapped[w] is cleared.
 *        - The corresponding ob_mask bit is cleared so the guest sees the
 *          window as inactive.
 *
 *   3. Compact the rc_mrs[] table by shifting all entries after the removed
 *      slot one position toward index zero, decrementing rc_mr_idx, and
 *      zeroing the now-unused last slot.
 *
 * NOTE: Only windows whose PCI range is *fully* contained within the
 * departing region are torn down. Partially overlapping windows are left
 * in place; such overlap should not occur with a correct EP driver.
 *
 * TODO: Consider propagating a notification to the guest when OB windows are
 *       forcibly torn down, so that the EP driver can remap them without a
 *       full device reset.
 */
void qepc_dma_unregister(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info) {
  QEPCState *s = vfu_get_private(vfu_ctx);

  qemu_epc_debug("%s: unregister iova=[0x%lx..0x%lx) len=0x%lx vaddr=%p",
                 __func__, (uint64_t)info->iova.iov_base,
                 (uint64_t)info->iova.iov_base + info->iova.iov_len,
                 (uint64_t)info->iova.iov_len, info->vaddr);

  /*
   * Walk rc_mrs[] to find the entry that matches the unregistered region.
   * All three fields (rc_phys, size, vaddr) must match to ensure we do not
   * accidentally discard an entry for a different but overlapping mapping.
   */
  for (int i = 0; i < s->rc_mr_idx; i++) {
    if (s->rc_mrs[i].rc_phys == (uint64_t)info->iova.iov_base &&
        s->rc_mrs[i].size    == info->iova.iov_len             &&
        s->rc_mrs[i].vaddr   == info->vaddr) {

      qemu_epc_debug(
          "%s: found matching rc_mrs[%d], rc_phys=0x%lx size=0x%lx vaddr=%p",
          __func__, i, (unsigned long)s->rc_mrs[i].rc_phys,
          (unsigned long)s->rc_mrs[i].size, s->rc_mrs[i].vaddr);

      /*
       * Unmap any OB windows whose PCI address range lies entirely within
       * the departing DMA region. This prevents the guest from continuing
       * to access memory that the RC has just unmapped.
       */
      for (int w = 0; w < NUM_OB_WINDOW; w++) {
        if (!s->ob_fast_mapped[w]) {
          continue;
        }

        uint64_t pci = s->obs[w].pci;
        uint64_t sz  = s->obs[w].size;

        if (pci >= s->rc_mrs[i].rc_phys &&
            (pci + sz) <= (s->rc_mrs[i].rc_phys + s->rc_mrs[i].size)) {

          qemu_epc_debug("%s: removing fast mapping for ob window %d "
                         "(pci=0x%lx size=0x%lx)",
                         __func__, w, (unsigned long)pci, (unsigned long)sz);

          /* Remove the per-window subregion from the OB aperture container. */
          memory_region_del_subregion(&s->ob_window_mr, &s->rc_local_mr[w]);

          s->ob_fast_mapped[w] = false;
          s->ob_mask &= ~(1 << w);
        }
      }

      /*
       * Compact the rc_mrs[] table: shift every entry after position i one
       * slot toward the front, then zero the now-unused tail entry and
       * decrement rc_mr_idx.
       */
      for (int k = i; k + 1 < s->rc_mr_idx; k++) {
        s->rc_mrs[k] = s->rc_mrs[k + 1];
      }

      /* Decrement before zeroing so rc_mr_idx always points at the first
       * unused slot. Guard against underflow even though it should not
       * occur in practice. */
      s->rc_mr_idx--;
      if (s->rc_mr_idx >= 0) {
        s->rc_mrs[s->rc_mr_idx].rc_phys = 0;
        s->rc_mrs[s->rc_mr_idx].size    = 0;
        s->rc_mrs[s->rc_mr_idx].vaddr   = NULL;
      }

      qemu_epc_debug("%s: rc_mrs compacted, new rc_mr_idx=%d", __func__,
                     s->rc_mr_idx);
      return;
    }
  }

  /* No matching entry found — log for debugging; nothing else to do. */
  qemu_epc_debug("%s: no matching rc_mr found for unregister", __func__);
}