/*
 * QEMU PCI Endpoint Controller device
			__func__, is_write ? "write" : "read", offset, count);
 */

#include "qemu/osdep.h"
#include "qom/object.h"

#include "qemu/log.h"

#include "qapi/error.h"
#include "qapi/qapi-visit-sockets.h"

#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/remote/vfio-user-obj.h"

#include "libvfio-user.h"

#define DEBUG_QEMU_EPC
#ifdef DEBUG_QEMU_EPC
#define qemu_epc_debug(fmt, ...) qemu_log("qemu_epc: " fmt "\n", ## __VA_ARGS__)
#else
#define qemu_epc_debug(...)                                                    \
  do {                                                                         \
  } while (0)
#endif

#define NUM_OB_WINDOW 5
#define OB_WINDOW_SIZE 0x40000000ULL

#define SUPPORT_RC_NUM_MRS 5

struct QEPCState {
  /*< private >*/
  PCIDevice dev;

  vfu_ctx_t *vfu;
  int vfu_fd;

  const char *sock_path;

  /*< public >*/
  MemoryRegion ctrl_mr, pci_cfg_mr, bar_cfg_mr;
  MemoryRegion ob_window_mr;

  QemuThread thread;

  struct {
	uint64_t phys_addr;
	uint64_t size;
	uint8_t flags;
  } bars[6];

  uint8_t bar_mask;
  uint8_t bar_no;

  uint8_t pcie_config[PCIE_CONFIG_SPACE_SIZE];
  uint8_t irq_type;

  struct {
	  uint64_t phys;
	  uint64_t pci;
	  uint64_t size;
  } obs[NUM_OB_WINDOW];
  uint8_t ob_mask;
  uint8_t ob_idx;

  struct rc_mr {
	uint64_t rc_phys;
	void *vaddr;
	uint64_t size;
  } rc_mrs[SUPPORT_RC_NUM_MRS];
  int rc_mr_idx;

  MemoryRegion rc_local_mr[NUM_OB_WINDOW];
};

#define TYPE_QEMU_EPC "qemu-epc"
OBJECT_DECLARE_SIMPLE_TYPE(QEPCState, QEMU_EPC)

#define QEPC_REVISION 0x00

#define QEPC_BAR_CFG_SIZE 1024

enum {
  QEPC_BAR_CTRL = 0,
  QEPC_BAR_PCI_CFG = 1,
  QEPC_BAR_BAR_CFG = 2,
  QEPC_BAR_OB_WINDOWS = 3,
};

enum {
	QEPC_CTRL_OFF_START		= 0x00,
	QEPC_CTRL_OFF_WIN_START = 0x08,
	QEPC_CTRL_OFF_WIN_SIZE	= 0x10,
	QEPC_CTRL_OFF_IRQ_TYPE	= 0x18,
	QEPC_CTRL_OFF_IRQ_NUM	= 0x1c,
	QEPC_CTRL_OFF_OB_MASK	= 0x20,
  	QEPC_CTRL_OFF_OB_IDX	= 0x24,
    QEPC_CTRL_OFF_OB_PHYS	= 0x28,
    QEPC_CTRL_OFF_OB_PCI	= 0x30,
    QEPC_CTRL_OFF_OB_SIZE	= 0x38,

	QEPC_CTRL_SIZE	= QEPC_CTRL_OFF_OB_SIZE + sizeof(uint64_t)
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

static uint64_t qepc_ctrl_mmio_read(void *opaque, hwaddr addr, unsigned size) {
  QEPCState *s = opaque;

  qemu_epc_debug("CTRL read: addr 0x%lx, size 0x%x", addr, size);

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

static ssize_t qepc_pci_cfg_access_bar(QEPCState *s,  char *const buf,
                                   size_t count, loff_t offset,
                                   const bool is_write)
{
	uint32_t t1, t2;

    qemu_epc_debug("%s: %s: offset 0x%lx, size 0x%lx",
			__func__, is_write ? "write" : "read", offset, count);

	assert(count == sizeof(uint32_t));

	if (is_write) {
		switch(offset) {
			case 0x10: // BAR 0
				memcpy(&t1, buf, sizeof(t1));
				//t2 = 0xfffffff0 & t1;
				t2 = (~(uint32_t)s->bars[0].size + 1) & t1;
				memcpy(s->pcie_config + offset, &t2, sizeof(t2));
				break;
			case 0x18: // BAR 2
				memcpy(&t1, buf, sizeof(t1));
				//t2 = 0xfffffff0 & t1;
				t2 = (~(uint32_t)s->bars[2].size + 1) & t1;
				memcpy(s->pcie_config + offset, &t2, sizeof(t2));
				break;
			case 0x20: // BAR 4
				memcpy(&t1, buf, sizeof(t1));
				//t2 = 0xfffffff0 & t1;
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

static ssize_t qepc_pci_cfg_access(vfu_ctx_t *vfu_ctx, char *const buf,
                                   size_t count, loff_t offset,
                                   const bool is_write) {
    QEPCState *s = vfu_get_private(vfu_ctx);

    qemu_epc_debug("%s: %s: offset 0x%lx, size 0x%lx", __func__, is_write ? "write" : "read",
                   offset, count);

    if (offset + count > PCIE_CONFIG_SPACE_SIZE) {
        return 0;
    }
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

  s->vfu = vfu_create_ctx(VFU_TRANS_SOCK, s->sock_path,
                          LIBVFIO_USER_FLAG_ATTACH_NB, s, VFU_DEV_TYPE_PCI);
  if (!s->vfu) {
    return NULL;
  }

  err = vfu_pci_init(s->vfu, VFU_PCI_TYPE_EXPRESS, PCI_HEADER_TYPE_NORMAL, 0);
  if (err) {
    return NULL;
  }

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

static void qepc_vfu_run(void *opaque)
{
    QEPCState *s = opaque;
    int err = -1;

    qemu_epc_debug("starting vfu main loop");

    while(err != 0) {
        err = vfu_run_ctx(s->vfu);
        if (err < 0) {
            if (errno == EINTR) {
                continue;
            } else if (errno == ENOTCONN){
                break;
            } else {
                break;
            }
        }
    }
}

static void qepc_vfu_attach_ctx(void *opaque)
{
    QEPCState *s = opaque;
    int err;

    qemu_epc_debug("attach vfu client");
    qemu_set_fd_handler(s->vfu_fd, NULL, NULL, NULL);

retry:
    err = vfu_attach_ctx(s->vfu);
    if (err < 0) {
        if (err == EAGAIN || errno == EWOULDBLOCK) {
            goto retry;
        }
        return;
    }

    s->vfu_fd = vfu_get_poll_fd(s->vfu);
    if (s->vfu_fd < 0) {
        return;
    }

    qemu_set_fd_handler(s->vfu_fd, qepc_vfu_run, NULL, s);
}

static void qepc_vfu_log(vfu_ctx_t *vfu_ctx, int level, const char *msg) {
    qemu_epc_debug("vfu: %d: %s", level, msg);
}


static ssize_t qepc_bar_access(vfu_ctx_t *vfu_ctx, uint8_t barno,
		char *const buf, size_t count, loff_t offset, const bool is_write) {

    QEPCState *s = vfu_get_private(vfu_ctx);
	dma_addr_t addr;

	qemu_epc_debug("%s for bar %d:", __func__, barno);

	addr = s->bars[barno].phys_addr + offset;

	if (is_write)
		pci_dma_write(&s->dev, addr, buf, count);
	else
		pci_dma_read(&s->dev, addr, buf, count);

	return count;
}

#define QEPC_ACCESS_BAR(barno) \
	static ssize_t qepc_bar_access_ ## barno (vfu_ctx_t *vfu_ctx, char *const buf, \
                                   size_t count, loff_t offset, \
                                   const bool is_write) \
	{ return qepc_bar_access(vfu_ctx, barno, buf, count, offset, is_write); }

QEPC_ACCESS_BAR(0)
QEPC_ACCESS_BAR(1)
QEPC_ACCESS_BAR(2)
QEPC_ACCESS_BAR(3)
QEPC_ACCESS_BAR(4)
QEPC_ACCESS_BAR(5)

typedef ssize_t (*qepc_bar_access_handler_t)(vfu_ctx_t *vfu_ctx,
			char *const buf, size_t count, loff_t offset, const bool is_write);

static qepc_bar_access_handler_t qepc_bar_handlers[] = {
	qepc_bar_access_0,
	qepc_bar_access_1,
	qepc_bar_access_2,
	qepc_bar_access_3,
	qepc_bar_access_4,
	qepc_bar_access_5
};

static void qepc_dma_register(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info)
{
    QEPCState *s = vfu_get_private(vfu_ctx);

	qemu_epc_debug("%s: register 0x%lx, 0x%lx", __func__,
			(uint64_t)info->iova.iov_base, info->iova.iov_len);

	if (!info->vaddr) {
		qemu_epc_debug("unsupported");
		return;
	}

	qemu_epc_debug("%s: vaddr 0x%p", __func__, info->vaddr);

	if (s->rc_mr_idx == SUPPORT_RC_NUM_MRS) {
		qemu_epc_debug("unsupported");
		return;
	}

	s->rc_mrs[s->rc_mr_idx] =  (struct rc_mr){
		.rc_phys = (uint64_t)info->iova.iov_base,
		.size = info->iova.iov_len,
		.vaddr = info->vaddr,
	};
	s->rc_mr_idx++;
}

static void qepc_dma_unregister(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info)
{
    //QEPCState *s = vfu_get_private(vfu_ctx);
	qemu_epc_debug("%s: unregister 0x%lx, 0x%lx", __func__,
			(uint64_t)info->iova.iov_base, info->iova.iov_len);
}

static int qepc_ctrl_handle_start(QEPCState *s, uint64_t val) {
  int err;

  s->vfu = vfu_create_ctx(VFU_TRANS_SOCK, s->sock_path,
                          LIBVFIO_USER_FLAG_ATTACH_NB, s, VFU_DEV_TYPE_PCI);
  if (!s->vfu) {
    qemu_epc_debug("failed at vfu_create_ctx");
    return -1;
  }

  vfu_setup_log(s->vfu, qepc_vfu_log, LOG_DEBUG);

  err = vfu_pci_init(s->vfu, VFU_PCI_TYPE_EXPRESS, PCI_HEADER_TYPE_NORMAL, 0);
  if (err) {
    qemu_epc_debug("failed at vfu_pci_init");
    return -1;
  }

  err = vfu_setup_region(s->vfu, VFU_PCI_DEV_CFG_REGION_IDX,
                         PCIE_CONFIG_SPACE_SIZE, &qepc_pci_cfg_access,
                         VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB, NULL,
                         0, -1, 0);
  if (err) {
    qemu_epc_debug("failed at vfu_setup_region");
    return -1;
  }

  // setup bars
  for(int i=0; i < PCI_NUM_REGIONS; i++) {

	if (!(s->bar_mask & (1 << i)))
		continue;

	qemu_epc_debug("setup bar %d: size 0x%lx", i, s->bars[i].size);

	err = vfu_setup_region(s->vfu, VFU_PCI_DEV_BAR0_REGION_IDX + i,
			s->bars[i].size, qepc_bar_handlers[i],
			VFU_REGION_FLAG_RW | VFU_REGION_FLAG_ALWAYS_CB, NULL,
                         0, -1, 0);
	if (err) {
		qemu_epc_debug("failed at vfu_setup_region for bar");
		return -1;
	}
  }


  // setup irqs
  err = vfu_setup_device_nr_irqs(s->vfu, VFU_DEV_INTX_IRQ, 1);
  if (err < 0) {
	  qemu_epc_debug("failed to setup irq");
	  return err;
  }


  // setup for dma
  err = vfu_setup_device_dma(s->vfu, qepc_dma_register, qepc_dma_unregister);
  if (err) {
	  qemu_epc_debug("failed to setup dma");
	  return -1;
  }

  err = vfu_realize_ctx(s->vfu);
  if (err) {
    qemu_epc_debug("failed at vfu_realize_ctx");
    return -1;
  }

  s->vfu_fd = vfu_get_poll_fd(s->vfu);
  if (s->vfu_fd < 0) {
       qemu_epc_debug("failed at vfu_get_poll_fd");
        return -1;
  }

  qemu_epc_debug("listening vfu connection from %s", s->sock_path);
  qemu_set_fd_handler(s->vfu_fd, qepc_vfu_attach_ctx, NULL, s);

  return 0;
}

static void qepc_handle_ctrl_irq(QEPCState *s, int irq_num)
{
	vfu_irq_trigger(s->vfu, 0);
}

static void qepc_handle_enable_disale_ob(QEPCState *s, uint64_t val)
{
	uint8_t prev = s->ob_mask;

	for(int i=0; i<NUM_OB_WINDOW; i++) {
		uint8_t bit = (1 << i);

		if ((prev & bit) == (val & bit))
			continue;

		if (prev & bit) {
			qemu_epc_debug("disable ob is not supported yet");
		} else {
			qemu_epc_debug("enable ob");
		}

		for(int j=0; j < SUPPORT_RC_NUM_MRS; j++) {

			if (s->rc_mrs[j].rc_phys <= s->obs[i].pci
					&& (s->obs[i].pci + s->obs[i].size) <=
					s->rc_mrs[j].rc_phys + s->rc_mrs[j].size) {

				uint64_t off = s->obs[i].pci - s->rc_mrs[j].rc_phys;

				void *start = s->rc_mrs[j].vaddr + off;
				uint64_t size = s->obs[i].size;

				//sprintf(s->rc_local_mr_name, "qemu-epc/rc-local-%d", 0);
				memory_region_init_ram_ptr(&s->rc_local_mr[0], OBJECT(s),
						"qemu-epc/rc-local-0", size, start);

				AddressSpace *as = pci_device_iommu_address_space(&s->dev);

				memory_region_add_subregion(as->root, s->ob_window_mr.addr,
						&s->rc_local_mr[0]);
				goto done;
			}
		}
	}
done:
}

static void qepc_ctrl_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size) {
  QEPCState *s = opaque;
  uint64_t tmp;

  qemu_epc_debug("CTRL write: addr 0x%lx, size 0x%x", addr, size);

  switch (addr) {
  case QEPC_CTRL_OFF_START:
    qepc_ctrl_handle_start(s, val);
    return;
  case QEPC_CTRL_OFF_IRQ_TYPE:
    s->irq_type = val;
    break;
  case QEPC_CTRL_OFF_IRQ_NUM:
    qepc_handle_ctrl_irq(s, val);
    break;
  case QEPC_CTRL_OFF_OB_MASK:
	qepc_handle_enable_disale_ob(s, val);
	break;
  case QEPC_CTRL_OFF_OB_IDX:
    s->ob_idx = val;
    break;
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
  case QEPC_CTRL_OFF_OB_PCI:
    tmp = s->obs[s->ob_idx].pci;
    tmp = (tmp & ~0xffffffff) | (val & 0xffffffff);
    s->obs[s->ob_idx].pci = tmp;
    break;
  case QEPC_CTRL_OFF_OB_PCI + sizeof(uint32_t):
    tmp = s->obs[s->ob_idx].pci;
    tmp = (tmp & 0xffffffff) | (val << 32);
    s->obs[s->ob_idx].pci = tmp;
    qemu_epc_debug("ob map pci: %d: 0x%lx", s->ob_idx, tmp);
    break;
  case QEPC_CTRL_OFF_OB_SIZE:
    tmp = s->obs[s->ob_idx].size;
    tmp = (tmp & ~0xffffffff) | (val & 0xffffffff);
    s->obs[s->ob_idx].size = tmp;
    break;
  case QEPC_CTRL_OFF_OB_SIZE + sizeof(uint32_t):
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

static uint64_t qepc_cfg_bar_read(void *opaque, hwaddr addr, unsigned size) {
	QEPCState *s = opaque;

	switch(addr) {
		case QEPC_BAR_CFG_OFF_MASK:
			return s->bar_mask;
		default:
			break;
	}

	return 0;
}

static void qepc_cfg_bar_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size) {
  QEPCState *s = opaque;
  void *ptr;
  uint64_t tmp;

  if (addr + size > QEPC_BAR_CFG_OFF_SIZE + 8)
	  qemu_epc_debug("%s: overrun %ld", __func__, addr + size);

  switch(addr) {
	case QEPC_BAR_CFG_OFF_MASK:
		s->bar_mask = val;
		break;
	case QEPC_BAR_CFG_OFF_NUMBER:
		s->bar_no = val;
		break;
	case QEPC_BAR_CFG_OFF_FLAGS:
		s->bars[s->bar_no].flags = val;
		ptr = s->pcie_config + 0x10 + 4 * s->bar_no;
		memcpy(&tmp, ptr, sizeof(uint64_t));
		tmp = (val & 0xf) | (tmp & 0xfffffff0) | 0x4;
		memcpy(ptr, &tmp, sizeof(uint64_t));
		qemu_epc_debug("%s: bar[%d] 0x%lx (flags %lx)", __func__, s->bar_no, tmp, val);
		break;
	case QEPC_BAR_CFG_OFF_RSV:
		break;
	case QEPC_BAR_CFG_OFF_PHYS_ADDR:
		s->bars[s->bar_no].phys_addr = val;
		ptr = s->pcie_config + 0x10 + 4 * s->bar_no;
		memcpy(&tmp, ptr, sizeof(uint64_t));
		tmp = (val & 0xfffffff0) | (tmp & 0xf);
		memcpy(ptr, &tmp, sizeof(uint64_t));
		qemu_epc_debug("%s: bar[%d] 0x%lx(addr %lx)", __func__, s->bar_no, tmp, val);
		break;
	case QEPC_BAR_CFG_OFF_SIZE:
		s->bars[s->bar_no].size = val;
		break;
  }
}

static const MemoryRegionOps qemu_epc_mmio_bar_cfg_ops = {
	.read = qepc_cfg_bar_read,
	.write = qepc_cfg_bar_write,
	.endianness = DEVICE_LITTLE_ENDIAN,
};


static void qepc_realize(PCIDevice *pci_dev, Error **errp) {
	QEPCState *s = QEMU_EPC(pci_dev);

	qemu_epc_debug("realize");

	if (!s->sock_path) {
	  error_setg(errp, "qemu-epc: sock_path should be set");
	  return;
	}

	memory_region_init_io(&s->ctrl_mr, OBJECT(s), &qepc_ctrl_mmio_ops, s,
			"qemu-epc/ctrl", pow2ceil(QEPC_CTRL_SIZE));

	memory_region_init_ram_ptr(&s->pci_cfg_mr, OBJECT(s),
			"qemu-epc/cfg-cfg", PCIE_CONFIG_SPACE_SIZE, s->pcie_config);

	memory_region_init_io(&s->bar_cfg_mr, OBJECT(s),
			&qemu_epc_mmio_bar_cfg_ops,
			s, "qemu-epc/bar-cfg",
			pow2ceil(QEPC_BAR_CFG_SIZE));

	memory_region_init(&s->ob_window_mr, NULL, "qemu-epc/ob",
			pow2ceil(NUM_OB_WINDOW * OB_WINDOW_SIZE));
	pci_register_bar(pci_dev, QEPC_BAR_CTRL, PCI_BASE_ADDRESS_MEM_TYPE_32,
			&s->ctrl_mr);
	pci_register_bar(pci_dev, QEPC_BAR_PCI_CFG,
			PCI_BASE_ADDRESS_SPACE_MEMORY, &s->pci_cfg_mr);
	pci_register_bar(pci_dev, QEPC_BAR_BAR_CFG,
			PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar_cfg_mr);

	pci_register_bar(pci_dev, QEPC_BAR_OB_WINDOWS, PCI_BASE_ADDRESS_MEM_TYPE_64,
			&s->ob_window_mr);
}

static void qepc_object_set_path (Object *obj, const char *str,
                                   Error **errp)
{
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

static void qepc_class_init(ObjectClass *klass, void *data) {
  DeviceClass *dc = DEVICE_CLASS(klass);
  PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

  qemu_epc_debug("initialize class");

  object_class_property_add_str(klass, "path", NULL,
                            qepc_object_set_path);

  k->realize = qepc_realize;
  // k->exit = qepc_exit;
  k->vendor_id = PCI_VENDOR_ID_REDHAT;
  k->device_id = PCI_DEVICE_ID_REDHAT_QEMU_EPC;
  k->revision = QEPC_REVISION;
  k->class_id = PCI_CLASS_OTHERS;

  dc->desc = "QEMU Endpoint Controller device";
  set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo qepc_info = {
    .name = TYPE_QEMU_EPC,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(QEPCState),
    .class_init = qepc_class_init,
    .interfaces = (InterfaceInfo[]){{INTERFACE_CONVENTIONAL_PCI_DEVICE}, {}},
};

static void qemu_epc_register_type(void) { type_register_static(&qepc_info); }

type_init(qemu_epc_register_type);
