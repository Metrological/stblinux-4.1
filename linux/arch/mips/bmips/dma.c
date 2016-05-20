/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2014 Kevin Cernekee <cernekee@gmail.com>
 */

#define pr_fmt(fmt)		"bmips-dma: " fmt

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <dma-coherence.h>

/*
 * BCM338x has configurable address translation windows which allow the
 * peripherals' DMA addresses to be different from the Zephyr-visible
 * physical addresses.  e.g. usb_dma_addr = zephyr_pa ^ 0x08000000
 *
 * If the "brcm,ubus" node has a "dma-ranges" property we will enable this
 * translation globally using the provided information.  This implements a
 * very limited subset of "dma-ranges" support and it will probably be
 * replaced by a more generic version later.
 */

struct bmips_dma_range {
	u32			child_addr;
	u32			parent_addr;
	u32			size;
};

struct bmips_dma_ops {
	dma_addr_t (*phys_to_dma)(struct device *dev, phys_addr_t pa);
	unsigned long (*addr_to_phys)(struct device *dev, dma_addr_t dma);
};

static inline dma_addr_t bmips_noop_phys_to_dma(struct device *dev,
						phys_addr_t pa)
{
	return pa;
}

static inline unsigned long bmips_noop_addr_to_phys(struct device *dev,
						    dma_addr_t dma_addr)
{
	return dma_addr;
}

static struct bmips_dma_range *bmips_dma_ranges;

static struct bmips_dma_ops bmips_dma_ops = {
	.phys_to_dma = bmips_noop_phys_to_dma,
	.addr_to_phys = bmips_noop_addr_to_phys,
};

#define FLUSH_RAC		0x100

static dma_addr_t bmips_ubus_phys_to_dma(struct device *dev, phys_addr_t pa)
{
	struct bmips_dma_range *r;

	for (r = bmips_dma_ranges; r && r->size; r++) {
		if (pa >= r->child_addr &&
		    pa < (r->child_addr + r->size))
			return pa - r->child_addr + r->parent_addr;
	}
	return pa;
}

static unsigned long bmips_ubus_dma_addr_to_phys(struct device *dev,
						 dma_addr_t dma_addr)
{
	struct bmips_dma_range *r;

	for (r = bmips_dma_ranges; r && r->size; r++) {
		if (dma_addr >= r->parent_addr &&
		    dma_addr < (r->parent_addr + r->size))
			return dma_addr - r->parent_addr + r->child_addr;
	}
	return dma_addr;
}

#ifdef CONFIG_PCI
/* These two offsets depend on the chip we are running on, namely
 * if the processor supports 1GB or 2GB on MEMC0
 */
static unsigned long memc1_start;
static unsigned long memc1_pci_offset;

/* This is a constant on all Broadcom STB chips, spans the GISB register
 * and EBI space
 */
#define BRCM_PCI_HOLE_START	_AC(0x10000000, UL)
#define BRCM_PCI_HOLE_SIZE	_AC(0x10000000, UL)

/*
 * PCIe inbound BARs collapse the "holes" in the chip's SCB address space.
 * Therefore the PCI addresses need to be adjusted as they will not match
 * the SCB addresses (MIPS physical addresses).
 */
static int dev_collapses_memory_hole(struct device *dev)
{
#if defined(CONFIG_HIGHMEM)
	if (unlikely(dev == NULL) ||
	    likely(dev->bus != &pci_bus_type))
		return 0;

	return 1;
#else
	return 0;
#endif
}

static dma_addr_t bmips_pci_phys_to_dma(struct device *dev, phys_addr_t pa)
{
	if (!dev_collapses_memory_hole(dev))
		return pa;
	if (pa >= memc1_start)
		return pa - memc1_pci_offset;
	if (pa >= (BRCM_PCI_HOLE_START + BRCM_PCI_HOLE_SIZE))
		return pa - BRCM_PCI_HOLE_SIZE;
	return pa;
}

static unsigned long bmips_pci_dma_addr_to_phys(struct device *dev,
						dma_addr_t dma_addr)
{
	if (!dev_collapses_memory_hole(dev))
		return dma_addr;
	if (dma_addr >= (memc1_start - memc1_pci_offset))
		return dma_addr + memc1_pci_offset;
	if (dma_addr >= BRCM_PCI_HOLE_START)
		return dma_addr + BRCM_PCI_HOLE_SIZE;
	return dma_addr;
}
#endif /* CONFIG_PCI */

static dma_addr_t bmips_phys_to_dma(struct device *dev, phys_addr_t pa)
{
	return bmips_dma_ops.phys_to_dma(dev, pa);
}

dma_addr_t plat_map_dma_mem(struct device *dev, void *addr, size_t size)
{
	return bmips_phys_to_dma(dev, virt_to_phys(addr));
}

dma_addr_t plat_map_dma_mem_page(struct device *dev, struct page *page)
{
	return bmips_phys_to_dma(dev, page_to_phys(page));
}

unsigned long plat_dma_addr_to_phys(struct device *dev, dma_addr_t dma_addr)
{
	return bmips_dma_ops.addr_to_phys(dev, dma_addr);
}

static int __init bmips_init_dma_ranges(void)
{
	struct device_node *np =
		of_find_compatible_node(NULL, NULL, "brcm,ubus");
	const __be32 *data;
	struct bmips_dma_range *r;
	int len;

	if (!np)
		return 0;

	data = of_get_property(np, "dma-ranges", &len);
	if (!data)
		goto out_good;

	len /= sizeof(*data) * 3;
	if (!len)
		goto out_bad;

	/* add a dummy (zero) entry at the end as a sentinel */
	bmips_dma_ranges = kzalloc(sizeof(struct bmips_dma_range) * (len + 1),
				   GFP_KERNEL);
	if (!bmips_dma_ranges)
		goto out_bad;

	for (r = bmips_dma_ranges; len; len--, r++) {
		r->child_addr = be32_to_cpup(data++);
		r->parent_addr = be32_to_cpup(data++);
		r->size = be32_to_cpup(data++);
	}

	bmips_dma_ops.phys_to_dma = bmips_ubus_phys_to_dma;
	bmips_dma_ops.addr_to_phys = bmips_ubus_dma_addr_to_phys;

out_good:
	of_node_put(np);
	return 0;

out_bad:
	pr_err("error parsing dma-ranges property\n");
	of_node_put(np);
	return -EINVAL;
}
arch_initcall(bmips_init_dma_ranges);

#ifdef CONFIG_PCI
static int __init bmips_init_pci_ranges(void)
{
	struct device_node *np = of_find_node_by_name(NULL, "rdb");
	struct cpuinfo_mips *c = &current_cpu_data;
	int rev = 0;

	/* Likely not running on a STB chip, bail out */
	if (!np || c->cputype != CPU_BMIPS5000)
		return 0;

	rev = c->processor_id & PRID_REV_MASK;

	/* Early BMIPS5000, only supports up to 1GB on MEMC0 */
	if (rev <= 0x2 &&
	   (c->processor_id & PRID_IMP_MASK) == PRID_IMP_BMIPS5000) {
		memc1_start = _AC(0x60000000, UL);
		memc1_pci_offset = _AC(0x20000000, UL);
	} else {
		memc1_start = _AC(0x90000000, UL);
		memc1_pci_offset = _AC(0x10000000, UL);
	}

	pr_info("MEMC1 start: 0x%lx, PCI offset: 0x%lx\n",
		memc1_start, memc1_pci_offset);

	bmips_dma_ops.phys_to_dma = bmips_pci_phys_to_dma;
	bmips_dma_ops.addr_to_phys = bmips_pci_dma_addr_to_phys;

	return 0;
}
arch_initcall(bmips_init_pci_ranges);
#endif
