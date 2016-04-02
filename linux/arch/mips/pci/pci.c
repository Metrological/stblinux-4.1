/*
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Copyright (C) 2003, 04, 11 Ralf Baechle (ralf@linux-mips.org)
 * Copyright (C) 2011 Wind River Systems,
 *   written by Ralf Baechle (ralf@linux-mips.org)
 */
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/of_address.h>

#include <asm/cpu-info.h>

/*
 * If PCI_PROBE_ONLY in pci_flags is set, we don't change any PCI resource
 * assignments.
 */

/*
 * The PCI controller list.
 */

struct pci_controller *hose_head;
static struct pci_controller **hose_tail = &hose_head;

unsigned long PCIBIOS_MIN_IO;
unsigned long PCIBIOS_MIN_MEM;

/*
 * We need to avoid collisions with `mirrored' VGA ports
 * and other strange ISA hardware, so we always want the
 * addresses to be allocated in the 0x000-0x0ff region
 * modulo 0x400.
 *
 * Why? Because some silly external IO cards only decode
 * the low 10 bits of the IO address. The 0x00-0xff region
 * is reserved for motherboard devices that decode all 16
 * bits, so it's ok to allocate at, say, 0x2800-0x28ff,
 * but we want to try to avoid allocating at 0x2900-0x2bff
 * which might have be mirrored at 0x0100-0x03ff..
 */
resource_size_t
pcibios_align_resource(void *data, const struct resource *res,
		       resource_size_t size, resource_size_t align)
{
	struct pci_dev *dev = data;
	struct pci_controller *hose = sysdata_to_hose(dev->sysdata);
	resource_size_t start = res->start;

	if (res->flags & IORESOURCE_IO) {
		/* Make sure we start at our min on all hoses */
		if (start < PCIBIOS_MIN_IO + hose->io_resource->start)
			start = PCIBIOS_MIN_IO + hose->io_resource->start;

		/*
		 * Put everything into 0x00-0xff region modulo 0x400
		 */
		if (start & 0x300)
			start = (start + 0x3ff) & ~0x3ff;
	} else if (res->flags & IORESOURCE_MEM) {
		/* Make sure we start at our min on all hoses */
		if (start < PCIBIOS_MIN_MEM + hose->mem_resource->start)
			start = PCIBIOS_MIN_MEM + hose->mem_resource->start;
	}

	if (hose->align_resource)
		return hose->align_resource(dev, res, start, size, align);

	return start;
}

void add_pci_controller(struct pci_controller *hose)
{
	*hose_tail = hose;
	hose_tail = &hose->next;
}

static int __init pcibios_set_cache_line_size(void)
{
	struct cpuinfo_mips *c = &current_cpu_data;
	unsigned int lsize;

	/*
	 * Set PCI cacheline size to that of the highest level in the
	 * cache hierarchy.
	 */
	lsize = c->dcache.linesz;
	lsize = c->scache.linesz ? : lsize;
	lsize = c->tcache.linesz ? : lsize;

	BUG_ON(!lsize);

	pci_dfl_cache_line_size = lsize >> 2;

	pr_debug("PCI: pci_cache_line_size set to %d bytes\n", lsize);
	return 0;
}
arch_initcall(pcibios_set_cache_line_size);

unsigned int pcibios_assign_all_busses(void)
{
	return 1;
}

EXPORT_SYMBOL(PCIBIOS_MIN_IO);
EXPORT_SYMBOL(PCIBIOS_MIN_MEM);

int pci_mmap_page_range(struct pci_dev *dev, struct vm_area_struct *vma,
			enum pci_mmap_state mmap_state, int write_combine)
{
	unsigned long prot;

	/*
	 * I/O space can be accessed via normal processor loads and stores on
	 * this platform but for now we elect not to do this and portable
	 * drivers should not do this anyway.
	 */
	if (mmap_state == pci_mmap_io)
		return -EINVAL;

	/*
	 * Ignore write-combine; for now only return uncached mappings.
	 */
	prot = pgprot_val(vma->vm_page_prot);
	prot = (prot & ~_CACHE_MASK) | _CACHE_UNCACHED;
	vma->vm_page_prot = __pgprot(prot);

	return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

char * (*pcibios_plat_setup)(char *str) __initdata;

char *__init pcibios_setup(char *str)
{
	if (pcibios_plat_setup)
		return pcibios_plat_setup(str);
	return str;
}
