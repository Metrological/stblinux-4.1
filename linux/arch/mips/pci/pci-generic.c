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

#include <linux/pci.h>

int __weak pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pci_sys_data *sysdata = dev->sysdata;
	struct pci_controller *ctl = sysdata_to_hose(sysdata);

	if (!ctl->map_irq)
		return -1;

	return ctl->map_irq(dev, slot, pin);
}

void pci_common_init_dev(struct device *parent, struct hw_pci *hw)
{
	struct pci_controller *ctl;
	struct resource_entry *window;
	struct resource **ctl_res;
	int i, ret, next_busnr = 0;

	for (i = 0; i < hw->nr_controllers; i++) {
		ctl = kzalloc(sizeof(*ctl), GFP_KERNEL);
		if (!ctl) {
			pr_err("%s: unable to allocate pci_controller\n",
			       __func__);
			continue;
		}

		ctl->pci_ops = hw->ops;
		ctl->align_resource = hw->align_resource;
		ctl->map_irq = hw->map_irq;

		INIT_LIST_HEAD(&ctl->sysdata.resources);

		if (hw->private_data)
			ctl->sysdata.private_data = hw->private_data[i];

		ctl->sysdata.busnr = next_busnr;
#ifdef CONFIG_PCI_MSI
		ctl->sysdata.msi_ctrl = hw->msi_ctrl;
#endif

		if (hw->preinit)
			hw->preinit();

		ret = hw->setup(i, &ctl->sysdata);
		if (ret <= 0) {
			pr_err("%s: unable to setup controller: %d\n",
			       __func__, ret);
			kfree(ctl);
			continue;
		}

		resource_list_for_each_entry(window, &ctl->sysdata.resources) {
			switch (resource_type(window->res)) {
			case IORESOURCE_IO:
				ctl_res = &ctl->io_resource;
				break;

			case IORESOURCE_MEM:
				ctl_res = &ctl->mem_resource;
				break;

			default:
				ctl_res = NULL;
			}

			if (!ctl_res)
				continue;

			if (*ctl_res) {
				pr_warn("%s: multiple resources of type 0x%lx\n",
					__func__, resource_type(window->res));
				continue;
			}

			*ctl_res = window->res;
		}

		if (hw->scan)
			ctl->bus = hw->scan(i, &ctl->sysdata);
		else
			ctl->bus = pci_scan_root_bus(parent,
					ctl->sysdata.busnr,
					hw->ops, &ctl->sysdata,
					&ctl->sysdata.resources);

		add_pci_controller(ctl);

		if (hw->postinit)
			hw->postinit();

		pci_fixup_irqs(pci_common_swizzle, pcibios_map_irq);

		if (ctl->bus) {
			if (!pci_has_flag(PCI_PROBE_ONLY)) {
				pci_bus_size_bridges(ctl->bus);
				pci_bus_assign_resources(ctl->bus);
			}

			pci_bus_add_devices(ctl->bus);
			next_busnr = ctl->bus->busn_res.end + 1;
		}
	}
}

void pcibios_fixup_bus(struct pci_bus *bus)
{
	pci_read_bridge_bases(bus);
}

int pcibios_enable_device(struct pci_dev *dev, int mask)
{
	if (pci_has_flag(PCI_PROBE_ONLY))
		return 0;

	return pci_enable_resources(dev, mask);
}

#ifdef CONFIG_PCI_MSI
struct msi_controller *pcibios_msi_controller(struct pci_dev *dev)
{
	struct pci_sys_data *sysdata = dev->sysdata;

	return sysdata->msi_ctrl;
}
#endif
