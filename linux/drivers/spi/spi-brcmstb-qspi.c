/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include "spi-bcm-qspi.h"

/* HIF INTR2 offsets */
#define HIF_SPI_INTR2_CPU_STATUS               0x00
#define HIF_SPI_INTR2_CPU_SET                  0x04
#define HIF_SPI_INTR2_CPU_CLEAR                0x08
#define HIF_SPI_INTR2_CPU_MASK_STATUS          0x0c
#define HIF_SPI_INTR2_CPU_MASK_SET             0x10
#define HIF_SPI_INTR2_CPU_MASK_CLEAR           0x14

struct brcmstb_qspi_intc {
	struct bcm_qspi_soc_intc soc_intc;
	struct platform_device *pdev;
	void __iomem *hif_intr2_reg;
	spinlock_t soclock;
	bool big_endian;
};

static const struct of_device_id brcmstb_qspi_of_match[] = {
	{ .compatible = "brcm,spi-brcmstb" },
	{ .compatible = "brcm,spi-brcmstb-qspi" },
#ifdef CONFIG_SPI_BRCMSTB_MSPI
	{ .compatible = "brcm,spi-brcmstb-mspi" },
#endif
	{},
};
MODULE_DEVICE_TABLE(of, brcmstb_qspi_of_match);

static u32 brcmstb_qspi_qspi_get_l2_int_status(
	struct bcm_qspi_soc_intc *soc_intc)
{
	struct brcmstb_qspi_intc *priv = container_of(soc_intc,
						      struct brcmstb_qspi_intc,
						      soc_intc);
	void __iomem *mmio = priv->hif_intr2_reg;
	u32 val = 0, sts = 0;

	val = bcm_qspi_readl(priv->big_endian,
			     (mmio + HIF_SPI_INTR2_CPU_STATUS));

	if (val & INTR_MSPI_DONE_MASK)
		sts |= MSPI_DONE;

	if (val & BSPI_LR_INTERRUPTS_ALL)
		sts |= BSPI_DONE;

	if (val & BSPI_LR_INTERRUPTS_ERROR)
		sts |= BSPI_ERR;

	return sts;
}

static void brcmstb_qspi_qspi_int_ack(struct bcm_qspi_soc_intc *soc_intc,
				      int type)
{
	struct brcmstb_qspi_intc *priv = container_of(soc_intc,
						      struct brcmstb_qspi_intc,
						      soc_intc);
	void __iomem *mmio = priv->hif_intr2_reg;
	u32 mask = get_qspi_mask(type);

	bcm_qspi_writel(priv->big_endian, mask,
			(mmio + HIF_SPI_INTR2_CPU_CLEAR));
}

static void brcmstb_qspi_qspi_int_set(struct bcm_qspi_soc_intc *soc_intc,
				      int type,
				      bool en)
{
	struct brcmstb_qspi_intc *priv = container_of(soc_intc,
						      struct brcmstb_qspi_intc,
						      soc_intc);
	void __iomem *mmio = priv->hif_intr2_reg;
	u32 mask = get_qspi_mask(type);
	unsigned long flags;

	spin_lock_irqsave(&priv->soclock, flags);

	if (en)
		bcm_qspi_writel(priv->big_endian, mask,
				(mmio + HIF_SPI_INTR2_CPU_MASK_CLEAR));
	else
		bcm_qspi_writel(priv->big_endian, mask,
				(mmio + HIF_SPI_INTR2_CPU_MASK_SET));

	spin_unlock_irqrestore(&priv->soclock, flags);
}

static int brcmstb_qspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct brcmstb_qspi_intc *priv;
	struct bcm_qspi_soc_intc *soc_intc = NULL;
	struct resource *res;
	int use_soc = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->pdev = pdev;

	spin_lock_init(&priv->soclock);

	use_soc = of_device_is_compatible(dev->of_node, "brcm,spi-brcmstb");
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "hif_spi_intr2");
	if (use_soc && res) {
		priv->hif_intr2_reg = devm_ioremap_resource(dev, res);
		if (IS_ERR(priv->hif_intr2_reg))
			return PTR_ERR(priv->hif_intr2_reg);

		priv->big_endian = of_device_is_big_endian(dev->of_node);
		soc_intc = &priv->soc_intc;
		brcmstb_qspi_qspi_int_set(soc_intc, MSPI_BSPI_DONE, false);
		brcmstb_qspi_qspi_int_ack(soc_intc, MSPI_BSPI_DONE);

		soc_intc->bcm_qspi_int_ack = brcmstb_qspi_qspi_int_ack;
		soc_intc->bcm_qspi_int_set = brcmstb_qspi_qspi_int_set;
		soc_intc->bcm_qspi_get_int_status =
			brcmstb_qspi_qspi_get_l2_int_status;
	}

	return bcm_qspi_probe(pdev, soc_intc);
}

static int brcmstb_qspi_remove(struct platform_device *pdev)
{
	return bcm_qspi_remove(pdev);
}

static struct platform_driver brcmstb_qspi_driver = {
	.probe			= brcmstb_qspi_probe,
	.remove			= brcmstb_qspi_remove,
	.driver = {
		.name		= "brcmstb_qspi",
		.pm		= &bcm_qspi_pm_ops,
		.of_match_table = brcmstb_qspi_of_match,
	}
};
module_platform_driver(brcmstb_qspi_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kamal Dasu");
MODULE_DESCRIPTION("Broadcom SPI driver for settop SoC");
