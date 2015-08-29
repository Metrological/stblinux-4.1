/*
 * sdhci-brcmstb.c Support for SDHCI on Broadcom SoC's
 *
 * Copyright (C) 2013 Broadcom Corporation
 *
 * Author: Al Cooper <acooper@broadcom.com>
 * Based on sdhci-dove.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/brcmstb/brcmstb.h>

#include "sdhci-pltfm.h"

#define SDIO_CFG_REG(x, y)	(x + BCHP_SDIO_0_CFG_##y -	\
				BCHP_SDIO_0_CFG_REG_START)

#if defined(CONFIG_BCM7439A0) || defined(CONFIG_BCM74371A0)
/*
 * HW7445-1183
 * Setting the RESET_ALL or RESET_DATA bits will hang the SDIO
 * core so don't allow these bits to be set. This workaround
 * allows the driver to be used for development and testing
 * but will prevent recovery from normally recoverable errors
 * and should NOT be used in production systems.
 */
static void sdhci_brcmstb_writeb(struct sdhci_host *host, u8 val, int reg)
{
	if (reg == SDHCI_SOFTWARE_RESET)
		val &= ~(SDHCI_RESET_ALL | SDHCI_RESET_DATA);
	writeb(val, host->ioaddr + reg);
}

static struct sdhci_ops sdhci_brcmstb_ops = {
	.write_b	= sdhci_brcmstb_writeb,
};
#endif

static struct sdhci_pltfm_data sdhci_brcmstb_pdata = {
};

#if defined(CONFIG_BCM3390A0) || defined(CONFIG_BCM7145B0) ||		\
	defined(CONFIG_BCM7250B0) || defined(CONFIG_BCM7364A0) ||	\
	defined(CONFIG_BCM7439B0) || defined(CONFIG_BCM7445D0)
static int sdhci_override_caps(struct platform_device *pdev,
			uint32_t cap0_setbits,
			uint32_t cap0_clearbits,
			uint32_t cap1_setbits,
			uint32_t cap1_clearbits)
{
	uint32_t val;
	struct resource *iomem;
	uintptr_t cfg_base;
	struct sdhci_host *host = platform_get_drvdata(pdev);

	/*
	 * The CAP's override bits in the CFG registers default to all
	 * zeros so start by getting the correct settings from the HOST
	 * CAPS registers and then modify the requested bits and write
	 * them to the override CFG registers.
	 */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!iomem)
		return -EINVAL;
	cfg_base = iomem->start;
	val = sdhci_readl(host, SDHCI_CAPABILITIES);
	val &= ~cap0_clearbits;
	val |= cap0_setbits;
	BDEV_WR(SDIO_CFG_REG(cfg_base, CAP_REG0), val);
	val = sdhci_readl(host, SDHCI_CAPABILITIES_1);
	val &= ~cap1_clearbits;
	val |= cap1_setbits;
	BDEV_WR(SDIO_CFG_REG(cfg_base, CAP_REG1), val);
	BDEV_WR(SDIO_CFG_REG(cfg_base, CAP_REG_OVERRIDE),
		BCHP_SDIO_0_CFG_CAP_REG_OVERRIDE_CAP_REG_OVERRIDE_MASK);
	return 0;
}

static int sdhci_fix_caps(struct platform_device *pdev)
{
	/* Disable SDR50 support because tuning is broken. */
	return sdhci_override_caps(pdev, 0, 0, 0, SDHCI_SUPPORT_SDR50);
}
#else
static int sdhci_fix_caps(struct platform_device *pdev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP

static int sdhci_brcmstb_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int res;

	res = sdhci_suspend_host(host);
	if (res)
		return res;
	clk_disable(pltfm_host->clk);
	return res;
}

static int sdhci_brcmstb_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int err;

	err = clk_enable(pltfm_host->clk);
	if (err)
		return err;
	return sdhci_resume_host(host);
}

#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sdhci_brcmstb_pmops, sdhci_brcmstb_suspend,
			sdhci_brcmstb_resume);

static int sdhci_brcmstb_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct clk *clk;
	int res;

	clk = of_clk_get_by_name(dn, "sw_sdio");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Clock not found in Device Tree\n");
		clk = NULL;
	}
	res = clk_prepare_enable(clk);
	if (res)
		goto undo_clk_get;

/* Only enable reset workaround for 7439a0 and 74371a0 senior */
#if defined(CONFIG_BCM7439A0) || defined(CONFIG_BCM74371A0)
	if (BRCM_CHIP_ID() == 0x7439)
		sdhci_brcmstb_pdata.ops = &sdhci_brcmstb_ops;
#endif
	host = sdhci_pltfm_init(pdev, &sdhci_brcmstb_pdata, 0);
	if (IS_ERR(host)) {
		res = PTR_ERR(host);
		goto undo_clk_prep;
	}
	sdhci_get_of_property(pdev);
	mmc_of_parse(host->mmc);
	res = sdhci_fix_caps(pdev);
	if (res)
		goto undo_pltfm_init;

	res = sdhci_add_host(host);
	if (res)
		goto undo_pltfm_init;

	pltfm_host = sdhci_priv(host);
	pltfm_host->clk = clk;
	return res;

undo_pltfm_init:
	sdhci_pltfm_free(pdev);
undo_clk_prep:
	clk_disable_unprepare(clk);
undo_clk_get:
	clk_put(clk);
	return res;
}

static int sdhci_brcmstb_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int res;
	res = sdhci_pltfm_unregister(pdev);
	clk_disable_unprepare(pltfm_host->clk);
	clk_put(pltfm_host->clk);
	return res;
}


static const struct of_device_id sdhci_brcm_of_match[] = {
	{ .compatible = "brcm,sdhci-brcmstb" },
	{},
};

static struct platform_driver sdhci_brcmstb_driver = {
	.driver		= {
		.name	= "sdhci-brcmstb",
		.owner	= THIS_MODULE,
		.pm	= &sdhci_brcmstb_pmops,
		.of_match_table = of_match_ptr(sdhci_brcm_of_match),
	},
	.probe		= sdhci_brcmstb_probe,
	.remove		= sdhci_brcmstb_remove,
};

module_platform_driver(sdhci_brcmstb_driver);

MODULE_DESCRIPTION("SDHCI driver for Broadcom");
MODULE_AUTHOR("Al Cooper <acooper@broadcom.com>");
MODULE_LICENSE("GPL v2");
