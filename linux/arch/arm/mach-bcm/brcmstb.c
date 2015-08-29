/*
 * Copyright (C) 2013-2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of_platform.h>

#if defined(CONFIG_BRCMSTB)
#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/bmem.h>
#include <linux/brcmstb/cma_driver.h>
#include <linux/clk/clk-brcmstb.h>
#include <linux/clocksource.h>
#include <linux/console.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <asm/mach/map.h>
#endif
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

static void __init brcmstb_init_irq(void)
{
	gic_set_irqchip_flags(IRQCHIP_MASK_ON_SUSPEND);
	irqchip_init();
}

static const char *const brcmstb_match[] __initconst = {
	"brcm,bcm7445",
	"brcm,brcmstb",
	NULL
};

#if defined(CONFIG_BRCMSTB)
/*
 * HACK: The following drivers are still using BDEV macros:
 * - XPT DMA
 * - SPI
 * - NAND
 * - SDHCI
 * - MoCA
 *
 * Once these drivers have migrated over to using 'of_iomap()' and standard
 * register accessors, we can eliminate this static mapping.
 */
static struct map_desc brcmstb_io_map[] __initdata = {
	{
	.virtual = (unsigned long)BRCMSTB_PERIPH_VIRT,
	.pfn     = __phys_to_pfn(BRCMSTB_PERIPH_PHYS),
	.length  = BRCMSTB_PERIPH_LENGTH,
	.type    = MT_DEVICE,
	},
};

static void __init brcmstb_map_io(void)
{
	iotable_init(brcmstb_io_map, ARRAY_SIZE(brcmstb_io_map));
}

static void __init brcmstb_reserve(void)
{
	brcmstb_memory_reserve();
	cma_reserve();
	bmem_reserve();
}

#define CPU_CREDIT_REG_OFFSET 0x184
#define  CPU_CREDIT_REG_MCPx_WR_PAIRING_EN_MASK 0x70000000

static void __iomem *cpubiuctrl_base;
static unsigned int mcp_wr_pairing_en;

/*
 * HW7445-1920: On affected chips, disable MCP write pairing to improve
 * stability on long term stress test.
 */
static int __init mcp_write_pairing_set(void)
{
	u32 creds = 0;

	if (!cpubiuctrl_base)
		return -1;

	creds = __raw_readl(cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
	if (mcp_wr_pairing_en) {
		pr_info("MCP: Enabling write pairing\n");
		__raw_writel(creds | CPU_CREDIT_REG_MCPx_WR_PAIRING_EN_MASK,
			     cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
	} else if (creds & CPU_CREDIT_REG_MCPx_WR_PAIRING_EN_MASK) {
		pr_info("MCP: Disabling write pairing\n");
		__raw_writel(creds & ~CPU_CREDIT_REG_MCPx_WR_PAIRING_EN_MASK,
				cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
	} else {
		pr_info("MCP: Write pairing already disabled\n");
	}

	return 0;
}

static void __init setup_hifcpubiuctrl_regs(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "brcm,brcmstb-cpu-biu-ctrl");
	if (!np)
		pr_err("missing BIU control node\n");

	cpubiuctrl_base = of_iomap(np, 0);
	if (!cpubiuctrl_base)
		pr_err("failed to remap BIU control base\n");

	mcp_wr_pairing_en = of_property_read_bool(np, "brcm,write-pairing");

	of_node_put(np);
}

#ifdef CONFIG_PM_SLEEP
static u32 cpu_credit_reg_dump;  /* for save/restore */

static int brcmstb_cpu_credit_reg_suspend(void)
{
	if (cpubiuctrl_base)
		cpu_credit_reg_dump =
			__raw_readl(cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
	return 0;
}

static void brcmstb_cpu_credit_reg_resume(void)
{
	if (cpubiuctrl_base)
		__raw_writel(cpu_credit_reg_dump,
				cpubiuctrl_base + CPU_CREDIT_REG_OFFSET);
}

static struct syscore_ops brcmstb_cpu_credit_syscore_ops = {
	.suspend = brcmstb_cpu_credit_reg_suspend,
	.resume = brcmstb_cpu_credit_reg_resume,
};
#endif

static void __init brcmstb_init_machine(void)
{
	struct platform_device_info devinfo = { .name = "cpufreq-dt", };

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register_full(&devinfo);
	cma_register();
#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&brcmstb_cpu_credit_syscore_ops);
#endif
}

static void __init brcmstb_init_early(void)
{
	setup_hifcpubiuctrl_regs();
	if (mcp_write_pairing_set())
		pr_err("MCP: Unable to disable write pairing!\n");
	add_preferred_console("ttyS", 0, "115200");
}

static void __init brcmstb_init_time(void)
{
	brcmstb_clocks_init();
	clocksource_of_init();
}
#endif

DT_MACHINE_START(BRCMSTB, "Broadcom STB (Flattened Device Tree)")
	.dt_compat	= brcmstb_match,
	.init_irq	= brcmstb_init_irq,
#if defined(CONFIG_BRCMSTB)
	.map_io		= brcmstb_map_io,
	.reserve	= brcmstb_reserve,
	.init_machine	= brcmstb_init_machine,
	.init_early	= brcmstb_init_early,
	.init_time	= brcmstb_init_time,
#endif
MACHINE_END
