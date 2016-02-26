/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 * Copyright (C) 2014 Kevin Cernekee <cernekee@gmail.com>
 */

#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/bootmem.h>
#include <linux/clk-provider.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/smp.h>
#include <linux/libfdt.h>
#include <asm/addrspace.h>
#include <asm/bmips.h>
#include <asm/bootinfo.h>
#include <asm/cpu-type.h>
#include <asm/mipsregs.h>
#include <asm/prom.h>
#include <asm/smp-ops.h>
#include <asm/time.h>
#include <asm/traps.h>
#include <asm/fw/cfe/cfe_api.h>
#include <asm/fw/cfe/cfe_error.h>

#define RELO_NORMAL_VEC		BIT(18)

#define REG_BCM6328_OTP		((void __iomem *)CKSEG1ADDR(0x1000062c))
#define BCM6328_TP1_DISABLED	BIT(9)

/* define default holes, there is no BMIPS_MAP2_SZ because that
 * map takes everything else that was not taken by MAP[01]
 */
#define BMIPS_MAP0_OFF 0x00000000
#define BMIPS_MAP0_SZ  0x10000000
#define BMIPS_MAP1_OFF 0x20000000
#define BMIPS_MAP1_SZ  0x30000000
#define BMIPS_MAP2_OFF 0x90000000

static const unsigned long kbase = VMLINUX_LOAD_ADDRESS & 0xfff00000;

struct bmips_quirk {
	const char		*compatible;
	void			(*quirk_fn)(void);
};

static void kbase_setup(void)
{
	__raw_writel(kbase | RELO_NORMAL_VEC,
		     BMIPS_GET_CBR() + BMIPS_RELO_VECTOR_CONTROL_1);
	ebase = kbase;
}

static void bcm3384_viper_quirks(void)
{
	/*
	 * Some experimental CM boxes are set up to let CM own the Viper TP0
	 * and let Linux own TP1.  This requires moving the kernel
	 * load address to a non-conflicting region (e.g. via
	 * CONFIG_PHYSICAL_START) and supplying an alternate DTB.
	 * If we detect this condition, we need to move the MIPS exception
	 * vectors up to an area that we own.
	 *
	 * This is distinct from the OTHER special case mentioned in
	 * smp-bmips.c (boot on TP1, but enable SMP, then TP0 becomes our
	 * logical CPU#1).  For the Viper TP1 case, SMP is off limits.
	 *
	 * Also note that many BMIPS435x CPUs do not have a
	 * BMIPS_RELO_VECTOR_CONTROL_1 register, so it isn't safe to just
	 * write VMLINUX_LOAD_ADDRESS into that register on every SoC.
	 */
	board_ebase_setup = &kbase_setup;
	bmips_smp_enabled = 0;
}

static void bcm63xx_fixup_cpu1(void)
{
	/*
	 * The bootloader has set up the CPU1 reset vector at
	 * 0xa000_0200.
	 * This conflicts with the special interrupt vector (IV).
	 * The bootloader has also set up CPU1 to respond to the wrong
	 * IPI interrupt.
	 * Here we will start up CPU1 in the background and ask it to
	 * reconfigure itself then go back to sleep.
	 */
	memcpy((void *)0xa0000200, &bmips_smp_movevec, 0x20);
	__sync();
	set_c0_cause(C_SW0);
	cpumask_set_cpu(1, &bmips_booted_mask);
}

static void bcm6328_quirks(void)
{
	/* Check CPU1 status in OTP (it is usually disabled) */
	if (__raw_readl(REG_BCM6328_OTP) & BCM6328_TP1_DISABLED)
		bmips_smp_enabled = 0;
	else
		bcm63xx_fixup_cpu1();
}

static void bcm6368_quirks(void)
{
	bcm63xx_fixup_cpu1();
}

static const struct bmips_quirk bmips_quirk_list[] = {
	{ "brcm,bcm3384-viper",		&bcm3384_viper_quirks		},
	{ "brcm,bcm33843-viper",	&bcm3384_viper_quirks		},
	{ "brcm,bcm6328",		&bcm6328_quirks			},
	{ "brcm,bcm6368",		&bcm6368_quirks			},
	{ },
};

void __init prom_init(void)
{
	register_bmips_smp_ops();
}

void __init prom_free_prom_memory(void)
{
}

const char *get_system_type(void)
{
	return "Generic BMIPS kernel";
}

void __init plat_time_init(void)
{
	struct device_node *np;
	u32 freq;

	np = of_find_node_by_name(NULL, "cpus");
	if (!np)
		panic("missing 'cpus' DT node");
	if (of_property_read_u32(np, "mips-hpt-frequency", &freq) < 0)
		panic("missing 'mips-hpt-frequency' property");
	of_node_put(np);

	mips_hpt_frequency = freq;
}

void __init transfer_mem_cfe_to_dt(void *dtb)
{
	int mem, len, i, rc;
	uint dram_sz = 0, tmp, base, size;
	uint new_map[3][2] = {{0,},};
	const ulong *map;

	if (cfe_getenv("DRAM0_SIZE", arcs_cmdline, COMMAND_LINE_SIZE) == CFE_OK) {
		if (kstrtouint(arcs_cmdline, 10, &tmp)) {
			printk(KERN_INFO "can't extract DRAM0_SIZE\n");
		} else {
			dram_sz = tmp*(1024*1024);
		}
	}
	if (cfe_getenv("DRAM1_SIZE", arcs_cmdline, COMMAND_LINE_SIZE) == CFE_OK) {
		if (kstrtouint(arcs_cmdline, 10, &tmp)) {
			printk(KERN_INFO "can't extract DRAM1_SIZE\n");
		} else {
			dram_sz += tmp*(1024*1024);
		}
	}
	if (!dram_sz) {
		printk(KERN_INFO "cfe dram size is 0\n");
		goto bail_mem_cfe_to_dt;
	}

	mem = fdt_path_offset(dtb, "/memory");
	if (mem < 0) {
		printk(KERN_INFO "could not find /memory value\n");
		goto bail_mem_cfe_to_dt;
	}

	map = (const ulong*)fdt_getprop(dtb, mem, "reg", &len);
	if (!map) {
		printk(KERN_INFO "could not find /memory reg\n");
		goto bail_mem_cfe_to_dt;
	}
	if (len != sizeof(new_map)) {
		printk(KERN_INFO "Not compatible with dt /memory/reg@ type, skipping re-map...\n");
		goto bail_mem_cfe_to_dt;
	}
	pr_debug("dt map:\n");
	for (i = 0 ; i < len/sizeof(base); i += 2) {
		base = fdt32_to_cpu(map[i]);
		size = fdt32_to_cpu(map[i+1]);
		pr_debug("  reg %08x @ %08x\n", size, base);
	}
	pr_debug("cfe total memory: 0x%x\n", dram_sz);

	for (i = 0; dram_sz; i++) {
		switch (i) {
		case 0:
			/* setup first map hole */
			size = (dram_sz < BMIPS_MAP0_SZ)?dram_sz:BMIPS_MAP0_SZ;
			new_map[i][0] = cpu_to_fdt32(BMIPS_MAP0_OFF);
			new_map[i][1] = cpu_to_fdt32(size);
			dram_sz -= size;
		break;
		case 1:
			/* setup second map hole */
			size = (dram_sz < BMIPS_MAP1_SZ)?dram_sz:BMIPS_MAP1_SZ;
			new_map[i][0] = cpu_to_fdt32(BMIPS_MAP1_OFF);
			new_map[i][1] = cpu_to_fdt32(size);
			dram_sz -= size;
		break;
		case 2:
			/* setup third map hole; everything else */
			new_map[i][0] = cpu_to_fdt32(BMIPS_MAP2_OFF);
			new_map[i][1] = cpu_to_fdt32(dram_sz);
			dram_sz = 0;
		break;
		default:
			printk(KERN_INFO "dt unexpected case\n");
			goto bail_mem_cfe_to_dt;
		}
	}

	rc = fdt_setprop_inplace(dtb, mem, "reg", &new_map, sizeof(new_map));
	if (rc != 0) {
		printk(KERN_INFO "set prop failed[%d]\n", rc);
		goto bail_mem_cfe_to_dt;
	}

	pr_debug("cfe re-map:\n");
	for (i = 0 ; i < len/sizeof(base); i += 2) {
		base = fdt32_to_cpu(map[i]);
		size = fdt32_to_cpu(map[i+1]);
		if (size)
			pr_debug("  reg %08x @ %08x\n", size, base);
	}
	return;
bail_mem_cfe_to_dt:
	printk(KERN_INFO "bailing cfe mem to dt re-map\n");
}

void __init transfer_net_cfe_to_dt(void *dtb)
{
	int eth, rc, len;
	u8 base_addr[6];
	const u8 *mac;

	if (cfe_getenv("ETH0_HWADDR", arcs_cmdline, COMMAND_LINE_SIZE) != CFE_OK) {
		printk(KERN_INFO "can't get cfe base mac address\n");
		goto bail_net_cfe_to_dt;
	}
	mac_pton(arcs_cmdline, base_addr);

	eth = fdt_path_offset(dtb, "/rdb");
	if (eth < 0) {
		printk(KERN_INFO "could not find /rdb value\n");
		goto bail_net_cfe_to_dt;
	}
	eth = fdt_subnode_offset(dtb, eth, "ethernet");
	if (eth < 0) {
		printk(KERN_INFO "could not find ethernet value\n");
		goto bail_net_cfe_to_dt;
	}

	mac = (const u8*)fdt_getprop(dtb, eth, "mac-address", &len);
	if (!mac) {
		printk(KERN_INFO "could not find mac-address property\n");
		goto bail_net_cfe_to_dt;
	}
	pr_debug("dt map:\n");
	pr_debug("  mac %pM\n", mac);

	rc = fdt_setprop_inplace(dtb, eth, "mac-address", base_addr, sizeof(base_addr));
	if (rc != 0) {
		printk(KERN_INFO "set mac prop failed[%d]\n", rc);
		goto bail_net_cfe_to_dt;
	}

	pr_debug("cfe re-map:\n");
	pr_debug("  mac %pM\n", base_addr);
	return;
bail_net_cfe_to_dt:
	printk(KERN_INFO "bailing cfe net to dt re-map\n");
}

#ifdef CONFIG_DT_BCM974XX
extern const char __dtb_bcm97425svmb_begin;
extern const char __dtb_bcm97429svmb_begin;
extern const char __dtb_bcm97435svmb_begin;

void __init pick_board_dt(void *orig_dtb)
{
	const void *dtb;

	if (cfe_getenv("CFE_BOARDNAME", arcs_cmdline, COMMAND_LINE_SIZE) != CFE_OK) {
		printk(KERN_INFO "can't get cfe board name\n");
		goto bail_pick_board_dt;
	}
	if (!strcmp(arcs_cmdline, "BCM97425SVMB")) {
		dtb = &__dtb_bcm97425svmb_begin;
	} else if (!strcmp(arcs_cmdline, "BCM97429SV")) {
		dtb = &__dtb_bcm97429svmb_begin;
	} else if (!strcmp(arcs_cmdline, "BCM97435SVMB")) {
		dtb = &__dtb_bcm97435svmb_begin;
	} else {
		printk(KERN_INFO "unkown board [%s]\n", arcs_cmdline);
		goto bail_pick_board_dt;
	}
	printk(KERN_INFO "cfe re-map:\n");
	printk(KERN_INFO "  board %s\n", arcs_cmdline);
	initial_boot_params = (void *)dtb;
	return;
bail_pick_board_dt:
	printk(KERN_INFO "bailing dt board selection\n");
	initial_boot_params = (void *)orig_dtb;
}
#else
static inline void pick_board_dt(void *orig_dtb)
{
	initial_boot_params = (void *)orig_dtb;
}
#endif /* CONFIG_DT_BCM974XX */

void __init transfer_cfe_to_dt(void *dtb)
{
	u64 h = (u64)(long)fw_arg0;
	u64 env =  (long)(void*)fw_arg2;

	if (fw_arg3 == CFE_EPTSEAL) {
		cfe_init(h, env);
		pick_board_dt(dtb);
		transfer_mem_cfe_to_dt(initial_boot_params);
		transfer_net_cfe_to_dt(initial_boot_params);
	}

}

void __init plat_mem_setup(void)
{
	void *dtb;
	const struct bmips_quirk *q;

	set_io_port_base(0);
	ioport_resource.start = 0;
	ioport_resource.end = ~0;

	/* intended to somewhat resemble ARM; see Documentation/arm/Booting */
	if (fw_arg0 == 0 && fw_arg1 == 0xffffffff)
		dtb = phys_to_virt(fw_arg2);
	else if (__dtb_start != __dtb_end)
		dtb = (void *)__dtb_start;
	else
		panic("no dtb found");

	transfer_cfe_to_dt(dtb);
	__dt_setup_arch(initial_boot_params);
	strlcpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);

	for (q = bmips_quirk_list; q->quirk_fn; q++) {
		if (of_flat_dt_is_compatible(of_get_flat_dt_root(),
					     q->compatible)) {
			q->quirk_fn();
		}
	}
}

void __init device_tree_init(void)
{
	struct device_node *np;

	unflatten_and_copy_device_tree();

	/* Disable SMP boot unless both CPUs are listed in DT and !disabled */
	np = of_find_node_by_name(NULL, "cpus");
	if (np && of_get_available_child_count(np) <= 1)
		bmips_smp_enabled = 0;
	of_node_put(np);
}

int __init plat_of_setup(void)
{
	return __dt_register_buses("simple-bus", NULL);
}

arch_initcall(plat_of_setup);

static int __init plat_dev_init(void)
{
	of_clk_init(NULL);
	return 0;
}

device_initcall(plat_dev_init);
