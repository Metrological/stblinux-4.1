/*
 * Broadcom STB CFE to Device Tree patching
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>
#include <asm/bootinfo.h>
#include <asm/fw/cfe/cfe_api.h>
#include <asm/fw/cfe/cfe_error.h>

/* define default holes, there is no BMIPS_MAP2_SZ because that
 * map takes everything else that was not taken by MAP[01]
 */
#define BMIPS_MAP0_OFF 0x00000000
#define BMIPS_MAP0_SZ  0x10000000
#define BMIPS_MAP1_OFF 0x20000000
#define BMIPS_MAP1_SZ  0x30000000
#define BMIPS_MAP2_OFF 0x90000000

static void __init transfer_mem_cfe_to_dt(void *dtb)
{
	int mem, len, i, rc;
	uint dram_sz = 0, tmp, base, size;
	uint new_map[3][2] = {{0,},};
	const ulong *map;
	char dram0_size[32];
	char dram1_size[32];

	if (cfe_getenv("DRAM0_SIZE", dram0_size, sizeof(dram0_size)) == CFE_OK) {
		if (kstrtouint(dram0_size, 10, &tmp)) {
			printk(KERN_INFO "can't extract DRAM0_SIZE\n");
		} else {
			dram_sz = tmp*(1024*1024);
		}
	}
	if (cfe_getenv("DRAM1_SIZE", dram1_size, sizeof(dram1_size)) == CFE_OK) {
		if (kstrtouint(dram1_size, 10, &tmp)) {
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

static int __init alloc_mac(void *dtb, int offset, const u8 *base_addr)
{
	const struct fdt_property *prop;
	int rc;

	/* Check whether this interface is enabled or not */
	prop = fdt_get_property(dtb, offset, "status", NULL);
	if (!strncmp(prop->data, "ok", strlen("ok"))) {
		fdt_delprop(dtb, offset, "mac-address");
		rc = fdt_setprop(dtb, offset, "mac-address",
				 base_addr, 6);
		return rc;
	}

	return -1;
}

static int __init moca_set_chip_id(void *dtb, int offset)
{
	const struct fdt_property *prop;
	char chip_str[9];
	unsigned long chip_id;
	int rc;

	rc = cfe_getenv("CHIP_PRODUCT_ID", chip_str, sizeof(chip_str));
	if (rc != CFE_OK)
		return -1;

	rc = kstrtoul(chip_str, 16, &chip_id);
	if (rc)
		return rc;

	chip_id = cpu_to_fdt32(chip_id);

	prop = fdt_get_property(dtb, offset, "status", NULL);
	if (!strncmp(prop->data, "ok", strlen("ok"))) {
		fdt_delprop(dtb, offset, "chip-id");
		rc = fdt_setprop(dtb, offset, "chip-id",
				 &chip_id, sizeof(chip_id));
		return rc;
	}

	return 0;
}

static void __init transfer_net_cfe_to_dt(void *dtb)
{
	const char *genet_match = "brcm,genet-v";
	const char *moca_match = "brcm,bmoca-instance";
	int offset = 0, rc, ret, genet0_off;
	const char *match = genet_match;
	const struct fdt_property *prop;
	unsigned int times = 0;
	char eth0_hwaddr[18];
	u8 base_addr[6];

	if (cfe_getenv("ETH0_HWADDR", eth0_hwaddr, sizeof(eth0_hwaddr)) != CFE_OK) {
		pr_err("Can't get CFE base MAC address\n");
		return;
	}

	mac_pton(eth0_hwaddr, base_addr);

	rc = fdt_path_offset(dtb, "/rdb");
	if (rc < 0) {
		pr_err("could not find /rdb value\n");
		return;
	}

again:
	/* Provide the same MAC address allocation as the stblinux-3.3
	 * BSP: ETH0_HWADDR is the base MAC address, GENET_0 gets its
	 * value, then we increment by 0x100, and take care of MoCA
	 */
	for (offset = fdt_next_node(dtb, rc, NULL); offset >= 0;
	     offset = fdt_next_node(dtb, offset, NULL)) {
		prop = fdt_get_property(dtb, offset, "compatible", NULL);
		if (prop && !strncmp(prop->data, match, strlen(match))) {
			ret = alloc_mac(dtb, offset, base_addr);
			if (ret == 0)
				base_addr[4]++;

			/* Save GENET_0 offset for next GENET iteration */
			if (times == 0)
				genet0_off = offset;

			times++;

			/* Start again, but match MOCA now */
			if (times == 1)
				match = moca_match;

			/* Second time matching GENET */
			else if (times == 2) {
				ret = moca_set_chip_id(dtb, offset);
				if (ret)
					break;
				match = genet_match;
				rc = genet0_off;
			/* Then stop here */
			} else
				break;

			goto again;
		}
	}
}

struct part_info {
	char *env_name;
	unsigned long int value;
};

static int __init get_part_infos(struct part_info *info, size_t size)
{
	unsigned int i;
	char buf[512];
	int rc, num_parts = 0;

	for (i = 0; i < size; i++) {
		rc = cfe_getenv(info[i].env_name, buf, sizeof(buf));
		if (rc != CFE_OK)
			continue;

		rc = kstrtoul(buf, 16, &info[i].value);
		if (rc)
			continue;

		num_parts++;
	}

	return num_parts;
}

static void __init alloc_mtd_partitions(void *dtb)
{
	struct part_info parts[] = {
		{ "LINUX_FFS_STARTAD", },
		{ "LINUX_FFS_SIZE", },
		{ "LINUX_PART_STARTAD" },
		{ "LINUX_PART_SIZE" },
	};
#define NUM_PARTS	4
	const struct fdt_property *prop = NULL;
	const char *match = "brcm,nandcs";
	unsigned int idx = 0, address = 2, size = 2;
	bool found = false;
	u32 reg[NUM_PARTS];
	int rc, offset;

	rc = get_part_infos(parts, NUM_PARTS);
	if (rc != NUM_PARTS) {
		pr_err("Could not get partitions from CFE: %d\n", rc);
		return;
	}

	rc = fdt_path_offset(dtb, "/rdb");
	if (rc < 0)
		return;

	for (offset = fdt_next_node(dtb, rc, NULL); offset >= 0;
	     offset = fdt_next_node(dtb, offset, NULL)) {
		prop = fdt_get_property(dtb, offset, "compatible", NULL);
		if (prop && !strncmp(prop->data, match, strlen(match))) {
			found = true;
			break;
		}
	}

	if (!found) {
		pr_warn("Could not find %s node in tree\n", match);
		return;
	}

	/* Get the address and size cells to make the code a bit robust */
	address = be32_to_cpup(fdt_getprop(dtb, offset,
			       "#address-cells", NULL));
	size = be32_to_cpup(fdt_getprop(dtb, offset,
			    "#size-cells", NULL));

	/* Now descend again into the partition sub-nodes */
	for (rc = fdt_next_node(dtb, offset, NULL); rc >= 0;
	     rc = fdt_next_node(dtb, rc, NULL)) {
		memset(&reg, 0, sizeof(reg));
		reg[address - 1] = cpu_to_fdt32(parts[idx].value);
		reg[address + size - 1] = cpu_to_fdt32(parts[idx + 1].value);
		fdt_setprop_inplace(dtb, rc, "reg", &reg,
				    sizeof(u32) * (address + size));
		idx += NUM_PARTS / 2;
		if (idx == NUM_PARTS)
			break;
	}
}

#ifdef CONFIG_DT_BCM974XX
extern const char __dtb_bcm97425svmb_begin;
extern const char __dtb_bcm97429svmb_begin;
extern const char __dtb_bcm97435svmb_begin;

static void __init pick_board_dt(void *orig_dtb)
{
	const void *dtb;
	char board_name[512];

	if (cfe_getenv("CFE_BOARDNAME", board_name, sizeof(board_name)) != CFE_OK) {
		printk(KERN_INFO "can't get cfe board name\n");
		goto bail_pick_board_dt;
	}
	if (!strcmp(board_name, "BCM97425SVMB")) {
		dtb = &__dtb_bcm97425svmb_begin;
	} else if (!strcmp(board_name, "BCM97429SV")) {
		dtb = &__dtb_bcm97429svmb_begin;
	} else if (!strcmp(board_name, "BCM97435SVMB")) {
		dtb = &__dtb_bcm97435svmb_begin;
	} else {
		printk(KERN_INFO "unkown board [%s]\n", board_name);
		goto bail_pick_board_dt;
	}
	printk(KERN_INFO "cfe re-map:\n");
	printk(KERN_INFO "  board %s\n", board_name);
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
		alloc_mtd_partitions(initial_boot_params);
		cfe_getenv("BOOT_FLAGS", arcs_cmdline, COMMAND_LINE_SIZE);
	}

}

