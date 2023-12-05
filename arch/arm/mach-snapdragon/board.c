// SPDX-License-Identifier: GPL-2.0+
/*
 * Common initialisation for Qualcomm Snapdragon boards.
 *
 * Copyright (c) 2023 Linaro Ltd.
 * Author: Caleb Connolly <caleb.connolly@linaro.org>
 */

#define LOG_DEBUG

#include <asm/armv8/mmu.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/psci.h>
#include <asm/system.h>
#include <dm/device.h>
#include <env.h>
#include <init.h>
#include <linux/arm-smccc.h>
#include <linux/bug.h>
#include <linux/psci.h>
#include <linux/sizes.h>
#include <lmb.h>
#include <malloc.h>

DECLARE_GLOBAL_DATA_PTR;

static struct mm_region rbx_mem_map[CONFIG_NR_DRAM_BANKS + 2] = { { 0 } };

struct mm_region *mem_map = rbx_mem_map;

int dram_init(void)
{
	return fdtdec_setup_mem_size_base();
}

int dram_init_banksize(void)
{
	int ret;
	phys_addr_t start, size;

	ret = fdtdec_setup_memory_banksize();
	if (ret < 0)
		return ret;

	if (WARN(CONFIG_NR_DRAM_BANKS < 2, "CONFIG_NR_DRAM_BANKS should be at least 2"))
		return 0;

	/* Some bootloaders populate the RAM banks in the wrong order -_- */
	start = gd->bd->bi_dram[1].start;
	size = gd->bd->bi_dram[1].size;
	if (size && start < gd->bd->bi_dram[0].start) {
		debug("Swapping DRAM banks\n");
		gd->bd->bi_dram[1].start = gd->bd->bi_dram[0].start;
		gd->bd->bi_dram[1].size = gd->bd->bi_dram[0].size;
		gd->bd->bi_dram[0].start = start;
		gd->bd->bi_dram[0].size = size;
	}

	printf("%s done\n", __func__);
	return 0;
}

static void show_psci_version(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(ARM_PSCI_0_2_FN_PSCI_VERSION, 0, 0, 0, 0, 0, 0, 0, &res);

	debug("PSCI:  v%ld.%ld\n",
	      PSCI_VERSION_MAJOR(res.a0),
	      PSCI_VERSION_MINOR(res.a0));
}

void *board_fdt_blob_setup(int *err)
{
	phys_addr_t fdt;
	/* Return DTB pointer passed by ABL */
	*err = 0;
	fdt = get_prev_bl_fdt_addr();

	/*
	 * If we bail then the board will simply not boot, instead let's
	 * try and use the FDT built into U-Boot if there is one...
	 * This avoids having a hard dependency on the previous stage bootloader
	 */
	if (IS_ENABLED(CONFIG_OF_SEPARATE) && (!fdt || fdt != ALIGN(fdt, SZ_4K))) {
		debug("%s: Using built in FDT, bootloader gave us %#llx\n", __func__, fdt);
		return (void *)gd->fdt_blob;
	}

	return (void *)fdt;
}

void reset_cpu(void)
{
	psci_system_reset();
}

/*
 * Some boards still need board specific init code, they can implement that by
 * overriding this function.
 *
 * FIXME: get rid of board specific init code
 */
void __weak qcom_board_init(void)
{
}

int board_init(void)
{
	show_psci_version();
	qcom_board_init();
	return 0;
}

/* Sets up the "board", and "soc" environment variables as well as constructing the devicetree
 * path, with a few quirks to handle non-standard dtb filenames. This is not meant to be a
 * comprehensive solution to automatically picking the DTB, but aims to be correct for the
 * majority case. For most devices it should be possible to make this algorithm work by
 * adjusting the root compatible property in the U-Boot DTS. Handling devices with multiple
 * variants that are all supported by a single U-Boot image will require implementing device-
 * specific detection.
 */
static void configure_env(void)
{
	const char *first_compat, *last_compat;
	char *tmp;
	char buf[32] = { 0 };
	/*
	 * Most DTB filenames follow the scheme: qcom/<soc>-[vendor]-<board>.dtb
	 * The vendor is skipped when it's a Qualcomm reference board, or the
	 * db845c.
	 */
	char dt_path[64] = { 0 };
	int compat_count, ret;
	ofnode root;

	root = ofnode_root();
	/* This is almost always 2, but be explicit that we want the first and last compatibles
	 * not the first and second.
	 */
	compat_count = ofnode_read_string_count(root, "compatible");
	if (compat_count < 2) {
		log_warning("%s: only one root compatible bailing!\n", __func__);
		return;
	}

	/* The most specific device compatible (e.g. "thundercomm,db845c") */
	ret = ofnode_read_string_index(root, "compatible", 0, &first_compat);
	if (ret < 0) {
		log_warning("Can't read first compatible\n");
		return;
	}

	/* The last compatible is always the SoC compatible */
	ret = ofnode_read_string_index(root, "compatible", compat_count - 1, &last_compat);
	if (ret < 0) {
		log_warning("Can't read second compatible\n");
		return;
	}

	/* Copy the second compat (e.g. "qcom,sdm845") into buf */
	strlcpy(buf, last_compat, sizeof(buf) - 1);
	tmp = buf;

	/* strsep() is destructive, it replaces the comma with a \0 */
	if (!strsep(&tmp, ",")) {
		log_warning("second compatible '%s' has no ','\n", buf);
		return;
	}

	/* tmp now points to just the "sdm845" part of the string */
	env_set("soc", tmp);

	/* Now figure out the "board" part from the first compatible */
	memset(buf, 0, sizeof(buf));
	strlcpy(buf, first_compat, sizeof(buf) - 1);
	tmp = buf;

	/* The Qualcomm reference boards (RBx, HDK, etc)  */
	if (!strncmp("qcom", buf, strlen("qcom"))) {
		/*
		 * They all have the first compatible as "qcom,<soc>-<board>"
		 * (e.g. "qcom,qrb5165-rb5"). We extract just the part after
		 * the dash.
		 */
		if (!strsep(&tmp, "-")) {
			log_warning("compatible '%s' has no '-'\n", buf);
			return;
		}
		/* tmp is now "rb5" */
		env_set("board", tmp);
	} else {
		if (!strsep(&tmp, ",")) {
			log_warning("compatible '%s' has no ','\n", buf);
			return;
		}
		/* for thundercomm we just want the bit after the comma (e.g. "db845c"),
		 * for all other boards we replace the comma with a '-' and take both
		 * (e.g. "oneplus-enchilada")
		 */
		if (!strncmp("thundercomm", buf, strlen("thundercomm"))) {
			env_set("board", tmp);
		} else {
			*(tmp - 1) = '-';
			env_set("board", buf);
		}
	}

	/* Now build the full path name */
	snprintf(dt_path, sizeof(dt_path), "qcom/%s-%s.dtb",
		 env_get("soc"), env_get("board"));
	env_set("fdtfile", dt_path);
}

void __weak qcom_late_init(void)
{
}

#define KERNEL_COMP_SIZE	SZ_64M
#define SZ_96M			(SZ_64M + SZ_32M)
#ifdef CONFIG_FASTBOOT_BUF_SIZE
#define FASTBOOT_BUF_SIZE CONFIG_FASTBOOT_BUF_SIZE
#else
#define FASTBOOT_BUF_SIZE 0
#endif

#define addr_alloc(lmb, size) lmb_alloc(lmb, size, SZ_2M)

/* Stolen from arch/arm/mach-apple/board.c */
int board_late_init(void)
{
	struct lmb lmb;
	u32 status = 0;

	lmb_init_and_reserve(&lmb, gd->bd, (void *)gd->fdt_blob);

	/* We need to be fairly conservative here as we support boards with just 1G of TOTAL RAM */
	status |= env_set_hex("kernel_addr_r", addr_alloc(&lmb, SZ_128M));
	status |= env_set_hex("ramdisk_addr_r", addr_alloc(&lmb, SZ_96M));
	status |= env_set_hex("kernel_comp_addr_r", addr_alloc(&lmb, KERNEL_COMP_SIZE));
	status |= env_set_hex("kernel_comp_size", KERNEL_COMP_SIZE);
	if (IS_ENABLED(CONFIG_FASTBOOT))
		status |= env_set_hex("fastboot_addr_r", addr_alloc(&lmb, FASTBOOT_BUF_SIZE));
	status |= env_set_hex("scriptaddr", addr_alloc(&lmb, SZ_4M));
	status |= env_set_hex("pxefile_addr_r", addr_alloc(&lmb, SZ_4M));
	status |= env_set_hex("fdt_addr_r", addr_alloc(&lmb, SZ_2M));

	if (status)
		log_warning("%s: Failed to set run time variables\n", __func__);

	configure_env();
	qcom_late_init();

	return 0;
}

static void build_mem_map(void)
{
	int i;

	/*
	 * Ensure the peripheral block is sized to correctly cover the address range
	 * up to the first memory bank.
	 * Don't map the first page to ensure that we actually trigger an abort on a
	 * null pointer access rather than just hanging.
	 * FIXME: we should probably split this into more precise regions
	 */
	mem_map[0].phys = 0x1000;
	mem_map[0].virt = mem_map[0].phys;
	mem_map[0].size = gd->bd->bi_dram[0].start - mem_map[0].phys;
	mem_map[0].attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN;

	debug("Configured memory map:\n");
	debug("  0x%016llx - 0x%016llx: Peripheral block\n",
	      mem_map[0].phys, mem_map[0].phys + mem_map[0].size);

	/*
	 * Now add memory map entries for each DRAM bank, ensuring we don't
	 * overwrite the list terminator
	 */
	for (i = 0; i < ARRAY_SIZE(rbx_mem_map) - 2 && gd->bd->bi_dram[i].size; i++) {
		if (i == ARRAY_SIZE(rbx_mem_map) - 1) {
			log_warning("Too many DRAM banks!\n");
			break;
		}
		mem_map[i + 1].phys = gd->bd->bi_dram[i].start;
		mem_map[i + 1].virt = mem_map[i + 1].phys;
		mem_map[i + 1].size = gd->bd->bi_dram[i].size;
		mem_map[i + 1].attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
				     PTE_BLOCK_INNER_SHARE;

		debug("  0x%016llx - 0x%016llx: DDR bank %d\n",
		      mem_map[i + 1].phys, mem_map[i + 1].phys + mem_map[i + 1].size, i);
	}
}

u64 get_page_table_size(void)
{
	return SZ_64K;
}

void enable_caches(void)
{
	build_mem_map();

	icache_enable();
	dcache_enable();
}
