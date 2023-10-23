// SPDX-License-Identifier: GPL-2.0+
/*
 *  EFI application tables support
 *
 *  Copyright (c) 2016 Alexander Graf
 */

#define LOG_CATEGORY LOGC_EFI

#include <common.h>
#include <efi_loader.h>
#include <lmb.h>
#include <log.h>
#include <malloc.h>
#include <mapmem.h>
#include <smbios.h>
#include <linux/sizes.h>

enum {
	TABLE_SIZE	= SZ_4K,
};

/*
 * Install the SMBIOS table as a configuration table.
 *
 * Return:	status code
 */
efi_status_t efi_smbios_register(void)
{
	ulong addr;
	efi_status_t ret;

	addr = gd->arch.smbios_start;
	if (!addr) {
		log_err("No SMBIOS tables to install\n");
		return EFI_NOT_FOUND;
	}

	/* Mark space used for tables */
	ret = efi_add_memory_map(addr, TABLE_SIZE, EFI_RUNTIME_SERVICES_DATA);
	if (ret)
		return ret;

	log_debug("EFI using SMBIOS tables at %lx\n", addr);

	/* Install SMBIOS information as configuration table */
	return efi_install_configuration_table(&smbios_guid,
					       map_sysmem(addr, 0));
}

static int install_smbios_table(void)
{
	struct lmb lmb;
	phys_addr_t addr;

	if (!IS_ENABLED(CONFIG_GENERATE_SMBIOS_TABLE) || IS_ENABLED(CONFIG_X86))
		return 0;

	lmb_init_and_reserve(&lmb, gd->bd, (void*)gd->fdt_blob);

	addr = lmb_alloc_base(&lmb, TABLE_SIZE, SZ_4K, UINT_MAX);
	addr = map_to_sysmem((void*)addr);

	if (!write_smbios_table(addr)) {
		log_err("Failed to write SMBIOS table\n");
		return log_msg_ret("smbios", -EINVAL);
	}

	/* Make a note of where we put it */
	log_debug("SMBIOS tables written to %llx\n", addr);
	gd->arch.smbios_start = addr;

	return 0;
}
EVENT_SPY_SIMPLE(EVT_LAST_STAGE_INIT, install_smbios_table);
