// SPDX-License-Identifier: BSD-3-Clause
/*
 * Clock drivers for Qualcomm sc7280
 *
 * (C) Copyright 2023 Linaro Ltd.
 */

#define LOG_DEBUG

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <linux/delay.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bug.h>
#include <linux/bitops.h>
#include <dt-bindings/clock/qcom,gcc-sc7280.h>

#include "clock-qcom.h"

static ulong sc7280_set_rate(struct clk *clk, ulong rate)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	if (clk->id < priv->data->num_clks)
		debug("%s: %s, requested rate=%ld\n", __func__, priv->data->clks[clk->id].name, rate);

	switch (clk->id) {
	default:
		return 0;
	}
}

static const struct gate_clk sc7280_clks[] = {

};

static int sc7280_enable(struct clk *clk)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	if (priv->data->num_clks < clk->id) {
		debug("%s: unknown clk id %lu\n", __func__, clk->id);
		return 0;
	}

	debug("%s: clk %s\n", __func__, sc7280_clks[clk->id].name);

	switch (clk->id) {
	}

	qcom_gate_clk_en(priv, clk->id);

	return 0;
}

static const struct qcom_reset_map sc7280_gcc_resets[] = {
	[GCC_PCIE_0_BCR] = { 0x6b000 },
	[GCC_PCIE_0_PHY_BCR] = { 0x6c01c },
	[GCC_PCIE_1_BCR] = { 0x8d000 },
	[GCC_PCIE_1_PHY_BCR] = { 0x8e01c },
	[GCC_QUSB2PHY_PRIM_BCR] = { 0x12000 },
	[GCC_QUSB2PHY_SEC_BCR] = { 0x12004 },
	[GCC_SDCC1_BCR] = { 0x75000 },
	[GCC_SDCC2_BCR] = { 0x14000 },
	[GCC_SDCC4_BCR] = { 0x16000 },
	[GCC_UFS_PHY_BCR] = { 0x77000 },
	[GCC_USB30_PRIM_BCR] = { 0xf000 },
	[GCC_USB30_SEC_BCR] = { 0x9e000 },
	[GCC_USB3_DP_PHY_PRIM_BCR] = { 0x50008 },
	[GCC_USB3_PHY_PRIM_BCR] = { 0x50000 },
	[GCC_USB3PHY_PHY_PRIM_BCR] = { 0x50004 },
	[GCC_USB_PHY_CFG_AHB2PHY_BCR] = { 0x6a000 },
};

static struct msm_clk_data qcs404_gcc_data = {
	.resets = sc7280_gcc_resets,
	.num_resets = ARRAY_SIZE(sc7280_gcc_resets),
	.clks = sc7280_clks,
	.num_clks = ARRAY_SIZE(sc7280_clks),

	.enable = sc7280_enable,
	.set_rate = sc7280_set_rate,
};


static const struct udevice_id gcc_sc7280_of_match[] = {
	{
		.compatible = "qcom,gcc-sc7280",
		.data = (ulong)&qcs404_gcc_data,
	},
	{ }
};

U_BOOT_DRIVER(gcc_sc7280) = {
	.name		= "gcc_sc7280",
	.id		= UCLASS_NOP,
	.of_match	= gcc_sc7280_of_match,
	.bind		= qcom_cc_bind,
	.flags		= DM_FLAG_PRE_RELOC,
};
