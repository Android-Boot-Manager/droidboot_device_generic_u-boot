// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Bhupesh Sharma <bhupesh.sharma@linaro.org>
 *
 * Based on Linux driver
 */

#include <clk.h>
#include <clk-uclass.h>
#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <generic-phy.h>
#include <malloc.h>
#include <reset.h>

#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#define USB2_PHY_USB_PHY_UTMI_CTRL0		(0x3c)
#define SLEEPM					BIT(0)
#define OPMODE_MASK				GENMASK(4, 3)
#define OPMODE_NORMAL				(0x00)
#define OPMODE_NONDRIVING			BIT(3)
#define TERMSEL					BIT(5)

#define USB2_PHY_USB_PHY_UTMI_CTRL1		(0x40)
#define XCVRSEL					BIT(0)

#define USB2_PHY_USB_PHY_UTMI_CTRL5		(0x50)
#define POR					BIT(1)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON0	(0x54)
#define SIDDQ					BIT(2)
#define RETENABLEN				BIT(3)
#define FSEL_MASK				GENMASK(6, 4)
#define FSEL_DEFAULT				(0x3 << 4)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON1	(0x58)
#define VBUSVLDEXTSEL0				BIT(4)
#define PLLBTUNE				BIT(5)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON2	(0x5c)
#define VREGBYPASS				BIT(0)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL1		(0x60)
#define VBUSVLDEXT0				BIT(0)

#define USB2_PHY_USB_PHY_HS_PHY_CTRL2		(0x64)
#define USB2_AUTO_RESUME			BIT(0)
#define USB2_SUSPEND_N				BIT(2)
#define USB2_SUSPEND_N_SEL			BIT(3)

#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X0		(0x6c)
#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1		(0x70)
#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2		(0x74)
#define USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X3		(0x78)
#define PARAM_OVRD_MASK				0xFF

#define USB2_PHY_USB_PHY_CFG0			(0x94)
#define UTMI_PHY_DATAPATH_CTRL_OVERRIDE_EN	BIT(0)
#define UTMI_PHY_CMN_CTRL_OVERRIDE_EN		BIT(1)

#define USB2_PHY_USB_PHY_REFCLK_CTRL		(0xa0)
#define REFCLK_SEL_MASK				GENMASK(1, 0)
#define REFCLK_SEL_DEFAULT			(0x2 << 0)

#define HS_DISCONNECT_MASK			GENMASK(2, 0)
#define SQUELCH_DETECTOR_MASK			GENMASK(7, 5)

#define HS_AMPLITUDE_MASK			GENMASK(3, 0)
#define PREEMPHASIS_DURATION_MASK		BIT(5)
#define PREEMPHASIS_AMPLITUDE_MASK		GENMASK(7, 6)

#define HS_RISE_FALL_MASK			GENMASK(1, 0)
#define HS_CROSSOVER_VOLTAGE_MASK		GENMASK(3, 2)
#define HS_OUTPUT_IMPEDANCE_MASK		GENMASK(5, 4)

#define LS_FS_OUTPUT_IMPEDANCE_MASK		GENMASK(3, 0)

#define SNPS_HS_NUM_VREGS		ARRAY_SIZE(hs_7nm_vreg_names)

struct override_param {
	s32	value;
	u8	reg_val;
};

struct override_param_map {
	const char *prop_name;
	const struct override_param *param_table;
	u8 table_size;
	u8 reg_offset;
	u8 param_mask;
};

struct phy_override_seq {
	bool	need_update;
	u8	offset;
	u8	value;
	u8	mask;
};

#define NUM_HSPHY_TUNING_PARAMS	(9)

/* struct hs_7nm_phy_cfg - per-PHY initialization config */
struct hs_7nm_phy_cfg {
	/* clocks to be requested */
	struct clk *clks;
	int num_clks;

	/* resets to be requested */
	struct reset_ctl *resets;
	int num_resets;

	struct override_param_map *map_cfg;
	struct phy_override_seq update_seq_cfg[NUM_HSPHY_TUNING_PARAMS];
};

/**
 * struct hs_7nm_phy_priv - snps hs phy attributes
 */
struct hs_7nm_phy_priv {
	void __iomem *base;

	/* clocks to be requested */
	struct clk *clks;
	int num_clks;

	/* resets to be requested */
	struct reset_ctl *resets;
	int num_resets;

	struct hs_7nm_phy_cfg *cfg;
};

/* Enable DEBUG_HS_PHY to trace hs_7nm reg read/write */
#define DEBUG_HS_PHY

#ifdef DEBUG_HS_PHY
#define debug_hs_7nm printf
static unsigned int writel_count = 0;
static unsigned int readl_count = 0;
#endif

static inline void hs_7nm_write_mask(void __iomem *base, u32 offset,
						u32 mask, u32 val)
{
	u32 reg;

	reg = readl_relaxed(base + offset);

#ifdef DEBUG_HS_PHY
	debug_hs_7nm("%s: readl_cnt=%u, reg=0x%p, val=0x%x\n",
			__func__, readl_count++, base + offset, reg);
#endif

	reg &= ~mask;
	reg |= val & mask;
	writel_relaxed(reg, base + offset);

#ifdef DEBUG_HS_PHY
	debug_hs_7nm("%s: writel_cnt=%u, reg=0x%p, val=0x%x\n",
			__func__, writel_count++, base + offset, reg);
#endif

	/* Ensure above write is completed */
	readl_relaxed(base + offset);
}

static const struct override_param hs_disconnect_sc7280[] = {
	{ -272, 0 },
	{ 0, 1 },
	{ 317, 2 },
	{ 630, 3 },
	{ 973, 4 },
	{ 1332, 5 },
	{ 1743, 6 },
	{ 2156, 7 },
};

static const struct override_param squelch_det_threshold_sc7280[] = {
	{ -2090, 7 },
	{ -1560, 6 },
	{ -1030, 5 },
	{ -530, 4 },
	{ 0, 3 },
	{ 530, 2 },
	{ 1060, 1 },
	{ 1590, 0 },
};

static const struct override_param hs_amplitude_sc7280[] = {
	{ -660, 0 },
	{ -440, 1 },
	{ -220, 2 },
	{ 0, 3 },
	{ 230, 4 },
	{ 440, 5 },
	{ 650, 6 },
	{ 890, 7 },
	{ 1110, 8 },
	{ 1330, 9 },
	{ 1560, 10 },
	{ 1780, 11 },
	{ 2000, 12 },
	{ 2220, 13 },
	{ 2430, 14 },
	{ 2670, 15 },
};

static const struct override_param preemphasis_duration_sc7280[] = {
	{ 10000, 1 },
	{ 20000, 0 },
};

static const struct override_param preemphasis_amplitude_sc7280[] = {
	{ 10000, 1 },
	{ 20000, 2 },
	{ 30000, 3 },
	{ 40000, 0 },
};

static const struct override_param hs_rise_fall_time_sc7280[] = {
	{ -4100, 3 },
	{ 0, 2 },
	{ 2810, 1 },
	{ 5430, 0 },
};

static const struct override_param hs_crossover_voltage_sc7280[] = {
	{ -31000, 1 },
	{ 0, 3 },
	{ 28000, 2 },
};

static const struct override_param hs_output_impedance_sc7280[] = {
	{ -2300000, 3 },
	{ 0, 2 },
	{ 2600000, 1 },
	{ 6100000, 0 },
};

static const struct override_param ls_fs_output_impedance_sc7280[] = {
	{ -1053, 15 },
	{ -557, 7 },
	{ 0, 3 },
	{ 612, 1 },
	{ 1310, 0 },
};

static const struct override_param_map sc7280_snps_7nm_phy[] = {
	{
		"qcom,hs-disconnect-bp",
		hs_disconnect_sc7280,
		ARRAY_SIZE(hs_disconnect_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X0,
		HS_DISCONNECT_MASK
	},
	{
		"qcom,squelch-detector-bp",
		squelch_det_threshold_sc7280,
		ARRAY_SIZE(squelch_det_threshold_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X0,
		SQUELCH_DETECTOR_MASK
	},
	{
		"qcom,hs-amplitude-bp",
		hs_amplitude_sc7280,
		ARRAY_SIZE(hs_amplitude_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1,
		HS_AMPLITUDE_MASK
	},
	{
		"qcom,pre-emphasis-duration-bp",
		preemphasis_duration_sc7280,
		ARRAY_SIZE(preemphasis_duration_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1,
		PREEMPHASIS_DURATION_MASK,
	},
	{
		"qcom,pre-emphasis-amplitude-bp",
		preemphasis_amplitude_sc7280,
		ARRAY_SIZE(preemphasis_amplitude_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X1,
		PREEMPHASIS_AMPLITUDE_MASK,
	},
	{
		"qcom,hs-rise-fall-time-bp",
		hs_rise_fall_time_sc7280,
		ARRAY_SIZE(hs_rise_fall_time_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2,
		HS_RISE_FALL_MASK
	},
	{
		"qcom,hs-crossover-voltage-microvolt",
		hs_crossover_voltage_sc7280,
		ARRAY_SIZE(hs_crossover_voltage_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2,
		HS_CROSSOVER_VOLTAGE_MASK
	},
	{
		"qcom,hs-output-impedance-micro-ohms",
		hs_output_impedance_sc7280,
		ARRAY_SIZE(hs_output_impedance_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X2,
		HS_OUTPUT_IMPEDANCE_MASK,
	},
	{
		"qcom,ls-fs-output-impedance-bp",
		ls_fs_output_impedance_sc7280,
		ARRAY_SIZE(ls_fs_output_impedance_sc7280),
		USB2_PHY_USB_PHY_HS_PHY_OVERRIDE_X3,
		LS_FS_OUTPUT_IMPEDANCE_MASK,
	},
	{},
};

static int hs_7nm_usb_init(struct phy *phy)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(phy->dev);
	int i;

	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_CFG0,
					UTMI_PHY_CMN_CTRL_OVERRIDE_EN,
					UTMI_PHY_CMN_CTRL_OVERRIDE_EN);
	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_UTMI_CTRL5,
							POR, POR);
	hs_7nm_write_mask(hs_7nm->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON0,
					FSEL_MASK, 0);
	hs_7nm_write_mask(hs_7nm->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON1,
					PLLBTUNE, PLLBTUNE);
	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_REFCLK_CTRL,
					REFCLK_SEL_DEFAULT, REFCLK_SEL_MASK);
	hs_7nm_write_mask(hs_7nm->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON1,
					VBUSVLDEXTSEL0, VBUSVLDEXTSEL0);
	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_HS_PHY_CTRL1,
					VBUSVLDEXT0, VBUSVLDEXT0);

	for (i = 0; i < ARRAY_SIZE(hs_7nm->cfg->update_seq_cfg); i++) {
		if (hs_7nm->cfg->update_seq_cfg[i].need_update)
			hs_7nm_write_mask(hs_7nm->base,
					hs_7nm->cfg->update_seq_cfg[i].offset,
					hs_7nm->cfg->update_seq_cfg[i].mask,
					hs_7nm->cfg->update_seq_cfg[i].value);
	}

	hs_7nm_write_mask(hs_7nm->base,
					USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON2,
					VREGBYPASS, VREGBYPASS);

	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_HS_PHY_CTRL2,
					USB2_SUSPEND_N_SEL | USB2_SUSPEND_N,
					USB2_SUSPEND_N_SEL | USB2_SUSPEND_N);

	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_UTMI_CTRL0,
					SLEEPM, SLEEPM);

	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_HS_PHY_CTRL_COMMON0,
				   SIDDQ, 0);

	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_UTMI_CTRL5,
					POR, 0);

	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_HS_PHY_CTRL2,
					USB2_SUSPEND_N_SEL, 0);

	hs_7nm_write_mask(hs_7nm->base, USB2_PHY_USB_PHY_CFG0,
					UTMI_PHY_CMN_CTRL_OVERRIDE_EN, 0);

	return 0;
}

static int hs_7nm_phy_do_reset(struct hs_7nm_phy_priv *hs_7nm)
{
	int i, ret;

	for (i = 0; i < hs_7nm->num_resets; i++) {
		ret = reset_assert(&hs_7nm->resets[i]);
		if (ret)
			return ret;
	}

	udelay(10);

	for (i = 0; i < hs_7nm->num_resets; i++) {
		ret = reset_deassert(&hs_7nm->resets[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int hs_7nm_phy_power_on(struct phy *phy)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(phy->dev);
	int ret;

	ret = hs_7nm_phy_do_reset(hs_7nm);
	if (ret)
		return ret;
	
	ret = hs_7nm_usb_init(phy);
	if (ret)
		return ret;

	return 0;
}

static int hs_7nm_phy_power_off(struct phy *phy)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(phy->dev);

	reset_release_all(hs_7nm->resets, hs_7nm->num_resets);
	clk_release_all(hs_7nm->clks, hs_7nm->num_clks);

	return 0;
}

static int hs_7nm_phy_reset_init(struct udevice *dev, struct hs_7nm_phy_priv *hs_7nm)
{
	const struct hs_7nm_phy_cfg *cfg = hs_7nm->cfg;
	int num = cfg->num_resets;
	int i, ret;

	hs_7nm->num_resets = 0;
	hs_7nm->resets = devm_kcalloc(dev, num, sizeof(*hs_7nm->resets), GFP_KERNEL);
	if (!hs_7nm->resets)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		ret = reset_get_by_index(dev, i, &hs_7nm->resets[i]);
		if (ret < 0)
			goto reset_get_err;

		++hs_7nm->num_resets;
	}

	return 0;

reset_get_err:
	ret = reset_release_all(hs_7nm->resets, hs_7nm->num_resets);
	if (ret)
		debug("failed to disable all resets\n");

	return ret;
}

static int hs_7nm_phy_clk_init(struct udevice *dev, struct hs_7nm_phy_priv *hs_7nm)
{
	const struct hs_7nm_phy_cfg *cfg = hs_7nm->cfg;
	int num = cfg->num_clks;
	int i, ret;

	hs_7nm->num_clks = 0;
	hs_7nm->clks = devm_kcalloc(dev, num, sizeof(*hs_7nm->clks), GFP_KERNEL);
	if (!hs_7nm->clks)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		ret = clk_get_by_index(dev, i, &hs_7nm->clks[i]);
		if (ret < 0)
			goto clk_get_err;

		ret = clk_enable(&hs_7nm->clks[i]);
		if (ret && ret != -ENOSYS) {
			debug("failed to enable clock %d\n", i);
			clk_free(&hs_7nm->clks[i]);
			goto clk_get_err;
		}

		++hs_7nm->num_clks;
	}

	return 0;

clk_get_err:
	ret = clk_release_all(hs_7nm->clks, hs_7nm->num_clks);
	if (ret)
		debug("failed to disable all clocks\n");

	return ret;
}


static void hs_7nm_override_param_update_val(
			struct override_param_map map,
			s32 dt_val, struct phy_override_seq *seq_entry)
{
	int i;

	/*
	 * Param table for each param is in increasing order
	 * of dt values. We need to iterate over the list to
	 * select the entry that matches the dt value and pick
	 * up the corresponding register value.
	 */
	for (i = 0; i < map.table_size - 1; i++) {
		if (map.param_table[i].value == dt_val)
			break;
	}

	seq_entry->need_update = true;
	seq_entry->offset = map.reg_offset;
	seq_entry->mask = map.param_mask;
	seq_entry->value = map.param_table[i].reg_val << __ffs(map.param_mask);
}

static void hs_7nm_read_override_param_seq(struct udevice *dev)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(dev);
	ofnode dp = dev_ofnode(dev);
	s32 val;
	int ret, i;
	const struct override_param_map *map_cfg = hs_7nm->cfg->map_cfg;

	if (!map_cfg)
		return;

	for (i = 0; map_cfg[i].prop_name != NULL; i++) {
		ret = ofnode_read_s32(dp, map_cfg[i].prop_name, &val);
		if (ret)
			continue;

		hs_7nm_override_param_update_val(map_cfg[i], val,
					&hs_7nm->cfg->update_seq_cfg[i]);
		printf("Read param: %s dt_val: %d reg_val: 0x%x\n",
			map_cfg[i].prop_name, val, hs_7nm->cfg->update_seq_cfg[i].value);

	}
}

static int hs_7nm_phy_probe(struct udevice *dev)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(dev);
	int ret;

#ifdef DEBUG_HS_PHY
	debug_hs_7nm("%s: Entered function\n", __func__);
#endif

	hs_7nm->base = (void __iomem *)dev_read_addr(dev);
	if (IS_ERR(hs_7nm->base))
		return PTR_ERR(hs_7nm->base);

	hs_7nm->cfg = (struct hs_7nm_phy_cfg *)dev_get_driver_data(dev);
	
	ret = hs_7nm_phy_clk_init(dev, hs_7nm);
	if (ret)
		return ret;

	ret = hs_7nm_phy_reset_init(dev, hs_7nm);
	if (ret)
		return ret;

#ifdef DEBUG_HS_PHY
	debug_hs_7nm("%s: Acquired clks and resets\n", __func__);
#endif

	hs_7nm_read_override_param_seq(dev);

#ifdef DEBUG_HS_PHY
	debug_hs_7nm("%s: Exiting function with success\n", __func__);
#endif

	return 0;
}

static struct phy_ops hs_7nm_phy_ops = {
	.power_on = hs_7nm_phy_power_on,
	.power_off = hs_7nm_phy_power_off,
};

static const struct udevice_id hs_7nm_phy_ids[] = {
	{ .compatible	= "qcom,sm8150-usb-hs-phy", },
	{ .compatible	= "qcom,usb-snps-hs-5nm-phy", },
	{
		.compatible	= "qcom,usb-snps-hs-7nm-phy",
		.data		= (ulong)&sc7280_snps_7nm_phy,
	},
	{ .compatible	= "qcom,usb-snps-femto-v2-phy",	},
	{ }
};

U_BOOT_DRIVER(qcom_usb_hs_7nm) = {
	.name		= "qcom-usb-hs-7nm",
	.id		= UCLASS_PHY,
	.of_match	= hs_7nm_phy_ids,
	.ops		= &hs_7nm_phy_ops,
	.probe		= hs_7nm_phy_probe,
	.priv_auto	= sizeof(struct hs_7nm_phy_priv),
};
