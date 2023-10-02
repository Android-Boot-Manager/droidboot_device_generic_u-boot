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
	struct clk_bulk clks;

	/* resets to be requested */
	struct reset_ctl_bulk resets;

	struct hs_7nm_phy_cfg *cfg;
};

static inline void hs_7nm_write_mask(void __iomem *base, u32 offset,
						u32 mask, u32 val)
{
	u32 reg;

	reg = readl_relaxed(base + offset);

	reg &= ~mask;
	reg |= val & mask;
	writel_relaxed(reg, base + offset);

	/* Ensure above write is completed */
	readl_relaxed(base + offset);
}

static int hs_7nm_usb_init(struct phy *phy)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(phy->dev);

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

static int hs_7nm_phy_power_on(struct phy *phy)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(phy->dev);
	int ret;

	clk_enable_bulk(&hs_7nm->clks);

	ret = reset_deassert_bulk(&hs_7nm->resets);
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

	reset_assert_bulk(&hs_7nm->resets);
	clk_disable_bulk(&hs_7nm->clks);

	return 0;
}

static int hs_7nm_phy_clk_init(struct udevice *dev, struct hs_7nm_phy_priv *hs_7nm)
{
	int ret;

	ret = clk_get_bulk(dev, &hs_7nm->clks);
	/* We may have no clocks */
	if (ret == -ENOENT) {
		debug("%s: no clocks\n", __func__);
		return 0;
	}
	if (ret < 0) {
		printf("%s: Failed to get clocks %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int hs_7nm_phy_probe(struct udevice *dev)
{
	struct hs_7nm_phy_priv *hs_7nm = dev_get_priv(dev);
	int ret;

	hs_7nm->base = (void __iomem *)dev_read_addr(dev);
	if (IS_ERR(hs_7nm->base))
		return PTR_ERR(hs_7nm->base);

	hs_7nm->cfg = (struct hs_7nm_phy_cfg *)dev_get_driver_data(dev);

	ret = hs_7nm_phy_clk_init(dev, hs_7nm);
	if (ret) {
		printf("%s: hs_7nm_phy_clk_init %d\n", __func__, ret);
		return ret;
	}

	ret = reset_get_bulk(dev, &hs_7nm->resets);
	if (ret < 0) {
		printf("failed to get resets, ret = %d\n", ret);
		return ret;
	}

	clk_enable_bulk(&hs_7nm->clks);
	reset_deassert_bulk(&hs_7nm->resets);

	return 0;
}

static struct phy_ops hs_7nm_phy_ops = {
	.power_on = hs_7nm_phy_power_on,
	.power_off = hs_7nm_phy_power_off,
};

static const struct udevice_id hs_7nm_phy_ids[] = {
	{ .compatible	= "qcom,sm8150-usb-hs-phy", },
	{ .compatible	= "qcom,usb-snps-hs-5nm-phy", },
	{ .compatible	= "qcom,usb-snps-hs-7nm-phy", },
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
