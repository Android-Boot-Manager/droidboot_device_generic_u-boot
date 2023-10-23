// SPDX-License-Identifier: BSD-3-Clause AND GPL-2.0
/*
 * Clock and reset drivers for Qualcomm platforms Global Clock
 * Controller (GCC).
 *
 * (C) Copyright 2015 Mateusz Kulikowski <mateusz.kulikowski@gmail.com>
 * (C) Copyright 2020 Sartura Ltd. (reset driver)
 *     Author: Robert Marko <robert.marko@sartura.hr>
 * (C) Copyright 2022 Linaro Ltd. (reset driver)
 *     Author: Sumit Garg <sumit.garg@linaro.org>
 *
 * Based on Little Kernel driver, simplified
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <reset-uclass.h>

#include "clock-qcom.h"

#include "debugcc/debugcc.h"

/* CBCR register fields */
#define CBCR_BRANCH_ENABLE_BIT  BIT(0)
#define CBCR_BRANCH_OFF_BIT     BIT(31)

/* Enable clock controlled by CBC soft macro */
void clk_enable_cbc(phys_addr_t cbcr)
{
	setbits_le32(cbcr, CBCR_BRANCH_ENABLE_BIT);

	while (readl(cbcr) & CBCR_BRANCH_OFF_BIT)
		;
}

void gdsc_enable(phys_addr_t gdscr)
{
	uint32_t count;
	clrbits_le32(gdscr, GDSC_SW_COLLAPSE);
	for (count = 0; count < 1500; count++) {
		if (readl(gdscr) & GDSC_PWR_ON)
			break;
		udelay(1);
	}
	WARN(count == 1500, "WARNING: GDSC @ %#llx stuck at off\n", gdscr);
}

void clk_enable_gpll0(phys_addr_t base, const struct pll_vote_clk *gpll0)
{
	if (readl(base + gpll0->status) & gpll0->status_bit)
		return; /* clock already enabled */

	setbits_le32(base + gpll0->ena_vote, gpll0->vote_bit);

	while ((readl(base + gpll0->status) & gpll0->status_bit) == 0)
		;
}

#define BRANCH_ON_VAL (0)
#define BRANCH_NOC_FSM_ON_VAL BIT(29)
#define BRANCH_CHECK_MASK GENMASK(31, 28)

void clk_enable_vote_clk(phys_addr_t base, const struct vote_clk *vclk)
{
	u32 val;

	setbits_le32(base + vclk->ena_vote, vclk->vote_bit);
	do {
		val = readl(base + vclk->cbcr_reg);
		val &= BRANCH_CHECK_MASK;
	} while ((val != BRANCH_ON_VAL) && (val != BRANCH_NOC_FSM_ON_VAL));
}

#define APPS_CMD_RCGR_UPDATE BIT(0)

/* Update clock command via CMD_RCGR */
void clk_bcr_update(phys_addr_t apps_cmd_rcgr)
{
	u32 count;
	setbits_le32(apps_cmd_rcgr, APPS_CMD_RCGR_UPDATE);

	/* Wait for frequency to be updated. */
	for (count = 0; count < 50000; count++) {
		if (!(readl(apps_cmd_rcgr) & APPS_CMD_RCGR_UPDATE))
			break;
		udelay(1);
	}
	WARN(count == 50000, "WARNING: RCG @ %#llx [%#010x] stuck at off\n",
	     apps_cmd_rcgr, readl(apps_cmd_rcgr));
}

#define CFG_SRC_DIV_MASK	0b11111
#define CFG_SRC_SEL_SHIFT	8
#define CFG_SRC_SEL_MASK	(0x7 << CFG_SRC_SEL_SHIFT)
#define CFG_MODE_SHIFT		12
#define CFG_MODE_MASK		(0x3 << CFG_MODE_SHIFT)
#define CFG_MODE_DUAL_EDGE	(0x2 << CFG_MODE_SHIFT)
#define CFG_HW_CLK_CTRL_MASK	BIT(20)

/*
 * root set rate for clocks with half integer and MND divider
 * div should be pre-calculated ((div * 2) - 1)
 */
void clk_rcg_set_rate_mnd(phys_addr_t base, uint32_t cmd_rcgr,
			  int div, int m, int n, int source, u8 mnd_width)
{
	u32 cfg;
	/* M value for MND divider. */
	u32 m_val = m;
	u32 n_minus_m = n - m;
	/* NOT(N-M) value for MND divider. */
	u32 n_val = ~n_minus_m * !!(n);
	/* NOT 2D value for MND divider. */
	u32 d_val = ~(clamp_t(u32, n, m, n_minus_m));
	u32 mask = BIT(mnd_width) - 1;

	debug("m %#x n %#x d %#x div %#x mask %#x\n", m_val, n_val, d_val, div, mask);

	/* Program MND values */
	writel(m_val & mask, base + cmd_rcgr + RCG_M_REG);
	writel(n_val & mask, base + cmd_rcgr + RCG_N_REG);
	writel(d_val & mask, base + cmd_rcgr + RCG_D_REG);

	/* setup src select and divider */
	cfg  = readl(base + cmd_rcgr + RCG_CFG_REG);
	cfg &= ~(CFG_SRC_SEL_MASK | CFG_MODE_MASK | CFG_HW_CLK_CTRL_MASK);
	cfg |= source & CFG_SRC_SEL_MASK; /* Select clock source */

	if (div)
		cfg |= div & CFG_SRC_DIV_MASK;

	if (n && n != m)
		cfg |= CFG_MODE_DUAL_EDGE;

	writel(cfg, base + cmd_rcgr + RCG_CFG_REG); /* Write new clock configuration */

	/* Inform h/w to start using the new config. */
	clk_bcr_update(base + cmd_rcgr);
}

/* root set rate for clocks with half integer and mnd_width=0 */
void clk_rcg_set_rate(phys_addr_t base, uint32_t cmd_rcgr, int div,
		      int source)
{
	u32 cfg;

	/* setup src select and divider */
	cfg  = readl(base + cmd_rcgr + RCG_CFG_REG);
	cfg &= ~(CFG_SRC_SEL_MASK | CFG_MODE_MASK | CFG_HW_CLK_CTRL_MASK);
	cfg |= source & CFG_CLK_SRC_MASK; /* Select clock source */

	/*
	 * Set the divider; HW permits fraction dividers (+0.5), but
	 * for simplicity, we will support integers only
	 */
	if (div)
		cfg |= (2 * div - 1) & CFG_SRC_DIV_MASK;

	writel(cfg, base + cmd_rcgr + RCG_CFG_REG); /* Write new clock configuration */

	/* Inform h/w to start using the new config. */
	clk_bcr_update(base + cmd_rcgr);
}

const struct freq_tbl *qcom_find_freq(const struct freq_tbl *f, uint rate)
{
	if (!f)
		return NULL;

	if (!f->freq)
		return f;

	for (; f->freq; f++)
		if (rate <= f->freq)
			return f;

	/* Default to our fastest rate */
	return f - 1;
}

static int msm_clk_probe(struct udevice *dev)
{
	struct msm_clk_data *data = (struct msm_clk_data *)dev_get_driver_data(dev);
	struct msm_clk_priv *priv = dev_get_priv(dev);

	priv->base = dev_read_addr(dev);
	if (priv->base == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->data = data;

	return 0;
}

static ulong msm_clk_set_rate(struct clk *clk, ulong rate)
{
	struct msm_clk_data *data = (struct msm_clk_data *)dev_get_driver_data(clk->dev);

	if (data->set_rate)
		return data->set_rate(clk, rate);

	return 0;
}

static int msm_clk_enable(struct clk *clk)
{
	struct msm_clk_data *data = (struct msm_clk_data *)dev_get_driver_data(clk->dev);

	if (data->enable)
		return data->enable(clk);

	return 0;
}

static void dump_gplls(struct udevice *dev, phys_addr_t base) {
	static phys_addr_t gplls[] = {
		0x00100000,
		0x00101000,
		0x00102000,
		0x00103000,
		0x00176000,
		0x00174000,
		0x00113000,
		0x0011a000,
		0x0011b000,
		0x0011c000,
		0x0011d000,
		0x0014a000,
	};
	uint32_t i;
	bool locked;
	uint64_t l, a, xo_rate = 19200000;
	struct clk clk;
	int ret;
	u32 pll_branch = readl(0x00152018);

	ret = clk_get_by_name(dev, "xo_board", &clk);
	if (ret < 0) {
		ret = clk_get_by_name(dev, "xo-board", &clk);
		if (ret < 0)
			printf("Can't find XO clock, XO_BOARD rate may be wrong\n");
	}

	if (ret >= 0)
		xo_rate = clk_get_rate(&clk);

	printf("| GPLL   | LOCKED | GATE | XO_BOARD  |  PLL_L     | ALPHA          |\n");
	printf("+--------+--------+------+-----------+------------+----------------+\n");
	for (i = 0; i < ARRAY_SIZE(gplls); i++) {
		locked = !!(readl(gplls[i]) & BIT(31));
		l = readl(gplls[i] + 4) & (BIT(16)-1);
		a = readq(gplls[i] + 40) & (BIT(16)-1);
		printf("| GPLL%-2d | %-6s | %-4s | %9llu * (%#-9llx + %#-13llx  * 2 ** -40 ) / 1000000\n",
			i, locked ? "X" : "", pll_branch & BIT(i) ? "X" : "", xo_rate, l, a);
	}
}

static void dump_rcgs(void) {
	static phys_addr_t rcgs[] = {
		// 0x0010f018, // RB2
		// 0x0010f030,
		// 0x0010f05c,

		// RB5
		// 0x00175024, // GCC_UFS_CARD_AXI_CMD_RCGR
		// 0x0017506c, // GCC_UFS_CARD_ICE_CORE_CMD_RCGR
		// 0x00175084, // GCC_UFS_CARD_UNIPRO_CORE_CMD_RCGR
		// 0x001750a0, // GCC_UFS_CARD_PHY_AUX_CMD_RCGR
		// 0x00177024, // GCC_UFS_PHY_AXI_CMD_RCGR
		// 0x0017706c, // GCC_UFS_PHY_ICE_CORE_CMD_RCGR
		// 0x00177084, // GCC_UFS_PHY_UNIPRO_CORE_CMD_RCGR
		// 0x001770a0, // GCC_UFS_PHY_PHY_AUX_CMD_RCGR
		0x0011400c, // GCC_SDCC2_APPS_CMD_RCGR
		//0x001184D0

		// RB3
		//0x00118148
	};
	static const char * const rcg_names[] = {
		// "USB30_PRIM_MASTER", // RB2
		// "USB30_PRIM_MOCK_UTMI",
		// "USB3_PRIM_PHY_AUX",

		// RB5
		// "GCC_UFS_CARD_AXI_CMD_RCGR",
		// "GCC_UFS_CARD_ICE_CORE_CMD_RCGR",
		// "GCC_UFS_CARD_UNIPRO_CORE_CMD_RCGR",
		// "GCC_UFS_CARD_PHY_AUX_CMD_RCGR",
		// "GCC_UFS_PHY_AXI_CMD_RCGR",
		// "GCC_UFS_PHY_ICE_CORE_CMD_RCGR",
		// "GCC_UFS_PHY_UNIPRO_CORE_CMD_RCGR",
		// "GCC_UFS_PHY_PHY_AUX_CMD_RCGR",
		"GCC_SDCC2_APPS_CMD_RCGR",
		//"UART",
	};
	int i;
	uint32_t cmd;
	uint32_t cfg;
	uint32_t not_n_minus_m;
	uint32_t src, m, n, div;
	bool root_on, d_odd;
	printf("\nRCGs:\n");

	for (i = 0; i < ARRAY_SIZE(rcgs); i++) {
		cmd = readl(rcgs[i]);
		cfg = readl(rcgs[i] + 0x4);
		m = readl(rcgs[i] + 0x8);
		not_n_minus_m = readl(rcgs[i] + 0xc);

		root_on = !(cmd & BIT(31)); // ROOT_OFF
		src = (cfg >> 8) & 7;

		if (not_n_minus_m)
			n = (~not_n_minus_m & 0xffff) + m;
		else
			n = 0;

		div = ((cfg & 0b11111) + 1) / 2;
		d_odd = ((cfg & 0b11111) + 1) % 2 == 1;
		printf("%#010x %#010x %#010x %#010x %#010x\n", cmd, cfg, m, not_n_minus_m, readl(rcgs[i] + 0x10));
		printf("%-32s: %-1s src %d | input_freq * (%#x/%#x) * (1/%d%s)",
			rcg_names[i], root_on ? "X" : "", src, m ?: 1, n ?: 1, div, d_odd ? ".5" : "");
		printf("  [%#010X]\n", cmd);
	}

	printf("\n");
}

static void msm_dump_clks(struct udevice *dev)
{
	struct msm_clk_data *data = (struct msm_clk_data *)dev_get_driver_data(dev);
	struct msm_clk_priv *priv = dev_get_priv(dev);
	const struct gate_clk *sclk;
	const struct qcom_reset_map *rst;
	int val, i;

	if (!data->clks) {
		printf("No clocks\n");
		return;
	}

	for (i = 0; i < data->num_clks; i++) {
		sclk = &data->clks[i];
		if (!sclk->name)
			continue;
		printf("%-32s: ", sclk->name);
		val = readl(priv->base + sclk->reg) & sclk->en_val;
		printf("%s\n", val ? "ON" : "");
	}

	for (i = 0; i < data->num_resets; i++) {
		rst = &data->resets[i];
		printf("%#05x: ", rst->reg);
		val = readl(priv->base + rst->reg);
		printf("%s\n", val > 0 ? "ON" : "");
	}

	dump_gplls(dev, priv->base);
	dump_rcgs();
}

static void msm_debug_clks(struct udevice *dev, int argc, char *const argv[])
{
	if (!IS_ENABLED(CONFIG_CLK_QCOM_DEBUG)) {
		printf("Enable CONFIG_CLK_QCOM_DEBUG to debug GCC\n");
		return;
	}

	qcom_debugcc_run(argc, argv);
}

static struct clk_ops msm_clk_ops = {
	.set_rate = msm_clk_set_rate,
	.enable = msm_clk_enable,
	.dump_clks = msm_dump_clks,
	.debug_clks = msm_debug_clks,
};

U_BOOT_DRIVER(qcom_clk) = {
	.name		= "qcom_clk",
	.id		= UCLASS_CLK,
	.ops		= &msm_clk_ops,
	.priv_auto	= sizeof(struct msm_clk_priv),
	.probe		= msm_clk_probe,
};

int qcom_cc_bind(struct udevice *parent)
{
	struct msm_clk_data *data = (struct msm_clk_data *)dev_get_driver_data(parent);
	struct udevice *clkdev, *rstdev;
	struct driver *drv;
	int ret;

	/* Get a handle to the common clk handler */
	drv = lists_driver_lookup_name("qcom_clk");
	if (!drv)
		return -ENOENT;

	/* Register the clock controller */
	ret = device_bind_with_driver_data(parent, drv, "qcom_clk", (ulong)data,
					   dev_ofnode(parent), &clkdev);
	if (ret)
		return ret;

	/* Bail out early if resets are not specified for this platform */
	if (!data->resets)
		return ret;

	/* Get a handle to the common reset handler */
	drv = lists_driver_lookup_name("qcom_reset");
	if (!drv)
		return -ENOENT;

	/* Register the reset controller */
	ret = device_bind_with_driver_data(parent, drv, "qcom_reset", (ulong)data,
					   dev_ofnode(parent), &rstdev);
	if (ret)
		device_unbind(clkdev);

	return ret;
}

static int qcom_reset_set(struct reset_ctl *rst, bool assert)
{
	struct msm_clk_data *data = (struct msm_clk_data *)dev_get_driver_data(rst->dev);
	void __iomem *base = dev_get_priv(rst->dev);
	const struct qcom_reset_map *map;
	u32 value;

	map = &data->resets[rst->id];

	value = readl(base + map->reg);

	if (assert)
		value |= BIT(map->bit);
	else
		value &= ~BIT(map->bit);

	writel(value, base + map->reg);

	return 0;
}

static int qcom_reset_assert(struct reset_ctl *rst)
{
	return qcom_reset_set(rst, true);
}

static int qcom_reset_deassert(struct reset_ctl *rst)
{
	return qcom_reset_set(rst, false);
}

static const struct reset_ops qcom_reset_ops = {
	.rst_assert = qcom_reset_assert,
	.rst_deassert = qcom_reset_deassert,
};

static int qcom_reset_probe(struct udevice *dev)
{
	/* Set our priv pointer to the base address */
	dev_set_priv(dev, (void *)dev_read_addr(dev));

	return 0;
}

U_BOOT_DRIVER(qcom_reset) = {
	.name = "qcom_reset",
	.id = UCLASS_RESET,
	.ops = &qcom_reset_ops,
	.probe = qcom_reset_probe,
};
