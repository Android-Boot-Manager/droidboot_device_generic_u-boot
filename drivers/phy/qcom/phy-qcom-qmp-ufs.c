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
#include <linux/ioport.h>

#include <dt-bindings/clock/qcom,gcc-sm6115.h>

#include "phy-qcom-qmp.h"
#include "phy-qcom-qmp-pcs-ufs-v2.h"
#include "phy-qcom-qmp-pcs-ufs-v3.h"
#include "phy-qcom-qmp-pcs-ufs-v4.h"
#include "phy-qcom-qmp-pcs-ufs-v5.h"
#include "phy-qcom-qmp-pcs-ufs-v6.h"

#include "phy-qcom-qmp-qserdes-com-v4.h"
#include "phy-qcom-qmp-qserdes-txrx-v4.h"
#include "phy-qcom-qmp-qserdes-txrx-ufs-v6.h"

/* QPHY_SW_RESET bit */
#define SW_RESET				BIT(0)
/* QPHY_POWER_DOWN_CONTROL */
#define SW_PWRDN				BIT(0)
/* QPHY_START_CONTROL bits */
#define SERDES_START				BIT(0)
#define PCS_START				BIT(1)
/* QPHY_PCS_READY_STATUS bit */
#define PCS_READY				BIT(0)

#define PHY_INIT_COMPLETE_TIMEOUT		(200 * 10000)

struct qmp_ufs_init_tbl {
	unsigned int offset;
	unsigned int val;
	/*
	 * mask of lanes for which this register is written
	 * for cases when second lane needs different values
	 */
	u8 lane_mask;
};

#define QMP_PHY_INIT_CFG(o, v)		\
	{				\
		.offset = o,		\
		.val = v,		\
		.lane_mask = 0xff,	\
	}

#define QMP_PHY_INIT_CFG_LANE(o, v, l)	\
	{				\
		.offset = o,		\
		.val = v,		\
		.lane_mask = l,		\
	}

enum ufs_hs_gear_tag {
	UFS_HS_DONT_CHANGE,	/* Don't change Gear */
	UFS_HS_G1,		/* HS Gear 1 (default for reset) */
	UFS_HS_G2,		/* HS Gear 2 */
	UFS_HS_G3,		/* HS Gear 3 */
	UFS_HS_G4,		/* HS Gear 4 */
	UFS_HS_G5		/* HS Gear 5 */
};

/* set of registers with offsets different per-PHY */
enum qphy_reg_layout {
	/* PCS registers */
	QPHY_SW_RESET,
	QPHY_START_CTRL,
	QPHY_PCS_READY_STATUS,
	QPHY_PCS_POWER_DOWN_CONTROL,
	/* Keep last to ensure regs_layout arrays are properly initialized */
	QPHY_LAYOUT_SIZE
};

static const unsigned int ufsphy_v2_regs_layout[QPHY_LAYOUT_SIZE] = {
	[QPHY_START_CTRL]		= QPHY_V2_PCS_UFS_PHY_START,
	[QPHY_PCS_READY_STATUS]		= QPHY_V2_PCS_UFS_READY_STATUS,
	[QPHY_PCS_POWER_DOWN_CONTROL]	= QPHY_V2_PCS_UFS_POWER_DOWN_CONTROL,
};

static const unsigned int ufsphy_v3_regs_layout[QPHY_LAYOUT_SIZE] = {
	[QPHY_START_CTRL]		= QPHY_V3_PCS_UFS_PHY_START,
	[QPHY_PCS_READY_STATUS]		= QPHY_V3_PCS_UFS_READY_STATUS,
	[QPHY_PCS_POWER_DOWN_CONTROL]	= QPHY_V3_PCS_UFS_POWER_DOWN_CONTROL,
};

static const unsigned int ufsphy_v4_regs_layout[QPHY_LAYOUT_SIZE] = {
	[QPHY_START_CTRL]		= QPHY_V4_PCS_UFS_PHY_START,
	[QPHY_PCS_READY_STATUS]		= QPHY_V4_PCS_UFS_READY_STATUS,
	[QPHY_SW_RESET]			= QPHY_V4_PCS_UFS_SW_RESET,
	[QPHY_PCS_POWER_DOWN_CONTROL]	= QPHY_V4_PCS_UFS_POWER_DOWN_CONTROL,
};

static const struct qmp_ufs_init_tbl sdm845_ufsphy_serdes[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_SYS_CLK_CTRL, 0x02),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_BIAS_EN_CLKBUFLR_EN, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_BG_TIMER, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_PLL_IVCO, 0x07),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_CMN_CONFIG, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_SYSCLK_EN_SEL, 0xd5),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_RESETSM_CNTRL, 0x20),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_CLK_SELECT, 0x30),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_HSCLK_SEL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_LOCK_CMP_EN, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE_CTRL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_CORE_CLK_EN, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE_MAP, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_SVS_MODE_CLK_SEL, 0x05),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE_INITVAL1, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE_INITVAL2, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_DEC_START_MODE0, 0x82),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_CP_CTRL_MODE0, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_PLL_RCTRL_MODE0, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_PLL_CCTRL_MODE0, 0x36),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_INTEGLOOP_GAIN0_MODE0, 0x3f),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_INTEGLOOP_GAIN1_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE1_MODE0, 0xda),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE2_MODE0, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_LOCK_CMP1_MODE0, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_LOCK_CMP2_MODE0, 0x0c),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_DEC_START_MODE1, 0x98),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_CP_CTRL_MODE1, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_PLL_RCTRL_MODE1, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_PLL_CCTRL_MODE1, 0x36),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_INTEGLOOP_GAIN0_MODE1, 0x3f),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_INTEGLOOP_GAIN1_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE1_MODE1, 0xc1),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE2_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_LOCK_CMP1_MODE1, 0x32),
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_LOCK_CMP2_MODE1, 0x0f),
};

static const struct qmp_ufs_init_tbl sdm845_ufsphy_hs_b_serdes[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_COM_VCO_TUNE_MAP, 0x44),
};

static const struct qmp_ufs_init_tbl sdm845_ufsphy_tx[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_LANE_MODE_1, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RES_CODE_LANE_OFFSET_TX, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RES_CODE_LANE_OFFSET_RX, 0x07),
};

static const struct qmp_ufs_init_tbl sdm845_ufsphy_rx[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_LVL, 0x24),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_CNTRL, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_DEGLITCH_CNTRL, 0x1e),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_INTERFACE_MODE, 0x40),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_FO_GAIN, 0x0b),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_TERM_BW, 0x5b),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL2, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL3, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL4, 0x1b),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SVS_SO_GAIN_HALF, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SVS_SO_GAIN_QUARTER, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SVS_SO_GAIN, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x4b),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_PI_CONTROLS, 0x81),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_COUNT_LOW, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_MODE_00, 0x59),
};

static const struct qmp_ufs_init_tbl sdm845_ufsphy_pcs[] = {
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_RX_SIGDET_CTRL2, 0x6e),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_TX_LARGE_AMP_DRV_LVL, 0x0a),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_TX_SMALL_AMP_DRV_LVL, 0x02),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_RX_SYM_RESYNC_CTRL, 0x03),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_TX_MID_TERM_CTRL1, 0x43),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_RX_SIGDET_CTRL1, 0x0f),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_RX_MIN_HIBERN8_TIME, 0x9a),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_UFS_MULTI_LANE_CTRL1, 0x02),
};

static const struct qmp_ufs_init_tbl sm6115_ufsphy_serdes[] = {
	QMP_PHY_INIT_CFG(QSERDES_COM_CMN_CONFIG, 0x0e),
	QMP_PHY_INIT_CFG(QSERDES_COM_SYSCLK_EN_SEL, 0x14),
	QMP_PHY_INIT_CFG(QSERDES_COM_CLK_SELECT, 0x30),
	QMP_PHY_INIT_CFG(QSERDES_COM_SYS_CLK_CTRL, 0x02),
	QMP_PHY_INIT_CFG(QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x08),
	QMP_PHY_INIT_CFG(QSERDES_COM_BG_TIMER, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_COM_HSCLK_SEL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_CORECLK_DIV, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_COM_CORECLK_DIV_MODE1, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP_EN, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_CTRL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_RESETSM_CNTRL, 0x20),
	QMP_PHY_INIT_CFG(QSERDES_COM_CORE_CLK_EN, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP_CFG, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_TIMER1, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_TIMER2, 0x3f),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_MAP, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_COM_SVS_MODE_CLK_SEL, 0x05),
	QMP_PHY_INIT_CFG(QSERDES_COM_DEC_START_MODE0, 0x82),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START1_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START2_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START3_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_CP_CTRL_MODE0, 0x0b),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_RCTRL_MODE0, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_CCTRL_MODE0, 0x28),
	QMP_PHY_INIT_CFG(QSERDES_COM_INTEGLOOP_GAIN0_MODE0, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_COM_INTEGLOOP_GAIN1_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE1_MODE0, 0x28),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE2_MODE0, 0x02),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP1_MODE0, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP2_MODE0, 0x0c),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP3_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_DEC_START_MODE1, 0x98),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START1_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START2_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_DIV_FRAC_START3_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_CP_CTRL_MODE1, 0x0b),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_RCTRL_MODE1, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_CCTRL_MODE1, 0x28),
	QMP_PHY_INIT_CFG(QSERDES_COM_INTEGLOOP_GAIN0_MODE1, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_COM_INTEGLOOP_GAIN1_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE1_MODE1, 0xd6),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE2_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP1_MODE1, 0x32),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP2_MODE1, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_COM_LOCK_CMP3_MODE1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_COM_PLL_IVCO, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_COM_BG_TRIM, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_INITVAL1, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_INITVAL2, 0x00),
};

static const struct qmp_ufs_init_tbl sm6115_ufsphy_hs_b_serdes[] = {
	QMP_PHY_INIT_CFG(QSERDES_COM_VCO_TUNE_MAP, 0x44),
};

static const struct qmp_ufs_init_tbl sm6115_ufsphy_tx[] = {
	QMP_PHY_INIT_CFG(QSERDES_TX_HIGHZ_TRANSCEIVEREN_BIAS_DRVR_EN, 0x45),
	QMP_PHY_INIT_CFG(QSERDES_TX_LANE_MODE, 0x06),
};

static const struct qmp_ufs_init_tbl sm6115_ufsphy_rx[] = {
	QMP_PHY_INIT_CFG(QSERDES_RX_SIGDET_LVL, 0x24),
	QMP_PHY_INIT_CFG(QSERDES_RX_SIGDET_CNTRL, 0x0F),
	QMP_PHY_INIT_CFG(QSERDES_RX_RX_INTERFACE_MODE, 0x40),
	QMP_PHY_INIT_CFG(QSERDES_RX_SIGDET_DEGLITCH_CNTRL, 0x1E),
	QMP_PHY_INIT_CFG(QSERDES_RX_UCDR_FASTLOCK_FO_GAIN, 0x0B),
	QMP_PHY_INIT_CFG(QSERDES_RX_RX_TERM_BW, 0x5B),
	QMP_PHY_INIT_CFG(QSERDES_RX_RX_EQ_GAIN1_LSB, 0xFF),
	QMP_PHY_INIT_CFG(QSERDES_RX_RX_EQ_GAIN1_MSB, 0x3F),
	QMP_PHY_INIT_CFG(QSERDES_RX_RX_EQ_GAIN2_LSB, 0xFF),
	QMP_PHY_INIT_CFG(QSERDES_RX_RX_EQ_GAIN2_MSB, 0x3F),
	QMP_PHY_INIT_CFG(QSERDES_RX_RX_EQU_ADAPTOR_CNTRL2, 0x0D),
	QMP_PHY_INIT_CFG(QSERDES_RX_UCDR_SVS_SO_GAIN_HALF, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_RX_UCDR_SVS_SO_GAIN_QUARTER, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_RX_UCDR_SVS_SO_GAIN, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x5B),
};

static const struct qmp_ufs_init_tbl sm6115_ufsphy_pcs[] = {
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_RX_PWM_GEAR_BAND, 0x15),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_RX_SIGDET_CTRL2, 0x6d),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_TX_LARGE_AMP_DRV_LVL, 0x0f),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_TX_SMALL_AMP_DRV_LVL, 0x02),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_RX_MIN_STALL_NOCONFIG_TIME_CAP, 0x28),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_RX_SYM_RESYNC_CTRL, 0x03),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_TX_LARGE_AMP_POST_EMP_LVL, 0x12),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_TX_SMALL_AMP_POST_EMP_LVL, 0x0f),
	QMP_PHY_INIT_CFG(QPHY_V2_PCS_UFS_RX_MIN_HIBERN8_TIME, 0x9a), /* 8 us */
};

static const struct qmp_ufs_init_tbl sm8150_ufsphy_serdes[] = {
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_SYSCLK_EN_SEL, 0xd9),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_HSCLK_SEL, 0x11),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_HSCLK_HS_SWITCH_SEL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_LOCK_CMP_EN, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_VCO_TUNE_MAP, 0x02),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_PLL_IVCO, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_VCO_TUNE_INITVAL2, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_BIN_VCOCAL_HSCLK_SEL, 0x11),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_DEC_START_MODE0, 0x82),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_CP_CTRL_MODE0, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_PLL_RCTRL_MODE0, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_PLL_CCTRL_MODE0, 0x36),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_LOCK_CMP1_MODE0, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_LOCK_CMP2_MODE0, 0x0c),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_BIN_VCOCAL_CMP_CODE1_MODE0, 0xac),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_BIN_VCOCAL_CMP_CODE2_MODE0, 0x1e),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_DEC_START_MODE1, 0x98),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_CP_CTRL_MODE1, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_PLL_RCTRL_MODE1, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_PLL_CCTRL_MODE1, 0x36),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_LOCK_CMP1_MODE1, 0x32),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_LOCK_CMP2_MODE1, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_BIN_VCOCAL_CMP_CODE1_MODE1, 0xdd),
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_BIN_VCOCAL_CMP_CODE2_MODE1, 0x23),
};

static const struct qmp_ufs_init_tbl sm8150_ufsphy_hs_b_serdes[] = {
	QMP_PHY_INIT_CFG(QSERDES_V4_COM_VCO_TUNE_MAP, 0x06),
};

static const struct qmp_ufs_init_tbl sm8150_ufsphy_tx[] = {
	QMP_PHY_INIT_CFG(QSERDES_V4_TX_PWM_GEAR_1_DIVIDER_BAND0_1, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V4_TX_PWM_GEAR_2_DIVIDER_BAND0_1, 0x03),
	QMP_PHY_INIT_CFG(QSERDES_V4_TX_PWM_GEAR_3_DIVIDER_BAND0_1, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V4_TX_PWM_GEAR_4_DIVIDER_BAND0_1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V4_TX_LANE_MODE_1, 0x05),
	QMP_PHY_INIT_CFG(QSERDES_V4_TX_TRAN_DRVR_EMP_EN, 0x0c),
};

static const struct qmp_ufs_init_tbl sm8150_ufsphy_rx[] = {
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_SIGDET_LVL, 0x24),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_SIGDET_CNTRL, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_SIGDET_DEGLITCH_CNTRL, 0x1e),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_BAND, 0x18),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_FASTLOCK_FO_GAIN, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x4b),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_PI_CONTROLS, 0xf1),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_FASTLOCK_COUNT_LOW, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_PI_CTRL2, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_FO_GAIN, 0x0c),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_SO_GAIN, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_TERM_BW, 0x1b),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQU_ADAPTOR_CNTRL2, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQU_ADAPTOR_CNTRL3, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQU_ADAPTOR_CNTRL4, 0x1d),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_OFFSET_ADAPTOR_CNTRL2, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_IDAC_MEASURE_TIME, 0x10),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_IDAC_TSETTLE_LOW, 0xc0),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_IDAC_TSETTLE_HIGH, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_LOW, 0x36),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH, 0x36),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH2, 0xf6),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH3, 0x3b),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH4, 0x3d),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_LOW, 0xe0),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_HIGH, 0xc8),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_HIGH2, 0xc8),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_HIGH3, 0x3b),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_HIGH4, 0xb1),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_10_LOW, 0xe0),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_10_HIGH, 0xc8),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_10_HIGH2, 0xc8),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_10_HIGH3, 0x3b),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_10_HIGH4, 0xb1),
};

static const struct qmp_ufs_init_tbl sm8150_ufsphy_pcs[] = {
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_RX_SIGDET_CTRL2, 0x6d),
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_TX_LARGE_AMP_DRV_LVL, 0x0a),
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_TX_SMALL_AMP_DRV_LVL, 0x02),
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_TX_MID_TERM_CTRL1, 0x43),
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_DEBUG_BUS_CLKSEL, 0x1f),
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_RX_MIN_HIBERN8_TIME, 0xff),
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_MULTI_LANE_CTRL1, 0x02),
};

static const struct qmp_ufs_init_tbl sm8150_ufsphy_hs_g4_pcs[] = {
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_TX_LARGE_AMP_DRV_LVL, 0x10),
	QMP_PHY_INIT_CFG(QPHY_V4_PCS_UFS_BIST_FIXED_PAT_CTRL, 0x0a),
};

static const struct qmp_ufs_init_tbl sm8250_ufsphy_hs_g4_tx[] = {
	QMP_PHY_INIT_CFG(QSERDES_V4_TX_LANE_MODE_1, 0xe5),
};

static const struct qmp_ufs_init_tbl sm8250_ufsphy_hs_g4_rx[] = {
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x5a),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_PI_CTRL2, 0x81),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_UCDR_FO_GAIN, 0x0e),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_TERM_BW, 0x6f),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQU_ADAPTOR_CNTRL1, 0x04),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQU_ADAPTOR_CNTRL2, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQU_ADAPTOR_CNTRL3, 0x09),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQU_ADAPTOR_CNTRL4, 0x07),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1, 0x17),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_IDAC_MEASURE_TIME, 0x20),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_IDAC_TSETTLE_LOW, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_IDAC_TSETTLE_HIGH, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_LOW, 0x3f),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH2, 0xff),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH3, 0x7f),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_00_HIGH4, 0x2c),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_LOW, 0x6d),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_HIGH, 0x6d),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_HIGH2, 0xed),
	QMP_PHY_INIT_CFG(QSERDES_V4_RX_RX_MODE_01_HIGH4, 0x3c),
};

struct qmp_ufs_offsets {
	u16 serdes;
	u16 pcs;
	u16 tx;
	u16 rx;
	/* for PHYs with >= 2 lanes */
	u16 tx2;
	u16 rx2;
};

struct qmp_ufs_cfg_tbls {
	/* Init sequence for PHY blocks - serdes, tx, rx, pcs */
	const struct qmp_ufs_init_tbl *serdes;
	int serdes_num;
	const struct qmp_ufs_init_tbl *tx;
	int tx_num;
	const struct qmp_ufs_init_tbl *rx;
	int rx_num;
	const struct qmp_ufs_init_tbl *pcs;
	int pcs_num;
};

/* struct qmp_ufs_cfg - per-PHY initialization config */
struct qmp_ufs_cfg {
	int lanes;

	const struct qmp_ufs_offsets *offsets;

	/* Main init sequence for PHY blocks - serdes, tx, rx, pcs */
	const struct qmp_ufs_cfg_tbls tbls;
	/* Additional sequence for HS Series B */
	const struct qmp_ufs_cfg_tbls tbls_hs_b;
	/* Additional sequence for HS G4 */
	const struct qmp_ufs_cfg_tbls tbls_hs_g4;

	/* clock ids to be requested */
	const char * const *clk_list;
	int num_clks;
	/* regulators to be requested */
	const char * const *vreg_list;
	int num_vregs;
	/* resets to be requested */
	const char * const *reset_list;
	int num_resets;

	/* array of registers with different offsets */
	const unsigned int *regs;

	/* true, if PCS block has no separate SW_RESET register */
	bool no_pcs_sw_reset;
};

struct qmp_ufs_priv {
	struct phy *phy;

	void __iomem *serdes;
	void __iomem *pcs;
	void __iomem *pcs_misc;
	void __iomem *tx;
	void __iomem *rx;
	void __iomem *tx2;
	void __iomem *rx2;

	struct clk *clks;
	unsigned int clk_count;

	struct reset_ctl *resets;
	unsigned int reset_count;

	const struct qmp_ufs_cfg *cfg;

	struct udevice *dev;

	u32 mode;
	u32 submode;
};

static inline void qphy_setbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg |= val;
	writel(reg, base + offset);

	/* ensure that above write is through */
	readl(base + offset);
}

static inline void qphy_clrbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg &= ~val;
	writel(reg, base + offset);

	/* ensure that above write is through */
	readl(base + offset);
}

/* list of clocks required by phy */
static const char * const sdm845_ufs_phy_clk_l[] = {
	"ref", "ref_aux",
};

/* list of regulators */
static const char * const qmp_ufs_vreg_l[] = {
	"vdda-phy", "vdda-pll",
};

/* list of resets */
static const char * const qmp_ufs_reset_l[] = {
	"ufsphy",
};

static const struct qmp_ufs_offsets qmp_ufs_offsets = {
	.serdes		= 0,
	.pcs		= 0xc00,
	.tx		= 0x400,
	.rx		= 0x600,
	.tx2		= 0x800,
	.rx2		= 0xa00,
};

static const struct qmp_ufs_cfg sdm845_ufsphy_cfg = {
	.lanes			= 2,

	.tbls = {
		.serdes		= sdm845_ufsphy_serdes,
		.serdes_num	= ARRAY_SIZE(sdm845_ufsphy_serdes),
		.tx		= sdm845_ufsphy_tx,
		.tx_num		= ARRAY_SIZE(sdm845_ufsphy_tx),
		.rx		= sdm845_ufsphy_rx,
		.rx_num		= ARRAY_SIZE(sdm845_ufsphy_rx),
		.pcs		= sdm845_ufsphy_pcs,
		.pcs_num	= ARRAY_SIZE(sdm845_ufsphy_pcs),
	},
	.tbls_hs_b = {
		.serdes		= sdm845_ufsphy_hs_b_serdes,
		.serdes_num	= ARRAY_SIZE(sdm845_ufsphy_hs_b_serdes),
	},
	.clk_list		= sdm845_ufs_phy_clk_l,
	.num_clks		= ARRAY_SIZE(sdm845_ufs_phy_clk_l),
	.vreg_list		= qmp_ufs_vreg_l,
	.num_vregs		= ARRAY_SIZE(qmp_ufs_vreg_l),
	.regs			= ufsphy_v3_regs_layout,

	.no_pcs_sw_reset	= true,
};

static const struct qmp_ufs_cfg sm6115_ufsphy_cfg = {
	.lanes			= 1,

	.offsets		= &qmp_ufs_offsets,

	.tbls = {
		.serdes		= sm6115_ufsphy_serdes,
		.serdes_num	= ARRAY_SIZE(sm6115_ufsphy_serdes),
		.tx		= sm6115_ufsphy_tx,
		.tx_num		= ARRAY_SIZE(sm6115_ufsphy_tx),
		.rx		= sm6115_ufsphy_rx,
		.rx_num		= ARRAY_SIZE(sm6115_ufsphy_rx),
		.pcs		= sm6115_ufsphy_pcs,
		.pcs_num	= ARRAY_SIZE(sm6115_ufsphy_pcs),
	},
	.tbls_hs_b = {
		.serdes		= sm6115_ufsphy_hs_b_serdes,
		.serdes_num	= ARRAY_SIZE(sm6115_ufsphy_hs_b_serdes),
	},
	.clk_list		= sdm845_ufs_phy_clk_l,
	.num_clks		= ARRAY_SIZE(sdm845_ufs_phy_clk_l),
	.vreg_list		= qmp_ufs_vreg_l,
	.num_vregs		= ARRAY_SIZE(qmp_ufs_vreg_l),
	.reset_list		= qmp_ufs_reset_l,
	.num_resets		= ARRAY_SIZE(qmp_ufs_reset_l),
	.regs			= ufsphy_v2_regs_layout,

	.no_pcs_sw_reset	= true,
};

static const struct qmp_ufs_cfg sm8250_ufsphy_cfg = {
	.lanes			= 2,

	.tbls = {
		.serdes		= sm8150_ufsphy_serdes,
		.serdes_num	= ARRAY_SIZE(sm8150_ufsphy_serdes),
		.tx		= sm8150_ufsphy_tx,
		.tx_num		= ARRAY_SIZE(sm8150_ufsphy_tx),
		.rx		= sm8150_ufsphy_rx,
		.rx_num		= ARRAY_SIZE(sm8150_ufsphy_rx),
		.pcs		= sm8150_ufsphy_pcs,
		.pcs_num	= ARRAY_SIZE(sm8150_ufsphy_pcs),
	},
	.tbls_hs_b = {
		.serdes		= sm8150_ufsphy_hs_b_serdes,
		.serdes_num	= ARRAY_SIZE(sm8150_ufsphy_hs_b_serdes),
	},
	.tbls_hs_g4 = {
		.tx		= sm8250_ufsphy_hs_g4_tx,
		.tx_num		= ARRAY_SIZE(sm8250_ufsphy_hs_g4_tx),
		.rx		= sm8250_ufsphy_hs_g4_rx,
		.rx_num		= ARRAY_SIZE(sm8250_ufsphy_hs_g4_rx),
		.pcs		= sm8150_ufsphy_hs_g4_pcs,
		.pcs_num	= ARRAY_SIZE(sm8150_ufsphy_hs_g4_pcs),
	},
	.clk_list		= sdm845_ufs_phy_clk_l,
	.num_clks		= ARRAY_SIZE(sdm845_ufs_phy_clk_l),
	.vreg_list		= qmp_ufs_vreg_l,
	.num_vregs		= ARRAY_SIZE(qmp_ufs_vreg_l),
	.reset_list		= qmp_ufs_reset_l,
	.num_resets		= ARRAY_SIZE(qmp_ufs_reset_l),
	.regs			= ufsphy_v4_regs_layout,

	.no_pcs_sw_reset	= false,
};

static void qmp_ufs_configure_lane(void __iomem *base,
					const struct qmp_ufs_init_tbl tbl[],
					int num,
					u8 lane_mask)
{
	int i;
	const struct qmp_ufs_init_tbl *t = tbl;

	if (!t)
		return;

	for (i = 0; i < num; i++, t++) {
		if (!(t->lane_mask & lane_mask))
			continue;

		writel(t->val, base + t->offset);
	}
}

static void qmp_ufs_configure(void __iomem *base,
				   const struct qmp_ufs_init_tbl tbl[],
				   int num)
{
	qmp_ufs_configure_lane(base, tbl, num, 0xff);
}

static void qmp_ufs_serdes_init(struct qmp_ufs_priv *qmp, const struct qmp_ufs_cfg_tbls *tbls)
{
	void __iomem *serdes = qmp->serdes;

	qmp_ufs_configure(serdes, tbls->serdes, tbls->serdes_num);
}

static void qmp_ufs_lanes_init(struct qmp_ufs_priv *qmp, const struct qmp_ufs_cfg_tbls *tbls)
{
	const struct qmp_ufs_cfg *cfg = qmp->cfg;
	void __iomem *tx = qmp->tx;
	void __iomem *rx = qmp->rx;

	qmp_ufs_configure_lane(tx, tbls->tx, tbls->tx_num, 1);
	qmp_ufs_configure_lane(rx, tbls->rx, tbls->rx_num, 1);

	if (cfg->lanes >= 2) {
		qmp_ufs_configure_lane(qmp->tx2, tbls->tx, tbls->tx_num, 2);
		qmp_ufs_configure_lane(qmp->rx2, tbls->rx, tbls->rx_num, 2);
	}
}

static void qmp_ufs_pcs_init(struct qmp_ufs_priv *qmp, const struct qmp_ufs_cfg_tbls *tbls)
{
	void __iomem *pcs = qmp->pcs;

	qmp_ufs_configure(pcs, tbls->pcs, tbls->pcs_num);
}

static void qmp_ufs_init_registers(struct qmp_ufs_priv *qmp, const struct qmp_ufs_cfg *cfg)
{
	/* We support 'PHY_MODE_UFS_HS_B' mode & 'UFS_HS_G3' submode for now. */
	qmp_ufs_serdes_init(qmp, &cfg->tbls);
	qmp_ufs_serdes_init(qmp, &cfg->tbls_hs_b);
	qmp_ufs_lanes_init(qmp, &cfg->tbls);
	qmp_ufs_pcs_init(qmp, &cfg->tbls);
}

static int qmp_ufs_do_reset(struct qmp_ufs_priv *qmp)
{
	int i, ret;

	for (i = 0; i < qmp->reset_count; i++) {
		ret = reset_assert(&qmp->resets[i]);
		if (ret)
			return ret;
	}

	udelay(10);

	for (i = 0; i < qmp->reset_count; i++) {
		ret = reset_deassert(&qmp->resets[i]);
		if (ret)
			return ret;
	}

	udelay(50);

	return 0;
}

static int qmp_ufs_power_on(struct phy *phy)
{
	struct qmp_ufs_priv *qmp = dev_get_priv(phy->dev);
	const struct qmp_ufs_cfg *cfg = qmp->cfg;
	void __iomem *pcs = qmp->pcs;
	void __iomem *status;
	unsigned int val;
	int ret;

	/* Power down PHY */
	qphy_setbits(pcs, cfg->regs[QPHY_PCS_POWER_DOWN_CONTROL], SW_PWRDN);

	qmp_ufs_init_registers(qmp, cfg);

	if (cfg->no_pcs_sw_reset) {
		ret = qmp_ufs_do_reset(qmp);
		if (ret) {
			printf("%s: qmp reset failed\n", __func__);
			return ret;
		}
	}

	/* Pull PHY out of reset state */
	if (!cfg->no_pcs_sw_reset)
		qphy_clrbits(pcs, cfg->regs[QPHY_SW_RESET], SW_RESET);

	/* start SerDes */
	qphy_setbits(pcs, cfg->regs[QPHY_START_CTRL], SERDES_START);

	status = pcs + cfg->regs[QPHY_PCS_READY_STATUS];
	ret = readl_poll_timeout(status, val, (val & PCS_READY), PHY_INIT_COMPLETE_TIMEOUT);
	if (ret) {
		printf("%s: phy initialization timed-out\n", __func__);
		return ret;
	}

	return 0;
}

static int qmp_ufs_power_off(struct phy *phy)
{
	struct qmp_ufs_priv *qmp = dev_get_priv(phy->dev);
	const struct qmp_ufs_cfg *cfg = qmp->cfg;

	/* PHY reset */
	qphy_setbits(qmp->pcs, cfg->regs[QPHY_SW_RESET], SW_RESET);

	/* stop SerDes and Phy-Coding-Sublayer */
	qphy_clrbits(qmp->pcs, cfg->regs[QPHY_START_CTRL],
			SERDES_START | PCS_START);

	/* Put PHY into POWER DOWN state: active low */
	qphy_clrbits(qmp->pcs, cfg->regs[QPHY_PCS_POWER_DOWN_CONTROL],
			SW_PWRDN);

	clk_release_all(qmp->clks, qmp->clk_count);

	return 0;
}

static int qmp_ufs_vreg_init(struct udevice *dev, struct qmp_ufs_priv *qmp)
{
	// TBD: Add regulator support later - if needed
	return 0;
}

static int qmp_ufs_reset_init(struct udevice *dev, struct qmp_ufs_priv *qmp)
{
	const struct qmp_ufs_cfg *cfg = qmp->cfg;
	int num = cfg->num_resets;
	int i, ret;

	qmp->reset_count = 0;
	qmp->resets = devm_kcalloc(dev, num, sizeof(*qmp->resets), GFP_KERNEL);
	if (!qmp->resets)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		ret = reset_get_by_index(dev, i, &qmp->resets[i]);
		if (ret < 0) {
			printf("%s: failed to get reset %d\n", __func__, i);
			goto reset_get_err;
		}

		++qmp->reset_count;
	}

	return 0;

reset_get_err:
	ret = reset_release_all(qmp->resets, qmp->reset_count);
	if (ret)
		printf("%s: failed to disable all resets\n", __func__);

	return ret;
}

static int qmp_ufs_clk_init(struct udevice *dev, struct qmp_ufs_priv *qmp)
{
	const struct qmp_ufs_cfg *cfg = qmp->cfg;
	int num = cfg->num_clks;
	int i, ret;

	qmp->clk_count = 0;
	qmp->clks = devm_kcalloc(dev, num, sizeof(*qmp->clks), GFP_KERNEL);
	if (!qmp->clks)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		ret = clk_get_by_index(dev, i, &qmp->clks[i]);
		if (ret < 0)
			goto clk_get_err;

		ret = clk_enable(&qmp->clks[i]);
		if (ret && ret != -ENOSYS) {
			printf("%s: failed to enable clock %d\n", __func__, i);
			clk_free(&qmp->clks[i]);
			goto clk_get_err;
		}

		++qmp->clk_count;
	}

	return 0;

clk_get_err:
	ret = clk_release_all(qmp->clks, qmp->clk_count);
	if (ret)
		printf("%s: failed to disable all clocks\n", __func__);

	return ret;
}

static int qmp_ufs_probe_generic_child(struct udevice *dev,
				       ofnode child)
{
	struct qmp_ufs_priv *qmp = dev_get_priv(dev);
	const struct qmp_ufs_cfg *cfg = qmp->cfg;
	struct resource res;
	int ret;

	/*
	 * Get memory resources for the PHY:
	 * Resources are indexed as: tx -> 0; rx -> 1; pcs -> 2.
	 * For dual lane PHYs: tx2 -> 3, rx2 -> 4, pcs_misc (optional) -> 5
	 * For single lane PHYs: pcs_misc (optional) -> 3.
	 */
	ret = ofnode_read_resource(child, 0, &res);
	if (ret) {
		dev_err(dev, "can't get reg property of child %s\n",
			ofnode_get_name(child));
		return ret;
	}

	qmp->tx = (void __iomem *)res.start;

	ret = ofnode_read_resource(child, 1, &res);
	if (ret) {
		dev_err(dev, "can't get reg property of child %s\n",
			ofnode_get_name(child));
		return ret;
	}

	qmp->rx = (void __iomem *)res.start;

	ret = ofnode_read_resource(child, 2, &res);
	if (ret) {
		dev_err(dev, "can't get reg property of child %s\n",
			ofnode_get_name(child));
		return ret;
	}

	qmp->pcs = (void __iomem *)res.start;

	if (cfg->lanes >= 2) {
		ret = ofnode_read_resource(child, 3, &res);
		if (ret) {
			dev_err(dev, "can't get reg property of child %s\n",
				ofnode_get_name(child));
			return ret;
		}

		qmp->tx2 = (void __iomem *)res.start;

		ret = ofnode_read_resource(child, 4, &res);
		if (ret) {
			dev_err(dev, "can't get reg property of child %s\n",
				ofnode_get_name(child));
			return ret;
		}

		qmp->rx2 = (void __iomem *)res.start;

		ret = ofnode_read_resource(child, 5, &res);
		if (ret)
			qmp->pcs_misc = NULL;
	} else {
		ret = ofnode_read_resource(child, 3, &res);
		if (ret)
			qmp->pcs_misc = NULL;
	}

	if (!qmp->pcs_misc)
		dev_warn(qmp->dev, "PHY pcs_misc-reg not used\n");

	return 0;
}

static int qmp_ufs_probe_dt_children(struct udevice *dev)
{
	int ret;
	ofnode child;

	ofnode_for_each_subnode(child, dev_ofnode(dev)) {
		ret = qmp_ufs_probe_generic_child(dev, child);
		if (ret) {
			dev_err(dev, "Cannot parse child %s:%d\n",
				ofnode_get_name(child), ret);
			return ret;
		}
	}

	return 0;
}

static int qmp_ufs_probe(struct udevice *dev)
{
	struct qmp_ufs_priv *qmp = dev_get_priv(dev);
	int ret;

	qmp->serdes = (void __iomem *)dev_read_addr(dev);
	if (IS_ERR(qmp->serdes))
		return PTR_ERR(qmp->serdes);

	qmp->cfg = (const struct qmp_ufs_cfg *)dev_get_driver_data(dev);
	if (!qmp->cfg)
		return -EINVAL;

	ret = qmp_ufs_clk_init(dev, qmp);
	if (ret) {
		printf("%s: failed to get UFS clks\n", __func__);
		return ret;
	}

	ret = qmp_ufs_vreg_init(dev, qmp);
	if (ret) {
		printf("%s: failed to get UFS voltage regulators\n", __func__);
		return ret;
	}

	if (qmp->cfg->no_pcs_sw_reset) {
		ret = qmp_ufs_reset_init(dev, qmp);
		if (ret) {
			printf("%s: failed to get UFS resets\n", __func__);
			return ret;
		}
	}

	qmp->dev = dev;

	ret = qmp_ufs_probe_dt_children(dev);
	if (ret) {
		printf("%s: failed to get UFS dt regs\n", __func__);
		return ret;
	}

	return 0;
}

static struct phy_ops qmp_ufs_ops = {
	.power_on = qmp_ufs_power_on,
	.power_off = qmp_ufs_power_off,
};

static const struct udevice_id qmp_ufs_ids[] = {
	{ .compatible = "qcom,sdm845-qmp-ufs-phy", .data = (ulong)&sdm845_ufsphy_cfg },
	{ .compatible = "qcom,sm6115-qmp-ufs-phy", .data = (ulong)&sm6115_ufsphy_cfg },
	{ .compatible = "qcom,sm8250-qmp-ufs-phy", .data = (ulong)&sm8250_ufsphy_cfg },
	{ }
};

U_BOOT_DRIVER(qcom_qmp_ufs) = {
	.name		= "qcom-qmp-ufs",
	.id		= UCLASS_PHY,
	.of_match	= qmp_ufs_ids,
	.ops		= &qmp_ufs_ops,
	.probe		= qmp_ufs_probe,
	.priv_auto	= sizeof(struct qmp_ufs_priv),
};
