// SPDX-License-Identifier: GPL-2.0-only
/*
 * Alif Semiconductor SoC glue for the DesignWare MIPI DSI host.
 *
 * The Ensemble family has a single MIPI D-PHY macro pair. The TX macro is
 * owned by this driver: it drives the DSI display output and can also be
 * turned around and lent to the camera subsystem as an RX macro. The
 * dedicated RX macro stays with the standalone D-PHY driver.
 *
 * Copyright (C) 2026 Alif Semiconductor
 * Author: Yogender Kumar Arya <yogender.kumar@alifsemi.com>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/units.h>

#include <drm/bridge/dw_mipi_dsi.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>

/* SoC (EXPMST) D-PHY control registers */
#define DPHY_PLL_CTRL0			0x10
#define DPHY_PLL_CTRL1			0x14
#define DPHY_PLL_CTRL2			0x18
#define TX_DPHY_CTRL0			0x30
#define TX_DPHY_CTRL1			0x34
#define DSI_CTRL			0x44

#define DPHY_PLL_CTRL0_CLKSEL		GENMASK(21, 20)
#define DPHY_PLL_CLKSEL_PLL		1
#define DPHY_PLL_CLKSEL_CLKEXT		2	/* buffered clkext, HWRM 8.3.4.3.5 */
#define DPHY_PLL_CTRL0_SHADOW_CLR	BIT(12)
#define DPHY_PLL_CTRL0_UPDATE_PLL	BIT(8)
#define DPHY_PLL_CTRL0_SHADOW_CONTROL	BIT(4)

#define DPHY_PLL_CTRL0_STOPPED		0x10000
#define DPHY_PLL_CTRL1_BYPASS		0xf040
#define DPHY_PLL_CTRL2_BYPASS		0x03100400

#define DPHY_CTRL0_CFG_CLK_FREQ_RANGE	GENMASK(31, 24)
#define DPHY_CTRL0_HS_FREQ_RANGE	GENMASK(22, 16)
#define DPHY_CTRL0_BASE_DIR		GENMASK(13, 12)
#define DPHY_CTRL0_TXRXZ		BIT(8)
#define DPHY_CTRL0_TESTPORT_SEL		BIT(4)

#define DPHY_CTRL1_FORCE_RX_MODE	GENMASK(1, 0)

#define DSI_CTRL_CAM2_EN		BIT(9)
#define DSI_CTRL_PHYSEL_TX_RXN		BIT(10)

/* DSI host registers holding the TX D-PHY wrapper */
#define DSI_CMD_PKT_STATUS		0x74
#define DSI_CMD_PKT_STATUS_GEN_PLD_W_EMPTY	BIT(2)
#define DSI_CMD_PKT_STATUS_GEN_CMD_EMPTY	BIT(0)

#define DSI_LPCLK_CTRL			0x94
#define DSI_LPCLK_CTRL_TXREQUESTCLKHS	BIT(0)

#define DSI_PHY_RSTZ			0xa0
#define DSI_PHY_STATUS			0xb0
#define DSI_PHY_TST_CTRL0		0xb4
#define DSI_PHY_TST_CTRL1		0xb8

#define DSI_PHY_RSTZ_FORCEPLL		BIT(3)
#define DSI_PHY_RSTZ_ENABLECLK		BIT(2)
#define DSI_PHY_RSTZ_RSTZ		BIT(1)
#define DSI_PHY_RSTZ_SHUTDOWNZ		BIT(0)

#define DSI_PHY_STATUS_STOPSTATE1LANE	BIT(7)
#define DSI_PHY_STATUS_STOPSTATE0LANE	BIT(4)
#define DSI_PHY_STATUS_STOPSTATECLKLANE	BIT(2)
#define DSI_PHY_STATUS_PHY_LOCK		BIT(0)

/* CSI host registers holding the RX D-PHY wrapper */
#define CSI_PHY_SHUTDOWNZ		0x40
#define CSI_DPHY_RSTZ			0x44
#define CSI_PHY_RX			0x48
#define CSI_PHY_STOPSTATE		0x4c
#define CSI_PHY_TST_CTRL0		0x50
#define CSI_PHY_TST_CTRL1		0x54

#define CSI_PHY_SHUTDOWNZ_SHUTDOWNZ	BIT(0)
#define CSI_DPHY_RSTZ_RSTZ		BIT(0)

#define CSI_PHY_STOPSTATE_CLK		BIT(16)
#define CSI_PHY_STOPSTATE_DATA_1	BIT(1)
#define CSI_PHY_STOPSTATE_DATA_0	BIT(0)

/* VBAT power control */
#define VBAT_PWR_CTRL			0x00
#define VBAT_TX_DPHY_PWR_MASK		BIT(0)
#define VBAT_TX_DPHY_ISO		BIT(1)
#define VBAT_RX_DPHY_PWR_MASK		BIT(4)
#define VBAT_RX_DPHY_ISO		BIT(5)
#define VBAT_DPHY_PLL_PWR_MASK		BIT(8)
#define VBAT_DPHY_PLL_ISO		BIT(9)
#define VBAT_DPHY_VPH_1P8_BYP_EN	BIT(12)
#define VBAT_DPHY_VPH_1P8_BYP_VAL	BIT(13)
#define VBAT_UPHY_PWR_MASK		BIT(16)
#define VBAT_UPHY_ISO			BIT(17)
#define VBAT_VREG_AUX_1_1V8_EN		BIT(24)
#define VBAT_VREG_AUX_2_1V8_EN		BIT(25)

/*
 * Analog 1.8 V supplies only. USB PHY and RX D-PHY isolation live in the
 * same register (HWRM 8.3.7.3.3); those bits must be left alone.
 */
#define VBAT_PWR_ON_INIT		(VBAT_DPHY_VPH_1P8_BYP_EN | \
					 VBAT_DPHY_VPH_1P8_BYP_VAL | \
					 VBAT_VREG_AUX_1_1V8_EN | \
					 VBAT_VREG_AUX_2_1V8_EN)

static_assert(!(VBAT_PWR_ON_INIT &
		(VBAT_RX_DPHY_PWR_MASK | VBAT_RX_DPHY_ISO |
		 VBAT_UPHY_PWR_MASK | VBAT_UPHY_ISO)),
	      "VBAT init must not touch USB PHY or RX D-PHY isolation");

/* Test interface control bits, valid for both the DSI and CSI wrappers */
#define PHY_TST_CTRL0_CLK		BIT(1)
#define PHY_TST_CTRL0_CLR		BIT(0)
#define PHY_TST_CTRL1_TESTEN		BIT(16)
#define PHY_TST_CTRL1_TESTDOUT		GENMASK(15, 8)
#define PHY_TST_CTRL1_TESTDIN		GENMASK(7, 0)

/* D-PHY test interface register addresses */
#define TX_PLL_1			0x15e
#define TX_PLL_5			0x162
#define TX_PLL_9			0x166
#define TX_PLL_13			0x16a
#define TX_PLL_17			0x16e
#define TX_PLL_27			0x178
#define TX_PLL_28			0x179
#define TX_PLL_29			0x17a
#define TX_PLL_30			0x17b
#define TX_CB_0				0x1aa
#define TX_CB_1				0x1ab
#define TX_CB_2				0x1ac
#define TX_CB_3				0x1ad
#define TX_SLEW_0			0x26b
#define TX_SLEW_5			0x270
#define TX_SLEW_6			0x271
#define TX_SLEW_7			0x272
#define TX_CLK_TERMLOWCAP		0x402
#define TX_LANE1_SLEWRATE_0		0x70b
#define TX_LANE2_SLEWRATE_0		0x90b
#define TX_LANE3_SLEWRATE_0		0xb0b

#define RX_SYS_1			0x01f
#define RX_STARTUP_OVR_2		0x0e2
#define RX_STARTUP_OVR_3		0x0e3
#define RX_STARTUP_OVR_4		0x0e4
#define RX_CLKLANE_LANE_6		0x307

/*
 * Test-interface bitfields from the DWC MIPI D-PHY databook
 * (dphy4txtester / dphy4rxtester).
 */
#define TX_PLL_1_CPBIAS_CNTRL		GENMASK(6, 0)
#define TX_PLL_5_INT_CNTRL		GENMASK(7, 2)
#define TX_PLL_5_GMP_CNTRL		GENMASK(1, 0)
#define TX_PLL_9_LOCK_STATE_OVR_EN	BIT(3)
#define TX_PLL_13_MPLL_PROG		GENMASK(1, 0)
#define TX_PLL_17_PROP_CNTRL		GENMASK(5, 0)
#define TX_PLL_17_PWRON_OVR		BIT(7)
#define TX_PLL_17_PWRON_OVR_EN		BIT(6)
#define TX_PLL_27_N_OVR			GENMASK(6, 3)
#define TX_PLL_27_N_OVR_EN		BIT(7)
#define TX_PLL_28_M_OVR			GENMASK(7, 0)
#define TX_PLL_29_M_OVR			GENMASK(1, 0)
#define TX_PLL_30_M_OVR_EN		BIT(0)
#define TX_PLL_30_VCO_CNTRL_OVR		GENMASK(6, 1)
#define TX_PLL_30_VCO_CNTRL_OVR_EN	BIT(7)
#define TX_CB_0_SEL_VREFCD_LPRX		GENMASK(6, 5)
#define TX_CB_0_SEL_V400		GENMASK(4, 2)
#define TX_CB_0_SEL_CHOP_CLK		BIT(1)
#define TX_CB_0_CHOP_CLK_EN		BIT(0)
#define TX_CB_1_SEL_VREFLPTX		BIT(2)
#define TX_CB_1_SEL_VREF_LPRX		GENMASK(1, 0)
#define TX_CB_2_CLKDIV_CLK_EN		BIT(4)
#define TX_CB_3_VREF_MPLL_REG_SEL	GENMASK(2, 0)
#define TX_SLEW_0_SRCAL_EN_OVR_EN	BIT(2)
#define TX_SLEW_5_OSC_FREQ		GENMASK(7, 0)
#define TX_SLEW_6_OSC_FREQ		GENMASK(3, 0)
#define TX_SLEW_7_SR_SEL_TESTER		GENMASK(5, 4)
#define TX_SLEW_7_SR_RANGE		BIT(0)
#define TX_CLK_TERMLOWCAP_LP00_OVR_EN	BIT(1)
#define TX_LANE_SLEWRATE_SR_FINISHED_OVR_EN	BIT(3)
#define TX_LANE_SLEWRATE_SR_FINISHED_OVR	BIT(2)
#define TX_LANE_SLEWRATE_SRCAL_EN_OVR_EN	BIT(1)
#define RX_STARTUP_OVR_2_OSC_FREQ	GENMASK(7, 0)
#define RX_STARTUP_OVR_3_OSC_FREQ	GENMASK(3, 0)
#define RX_STARTUP_OVR_4_OSC_FREQ_OVR_EN	BIT(0)
#define RX_CLKLANE_LANE_6_HSRX_PULL_LONG	BIT(7)

/* Analog programming values from the D-PHY databook */
#define TX_PLL_13_MPLL_PROG_BYP_VREG	0x3	/* bits[1:0] = 2'b11 */
#define TX_CB_VREF_LPRX_325MV		0x2	/* 2'b10 */
#define TX_CB_VREFLPTX_1200MV		0x1
#define TX_CB_V400_400MV		0x4	/* 3'b1xx */
#define TX_SLEW_7_SR_SEL_ON		0x1	/* 2'b01: slew rate on */
#define TX_SLEW_OSC_FREQ_LT_1000	657
#define TX_SLEW_OSC_FREQ_LT_1500	920
#define RX_SYS_1_80MBPS			0x85

/* Park unused lanes: sr_finished_ovr{,_en} and srcal_en_ovr_en */
#define TX_LANE_SLEWRATE_PARK		(TX_LANE_SLEWRATE_SR_FINISHED_OVR_EN | \
					 TX_LANE_SLEWRATE_SR_FINISHED_OVR | \
					 TX_LANE_SLEWRATE_SRCAL_EN_OVR_EN)

/* Fixed PLL analog programming */
#define DPHY_CPBIAS_CNTRL		0x00
#define DPHY_GMP_CNTRL			0x01
#define DPHY_INT_CNTRL			0x04
#define DPHY_PROP_CNTRL			0x10
#define DPHY_CB_VREF_CNTRL		0x02

#define DPHY_PLL_N			3
#define DPHY_PLL_M_MAX			1023
#define DPHY_PLL_INPUT_MIN_KHZ		8000
#define DPHY_PLL_INPUT_MAX_KHZ		24000

#define DPHY_REF_FREQ_DEFAULT		38400000
#define DPHY_CFG_FREQ_DEFAULT		25000000
#define DPHY_CFG_CLK_MIN_MHZ		17
#define DPHY_CFG_CLK_RANGE_MULT		4

#define DPHY_MAX_DATA_LANES		2
#define DPHY_LOCK_TIMEOUT_US		1000000

enum alif_dsi_usage {
	ALIF_DSI_USAGE_IDLE = 0,
	ALIF_DSI_USAGE_DISPLAY,
	ALIF_DSI_USAGE_CAMERA,
};

struct alif_dphy_freq_range {
	u16 bitrate;
	u8 hsfreqrange;
	u16 osc_freq_target;
	u16 clk_lp2hs;
	u16 clk_hs2lp;
	u16 lane_lp2hs;
	u16 lane_hs2lp;
};

struct alif_dphy_vco_range {
	u32 min_khz;
	u32 max_khz;
	u8 vco_cntrl;
};

struct alif_dphy_pll {
	u16 m;
	u16 n;
	u8 p;
	u8 vco_cntrl;
	u32 fout_khz;
};

/* Test interface port, either the DSI or the CSI wrapper */
struct alif_testif {
	void __iomem *ctrl0;
	void __iomem *ctrl1;
};

struct alif_dw_dsi {
	struct device *dev;
	void __iomem *dsi_regs;
	void __iomem *soc_regs;
	void __iomem *vbat_regs;
	void __iomem *csi_regs;

	struct clk *pclk;
	struct clk *tx_clk;
	struct clk *pll_ref_clk;
	struct clk *pll_bypass_clk;

	u32 ref_freq;
	u32 cfg_freq;

	struct dw_mipi_dsi *dmd;
	struct dw_mipi_dsi_plat_data pdata;
	struct mipi_dsi_device *dsi_dev;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;

	/* TX macro turned around as an RX macro for the camera subsystem */
	struct phy *rx_phy;

	struct mutex usage_mutex;	/* serialises display and camera use */
	enum alif_dsi_usage usage;

	/* Display state */
	union phy_configure_opts phy_opts;
	const struct alif_dphy_freq_range *tx_range;
	struct alif_dphy_pll pll;
	bool tx_active;
	struct mux_state *mux_display;

	/* Camera state */
	const struct alif_dphy_freq_range *rx_range;
	unsigned int rx_lanes;
	struct mux_state *mux_camera;
};

static const struct alif_dphy_freq_range alif_dphy_freq_ranges[] = {
	{80,   0x00, 0x1e9, 21,  17,  15,  10},
	{90,   0x10, 0x1e9, 23,  17,  16,  10},
	{100,  0x20, 0x1e9, 22,  17,  16,  10},
	{110,  0x30, 0x1e9, 25,  18,  17,  11},
	{120,  0x01, 0x1e9, 26,  20,  18,  11},
	{130,  0x11, 0x1e9, 27,  19,  19,  11},
	{140,  0x21, 0x1e9, 27,  19,  19,  11},
	{150,  0x31, 0x1e9, 28,  20,  20,  12},
	{160,  0x02, 0x1e9, 30,  21,  22,  13},
	{170,  0x12, 0x1e9, 30,  21,  23,  13},
	{180,  0x22, 0x1e9, 31,  21,  23,  13},
	{190,  0x32, 0x1e9, 32,  22,  24,  13},
	{205,  0x03, 0x1e9, 35,  22,  25,  13},
	{220,  0x13, 0x1e9, 37,  26,  27,  15},
	{235,  0x23, 0x1e9, 38,  28,  27,  16},
	{250,  0x33, 0x1e9, 41,  29,  30,  17},
	{275,  0x04, 0x1e9, 43,  29,  32,  18},
	{300,  0x14, 0x1e9, 45,  32,  35,  19},
	{325,  0x25, 0x1e9, 48,  33,  36,  18},
	{350,  0x35, 0x1e9, 51,  35,  40,  20},
	{400,  0x05, 0x1e9, 59,  37,  44,  21},
	{450,  0x16, 0x1e9, 65,  40,  49,  23},
	{500,  0x26, 0x1e9, 71,  41,  54,  24},
	{550,  0x37, 0x1e9, 77,  44,  57,  26},
	{600,  0x07, 0x1e9, 82,  46,  64,  27},
	{650,  0x18, 0x1e9, 87,  48,  67,  28},
	{700,  0x28, 0x1e9, 94,  52,  71,  29},
	{750,  0x39, 0x1e9, 99,  52,  75,  31},
	{800,  0x09, 0x1e9, 105, 55,  82,  32},
	{850,  0x19, 0x1e9, 110, 58,  85,  32},
	{900,  0x29, 0x1e9, 115, 58,  88,  35},
	{950,  0x3a, 0x1e9, 120, 62,  93,  36},
	{1000, 0x0a, 0x1e9, 128, 63,  99,  38},
	{1050, 0x1a, 0x1e9, 132, 65,  102, 38},
	{1100, 0x2a, 0x1e9, 138, 67,  106, 39},
	{1150, 0x3b, 0x1e9, 146, 69,  112, 42},
	{1200, 0x0b, 0x1e9, 151, 71,  117, 43},
	{1250, 0x1b, 0x1e9, 153, 74,  120, 45},
	{1300, 0x2b, 0x1e9, 160, 73,  124, 46},
	{1350, 0x3c, 0x1e9, 165, 76,  130, 47},
	{1400, 0x0c, 0x1e9, 172, 78,  134, 49},
	{1450, 0x1c, 0x1e9, 177, 80,  138, 49},
	{1500, 0x2c, 0x1e9, 183, 81,  143, 52},
	{1550, 0x3d, 0x10f, 191, 84,  147, 52},
	{1600, 0x0d, 0x118, 194, 85,  152, 52},
	{1650, 0x1d, 0x121, 201, 86,  155, 53},
	{1700, 0x2e, 0x12a, 208, 88,  161, 53},
	{1750, 0x3e, 0x132, 212, 89,  165, 53},
	{1800, 0x0e, 0x13b, 220, 90,  171, 54},
	{1850, 0x1e, 0x144, 223, 92,  175, 54},
	{1900, 0x2f, 0x14d, 231, 91,  180, 55},
	{1950, 0x3f, 0x155, 236, 95,  185, 56},
	{2000, 0x0f, 0x15e, 243, 97,  190, 56},
	{2050, 0x40, 0x167, 248, 99,  194, 58},
	{2100, 0x41, 0x170, 252, 100, 199, 59},
	{2150, 0x42, 0x178, 259, 102, 204, 61},
	{2200, 0x43, 0x181, 266, 105, 210, 62},
	{2250, 0x44, 0x18a, 269, 109, 213, 63},
	{2300, 0x45, 0x193, 272, 109, 217, 65},
	{2350, 0x46, 0x19b, 281, 112, 225, 66},
	{2400, 0x47, 0x1a4, 283, 115, 226, 66},
	{2450, 0x48, 0x1ad, 282, 115, 226, 67},
	{2500, 0x49, 0x1e9, 281, 118, 227, 67},
};

/*
 * VCO ranges in kHz. vco_cntrl[5:3] selects the PLL output division factor,
 * so the same code can appear twice with different frequency windows.
 */
static const struct alif_dphy_vco_range alif_dphy_vco_ranges[] = {
	{1170000, 1250000, 0x03},
	{975000,  1230000, 0x07},
	{853125,  1025000, 0x08},
	{706875,  896875,  0x08},
	{585000,  743125,  0x0b},
	{487500,  615000,  0x0f},
	{426560,  512500,  0x10},
	{353400,  484400,  0x10},
	{292500,  371500,  0x13},
	{243750,  307500,  0x17},
	{213300,  256250,  0x18},
	{176720,  224200,  0x18},
	{146250,  185780,  0x1b},
	{121880,  153750,  0x1f},
	{106640,  125120,  0x20},
	{88360,   112100,  0x20},
	{73130,   92900,   0x23},
	{60930,   76870,   0x27},
	{53320,   64000,   0x28},
	{44180,   56000,   0x28},
	{40000,   46440,   0x2b},
};

/* vco_cntrl[5:3] encodes the output division factor as 2^(field + 1) */
static u8 alif_dphy_out_div(u8 vco_cntrl)
{
	return 1 << (((vco_cntrl >> 3) & 0x7) + 1);
}

static void alif_update_bits(void __iomem *reg, u32 mask, u32 val)
{
	u32 tmp = readl(reg);

	tmp &= ~mask;
	tmp |= val & mask;
	writel(tmp, reg);
}

static void alif_set_bits(void __iomem *reg, u32 mask)
{
	writel(readl(reg) | mask, reg);
}

static void alif_clear_bits(void __iomem *reg, u32 mask)
{
	writel(readl(reg) & ~mask, reg);
}

/*
 * The D-PHY analog and digital tuning registers are only reachable through
 * the DesignWare test interface, which shifts in a 16-bit address one byte
 * at a time before the data byte.
 */
static void alif_testif_addr(const struct alif_testif *ifx, u16 addr)
{
	alif_clear_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
	alif_clear_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTEN);
	alif_set_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTEN);
	alif_set_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
	alif_update_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTDIN, 0);
	alif_clear_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
	alif_clear_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTEN);
	alif_update_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTDIN, addr >> 8);
	alif_set_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
	alif_clear_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
	alif_set_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTEN);
	alif_set_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
	alif_update_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTDIN, addr & 0xff);
	alif_clear_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
}

static u8 alif_testif_read(const struct alif_testif *ifx, u16 addr)
{
	u8 val;

	alif_testif_addr(ifx, addr);
	val = FIELD_GET(PHY_TST_CTRL1_TESTDOUT, readl(ifx->ctrl1));
	alif_clear_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTEN);

	return val;
}

static void alif_testif_write(const struct alif_testif *ifx, u16 addr, u8 data)
{
	alif_testif_addr(ifx, addr);
	alif_clear_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTEN);
	alif_update_bits(ifx->ctrl1, PHY_TST_CTRL1_TESTDIN, data);
	alif_set_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
	alif_clear_bits(ifx->ctrl0, PHY_TST_CTRL0_CLK);
}

static void alif_testif_update(const struct alif_testif *ifx, u16 addr,
			       u8 field, u8 val)
{
	u8 tmp = alif_testif_read(ifx, addr);

	tmp &= ~field;
	tmp |= (val << __ffs(field)) & field;
	alif_testif_write(ifx, addr, tmp);
}

/*
 * Both test ports share the wrapper clear bit, so the reset has to be applied
 * once per port selection.
 */
static void alif_testif_reset(void __iomem *ctrl, void __iomem *tst_ctrl0)
{
	alif_set_bits(ctrl, DPHY_CTRL0_TESTPORT_SEL);
	alif_set_bits(tst_ctrl0, PHY_TST_CTRL0_CLR);
	alif_clear_bits(ctrl, DPHY_CTRL0_TESTPORT_SEL);
	alif_set_bits(tst_ctrl0, PHY_TST_CTRL0_CLR);

	fsleep(1);

	alif_set_bits(ctrl, DPHY_CTRL0_TESTPORT_SEL);
	alif_clear_bits(tst_ctrl0, PHY_TST_CTRL0_CLR);
	alif_clear_bits(ctrl, DPHY_CTRL0_TESTPORT_SEL);
	alif_clear_bits(tst_ctrl0, PHY_TST_CTRL0_CLR);
}

static const struct alif_dphy_freq_range *alif_dphy_get_range(struct alif_dw_dsi *dsi,
							      unsigned int mbps)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(alif_dphy_freq_ranges) - 1; i++)
		if (mbps <= alif_dphy_freq_ranges[i].bitrate)
			break;

	if (mbps > alif_dphy_freq_ranges[i].bitrate) {
		dev_err(dsi->dev, "unsupported lane rate %u Mbps\n", mbps);
		return NULL;
	}

	return &alif_dphy_freq_ranges[i];
}

static u32 alif_dphy_cfg_clk_range(struct alif_dw_dsi *dsi)
{
	u32 mhz = dsi->cfg_freq / HZ_PER_MHZ;

	if (mhz <= DPHY_CFG_CLK_MIN_MHZ)
		return 0;

	return (mhz - DPHY_CFG_CLK_MIN_MHZ) * DPHY_CFG_CLK_RANGE_MULT;
}

/*
 * Search the VCO table for the M/N/P combination whose output lands closest to
 * the requested DDR clock. N is fixed at 3, which keeps the PLL input inside
 * its 8-24 MHz window for every supported reference clock.
 */
static int alif_dphy_calc_pll(struct alif_dw_dsi *dsi, u32 fout_khz,
			      struct alif_dphy_pll *pll)
{
	u32 ref_khz = dsi->ref_freq / 1000;
	u32 best_delta = U32_MAX;
	unsigned int i;

	if (fout_khz > alif_dphy_vco_ranges[0].max_khz ||
	    fout_khz < alif_dphy_vco_ranges[ARRAY_SIZE(alif_dphy_vco_ranges) - 1].min_khz) {
		dev_dbg(dsi->dev, "PLL output %u kHz out of range\n", fout_khz);
		return -EINVAL;
	}

	if (ref_khz / DPHY_PLL_N > DPHY_PLL_INPUT_MAX_KHZ ||
	    ref_khz / DPHY_PLL_N < DPHY_PLL_INPUT_MIN_KHZ) {
		dev_dbg(dsi->dev, "reference clock %u kHz unusable\n", ref_khz);
		return -EINVAL;
	}

	pll->m = 0;

	for (i = 0; i < ARRAY_SIZE(alif_dphy_vco_ranges); i++) {
		const struct alif_dphy_vco_range *vco = &alif_dphy_vco_ranges[i];
		u32 div, delta, got_khz;
		u64 m;
		u8 p;

		if (fout_khz > vco->max_khz)
			break;
		if (fout_khz < vco->min_khz)
			continue;

		p = alif_dphy_out_div(vco->vco_cntrl);
		div = DPHY_PLL_N * 2 * p;

		m = DIV_ROUND_UP_ULL((u64)fout_khz * div, ref_khz);
		if (!m || m > DPHY_PLL_M_MAX)
			continue;

		got_khz = div_u64((u64)ref_khz * m, div);
		delta = abs_diff(got_khz, fout_khz);
		if (delta >= best_delta)
			continue;

		best_delta = delta;
		pll->m = m;
		pll->n = DPHY_PLL_N;
		pll->p = p;
		pll->vco_cntrl = vco->vco_cntrl;
		pll->fout_khz = got_khz;
	}

	if (!pll->m) {
		dev_dbg(dsi->dev, "no PLL setting for %u kHz\n", fout_khz);
		return -EINVAL;
	}

	return 0;
}

static void alif_dphy_config_pll(struct alif_dw_dsi *dsi,
				 const struct alif_testif *ifx)
{
	const struct alif_dphy_pll *pll = &dsi->pll;

	/* Program the PLL through the test interface rather than the SoC regs */
	alif_set_bits(dsi->soc_regs + DPHY_PLL_CTRL0,
		      DPHY_PLL_CTRL0_SHADOW_CONTROL);

	/* clksel: let the PLL generate the output clock */
	alif_update_bits(dsi->soc_regs + DPHY_PLL_CTRL0, DPHY_PLL_CTRL0_CLKSEL,
			 FIELD_PREP(DPHY_PLL_CTRL0_CLKSEL, DPHY_PLL_CLKSEL_PLL));

	alif_testif_update(ifx, TX_PLL_28, TX_PLL_28_M_OVR, pll->m);
	alif_testif_update(ifx, TX_PLL_29, TX_PLL_29_M_OVR, pll->m >> 8);

	/* Take the feedback divider from the values written above */
	alif_testif_update(ifx, TX_PLL_30, TX_PLL_30_M_OVR_EN, 1);
	alif_testif_update(ifx, TX_PLL_30, TX_PLL_30_VCO_CNTRL_OVR,
			   pll->vco_cntrl);
	alif_testif_update(ifx, TX_PLL_30, TX_PLL_30_VCO_CNTRL_OVR_EN, 1);

	alif_testif_update(ifx, TX_PLL_27, TX_PLL_27_N_OVR, pll->n - 1);
	alif_testif_update(ifx, TX_PLL_27, TX_PLL_27_N_OVR_EN, 1);

	alif_testif_update(ifx, TX_PLL_1, TX_PLL_1_CPBIAS_CNTRL,
			   DPHY_CPBIAS_CNTRL);
	alif_testif_update(ifx, TX_PLL_5, TX_PLL_5_INT_CNTRL, DPHY_INT_CNTRL);
	alif_testif_update(ifx, TX_PLL_5, TX_PLL_5_GMP_CNTRL, DPHY_GMP_CNTRL);
	alif_testif_update(ifx, TX_CB_3, TX_CB_3_VREF_MPLL_REG_SEL,
			   DPHY_CB_VREF_CNTRL);
	alif_testif_update(ifx, TX_PLL_17, TX_PLL_17_PROP_CNTRL,
			   DPHY_PROP_CNTRL);

	/* Enable PLL power */
	alif_testif_update(ifx, TX_PLL_17, TX_PLL_17_PWRON_OVR_EN, 1);
	alif_testif_update(ifx, TX_PLL_17, TX_PLL_17_PWRON_OVR, 1);
}

static void alif_dphy_pll_bypass(struct alif_dw_dsi *dsi)
{
	void __iomem *ctrl0 = dsi->soc_regs + DPHY_PLL_CTRL0;

	writel(DPHY_PLL_CTRL0_STOPPED, ctrl0);
	alif_set_bits(ctrl0, DPHY_PLL_CTRL0_SHADOW_CONTROL);
	fsleep(1);
	alif_set_bits(ctrl0, DPHY_PLL_CTRL0_SHADOW_CLR);
	fsleep(1);
	alif_clear_bits(ctrl0, DPHY_PLL_CTRL0_SHADOW_CLR);
	fsleep(1);
	writel(DPHY_PLL_CTRL1_BYPASS, dsi->soc_regs + DPHY_PLL_CTRL1);
	writel(DPHY_PLL_CTRL2_BYPASS, dsi->soc_regs + DPHY_PLL_CTRL2);
	alif_set_bits(ctrl0, DPHY_PLL_CTRL0_UPDATE_PLL);
	fsleep(1);
	alif_clear_bits(ctrl0, DPHY_PLL_CTRL0_UPDATE_PLL);
}

static int alif_dsi_claim(struct alif_dw_dsi *dsi, enum alif_dsi_usage usage)
{
	int ret = 0;

	mutex_lock(&dsi->usage_mutex);
	if (dsi->usage != ALIF_DSI_USAGE_IDLE && dsi->usage != usage)
		ret = -EBUSY;
	else
		dsi->usage = usage;
	mutex_unlock(&dsi->usage_mutex);

	return ret;
}

static void alif_dsi_release(struct alif_dw_dsi *dsi, enum alif_dsi_usage usage)
{
	mutex_lock(&dsi->usage_mutex);
	if (dsi->usage == usage)
		dsi->usage = ALIF_DSI_USAGE_IDLE;
	mutex_unlock(&dsi->usage_mutex);
}

static void alif_dphy_tx_power(struct alif_dw_dsi *dsi, bool on)
{
	u32 mask = VBAT_TX_DPHY_ISO | VBAT_TX_DPHY_PWR_MASK |
		   VBAT_DPHY_PLL_ISO | VBAT_DPHY_PLL_PWR_MASK;

	if (on)
		alif_clear_bits(dsi->vbat_regs + VBAT_PWR_CTRL, mask);
	else
		alif_set_bits(dsi->vbat_regs + VBAT_PWR_CTRL, mask);

	fsleep(10);
}

/*
 * Analog and slew-rate trimming that both the display TX path and the
 * turned-around RX path need.
 */
static void alif_dphy_tx_common_trim(const struct alif_testif *ifx)
{
	alif_testif_write(ifx, TX_PLL_13,
			  FIELD_PREP(TX_PLL_13_MPLL_PROG,
				     TX_PLL_13_MPLL_PROG_BYP_VREG));
	/* LP RX contention detector at 325 mV, LP TX reference at 1200 mV */
	alif_testif_write(ifx, TX_CB_1,
			  FIELD_PREP(TX_CB_1_SEL_VREF_LPRX,
				     TX_CB_VREF_LPRX_325MV) |
			  FIELD_PREP(TX_CB_1_SEL_VREFLPTX,
				     TX_CB_VREFLPTX_1200MV));
	alif_testif_write(ifx, TX_CB_0,
			  FIELD_PREP(TX_CB_0_SEL_VREFCD_LPRX,
				     TX_CB_VREF_LPRX_325MV) |
			  FIELD_PREP(TX_CB_0_SEL_V400, TX_CB_V400_400MV) |
			  TX_CB_0_SEL_CHOP_CLK | TX_CB_0_CHOP_CLK_EN);
}

static int alif_dphy_wait_stopstate(struct alif_dw_dsi *dsi, unsigned int lanes)
{
	u32 mask = DSI_PHY_STATUS_STOPSTATECLKLANE |
		   DSI_PHY_STATUS_STOPSTATE0LANE;
	u32 val;

	if (lanes > 1)
		mask |= DSI_PHY_STATUS_STOPSTATE1LANE;

	if (readl_poll_timeout(dsi->dsi_regs + DSI_PHY_STATUS, val,
			       (val & mask) == mask, 10, DPHY_LOCK_TIMEOUT_US)) {
		dev_err(dsi->dev,
			"clock/data lanes not in stop state, PHY status 0x%08x\n",
			val);
		return -ETIMEDOUT;
	}
	dev_dbg(dsi->dev, "clock/data lanes PHY status 0x%08x\n",
		readl(dsi->dsi_regs + DSI_PHY_STATUS));

	return 0;
}

static int alif_dphy_tx_setup(struct alif_dw_dsi *dsi)
{
	struct alif_testif ifx = {
		.ctrl0 = dsi->dsi_regs + DSI_PHY_TST_CTRL0,
		.ctrl1 = dsi->dsi_regs + DSI_PHY_TST_CTRL1,
	};
	void __iomem *ctrl0 = dsi->soc_regs + TX_DPHY_CTRL0;
	void __iomem *ctrl1 = dsi->soc_regs + TX_DPHY_CTRL1;
	unsigned int lanes = dsi->phy_opts.mipi_dphy.lanes;
	unsigned int mbps = dsi->tx_range->bitrate;
	u32 lane_mask = GENMASK(lanes - 1, 0);
	u32 val;
	int ret;

	alif_clear_bits(dsi->dsi_regs + DSI_PHY_RSTZ,
			DSI_PHY_RSTZ_RSTZ | DSI_PHY_RSTZ_SHUTDOWNZ);

	/* Master side calibration */
	alif_set_bits(ctrl0, DPHY_CTRL0_TXRXZ);
	alif_testif_reset(ctrl0, ifx.ctrl0);

	alif_update_bits(ctrl0, DPHY_CTRL0_HS_FREQ_RANGE,
			 FIELD_PREP(DPHY_CTRL0_HS_FREQ_RANGE,
				    dsi->tx_range->hsfreqrange));

	alif_dphy_tx_common_trim(&ifx);

	/* Below 450 Mbps the divided clock has to be enabled explicitly */
	if (mbps < 450)
		alif_testif_update(&ifx, TX_CB_2, TX_CB_2_CLKDIV_CLK_EN, 1);

	alif_testif_write(&ifx, TX_CLK_TERMLOWCAP,
			  TX_CLK_TERMLOWCAP_LP00_OVR_EN);

	if (mbps < 1000) {
		alif_testif_write(&ifx, TX_SLEW_5,
				  FIELD_GET(TX_SLEW_5_OSC_FREQ,
					    TX_SLEW_OSC_FREQ_LT_1000));
		alif_testif_write(&ifx, TX_SLEW_6,
				  FIELD_PREP(TX_SLEW_6_OSC_FREQ,
					     TX_SLEW_OSC_FREQ_LT_1000 >> 8));
		alif_testif_update(&ifx, TX_SLEW_7, TX_SLEW_7_SR_SEL_TESTER,
				   TX_SLEW_7_SR_SEL_ON);
		alif_testif_update(&ifx, TX_SLEW_7, TX_SLEW_7_SR_RANGE, 1);
	} else if (mbps < 1500) {
		alif_testif_write(&ifx, TX_SLEW_5,
				  FIELD_GET(TX_SLEW_5_OSC_FREQ,
					    TX_SLEW_OSC_FREQ_LT_1500));
		alif_testif_write(&ifx, TX_SLEW_6,
				  FIELD_PREP(TX_SLEW_6_OSC_FREQ,
					     TX_SLEW_OSC_FREQ_LT_1500 >> 8));
	}

	/* Park the slew-rate calibration of every unused lane */
	if (lanes == 1)
		alif_testif_write(&ifx, TX_LANE1_SLEWRATE_0,
				  TX_LANE_SLEWRATE_PARK);
	alif_testif_write(&ifx, TX_LANE2_SLEWRATE_0, TX_LANE_SLEWRATE_PARK);
	alif_testif_write(&ifx, TX_LANE3_SLEWRATE_0, TX_LANE_SLEWRATE_PARK);

	alif_update_bits(ctrl0, DPHY_CTRL0_CFG_CLK_FREQ_RANGE,
			 FIELD_PREP(DPHY_CTRL0_CFG_CLK_FREQ_RANGE,
				    alif_dphy_cfg_clk_range(dsi)));

	alif_dphy_config_pll(dsi, &ifx);

	/* Drive the lanes: base direction TX, no forced RX */
	alif_clear_bits(ctrl0, FIELD_PREP(DPHY_CTRL0_BASE_DIR, lane_mask));
	alif_clear_bits(ctrl1, FIELD_PREP(DPHY_CTRL1_FORCE_RX_MODE, lane_mask));

	/*
	 * The host must not request HS clock until the clock lane is in
	 * LP-11. A leftover TXREQUESTCLKHS keeps PHY_STOPSTATECLKLANE
	 * clear even after the PLL has locked.
	 */
	writel(0, dsi->dsi_regs + DSI_LPCLK_CTRL);

	/*
	 * Release the wrapper in one write. ENABLECLK while the PHY is
	 * still in shutdown/reset is not enough for the clock lane to
	 * finish its LP-11 transition; FORCEPLL is required as well.
	 */
	fsleep(1);
	writel(DSI_PHY_RSTZ_FORCEPLL | DSI_PHY_RSTZ_ENABLECLK |
	       DSI_PHY_RSTZ_RSTZ | DSI_PHY_RSTZ_SHUTDOWNZ,
	       dsi->dsi_regs + DSI_PHY_RSTZ);

	ret = readl_poll_timeout(dsi->dsi_regs + DSI_PHY_STATUS, val,
				 val & DSI_PHY_STATUS_PHY_LOCK, 10,
				 DPHY_LOCK_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev, "PLL failed to lock, PHY status 0x%08x\n",
			val);
		return ret;
	}

	return alif_dphy_wait_stopstate(dsi, lanes);
}

static int alif_dw_dsi_phy_init(void *priv_data)
{
	struct alif_dw_dsi *dsi = priv_data;
	int ret;

	/* A mode change without an intervening power_off keeps the clocks on */
	if (dsi->tx_active)
		return alif_dphy_tx_setup(dsi);

	ret = alif_dsi_claim(dsi, ALIF_DSI_USAGE_DISPLAY);
	if (ret) {
		dev_err(dsi->dev, "D-PHY is in use by the camera subsystem\n");
		return ret;
	}

	if (dsi->mux_display) {
		ret = mux_state_try_select(dsi->mux_display);
		if (ret)
			goto err_mux_display_release;
	}

	ret = clk_prepare_enable(dsi->pll_ref_clk);
	if (ret)
		goto err_release;

	ret = clk_prepare_enable(dsi->tx_clk);
	if (ret)
		goto err_ref_clk;

	alif_dphy_tx_power(dsi, true);

	/*
	 * Point the shared D-PHY at DSI TX. Reset default of PHYSEL is
	 * Rx, which leaves the pads in CSI-2 slave mode and the panel
	 * stays black even though /dev/fb0 is live.
	 */
	alif_update_bits(dsi->soc_regs + DSI_CTRL,
			 DSI_CTRL_PHYSEL_TX_RXN | DSI_CTRL_CAM2_EN,
			 DSI_CTRL_PHYSEL_TX_RXN);

	ret = alif_dphy_tx_setup(dsi);
	if (ret)
		goto err_power;

	dsi->tx_active = true;

	return 0;

err_power:
	alif_dphy_tx_power(dsi, false);
	clk_disable_unprepare(dsi->tx_clk);
err_ref_clk:
	clk_disable_unprepare(dsi->pll_ref_clk);
err_release:
	if (dsi->mux_display)
		mux_state_deselect(dsi->mux_display);
err_mux_display_release:
	alif_dsi_release(dsi, ALIF_DSI_USAGE_DISPLAY);

	return ret;
}

static void alif_dw_dsi_wait_lp_idle(struct alif_dw_dsi *dsi)
{
	unsigned int lanes = dsi->phy_opts.mipi_dphy.lanes;
	u32 fifo = DSI_CMD_PKT_STATUS_GEN_CMD_EMPTY |
		   DSI_CMD_PKT_STATUS_GEN_PLD_W_EMPTY;
	u32 stop = DSI_PHY_STATUS_STOPSTATE0LANE;
	u32 val;

	if (readl_poll_timeout(dsi->dsi_regs + DSI_CMD_PKT_STATUS, val,
			       (val & fifo) == fifo, 1000,
			       DPHY_LOCK_TIMEOUT_US))
		dev_err(dsi->dev, "command FIFO not empty before video\n");

	/*
	 * FIFO empty is not PHY-complete. The last LP DCS can still be
	 * on the wire; PWR_UP reset / TXREQUESTCLKHS would drop it.
	 */
	fsleep(50);

	if (lanes > 1)
		stop |= DSI_PHY_STATUS_STOPSTATE1LANE;

	if (readl_poll_timeout(dsi->dsi_regs + DSI_PHY_STATUS, val,
			       (val & stop) == stop, 10, DPHY_LOCK_TIMEOUT_US))
		dev_err(dsi->dev,
			"data lanes not idle before video, PHY 0x%08x\n", val);
}

static void alif_dw_dsi_phy_power_on(void *priv_data)
{
	struct alif_dw_dsi *dsi = priv_data;
	unsigned int lanes = dsi->phy_opts.mipi_dphy.lanes;

	if (!dsi->tx_active)
		return;

	/*
	 * dw_mipi_dsi_set_mode() always sets TXREQUESTCLKHS, even for
	 * command mode. Zephyr drops that request before LP DCS so the
	 * clock lane returns to LP-11; otherwise BTA/reads time out
	 * ("Read payload FIFO is empty") and the panel ignores 0x11/0x29.
	 * Video mode later raises TXREQUESTCLKHS again in set_mode().
	 */
	if (dsi->dsi_dev && (dsi->dsi_dev->mode_flags & MIPI_DSI_MODE_LPM)) {
		writel(0, dsi->dsi_regs + DSI_LPCLK_CTRL);
		if (!lanes)
			lanes = dsi->dsi_dev->lanes;
		if (lanes)
			alif_dphy_wait_stopstate(dsi, lanes);
	}
}

static void alif_dw_dsi_phy_power_off(void *priv_data)
{
	struct alif_dw_dsi *dsi = priv_data;

	if (!dsi->tx_active)
		return;

	alif_clear_bits(dsi->dsi_regs + DSI_PHY_RSTZ,
			DSI_PHY_RSTZ_FORCEPLL | DSI_PHY_RSTZ_ENABLECLK |
			DSI_PHY_RSTZ_RSTZ | DSI_PHY_RSTZ_SHUTDOWNZ);

	alif_dphy_tx_power(dsi, false);
	clk_disable_unprepare(dsi->tx_clk);
	clk_disable_unprepare(dsi->pll_ref_clk);

	dsi->tx_active = false;
	alif_dsi_release(dsi, ALIF_DSI_USAGE_DISPLAY);
	if (dsi->mux_display)
		mux_state_deselect(dsi->mux_display);
}

static int alif_dw_dsi_get_lane_mbps(void *priv_data,
				     const struct drm_display_mode *mode,
				     unsigned long mode_flags, u32 lanes,
				     u32 format, unsigned int *lane_mbps)
{
	struct alif_dw_dsi *dsi = priv_data;
	unsigned int mbps;
	int bpp, ret;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0) {
		dev_err(dsi->dev, "invalid pixel format %u\n", format);
		return bpp;
	}

	ret = phy_mipi_dphy_get_default_config(mode->clock * 100 * 12, bpp, lanes,
					       &dsi->phy_opts.mipi_dphy);
	if (ret) {
		dev_err(dsi->dev, "failed to get D-PHY config: %d\n", ret);
		return ret;
	}

	/* The PLL output is the DDR clock, half of the per-lane bit rate */
	ret = alif_dphy_calc_pll(dsi, DIV_ROUND_UP_ULL(dsi->phy_opts.mipi_dphy.hs_clk_rate,
						       2 * HZ_PER_KHZ),
				 &dsi->pll);
	if (ret) {
		dev_err(dsi->dev, "no PLL setting for %d kHz pixel clock\n",
			mode->clock);
		return ret;
	}

	mbps = DIV_ROUND_UP(dsi->pll.fout_khz * 2, KHZ_PER_MHZ);

	dsi->tx_range = alif_dphy_get_range(dsi, mbps);
	if (!dsi->tx_range)
		return -EINVAL;

	dsi->phy_opts.mipi_dphy.hs_clk_rate = (u64)dsi->pll.fout_khz * 2 * HZ_PER_KHZ;
	*lane_mbps = mbps;

	dev_dbg(dsi->dev,
		"lane rate %u Mbps, PLL m %u n %u p %u vco 0x%02x\n",
		mbps, dsi->pll.m, dsi->pll.n, dsi->pll.p, dsi->pll.vco_cntrl);

	return 0;
}

static int alif_dw_dsi_phy_get_timing(void *priv_data, unsigned int lane_mbps,
				      struct dw_mipi_dsi_dphy_timing *timing)
{
	struct alif_dw_dsi *dsi = priv_data;
	const struct alif_dphy_freq_range *range = dsi->tx_range;

	if (!range) {
		range = alif_dphy_get_range(dsi, lane_mbps);
		if (!range)
			return -EINVAL;
	}

	timing->clk_hs2lp = range->clk_hs2lp;
	timing->clk_lp2hs = range->clk_lp2hs;
	timing->data_hs2lp = range->lane_hs2lp;
	timing->data_lp2hs = range->lane_lp2hs;

	return 0;
}

static const struct dw_mipi_dsi_phy_ops alif_dw_dsi_phy_ops = {
	.init = alif_dw_dsi_phy_init,
	.power_on = alif_dw_dsi_phy_power_on,
	.power_off = alif_dw_dsi_phy_power_off,
	.get_lane_mbps = alif_dw_dsi_get_lane_mbps,
	.get_timing = alif_dw_dsi_phy_get_timing,
};

static enum drm_mode_status alif_dw_dsi_mode_valid(void *priv_data,
						   const struct drm_display_mode *mode,
						   unsigned long mode_flags,
						   u32 lanes, u32 format)
{
	struct alif_dw_dsi *dsi = priv_data;
	union phy_configure_opts opts;
	struct alif_dphy_pll pll;
	int bpp;

	bpp = mipi_dsi_pixel_format_to_bpp(format);
	if (bpp < 0)
		return MODE_ERROR;

	/* Validating the clock for 20% higher value so as to use burst mode. */
	if (phy_mipi_dphy_get_default_config(mode->clock * 100 * 12, bpp, lanes,
					     &opts.mipi_dphy))
		return MODE_CLOCK_RANGE;

	if (alif_dphy_calc_pll(dsi, DIV_ROUND_UP_ULL(opts.mipi_dphy.hs_clk_rate,
						     2 * HZ_PER_KHZ), &pll))
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

static int alif_dw_dsi_bridge_attach(struct drm_bridge *bridge,
				     enum drm_bridge_attach_flags flags)
{
	struct alif_dw_dsi *dsi = bridge->driver_private;

	return drm_bridge_attach(bridge->encoder, dsi->next_bridge, bridge,
				 flags);
}

static void alif_dw_dsi_bridge_enable(struct drm_bridge *bridge)
{
	/*
	 * Encoder-side of the DW host. This runs after panel prepare and
	 * before dw_mipi_dsi_set_mode(VIDEO), with no DW core hook.
	 */
	alif_dw_dsi_wait_lp_idle(bridge->driver_private);
}

static const struct drm_bridge_funcs alif_dw_dsi_bridge_funcs = {
	.attach = alif_dw_dsi_bridge_attach,
	.enable = alif_dw_dsi_bridge_enable,
};

/*
 * Insert this driver's bridge in front of the DW host so enable() can
 * drain LP TX after panel prepare. Must run after dw_mipi_dsi_probe()
 * returns: host_attach() is called from mipi_dsi_host_register() inside
 * that probe, when dsi->dmd is still NULL and the DW bridge of_node has
 * not been assigned yet. Looking the bridge up then returns NULL
 * (-ENODEV), the panel is removed, and tes-cdc later oopses on the
 * leftover DW bridge.
 */
static int alif_dw_dsi_publish_bridge(struct alif_dw_dsi *dsi)
{
	if (dsi->next_bridge)
		return 0;

	if (!dsi->dmd)
		return -ENODEV;

	dsi->next_bridge = dw_mipi_dsi_get_bridge(dsi->dmd);
	if (!dsi->next_bridge)
		return -ENODEV;

	dsi->bridge.funcs = &alif_dw_dsi_bridge_funcs;
	dsi->bridge.driver_private = dsi;
	dsi->bridge.of_node = dsi->next_bridge->of_node;
	dsi->next_bridge->of_node = NULL;
	drm_bridge_add(&dsi->bridge);

	return 0;
}

static int alif_dw_dsi_host_attach(void *priv_data, struct mipi_dsi_device *device)
{
	struct alif_dw_dsi *dsi = priv_data;

	if (mipi_dsi_pixel_format_to_bpp(device->format) < 0) {
		dev_err(dsi->dev, "unsupported pixel format %u\n",
			device->format);
		return -EINVAL;
	}

	dsi->dsi_dev = device;

	/* dsi->dmd is set only after dw_mipi_dsi_probe() returns. */
	if (!dsi->dmd) {
		dev_info(dsi->dev,
			 "panel %s recorded; publishing DSI bridge after probe\n",
			 dev_name(&device->dev));
		return 0;
	}

	return alif_dw_dsi_publish_bridge(dsi);
}

static int alif_dw_dsi_host_detach(void *priv_data, struct mipi_dsi_device *device)
{
	struct alif_dw_dsi *dsi = priv_data;

	if (device != dsi->dsi_dev)
		return -EINVAL;

	if (dsi->next_bridge) {
		drm_bridge_remove(&dsi->bridge);
		dsi->next_bridge->of_node = dsi->bridge.of_node;
		dsi->next_bridge = NULL;
		dsi->bridge.of_node = NULL;
	}
	dsi->dsi_dev = NULL;

	return 0;
}

static const struct dw_mipi_dsi_host_ops alif_dw_dsi_host_ops = {
	.attach = alif_dw_dsi_host_attach,
	.detach = alif_dw_dsi_host_detach,
};

/*
 * PHY provider: the TX macro turned around so the camera subsystem can
 * receive on the DSI lanes.
 */
static int alif_dphy_rx_init(struct phy *phy)
{
	struct alif_dw_dsi *dsi = phy_get_drvdata(phy);
	int ret;

	ret = alif_dsi_claim(dsi, ALIF_DSI_USAGE_CAMERA);
	if (ret) {
		dev_err(dsi->dev, "D-PHY is in use by the display\n");
		return ret;
	}

	if (dsi->mux_camera) {
		ret = mux_state_try_select(dsi->mux_camera);
		if (ret)
			goto err_mux_camera_select;
	}
	/*
	 * The turned-around macro is still gated by the DSI wrapper, so the
	 * DSI APB clock has to stay up for as long as the camera owns it. The
	 * DSI host only enables it around a modeset, which never happens here.
	 */
	ret = clk_prepare_enable(dsi->pclk);
	if (ret)
		goto err_clk_en;

	return 0;

err_clk_en:
	if (dsi->mux_camera)
		mux_state_deselect(dsi->mux_camera);
err_mux_camera_select:
	alif_dsi_release(dsi, ALIF_DSI_USAGE_CAMERA);

	return ret;
}

static int alif_dphy_rx_exit(struct phy *phy)
{
	struct alif_dw_dsi *dsi = phy_get_drvdata(phy);

	clk_disable_unprepare(dsi->pclk);
	alif_dsi_release(dsi, ALIF_DSI_USAGE_CAMERA);
	if (dsi->mux_camera)
		mux_state_deselect(dsi->mux_camera);

	return 0;
}

static int alif_dphy_rx_validate(struct phy *phy, enum phy_mode mode,
				 int submode, union phy_configure_opts *opts)
{
	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	return 0;
}

static int alif_dphy_rx_configure(struct phy *phy,
				  union phy_configure_opts *opts)
{
	struct alif_dw_dsi *dsi = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *cfg = &opts->mipi_dphy;
	unsigned int mbps;
	int ret;

	if (!cfg->lanes || cfg->lanes > DPHY_MAX_DATA_LANES) {
		dev_err(dsi->dev, "unsupported lane count %u\n", cfg->lanes);
		return -EINVAL;
	}

	ret = phy_mipi_dphy_config_validate(&opts->mipi_dphy);
	if (ret)
		return ret;

	mbps = div_u64(cfg->hs_clk_rate, HZ_PER_MHZ);

	dsi->rx_range = alif_dphy_get_range(dsi, mbps);
	if (!dsi->rx_range)
		return -EINVAL;

	dsi->rx_lanes = cfg->lanes;

	return 0;
}

static int alif_dphy_rx_setup(struct alif_dw_dsi *dsi)
{
	struct alif_testif ifx = {
		.ctrl0 = dsi->csi_regs + CSI_PHY_TST_CTRL0,
		.ctrl1 = dsi->csi_regs + CSI_PHY_TST_CTRL1,
	};
	void __iomem *ctrl0 = dsi->soc_regs + TX_DPHY_CTRL0;
	void __iomem *ctrl1 = dsi->soc_regs + TX_DPHY_CTRL1;
	u32 lane_mask = GENMASK(dsi->rx_lanes - 1, 0);
	u32 mask, val;
	int ret;

	alif_clear_bits(dsi->dsi_regs + DSI_PHY_RSTZ,
			DSI_PHY_RSTZ_RSTZ | DSI_PHY_RSTZ_SHUTDOWNZ);

	/* The lanes are clocked by the sensor, so stop the DSI PLL */
	alif_dphy_pll_bypass(dsi);

	/* Turn the TX macro around and feed it to the CSI receiver */
	alif_update_bits(dsi->soc_regs + DSI_CTRL,
			 DSI_CTRL_PHYSEL_TX_RXN | DSI_CTRL_CAM2_EN,
			 DSI_CTRL_CAM2_EN);

	alif_clear_bits(dsi->csi_regs + CSI_DPHY_RSTZ, CSI_DPHY_RSTZ_RSTZ);
	alif_clear_bits(dsi->csi_regs + CSI_PHY_SHUTDOWNZ,
			CSI_PHY_SHUTDOWNZ_SHUTDOWNZ);

	/* Slave side calibration */
	alif_clear_bits(ctrl0, DPHY_CTRL0_TXRXZ);
	alif_testif_reset(ctrl0, ifx.ctrl0);

	alif_update_bits(ctrl0, DPHY_CTRL0_HS_FREQ_RANGE,
			 FIELD_PREP(DPHY_CTRL0_HS_FREQ_RANGE,
				    dsi->rx_range->hsfreqrange));

	alif_dphy_tx_common_trim(&ifx);
	alif_testif_update(&ifx, TX_PLL_9, TX_PLL_9_LOCK_STATE_OVR_EN, 1);
	alif_testif_write(&ifx, TX_SLEW_0, TX_SLEW_0_SRCAL_EN_OVR_EN);

	/* The remaining trimming lives behind the RX test port */
	alif_set_bits(ctrl0, DPHY_CTRL0_TESTPORT_SEL);

	alif_testif_update(&ifx, RX_CLKLANE_LANE_6,
			   RX_CLKLANE_LANE_6_HSRX_PULL_LONG, 1);

	if (dsi->rx_range->bitrate == alif_dphy_freq_ranges[0].bitrate)
		alif_testif_write(&ifx, RX_SYS_1, RX_SYS_1_80MBPS);

	alif_testif_write(&ifx, RX_STARTUP_OVR_2,
			  FIELD_PREP(RX_STARTUP_OVR_2_OSC_FREQ,
				     dsi->rx_range->osc_freq_target));
	alif_testif_update(&ifx, RX_STARTUP_OVR_3, RX_STARTUP_OVR_3_OSC_FREQ,
			   dsi->rx_range->osc_freq_target >> 8);
	alif_testif_update(&ifx, RX_STARTUP_OVR_4,
			   RX_STARTUP_OVR_4_OSC_FREQ_OVR_EN, 1);

	alif_update_bits(ctrl0, DPHY_CTRL0_CFG_CLK_FREQ_RANGE,
			 FIELD_PREP(DPHY_CTRL0_CFG_CLK_FREQ_RANGE,
				    alif_dphy_cfg_clk_range(dsi)));

	/* Turn the lanes around: base direction RX, forced RX while starting */
	alif_set_bits(ctrl0, FIELD_PREP(DPHY_CTRL0_BASE_DIR, lane_mask));
	alif_set_bits(ctrl1, FIELD_PREP(DPHY_CTRL1_FORCE_RX_MODE, lane_mask));

	fsleep(1);
	alif_set_bits(dsi->csi_regs + CSI_PHY_SHUTDOWNZ,
		      CSI_PHY_SHUTDOWNZ_SHUTDOWNZ);
	fsleep(1);
	alif_set_bits(dsi->csi_regs + CSI_DPHY_RSTZ, CSI_DPHY_RSTZ_RSTZ);

	/* The TX wrapper still gates the macro, so release it too */
	alif_set_bits(dsi->dsi_regs + DSI_PHY_RSTZ,
		      DSI_PHY_RSTZ_SHUTDOWNZ | DSI_PHY_RSTZ_RSTZ |
		      DSI_PHY_RSTZ_FORCEPLL | DSI_PHY_RSTZ_ENABLECLK);

	mask = CSI_PHY_STOPSTATE_CLK | CSI_PHY_STOPSTATE_DATA_0;
	if (dsi->rx_lanes > 1)
		mask |= CSI_PHY_STOPSTATE_DATA_1;

	ret = readl_poll_timeout(dsi->csi_regs + CSI_PHY_STOPSTATE, val,
				 (val & mask) == mask, 10, DPHY_LOCK_TIMEOUT_US);
	if (ret) {
		dev_err(dsi->dev,
			"RX lanes not in stop state, stopstate 0x%08x rx 0x%08x\n",
			val, readl(dsi->csi_regs + CSI_PHY_RX));
		return ret;
	}

	/* Let the PHY track the real lane states from here on */
	alif_clear_bits(ctrl1, FIELD_PREP(DPHY_CTRL1_FORCE_RX_MODE, lane_mask));

	return 0;
}

static int alif_dphy_rx_power_on(struct phy *phy)
{
	struct alif_dw_dsi *dsi = phy_get_drvdata(phy);
	int ret;

	if (!dsi->rx_range) {
		dev_err(dsi->dev, "PHY not configured\n");
		return -EINVAL;
	}

	ret = clk_prepare_enable(dsi->pll_bypass_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(dsi->tx_clk);
	if (ret)
		goto err_bypass_clk;

	alif_dphy_tx_power(dsi, true);

	ret = alif_dphy_rx_setup(dsi);
	if (ret)
		goto err_power;

	return 0;

err_power:
	alif_dphy_tx_power(dsi, false);
	clk_disable_unprepare(dsi->tx_clk);
err_bypass_clk:
	clk_disable_unprepare(dsi->pll_bypass_clk);

	return ret;
}

static int alif_dphy_rx_power_off(struct phy *phy)
{
	struct alif_dw_dsi *dsi = phy_get_drvdata(phy);

	alif_clear_bits(dsi->csi_regs + CSI_DPHY_RSTZ, CSI_DPHY_RSTZ_RSTZ);
	alif_clear_bits(dsi->csi_regs + CSI_PHY_SHUTDOWNZ,
			CSI_PHY_SHUTDOWNZ_SHUTDOWNZ);
	alif_clear_bits(dsi->dsi_regs + DSI_PHY_RSTZ,
			DSI_PHY_RSTZ_FORCEPLL | DSI_PHY_RSTZ_ENABLECLK |
			DSI_PHY_RSTZ_RSTZ | DSI_PHY_RSTZ_SHUTDOWNZ);
	alif_clear_bits(dsi->soc_regs + DSI_CTRL, DSI_CTRL_CAM2_EN);

	alif_dphy_tx_power(dsi, false);
	clk_disable_unprepare(dsi->tx_clk);
	clk_disable_unprepare(dsi->pll_bypass_clk);

	return 0;
}

static const struct phy_ops alif_dphy_rx_ops = {
	.init = alif_dphy_rx_init,
	.exit = alif_dphy_rx_exit,
	.configure = alif_dphy_rx_configure,
	.validate = alif_dphy_rx_validate,
	.power_on = alif_dphy_rx_power_on,
	.power_off = alif_dphy_rx_power_off,
	.owner = THIS_MODULE,
};

static int alif_dw_dsi_map_resources(struct platform_device *pdev,
				     struct alif_dw_dsi *dsi)
{
	struct device *dev = &pdev->dev;
	struct device_node *dphy_np;
	struct resource *mem, res;
	u32 base;
	int ret;

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsi");
	if (mem)
		dsi->dsi_regs = devm_ioremap_resource(dev, mem);
	else
		dsi->dsi_regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsi->dsi_regs))
		return dev_err_probe(dev, PTR_ERR(dsi->dsi_regs),
				     "failed to map the DSI host registers\n");

	/*
	 * EXPMST D-PHY control and VBAT power live on dphy@4903f000.
	 * Map them without requesting the region so the D-PHY driver
	 * can own the same MMIO.
	 */
	dphy_np = of_parse_phandle(dev->of_node, "alif,dphy", 0);
	if (!dphy_np)
		return dev_err_probe(dev, -ENODEV, "missing alif,dphy phandle\n");

	ret = of_address_to_resource(dphy_np, 0, &res);
	if (ret) {
		of_node_put(dphy_np);
		return dev_err_probe(dev, ret,
				     "failed to get the SoC D-PHY registers\n");
	}
	dsi->soc_regs = devm_ioremap(dev, res.start, resource_size(&res));
	if (!dsi->soc_regs) {
		of_node_put(dphy_np);
		return dev_err_probe(dev, -ENOMEM,
				     "failed to map the SoC D-PHY registers\n");
	}

	ret = of_address_to_resource(dphy_np, 1, &res);
	if (ret) {
		of_node_put(dphy_np);
		return dev_err_probe(dev, ret,
				     "failed to get the VBAT registers\n");
	}
	dsi->vbat_regs = devm_ioremap(dev, res.start, resource_size(&res));
	of_node_put(dphy_np);
	if (!dsi->vbat_regs)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to map the VBAT registers\n");

	/*
	 * The CSI region is shared with the CSI-2 host, so map it without
	 * reserving it. It is only needed to lend the TX macro to the camera.
	 */
	if (!of_property_read_u32(dev->of_node, "alif,csi-base", &base)) {
		dsi->csi_regs = devm_ioremap(dev, base, SZ_4K);
		if (!dsi->csi_regs)
			return dev_err_probe(dev, -ENOMEM,
					     "failed to map the CSI registers\n");
	}

	return 0;
}

static int alif_dw_dsi_get_clocks(struct alif_dw_dsi *dsi)
{
	struct device *dev = dsi->dev;

	dsi->tx_clk = devm_clk_get(dev, "tx_clk");
	if (IS_ERR(dsi->tx_clk))
		return dev_err_probe(dev, PTR_ERR(dsi->tx_clk),
				     "failed to get tx_clk\n");

	dsi->pll_ref_clk = devm_clk_get(dev, "pll_ref_clk");
	if (IS_ERR(dsi->pll_ref_clk))
		return dev_err_probe(dev, PTR_ERR(dsi->pll_ref_clk),
				     "failed to get pll_ref_clk\n");

	/*
	 * Only the turned-around RX path needs these two. It clocks the macro
	 * from the bypass clock, and it programs the DSI wrapper outside of any
	 * modeset, so it cannot rely on the DSI host's own pclk reference.
	 */
	if (dsi->csi_regs) {
		dsi->pclk = devm_clk_get(dev, "pclk");
		if (IS_ERR(dsi->pclk))
			return dev_err_probe(dev, PTR_ERR(dsi->pclk),
					     "failed to get pclk\n");

		dsi->pll_bypass_clk = devm_clk_get(dev, "pll_bypass_clk");
		if (IS_ERR(dsi->pll_bypass_clk))
			return dev_err_probe(dev, PTR_ERR(dsi->pll_bypass_clk),
					     "failed to get pll_bypass_clk\n");
	}

	if (of_property_read_u32(dev->of_node, "ref-frequency", &dsi->ref_freq)) {
		dsi->ref_freq = clk_get_rate(dsi->pll_ref_clk);
		if (!dsi->ref_freq)
			dsi->ref_freq = DPHY_REF_FREQ_DEFAULT;
	}

	if (of_property_read_u32(dev->of_node, "cfg-clk-frequency",
				 &dsi->cfg_freq))
		dsi->cfg_freq = DPHY_CFG_FREQ_DEFAULT;

	return 0;
}

static int alif_dw_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct alif_dw_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;
	mutex_init(&dsi->usage_mutex);
	platform_set_drvdata(pdev, dsi);

	ret = alif_dw_dsi_map_resources(pdev, dsi);
	if (ret)
		return ret;

	ret = alif_dw_dsi_get_clocks(dsi);
	if (ret)
		return ret;

	if (of_property_present(dev->of_node, "mux-states")) {
		dsi->mux_display = devm_mux_state_get(dev, "display");
		if (IS_ERR(dsi->mux_display))
			return dev_err_probe(dev, PTR_ERR(dsi->mux_display),
					     "failed to get display mux state\n");

		dsi->mux_camera = devm_mux_state_get(dev, "camera");
		if (IS_ERR(dsi->mux_camera))
			return dev_err_probe(dev, PTR_ERR(dsi->mux_camera),
					     "failed to get camera mux state\n");
	}

	/* RMW: do not clear USB PHY or RX D-PHY isolation (HWRM 8.3.7.3.3). */
	alif_set_bits(dsi->vbat_regs + VBAT_PWR_CTRL, VBAT_PWR_ON_INIT);

	if (dsi->csi_regs) {
		dsi->rx_phy = devm_phy_create(dev, NULL, &alif_dphy_rx_ops);
		if (IS_ERR(dsi->rx_phy))
			return dev_err_probe(dev, PTR_ERR(dsi->rx_phy),
					     "failed to create the RX PHY\n");

		phy_set_drvdata(dsi->rx_phy, dsi);

		phy_provider = devm_of_phy_provider_register(dev,
							     of_phy_simple_xlate);
		if (IS_ERR(phy_provider))
			return dev_err_probe(dev, PTR_ERR(phy_provider),
					     "failed to register the PHY provider\n");
	} else {
		dev_dbg(dev,
			"no alif,csi-base, the TX D-PHY cannot be lent to the camera\n");
	}

	dsi->pdata.base = dsi->dsi_regs;
	dsi->pdata.max_data_lanes = DPHY_MAX_DATA_LANES;
	dsi->pdata.mode_valid = alif_dw_dsi_mode_valid;
	dsi->pdata.phy_ops = &alif_dw_dsi_phy_ops;
	dsi->pdata.host_ops = &alif_dw_dsi_host_ops;
	dsi->pdata.priv_data = dsi;

	dsi->dmd = dw_mipi_dsi_probe(pdev, &dsi->pdata);
	if (IS_ERR(dsi->dmd))
		return dev_err_probe(dev, PTR_ERR(dsi->dmd),
				     "failed to probe dw_mipi_dsi\n");

	if (dsi->dsi_dev) {
		ret = alif_dw_dsi_publish_bridge(dsi);
		if (ret) {
			dw_mipi_dsi_remove(dsi->dmd);
			return dev_err_probe(dev, ret,
					     "failed to publish the DSI bridge\n");
		}
	}

	return 0;
}

static void alif_dw_dsi_remove(struct platform_device *pdev)
{
	struct alif_dw_dsi *dsi = platform_get_drvdata(pdev);

	dw_mipi_dsi_remove(dsi->dmd);
}

static const struct of_device_id alif_dw_dsi_of_match[] = {
	{ .compatible = "alif,ensemble-dsi" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, alif_dw_dsi_of_match);

static struct platform_driver alif_dw_dsi_driver = {
	.probe = alif_dw_dsi_probe,
	.remove_new = alif_dw_dsi_remove,
	.driver = {
		.name = "alif-dw-mipi-dsi",
		.of_match_table = alif_dw_dsi_of_match,
	},
};
module_platform_driver(alif_dw_dsi_driver);

MODULE_AUTHOR("Yogender Kumar Arya <yogender.kumar@alifsemi.com>");
MODULE_DESCRIPTION("Alif DesignWare MIPI DSI host driver");
MODULE_LICENSE("GPL");
