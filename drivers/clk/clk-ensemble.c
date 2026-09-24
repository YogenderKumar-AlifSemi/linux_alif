// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2024 Alif Semiconductor.
 * Author: Harith George <harith.g@alifsemi.com>
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sizes.h>
#include <linux/clk/ensemble-clk.h>
#include <dt-bindings/clock/alif,ensemble-clock.h>

#define CCPSLV_BASE				0x4902F000
#define CCPMST_BASE				0x4903F000
#define CGU_BASE				0x1A602000
#define VBAT_BASE				0x1A609000
#define CLKCTL_SYS_BASE				0x1A010000
#define HOSTCPUCLK_CTRL				0x800
#define HOSTCPUCLK_DIV1				0x808
#define RTC_CLK_DIVIDER				14648
#define PCLK_FORCE				BIT(30)
#define IPCLK_FORCE				BIT(31)
#define HFXO_76M8_CLK				76800000
#define EXT_AUDIO_CLK				76800000
#define HOSTCPUCLK_DIV_MASK			0x1F

static struct clk_hw **hws;
static struct clk_hw_onecell_data *clk_hw_data;
static void __iomem *clkctl_sys_base;
static unsigned long pll_clk1_rate;

/**
 * Custom CPU clock structure for dynamic rate reading.
 * The standard clk_divider caches the rate, but we need clk_summary
 * to show the actual current frequency by reading the register.
 */
struct ensemble_cpu_clk {
	struct clk_hw hw;
	void __iomem *reg;
	spinlock_t *lock; /* Protects CPU clock rate updates */
};

#define to_ensemble_cpu_clk(_hw) container_of(_hw, struct ensemble_cpu_clk, hw)

static unsigned long ensemble_cpu_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ensemble_cpu_clk *cpu = to_ensemble_cpu_clk(hw);
	u32 reg, div;

	reg = readl(cpu->reg);
	/* Zero-based: reg_val + 1 = divider */
	div = (reg & HOSTCPUCLK_DIV_MASK) + 1;

	return parent_rate / div;
}

static long ensemble_cpu_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	unsigned long div;

	if (rate == 0)
		return *parent_rate;

	if (rate == 0)
		div = 1;
	else
		div = DIV_ROUND_CLOSEST(*parent_rate, rate);

	return *parent_rate / div;
}

static int ensemble_cpu_clk_hw_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct ensemble_cpu_clk *cpu = to_ensemble_cpu_clk(hw);
	unsigned long flags;
	u32 div, reg;

	if (rate == 0)
		div = 1;
	else
		div = DIV_ROUND_CLOSEST(parent_rate, rate);

	spin_lock_irqsave(cpu->lock, flags);

	reg = readl(cpu->reg);
	reg &= ~(HOSTCPUCLK_DIV_MASK);
	reg |= (div - 1) & HOSTCPUCLK_DIV_MASK;
	writel(reg, cpu->reg);

	/* Readback barrier */
	(void)readl(cpu->reg);

	spin_unlock_irqrestore(cpu->lock, flags);

	return 0;
}

static const struct clk_ops ensemble_cpu_clk_ops = {
	.recalc_rate = ensemble_cpu_clk_recalc_rate,
	.round_rate = ensemble_cpu_clk_round_rate,
	.set_rate = ensemble_cpu_clk_hw_set_rate,
};

static struct ensemble_cpu_clk *cpu_clk_data;

static const char *const uart_clk_src_sels[] = { "hfxo", "syst_pclk", };
static const char *const canfd_clk_src_sels[] = { "hfosc_clk", "160m_clk", };
static const char *const audio_clk_src_sels[] = { "cgu_76m8_clk", "audio_clk", };
static const char *const pixclk_sels[] = {"syst_aclk", "pll_clk3", };
static const char *const eth_clk_sels[] = {"eth_refclk", "50m_clk"};
static const char *const adc_clk_sels[] = {"syst_pclk", "160m_clk"};
static const char *const cmp_clk_sels[] = {"syst_pclk", "160m_clk"};
static const char *const sd_clk_sels[] = {"syst_hclk", "100m_clk"};
struct clk_hw *__ensemble_clk_hw_composite(const char *name,
					   const char * const *parent_names,
					   int num_parents, void __iomem *reg,
					   u32 composite_flags,
					   unsigned long flags);

DEFINE_SPINLOCK(ensemble_ccps_lock);
EXPORT_SYMBOL_GPL(ensemble_ccps_lock);

static inline struct clk_hw *__ensemble_clk_hw_mux(const char *name, void __iomem *reg,
						   u8 shift, u8 width, const char * const *parents,
						   int num_parents, unsigned long flags,
						   unsigned long clk_mux_flags)
{
	return clk_hw_register_mux(NULL, name, parents, num_parents,
				   flags | CLK_SET_RATE_NO_REPARENT, reg, shift,
				   width, clk_mux_flags, &ensemble_ccps_lock);
}

static inline struct clk_hw *__ensemble_clk_hw_gate(const char *name, const char *parent,
						    void __iomem *reg, u8 shift,
						    unsigned long flags,
						    unsigned long clk_gate_flags)
{
	return clk_hw_register_gate(NULL, name, parent, flags | CLK_SET_RATE_PARENT, reg,
				    shift, clk_gate_flags, &ensemble_ccps_lock);
}

static inline struct clk_hw *ensemble_clk_hw_fixed_factor(const char *name, const char *parent,
							  unsigned int mult, unsigned int div)
{
	return clk_hw_register_fixed_factor(NULL, name, parent,
					    CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk_hw *ensemble_clk_hw_divider(const char *name, const char *parent_name,
						     void __iomem *reg, u8 shift, u8 width)
{
	return clk_hw_register_divider(NULL, name, parent_name,
				       CLK_SET_RATE_PARENT, reg,
				       shift, width, CLK_DIVIDER_ONE_BASED,
				       &ensemble_ccps_lock);
}

/**
 * find_child_clock_by_name - Find a child clock node by its output name
 * @parent: Parent device node to search under
 * @clk: Clock output name to match against "clock-output-names" property
 *
 * Returns the matching child device_node or NULL if not found.
 */
static struct device_node *find_child_clock_by_name(struct device_node *parent, const char *clk)
{
	struct device_node *child;
	const char *name;

	for_each_child_of_node(parent, child) {
		if (!of_property_read_string(child, "clock-output-names", &name)) {
			if (!strcmp(name, clk))
				return child;
		}
	}

	return NULL;
}

#define CC_DIV_SHIFT		16
#define CC_DIV_WIDTH		9
#define CC_DIV_MASK		(((1U << CC_DIV_WIDTH) - 1) << CC_DIV_SHIFT)

#define CC_MUX_SHIFT		4
#define CC_MUX_MASK		0x1

#define CC_GATE_SHIFT		0

struct clk_hw *__ensemble_clk_hw_composite(const char *name, const char * const *parent_names,
					   int num_parents, void __iomem *reg, u32 composite_flags,
					   unsigned long flags)
{
	struct clk_hw *hw = ERR_PTR(-ENOMEM), *mux_hw;
	struct clk_hw *div_hw, *gate_hw = NULL;
	struct clk_divider *div;
	struct clk_gate *gate = NULL;
	struct clk_mux *mux;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return ERR_CAST(hw);

	mux_hw = &mux->hw;
	mux->reg = reg;
	mux->shift = CC_MUX_SHIFT;
	mux->mask = CC_MUX_MASK;
	mux->lock = &ensemble_ccps_lock;

	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		goto free_mux;

	div_hw = &div->hw;
	div->reg = reg;
	div->shift = CC_DIV_SHIFT;
	div->width = CC_DIV_WIDTH;
	div->flags = CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_ONE_BASED;
	div->lock = &ensemble_ccps_lock;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		goto free_div;

	gate_hw = &gate->hw;
	gate->reg = reg;
	gate->bit_idx = CC_GATE_SHIFT;
	gate->lock = &ensemble_ccps_lock;

	hw = clk_hw_register_composite(NULL, name, parent_names,
				       num_parents, mux_hw, &clk_mux_ops, div_hw, &clk_divider_ops,
				       gate_hw, &clk_gate_ops, flags);
	if (IS_ERR(hw))
		goto free_gate;

	return hw;

free_gate:
	kfree(gate);
free_div:
	kfree(div);
free_mux:
	kfree(mux);
	return ERR_CAST(hw);
}

#define ensemble_clk_hw_mux(name, reg, shift, width, parents, num_parents) \
	__ensemble_clk_hw_mux(name, reg, shift, width, parents, num_parents, 0, 0)

#define ensemble_clk_hw_gate_clk_flags(name, parent, reg, shift, flags, gate_flags) \
	__ensemble_clk_hw_gate(name, parent, reg, shift, flags, gate_flags)

#define ensemble_clk_hw_gate_flags(name, parent, reg, shift, flags) \
	__ensemble_clk_hw_gate(name, parent, reg, shift, flags, 0)

#define ensemble_clk_hw_gate(name, parent, reg, shift) \
	ensemble_clk_hw_gate_flags(name, parent, reg, shift, 0)

#define _ensemble_clk_hw_composite(name, parent_names, num_parents, reg, composite_flags, flags) \
	__ensemble_clk_hw_composite(name, parent_names, num_parents, reg, composite_flags, flags)

#define ensemble_clk_hw_composite(name, parent_names, num_parents, reg) \
	_ensemble_clk_hw_composite(name, parent_names, num_parents, reg, \
				   0, CLK_SET_RATE_NO_REPARENT)

static void __init ensemble_clocks_init(struct device_node *ccps_node)
{
	struct device_node *np = ccps_node;
	void __iomem *base, *cgu_base, *ccpmst_base, *vbat_base;
	struct clk *clk;
	struct device_node *iter;
	u32 clk_div;
	u32 val;
	int ret;

	base = ioremap(CCPSLV_BASE, SZ_4K);
	cgu_base = ioremap(CGU_BASE, SZ_256);
	ccpmst_base = ioremap(CCPMST_BASE, SZ_4K);
	vbat_base = ioremap(VBAT_BASE, SZ_4K);
	/* Enable Peripheral functional clocks and APB interface clocks. */
	writel(PCLK_FORCE | IPCLK_FORCE, base);

	clk_hw_data = kzalloc(struct_size(clk_hw_data, hws,
					  ENSEMBLE_CLK_END), GFP_KERNEL);
	if (WARN_ON(!clk_hw_data))
		return;

	clk_hw_data->num = ENSEMBLE_CLK_END;
	hws = clk_hw_data->hws;

	hws[ENSEMBLE_DUMMY_CLK] = clk_hw_register_fixed_rate(NULL, "dummy", NULL, 0, 0);

	clk = of_clk_get_by_name(ccps_node, "hfxo");
	hws[ENSEMBLE_HFXO_CLK] =  __clk_get_hw(clk);
	clk = of_clk_get_by_name(ccps_node, "pll_clk1");
	hws[ENSEMBLE_PLL_CLK1] =  __clk_get_hw(clk);
	clk = of_clk_get_by_name(ccps_node, "pll_clk3");
	hws[ENSEMBLE_PLL_CLK3] =  __clk_get_hw(clk);

	/* Register the fixed 50MHz REFCLK from the ETH PHY */
	hws[ENSEMBLE_ETH_REFCLK] = clk_hw_register_fixed_rate(NULL, "eth_refclk", NULL, 0,
							      50000000);
	hws[ENSEMBLE_76M8_CLK] = clk_hw_register_fixed_rate(NULL, "76m8_clk", NULL, 0,
							    HFXO_76M8_CLK);
	hws[ENSEMBLE_AUDIO_CLK] = clk_hw_register_fixed_rate(NULL, "audio_clk", NULL, 0,
							     EXT_AUDIO_CLK);

	hws[ENSEMBLE_SYST_ACLK] =  ensemble_clk_hw_fixed_factor("syst_aclk", "pll_clk1", 1, 2);
	hws[ENSEMBLE_SYST_HCLK] = ensemble_clk_hw_fixed_factor("syst_hclk", "syst_aclk", 1, 2);
	hws[ENSEMBLE_SYST_PCLK] = ensemble_clk_hw_fixed_factor("syst_pclk", "syst_aclk", 1, 4);
	hws[ENSEMBLE_50M_CLK] = ensemble_clk_hw_fixed_factor("50m_clk", "pll_clk1", 1, 16);
	hws[ENSEMBLE_100M_SCLK] = ensemble_clk_hw_fixed_factor("100m_sclk", "pll_clk1", 1, 8);
	hws[ENSEMBLE_100M_CLK] = ensemble_clk_hw_gate("100m_clk", "100m_sclk", cgu_base + 0x14, 7);
	hws[ENSEMBLE_HFOSC_CLK] = ensemble_clk_hw_gate("hfosc_clk", "hfxo", cgu_base + 0x14, 23);
	hws[ENSEMBLE_160M_SCLK] = ensemble_clk_hw_fixed_factor("160m_sclk", "pll_clk1", 1, 5);
	hws[ENSEMBLE_160M_CLK] = ensemble_clk_hw_gate("160m_clk", "160m_sclk", cgu_base + 0x14, 20);
	hws[ENSEMBLE_USB_SCLK] = ensemble_clk_hw_fixed_factor("usb_sclk", "pll_clk3", 1, 24);
	hws[ENSEMBLE_USB_CLK] = ensemble_clk_hw_gate("usb_clk", "usb_sclk", cgu_base + 0x14, 22);
	hws[ENSEMBLE_266M_SCLK] = ensemble_clk_hw_fixed_factor("266m_sclk", "pll_clk1", 1, 3);
	hws[ENSEMBLE_266M_CLK] = ensemble_clk_hw_gate("266m_clk", "266m_sclk", cgu_base + 0x14, 21);

	hws[ENSEMBLE_RTC_CLK] = ensemble_clk_hw_fixed_factor("s32k_clk",
							     "pll_clk3", 1, RTC_CLK_DIVIDER);
	hws[ENSEMBLE_S32K_CLK] = ensemble_clk_hw_gate("timer", "s32k_clk", vbat_base + 0x10, 0);

	hws[ENSEMBLE_UART0_CLK_SEL] = ensemble_clk_hw_mux("uart0_sclk",
							  base + 0x8, 8, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));
	hws[ENSEMBLE_UART1_CLK_SEL] = ensemble_clk_hw_mux("uart1_sclk",
							  base + 0x8, 9, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));
	hws[ENSEMBLE_UART2_CLK_SEL] = ensemble_clk_hw_mux("uart2_sclk",
							  base + 0x8, 10, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));
	hws[ENSEMBLE_UART3_CLK_SEL] = ensemble_clk_hw_mux("uart3_sclk",
							  base + 0x8, 11, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));
	hws[ENSEMBLE_UART4_CLK_SEL] = ensemble_clk_hw_mux("uart4_sclk",
							  base + 0x8, 12, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));
	hws[ENSEMBLE_UART5_CLK_SEL] = ensemble_clk_hw_mux("uart5_sclk",
							  base + 0x8, 13, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));
	hws[ENSEMBLE_UART6_CLK_SEL] = ensemble_clk_hw_mux("uart6_sclk",
							  base + 0x8, 14, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));
	hws[ENSEMBLE_UART7_CLK_SEL] = ensemble_clk_hw_mux("uart7_sclk",
							  base + 0x8, 15, 1, uart_clk_src_sels,
							  ARRAY_SIZE(uart_clk_src_sels));

	clk_set_parent(hws[ENSEMBLE_UART0_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);
	clk_set_parent(hws[ENSEMBLE_UART1_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);
	clk_set_parent(hws[ENSEMBLE_UART2_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);
	clk_set_parent(hws[ENSEMBLE_UART3_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);
	clk_set_parent(hws[ENSEMBLE_UART4_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);
	clk_set_parent(hws[ENSEMBLE_UART5_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);
	clk_set_parent(hws[ENSEMBLE_UART6_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);
	clk_set_parent(hws[ENSEMBLE_UART7_CLK_SEL]->clk, hws[ENSEMBLE_SYST_PCLK]->clk);

	hws[ENSEMBLE_UART0_CLK] = ensemble_clk_hw_gate("uart0_clk", "uart0_sclk", base + 0x8, 0);
	hws[ENSEMBLE_UART1_CLK] = ensemble_clk_hw_gate("uart1_clk", "uart1_sclk", base + 0x8, 1);
	hws[ENSEMBLE_UART2_CLK] = ensemble_clk_hw_gate("uart2_clk", "uart2_sclk", base + 0x8, 2);
	hws[ENSEMBLE_UART3_CLK] = ensemble_clk_hw_gate("uart3_clk", "uart3_sclk", base + 0x8, 3);
	hws[ENSEMBLE_UART4_CLK] = ensemble_clk_hw_gate("uart4_clk", "uart4_sclk", base + 0x8, 4);
	hws[ENSEMBLE_UART5_CLK] = ensemble_clk_hw_gate("uart5_clk", "uart5_sclk", base + 0x8, 5);
	hws[ENSEMBLE_UART6_CLK] = ensemble_clk_hw_gate("uart6_clk", "uart6_sclk", base + 0x8, 6);
	hws[ENSEMBLE_UART7_CLK] = ensemble_clk_hw_gate("uart7_clk", "uart7_sclk", base + 0x8, 7);

	hws[ENSEMBLE_CANFD_CLK_SEL] = ensemble_clk_hw_mux("canfd_sclk",
							  base + 0xC, 16, 1, canfd_clk_src_sels,
							  ARRAY_SIZE(canfd_clk_src_sels));
	clk_set_parent(hws[ENSEMBLE_CANFD_CLK_SEL]->clk, hws[ENSEMBLE_HFOSC_CLK]->clk);
	hws[ENSEMBLE_CANFD_CLK] = ensemble_clk_hw_gate("canfd_clk", "canfd_sclk", base + 0xc, 12);

	hws[ENSEMBLE_I3C_CLK] = ensemble_clk_hw_gate("i3c_clk", "syst_pclk", base + 0x24, 0);
	hws[ENSEMBLE_DAC121_CLK] = ensemble_clk_hw_gate("dac121_clk", "syst_pclk", base + 0x34, 4);
	hws[ENSEMBLE_DAC120_CLK] = ensemble_clk_hw_gate("dac120_clk", "syst_pclk", base + 0x34, 0);
	hws[ENSEMBLE_DWC_USB_CLK] = ensemble_clk_hw_gate("dwc_clk", "syst_pclk",
							 ccpmst_base + 0xC, 20);

	/**
	 * camera_pixclk shares ccpmst_base + 0x0 bit 0 with the sensor's
	 * extclk gate. If the unused-clock cleanup disables camera_pixclk,
	 * it also clears that shared bit and turns off extclk, breaking
	 * ARX3A0 I2C. Keep this clock enabled until the gate sharing is
	 * modelled explicitly.
	 */
	hws[ENSEMBLE_CAMERA_PIXCLK] = _ensemble_clk_hw_composite("camera_pixclk", pixclk_sels,
								 ARRAY_SIZE(pixclk_sels),
								 ccpmst_base + 0x0,
								 0, CLK_IGNORE_UNUSED);
	hws[ENSEMBLE_CDC200_PIXCLK] = ensemble_clk_hw_composite("cdc200_pixclk", pixclk_sels,
								ARRAY_SIZE(pixclk_sels),
								ccpmst_base + 0x4);
	hws[ENSEMBLE_CSI_PIXCLK] = ensemble_clk_hw_composite("csi_pixclk", pixclk_sels,
							     ARRAY_SIZE(pixclk_sels),
							     ccpmst_base + 0x8);
	hws[ENSEMBLE_CSI_APB] =
		ensemble_clk_hw_gate("csi_apb", "syst_pclk", ccpmst_base + 0xC, 24);
	hws[ENSEMBLE_CPI_APB] =
		ensemble_clk_hw_gate("cpi_apb", "syst_pclk", ccpmst_base + 0xC, 0);

	hws[ENSEMBLE_DSI_APB] =
		ensemble_clk_hw_gate("dsi_apb", "syst_pclk", ccpmst_base + 0xC, 28);
	hws[ENSEMBLE_CDC200_APB] =
		ensemble_clk_hw_gate("cdc200_apb", "syst_pclk", ccpmst_base + 0xC, 1);

	hws[ENSEMBLE_MIPI_BYPASS_CLK] = ensemble_clk_hw_gate("mipi_bypass_clk",
							     "pll_clk1", ccpmst_base + 0x40, 12);
	hws[ENSEMBLE_MIPI_PLLREF_CLK] = ensemble_clk_hw_gate("mipi_pllref_clk",
							     "hfosc_clk", ccpmst_base + 0x40, 8);
	hws[ENSEMBLE_MIPI_RXDPHY_CLK] = ensemble_clk_hw_gate("mipi_rxdphy_clk",
							     "hfosc_clk", ccpmst_base + 0x40, 4);
	hws[ENSEMBLE_MIPI_TXDPHY_CLK] = ensemble_clk_hw_gate("mipi_txdphy_clk",
							     "hfosc_clk", ccpmst_base + 0x40, 0);

	hws[ENSEMBLE_ETH_CSR_CLK] = ensemble_clk_hw_gate("eth_csr_clk",
							 "syst_hclk", ccpmst_base + 0xC, 12);

	hws[ENSEMBLE_ETH_CLK] = ensemble_clk_hw_mux("eth_clk",
						    ccpmst_base + 0x80, 4, 1, eth_clk_sels,
						    ARRAY_SIZE(eth_clk_sels));

	clk_set_parent(hws[ENSEMBLE_ETH_CLK]->clk, hws[ENSEMBLE_ETH_REFCLK]->clk);

	hws[ENSEMBLE_ADC120_CLK] = ensemble_clk_hw_mux("adc120_clk",
						       base + 0x30, 0, 1, adc_clk_sels,
						       ARRAY_SIZE(adc_clk_sels));
	hws[ENSEMBLE_ADC121_CLK] = ensemble_clk_hw_mux("adc121_clk",
						       base + 0x30, 4, 1, adc_clk_sels,
						       ARRAY_SIZE(adc_clk_sels));
	hws[ENSEMBLE_ADC122_CLK] = ensemble_clk_hw_mux("adc122_clk",
						       base + 0x30, 8, 1, adc_clk_sels,
						       ARRAY_SIZE(adc_clk_sels));
	hws[ENSEMBLE_ADC24_CLK] = ensemble_clk_hw_mux("adc24_clk",
						      base + 0x30, 12, 1, adc_clk_sels,
						      ARRAY_SIZE(adc_clk_sels));

	clk_set_parent(hws[ENSEMBLE_ADC120_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_ADC121_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_ADC122_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_ADC24_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);

	hws[ENSEMBLE_CMP0_CLK] = ensemble_clk_hw_mux("cmp0_clk", base + 0x38, 0, 1, cmp_clk_sels,
						     ARRAY_SIZE(cmp_clk_sels));
	hws[ENSEMBLE_CMP1_CLK] = ensemble_clk_hw_mux("cmp1_clk", base + 0x38, 4, 1, cmp_clk_sels,
						     ARRAY_SIZE(cmp_clk_sels));
	hws[ENSEMBLE_CMP2_CLK] = ensemble_clk_hw_mux("cmp2_clk", base + 0x38, 8, 1, cmp_clk_sels,
						     ARRAY_SIZE(cmp_clk_sels));
	hws[ENSEMBLE_CMP3_CLK] = ensemble_clk_hw_mux("cmp3_clk", base + 0x38, 12, 1, cmp_clk_sels,
						     ARRAY_SIZE(cmp_clk_sels));

	clk_set_parent(hws[ENSEMBLE_CMP0_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_CMP1_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_CMP2_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_CMP3_CLK]->clk, hws[ENSEMBLE_160M_CLK]->clk);

	hws[ENSEMBLE_SPI0_SS_IN_SEL_MST_CLK] =
		ensemble_clk_hw_gate("spi0_ssi_in_sel_clk", "syst_hclk", base + 0x28, 0);
	hws[ENSEMBLE_SPI1_SS_IN_SEL_MST_CLK] =
		ensemble_clk_hw_gate("spi1_ssi_in_sel_clk", "syst_hclk", base + 0x28, 1);
	hws[ENSEMBLE_SPI2_SS_IN_SEL_MST_CLK] =
		ensemble_clk_hw_gate("spi2_ssi_in_sel_clk", "syst_hclk", base + 0x28, 2);
	hws[ENSEMBLE_SPI3_SS_IN_SEL_MST_CLK] =
		ensemble_clk_hw_gate("spi3_ssi_in_sel_clk", "syst_hclk", base + 0x28, 3);
	hws[ENSEMBLE_SPI0_SS_IN_VAL_MST_CLK] =
		ensemble_clk_hw_gate("spi0_ssi_in_val_clk", "spi0_ssi_in_sel_clk", base + 0x28, 8);
	hws[ENSEMBLE_SPI1_SS_IN_VAL_MST_CLK] =
		ensemble_clk_hw_gate("spi1_ssi_in_val_clk", "spi1_ssi_in_sel_clk", base + 0x28, 9);
	hws[ENSEMBLE_SPI2_SS_IN_VAL_MST_CLK] =
		ensemble_clk_hw_gate("spi2_ssi_in_val_clk", "spi2_ssi_in_sel_clk", base + 0x28, 10);
	hws[ENSEMBLE_SPI3_SS_IN_VAL_MST_CLK] =
		ensemble_clk_hw_gate("spi3_ssi_in_val_clk", "spi3_ssi_in_sel_clk", base + 0x28, 11);
	hws[ENSEMBLE_SPI0_SS_IN_SEL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi0_ssi_slv_in_sel_clk", "syst_hclk",
					       base + 0x28, 0, 0, CLK_GATE_SET_TO_DISABLE);
	hws[ENSEMBLE_SPI1_SS_IN_SEL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi1_ssi_slv_in_sel_clk", "syst_hclk",
					       base + 0x28, 1, 0, CLK_GATE_SET_TO_DISABLE);
	hws[ENSEMBLE_SPI2_SS_IN_SEL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi2_ssi_slv_in_sel_clk", "syst_hclk",
					       base + 0x28, 2, 0, CLK_GATE_SET_TO_DISABLE);
	hws[ENSEMBLE_SPI3_SS_IN_SEL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi3_ssi_slv_in_sel_clk", "syst_hclk",
					       base + 0x28, 3, 0, CLK_GATE_SET_TO_DISABLE);
	hws[ENSEMBLE_SPI0_SS_IN_VAL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi0_ssi_slv_in_val_clk", "spi0_ssi_slv_in_sel_clk",
					       base + 0x28, 8, 0, CLK_GATE_SET_TO_DISABLE);
	hws[ENSEMBLE_SPI1_SS_IN_VAL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi1_ssi_slv_in_val_clk", "spi1_ssi_slv_in_sel_clk",
					       base + 0x28, 9, 0, CLK_GATE_SET_TO_DISABLE);
	hws[ENSEMBLE_SPI2_SS_IN_VAL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi2_ssi_slv_in_val_clk", "spi2_ssi_slv_in_sel_clk",
					       base + 0x28, 10, 0, CLK_GATE_SET_TO_DISABLE);
	hws[ENSEMBLE_SPI3_SS_IN_VAL_SLV_CLK] =
		ensemble_clk_hw_gate_clk_flags("spi3_ssi_slv_in_val_clk", "spi3_ssi_slv_in_sel_clk",
					       base + 0x28, 11, 0, CLK_GATE_SET_TO_DISABLE);

	hws[ENSEMBLE_SD_CLK] = ensemble_clk_hw_mux("sdhci_clk",
						   ccpmst_base + 0xC, 16, 1, sd_clk_sels,
						   ARRAY_SIZE(sd_clk_sels));
	clk_set_parent(hws[ENSEMBLE_SD_CLK]->clk, hws[ENSEMBLE_100M_CLK]->clk);

	hws[ENSEMBLE_CGU_76M8_CLK] = ensemble_clk_hw_gate("cgu_76m8_clk",
							  "76m8_clk", cgu_base + 0x14, 24);

	hws[ENSEMBLE_PDM_SCLK] = ensemble_clk_hw_mux("pdm_sclk",
						     base + 0x0, 9, 1, audio_clk_src_sels,
						     ARRAY_SIZE(audio_clk_src_sels));

	clk_set_parent(hws[ENSEMBLE_PDM_SCLK]->clk, hws[ENSEMBLE_CGU_76M8_CLK]->clk);

	hws[ENSEMBLE_PDM_CLK] = ensemble_clk_hw_gate("pdm_clk", "pdm_sclk", base + 0x0, 8);

	hws[ENSEMBLE_I2S0_SCLK] = ensemble_clk_hw_mux("i2s0_sclk",
						      base + 0x10, 16, 1, audio_clk_src_sels,
						      ARRAY_SIZE(audio_clk_src_sels));
	hws[ENSEMBLE_I2S1_SCLK] = ensemble_clk_hw_mux("i2s1_sclk",
						      base + 0x14, 16, 1, audio_clk_src_sels,
						      ARRAY_SIZE(audio_clk_src_sels));
	hws[ENSEMBLE_I2S2_SCLK] = ensemble_clk_hw_mux("i2s2_sclk",
						      base + 0x18, 16, 1, audio_clk_src_sels,
						      ARRAY_SIZE(audio_clk_src_sels));
	hws[ENSEMBLE_I2S3_SCLK] = ensemble_clk_hw_mux("i2s3_sclk",
						      base + 0x1c, 16, 1, audio_clk_src_sels,
						      ARRAY_SIZE(audio_clk_src_sels));

	clk_set_parent(hws[ENSEMBLE_I2S0_SCLK]->clk, hws[ENSEMBLE_CGU_76M8_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_I2S1_SCLK]->clk, hws[ENSEMBLE_CGU_76M8_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_I2S2_SCLK]->clk, hws[ENSEMBLE_CGU_76M8_CLK]->clk);
	clk_set_parent(hws[ENSEMBLE_I2S3_SCLK]->clk, hws[ENSEMBLE_CGU_76M8_CLK]->clk);

	hws[ENSEMBLE_I2S0_SCLK_AON] = ensemble_clk_hw_gate("i2s0_aon_clk",
							   "i2s0_sclk", base + 0x10, 20);
	hws[ENSEMBLE_I2S1_SCLK_AON] = ensemble_clk_hw_gate("i2s1_aon_clk",
							   "i2s1_sclk", base + 0x14, 20);
	hws[ENSEMBLE_I2S2_SCLK_AON] = ensemble_clk_hw_gate("i2s2_aon_clk",
							   "i2s2_sclk", base + 0x18, 20);
	hws[ENSEMBLE_I2S3_SCLK_AON] = ensemble_clk_hw_gate("i2s3_aon_clk",
							   "i2s3_sclk", base + 0x1c, 20);

	hws[ENSEMBLE_I2S0_CLK] = ensemble_clk_hw_gate("i2s0_clk", "i2s0_aon_clk", base + 0x10, 12);
	hws[ENSEMBLE_I2S1_CLK] = ensemble_clk_hw_gate("i2s1_clk", "i2s1_aon_clk", base + 0x14, 12);
	hws[ENSEMBLE_I2S2_CLK] = ensemble_clk_hw_gate("i2s2_clk", "i2s2_aon_clk", base + 0x18, 12);
	hws[ENSEMBLE_I2S3_CLK] = ensemble_clk_hw_gate("i2s3_clk", "i2s3_aon_clk", base + 0x1c, 12);

	hws[ENSEMBLE_I2S0_BIT_CLK] = ensemble_clk_hw_divider("i2s0_bit_clk",
							     "76m8_clk", base + 0x10, 0, 10);
	hws[ENSEMBLE_I2S1_BIT_CLK] = ensemble_clk_hw_divider("i2s1_bit_clk",
							     "76m8_clk", base + 0x14, 0, 10);
	hws[ENSEMBLE_I2S2_BIT_CLK] = ensemble_clk_hw_divider("i2s2_bit_clk",
							     "76m8_clk", base + 0x18, 0, 10);
	hws[ENSEMBLE_I2S3_BIT_CLK] = ensemble_clk_hw_divider("i2s3_bit_clk",
							     "76m8_clk", base + 0x1c, 0, 10);
	hws[ENSEMBLE_DMA_ENA_CLK] = ensemble_clk_hw_gate("dma_clk",
							 "syst_aclk", ccpmst_base + 0xC, 4);

	iter = find_child_clock_by_name(ccps_node, "extclk");
	if (iter) {
		of_property_read_u32(iter, "clock-div", &clk_div);
		of_node_put(iter);

		clk_div &= ((1UL << CC_DIV_WIDTH) - 1);
		val = readl(ccpmst_base);
		val &= ~CC_DIV_MASK;
		val |= clk_div << CC_DIV_SHIFT;
		writel(val, ccpmst_base);

		hws[ENSEMBLE_EXT_CLK_DIV] =
			ensemble_clk_hw_divider("extclk_rate", "syst_aclk", ccpmst_base, 16, 9);

		hws[ENSEMBLE_EXT_CLK] =
			ensemble_clk_hw_gate("extclk", "extclk_rate", ccpmst_base + 0x0, 0);
	}

	/**
	 * Register CPU clock with custom ops for dynamic rate reading.
	 * The CPU clock is derived from pll_clk1 (800 MHz) divided by
	 * HOSTCPUCLK_DIV1 register at CLKCTL_SYS_BASE + 0x808.
	 * Bits [4:0] = divider value (zero-based: 0=div1, 1=div2, ..., 31=div32)
	 *
	 * We use a custom clock type instead of clk_hw_register_divider so that
	 * clk_summary always shows the actual current frequency by reading
	 * the register in recalc_rate.
	 */
	clkctl_sys_base = ioremap(CLKCTL_SYS_BASE, SZ_4K);
	if (!clkctl_sys_base) {
		pr_err("ensemble-clk: failed to ioremap CLKCTL_SYS_BASE\n");
		goto err_check_hws;
	}

	static const char * const cpu_clk_parent[] = { "pll_clk1" };
	static struct clk_init_data cpu_clk_init = {
		.name         = "cpu_clk",
		.ops          = &ensemble_cpu_clk_ops,
		.parent_names = cpu_clk_parent,
		.num_parents  = 1,
		.flags        = 0,
	};

	cpu_clk_data = kzalloc(sizeof(*cpu_clk_data), GFP_KERNEL);
	if (!cpu_clk_data)
		goto err_unmap;

	cpu_clk_data->reg  = clkctl_sys_base + HOSTCPUCLK_DIV1;
	cpu_clk_data->lock = &ensemble_ccps_lock;
	cpu_clk_data->hw.init = &cpu_clk_init;

	ret = clk_hw_register(NULL, &cpu_clk_data->hw);
	if (ret) {
		pr_err("ensemble-clk: failed to register CPU clock: %d\n", ret);
		goto err_free;
	}

	hws[ENSEMBLE_CPU_CLK] = &cpu_clk_data->hw;
	pll_clk1_rate = clk_hw_get_rate(hws[ENSEMBLE_PLL_CLK1]);
	pr_info("ensemble-clk: CPU clock registered, PLL rate=%lu Hz\n", pll_clk1_rate);

err_check_hws:
	for (int i = 0; i < ENSEMBLE_CLK_END; i++) {
		if (IS_ERR(hws[i]))
			pr_err("ensemble clk %u: register failed with %ld\n",
			       i, PTR_ERR(hws[i]));
	}

	if (!clk_hw_data) {
		pr_err("ensemble-clk: clk_hw_data is NULL, cannot register provider\n");
		return;
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_hw_data);
	if (ret)
		pr_err("ensemble-clk: failed to add hw provider: %d\n", ret);

	return;

err_free:
	kfree(cpu_clk_data);
	cpu_clk_data = NULL;

err_unmap:
	iounmap(clkctl_sys_base);
	clkctl_sys_base = NULL;
	goto err_check_hws;
}

/**
 * ensemble_cpu_clk_set_rate - Set CPU clock frequency
 * @rate_hz: Target frequency in Hz
 *
 * This function is called by the cpufreq driver to change the CPU frequency.
 * It uses the clock framework's clk_set_rate() which calls our custom
 * set_rate ops to program the HOSTCPUCLK_DIV1 register.
 *
 * Returns 0 on success, negative error code on failure.
 */
int ensemble_cpu_clk_set_rate(unsigned long rate_hz)
{
	struct clk *cpu_clk;
	int ret;

	if (!cpu_clk_data || !hws[ENSEMBLE_CPU_CLK])
		return -ENODEV;

	cpu_clk = hws[ENSEMBLE_CPU_CLK]->clk;
	if (!cpu_clk)
		return -ENODEV;

	ret = clk_set_rate(cpu_clk, rate_hz);
	if (ret)
		pr_err("ensemble-clk: failed to set CPU clock to %lu Hz: %d\n",
		       rate_hz, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(ensemble_cpu_clk_set_rate);

/**
 * ensemble_cpu_clk_get_rate - Get current CPU clock frequency
 *
 * Uses the clock framework to get the current CPU clock rate.
 * Our custom recalc_rate callback reads the HOSTCPUCLK_DIV1 register
 * to calculate the actual frequency.
 *
 * Returns the current CPU clock frequency in Hz, or 0 on error.
 */
unsigned long ensemble_cpu_clk_get_rate(void)
{
	if (!cpu_clk_data || !hws[ENSEMBLE_CPU_CLK])
		return 0;

	/* clk_hw_get_rate calls our recalc_rate which reads the register */
	return clk_hw_get_rate(hws[ENSEMBLE_CPU_CLK]);
}
EXPORT_SYMBOL_GPL(ensemble_cpu_clk_get_rate);

CLK_OF_DECLARE(ensemble, "alif,ensemble-ccps", ensemble_clocks_init);
