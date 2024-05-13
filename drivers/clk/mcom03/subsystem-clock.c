// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023-2024 RnD Center "ELVEES", JSC
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>

#include <dt-bindings/soc/elvees,mcom03.h>
#include <dt-bindings/clock/mcom03-clock.h>

#include "mcom03-clk.h"

#define SDR_URB_PCI0_CTL		0x50
#define SDR_URB_PCI1_CTL		0x54
#define SDR_URB_PCIE_CTL_PAD_EN		BIT(4)

struct mcom03_clk_provider {
	struct clk_hw_onecell_data *clk_data;
	struct regmap *urb;
	struct mcom03_subsystem_clk *sclk;
	unsigned int subsystem;
};

static struct mcom03_pll mcom03_service_plls[] = {
	{CLK_SERVICE_PLL0, "service_pll", 0x1000, 1, BIT(0)},
};

static struct mcom03_ucg_chan mcom03_service_ucg_chans[] = {
	{CLK_SERVICE_UCG0_APB, 0, 0, "service_apb_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_CORE, 0, 1, "service_core_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_QSPI0, 0, 2, "qspi0_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_BPAM, 0, 3, "bpam_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_RISC0, 0, 4, "risc0_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_MFBSP0, 0, 5, "mfbsp0_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_MFBSP1, 0, 6, "mfbsp1_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_MAILBOX0, 0, 7, "mailbox0_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_PVTCTR, 0, 8, "pvtctr_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_I2C4, 0, 9, "i2c4_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_TRNG, 0, 10, "trng_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_SPIOTP, 0, 11, "spiotp_clk", "service_pll", true},
	{CLK_SERVICE_UCG0_I2C4_EXT, 0, 12, "i2c4_ext_clk", "service_pll"},
	{CLK_SERVICE_UCG0_QSPI0_EXT, 0, 13, "qspi0_ext_clk", "service_pll"},
	{CLK_SERVICE_UCG0_CLKOUT_EXT, 0, 14, "clkout_clk", "service_pll"},
	{CLK_SERVICE_UCG0_RISC0_TCK, 0, 15, "risc0_tck_clk", "service_pll"},
};

static struct mcom03_pll mcom03_cpu_plls[] = {
	{CLK_CPU_PLL0, "cpu_pll", 0x50, 1, BIT(0)},
};

static struct mcom03_ucg_chan mcom03_cpu_ucg_chans[] = {
	/* Set all channels as fixed because any attempt to change frequency
	 * will result in a violation of frequency ratio and this will
	 * cause a hang.
	 */
	{CLK_CPU_UCG0_SYS, 0, 0, "cpu_sys_clk", "cpu_pll", true},
	{CLK_CPU_UCG0_CORE, 0, 1, "cpu_core_clk", "cpu_pll", true},
	{CLK_CPU_UCG0_DBUS, 0, 2, "cpu_dbus_clk", "cpu_pll", true},
};

static struct mcom03_pll mcom03_sdr_plls[] = {
	{CLK_SDR_PLL0, "sdr_pll0", 0x0, 1, BIT(0)},
	{CLK_SDR_PLL1, "sdr_pll1", 0x8, 1, 0},
	{CLK_SDR_PLL2, "sdr_pll2", 0x10, 1, 0},
};

static struct mcom03_ucg_chan mcom03_sdr_ucg_chans[] = {
	{CLK_SDR_UCG0_CFG, 0, 0, "sdr_cfg_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_EXT, 0, 1, "sdr_ext_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_INT, 0, 2, "sdr_int_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_PCI, 0, 3, "pci_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_VCUA, 0, 4, "vcu_aclk", "sdr_pll0"},
	{CLK_SDR_UCG0_ACC0, 0, 5, "acc0_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_ACC1, 0, 6, "acc1_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_ACC2, 0, 7, "acc2_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_AUX_PCI, 0, 8, "aux_pci_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_GNSS, 0, 9, "gnss_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_DFE_ALT, 0, 10, "dfe_alt_clk", "sdr_pll0"},
	{CLK_SDR_UCG0_VCU, 0, 11, "vcu_tclk", "sdr_pll0"},
	{CLK_SDR_UCG0_LVDS, 0, 12, "lvds_clk", "sdr_pll0"},
};

static struct mcom03_clk_gate mcom03_sdr_gates[] = {
	{CLK_SDR_GATE_DSP0, "dsp0_clk", "sdr_pll2", 0x4c, 8},
	{CLK_SDR_GATE_DSP1, "dsp1_clk", "sdr_pll2", 0x4c, 9},
	{CLK_SDR_GATE_PCIE0, "pci0_clk", "pci_clk", 0x50, 0},
	{CLK_SDR_GATE_PCIE1, "pci1_clk", "pci_clk", 0x54, 0},
};

static struct mcom03_pll mcom03_media_plls[] = {
	{CLK_MEDIA_PLL0, "media_pll0", 0x0, 1, BIT(0)},
	{CLK_MEDIA_PLL1, "media_pll1", 0x10, 1, BIT(1)},
	{CLK_MEDIA_PLL2, "media_pll2", 0x20, 1, BIT(2)},
	{CLK_MEDIA_PLL3, "media_pll3", 0x30, 1, BIT(3)},
};

static struct mcom03_ucg_chan mcom03_media_ucg_chans[] = {
	{CLK_MEDIA_UCG0_SYSA, 0, 0, "media_sys_aclk", "media_pll0"},
	{CLK_MEDIA_UCG0_ISP_SYS, 0, 1, "isp_sys_clk", "media_pll0"},
	{CLK_MEDIA_UCG1_DISP_A, 1, 0, "disp_aclk", "media_pll1"},
	{CLK_MEDIA_UCG1_DISP_M, 1, 1, "disp_mclk", "media_pll1"},
	{CLK_MEDIA_UCG1_DISP_PXL, 1, 2, "disp_pxlclk", "media_pll1"},
	{CLK_MEDIA_UCG2_GPU_SYS, 2, 0, "gpu_sys_clk", "media_pll2"},
	{CLK_MEDIA_UCG2_GPU_MEM, 2, 1, "gpu_mem_clk", "media_pll2"},
	{CLK_MEDIA_UCG2_GPU_CORE, 2, 2, "gpu_core_clk", "media_pll2"},
	{CLK_MEDIA_UCG3_MIPI_RX_REF, 3, 0, "mipi_rx_ref_clk", "media_pll3"},
	{CLK_MEDIA_UCG3_MIPI_RX0_CFG, 3, 1, "mipi_rx0_cfg_clk", "media_pll3"},
	{CLK_MEDIA_UCG3_MIPI_RX1_CFG, 3, 2, "mipi_rx1_cfg_clk", "media_pll3"},
	{CLK_MEDIA_UCG3_MIPI_TX_REF, 3, 3, "mipi_tx_ref_clk", "media_pll3"},
	{CLK_MEDIA_UCG3_MIPI_TX_CFG, 3, 4, "mipi_tx_cfg_clk", "media_pll3"},
	{CLK_MEDIA_UCG3_CMOS0, 3, 5, "cmos0_clk", "media_pll3"},
	{CLK_MEDIA_UCG3_CMOS1, 3, 6, "cmos1_clk", "media_pll3"},
	{CLK_MEDIA_UCG3_MIPI_TXCLKESC, 3, 7, "mipi_txclkesc", "media_pll3"},
	{CLK_MEDIA_UCG3_VPU, 3, 8, "vpu_clk", "media_pll3"},
};

static struct mcom03_pll mcom03_ddr_plls[] = {
	{CLK_DDR_PLL0, "ddr_pll0", 0x0, 1},
	{CLK_DDR_PLL1, "ddr_pll1", 0x8, 1},
};

static struct mcom03_ucg_chan mcom03_ddr_ucg_chans[] = {
	{CLK_DDR_UCG0_DDR0, 0, 0, "ddr0_clk", "ddr_pll0", true, false, true},
	{CLK_DDR_UCG0_DDR0_X4, 0, 1, "ddr0_x4_clk", "ddr_pll0",
	 true, false, true},
	{CLK_DDR_UCG0_DDR1, 0, 2, "ddr1_clk", "ddr_pll0", true, false, true},
	{CLK_DDR_UCG0_DDR1_X4, 0, 3, "ddr1_x4_clk", "ddr_pll0",
	 true, false, true},
	{CLK_DDR_UCG1_SYS, 1, 0, "ddr_sys_clk", "ddr_pll1", true, false, true},
	{CLK_DDR_UCG1_SDR, 1, 1, "ddr_sdr_clk", "ddr_pll1", true, false, true},
	{CLK_DDR_UCG1_PCIE, 1, 2, "ddr_pcie_clk", "ddr_pll1",
	 true, false, true},
	{CLK_DDR_UCG1_ISP, 1, 3, "ddr_isp_clk", "ddr_pll1", true, false, true},
	{CLK_DDR_UCG1_GPU, 1, 4, "ddr_gpu_clk", "ddr_pll1", true, false, true},
	{CLK_DDR_UCG1_VPU, 1, 5, "ddr_vpu_clk", "ddr_pll1", true, false, true},
	{CLK_DDR_UCG1_DP, 1, 6, "ddr_dp_clk", "ddr_pll1", true, false, true},
	{CLK_DDR_UCG1_CPU, 1, 7, "ddr_cpu_clk", "ddr_pll1", true, false, true},
	{CLK_DDR_UCG1_SERVICE, 1, 8, "ddr_service_clk", "ddr_pll1",
	 true, false, true},
	{CLK_DDR_UCG1_HSPERIPH, 1, 9, "ddr_hsperiph_clk", "ddr_pll1",
	 true, false, true},
	{CLK_DDR_UCG1_LSPERIPH0, 1, 10, "ddr_lsperiph0_clk", "ddr_pll1",
	 true, false, true},
	{CLK_DDR_UCG1_LSPERIPH1, 1, 11, "ddr_lsperiph1_clk", "ddr_pll1",
	 true, false, true},
};

static struct mcom03_pll mcom03_hsperiph_plls[] = {
	{CLK_HSP_PLL0, "hsperiph_pll", 0x0, 3, 0xf},
};

static struct mcom03_clk_refmux mcom03_hsperiph_remuxes[] = {
	{CLK_HSP_REFMUX0, 0, 0xc, 0, 0x3, "hsperiph_refmux0"},
	{CLK_HSP_REFMUX1, 1, 0xc, 2, 0x3, "hsperiph_refmux1"},
	{CLK_HSP_REFMUX2, 2, 0xc, 4, 0x3, "hsperiph_refmux2"},
	{CLK_HSP_REFMUX3, 3, 0xc, 6, 0x3, "hsperiph_refmux3"},
};

static struct mcom03_ucg_chan mcom03_hsperiph_ucg_chans[] = {
	{CLK_HSP_UCG0_SYS, 0, 0, "hsp_sys_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_DMA, 0, 1, "hsp_dma_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_CTR, 0, 2, "hsp_ctr_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_SPRAM, 0, 3, "spram_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_EMAC0, 0, 4, "emac0_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_EMAC1, 0, 5, "emac1_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_USB0, 0, 6, "usb0_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_USB1, 0, 7, "usb1_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_NFC, 0, 8, "nfc_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_PDMA2, 0, 9, "pdma2_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_SDMMC0, 0, 10, "sdmmc0_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_SDMMC1, 0, 11, "sdmmc1_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG0_QSPI, 0, 12, "qspi_clk", "hsperiph_refmux0"},
	{CLK_HSP_UCG1_SDMMC0_XIN, 1, 0, "sdmmc0_xin_clk", "hsperiph_refmux1"},
	{CLK_HSP_UCG1_SDMMC1_XIN, 1, 1, "sdmmc1_xin_clk", "hsperiph_refmux1"},
	{CLK_HSP_UCG1_NFC_FLASH, 1, 2, "nfc_clk_flash", "hsperiph_refmux1"},
	{CLK_HSP_UCG1_QSPI_EXT, 1, 3, "qspi_ext_clk", "hsperiph_refmux1"},
	{CLK_HSP_UCG1_UST, 1, 4, "ust_clk", "hsperiph_refmux1"},
	{CLK_HSP_UCG2_EMAC0_1588, 2, 0, "emac0_clk_1588", "hsperiph_refmux2"},
	{CLK_HSP_UCG2_EMAC0_RGMII_TXC, 2, 1, "emac0_rgmii_txc", "hsperiph_refmux2"},
	{CLK_HSP_UCG2_EMAC1_1588, 2, 2, "emac1_clk_1588", "hsperiph_refmux2"},
	{CLK_HSP_UCG2_EMAC1_RGMII_TXC, 2, 3, "emac1_rgmii_txc", "hsperiph_refmux2"},
	{CLK_HSP_UCG3_USB0_REF_ALT, 3, 0, "usb0_ref_alt_clk", "hsperiph_refmux3"},
	{CLK_HSP_UCG3_USB0_SUSPEND, 3, 1, "usb0_suspend_clk", "hsperiph_refmux3"},
	{CLK_HSP_UCG3_USB1_REF_ALT, 3, 2, "usb1_ref_alt_clk", "hsperiph_refmux3"},
	{CLK_HSP_UCG3_USB1_SUSPEND, 3, 3, "usb1_suspend_clk", "hsperiph_refmux3"},
};

static struct mcom03_pll mcom03_lsperiph0_plls[] = {
	{CLK_LSP0_PLL0, "lsperiph0_pll", 0x0, 1, 0x1},
};

static struct mcom03_ucg_chan mcom03_lsperiph0_ucg_chans[] = {
	{CLK_LSP0_UCG0_SYS, 0, 0, "lsp0_sys_clk", "lsperiph0_pll"},

	/* When driver UART requests to change clock and clock driver sets it
	 * lower than the requested one, baud rate divisor for the UART becomes
	 * less than one. With integer divisor set to zero, the baud clock is
	 * disabled and no serial communications will occur. Round up requested
	 * frequency for UARTs.
	 */
	{CLK_LSP0_UCG0_UART3, 0, 1, "uart3_clk", "lsperiph0_pll", false, true},
	{CLK_LSP0_UCG0_UART1, 0, 2, "uart1_clk", "lsperiph0_pll", false, true},
	{CLK_LSP0_UCG0_UART2, 0, 3, "uart2_clk", "lsperiph0_pll", false, true},
	{CLK_LSP0_UCG0_SPI0, 0, 4, "spi0_clk", "lsperiph0_pll"},
	{CLK_LSP0_UCG0_I2C0, 0, 5, "i2c0_clk", "lsperiph0_pll"},
	{CLK_LSP0_UCG0_GPIO0_DB, 0, 6, "gpio0_dbclk", "lsperiph0_pll"},
};

static struct mcom03_pll mcom03_lsperiph1_plls[] = {
	{CLK_LSP1_PLL0, "lsperiph1_pll", 0x0, 1, 0x3},
};

static struct mcom03_clk_refmux mcom03_lsperiph1_remuxes[] = {
	{CLK_LSP1_REFMUX_I2S, 1, 0x10, 0, 0x1, "lsperiph1_refmux_i2s"},
};

static struct mcom03_ucg_chan mcom03_lsperiph1_ucg_chans[] = {
	{CLK_LSP1_UCG0_SYS, 0, 0, "lsp1_sys_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_I2C1, 0, 1, "i2c1_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_I2C2, 0, 2, "i2c2_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_I2C3, 0, 3, "i2c3_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_GPIO1_DB, 0, 4, "gpio1_dbclk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_SSI1, 0, 5, "ssi1_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_UART0, 0, 6, "uart0_clk", "lsperiph1_pll", false, true},
	{CLK_LSP1_UCG0_TIMERS0, 0, 7, "timers0_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_PWM0, 0, 8, "pwm0_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG0_WDT1, 0, 9, "wdt1_clk", "lsperiph1_pll"},
	{CLK_LSP1_UCG_I2S_I2S0, 1, 0, "lsp1_i2s_clk", "lsperiph1_refmux_i2s"},
};

void mcom03_sdr_clk_init(struct mcom03_clk_provider *prov)
{
	/* Enable PCIe external gates */
	regmap_update_bits(prov->urb, SDR_URB_PCI0_CTL, SDR_URB_PCIE_CTL_PAD_EN,
			   SDR_URB_PCIE_CTL_PAD_EN);

	regmap_update_bits(prov->urb, SDR_URB_PCI1_CTL, SDR_URB_PCIE_CTL_PAD_EN,
			   SDR_URB_PCIE_CTL_PAD_EN);
}

struct mcom03_subsystem_clk mcom03_subsystems[] = {
	[MCOM03_SUBSYSTEM_SERVICE] = {
		.plls = mcom03_service_plls,
		.nr_plls = ARRAY_SIZE(mcom03_service_plls),

		.ucg_chans = mcom03_service_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_service_ucg_chans),
		.max_ucg_id = 0,

		.nr_clocks = CLK_SERVICE_NR_CLOCKS,
	},
	[MCOM03_SUBSYSTEM_CPU] = {
		.plls = mcom03_cpu_plls,
		.nr_plls = ARRAY_SIZE(mcom03_cpu_plls),

		.ucg_chans = mcom03_cpu_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_cpu_ucg_chans),
		.max_ucg_id = 0,

		.nr_clocks = CLK_CPU_NR_CLOCKS,
	},
	[MCOM03_SUBSYSTEM_SDR] = {
		.plls = mcom03_sdr_plls,
		.nr_plls = ARRAY_SIZE(mcom03_sdr_plls),

		.ucg_chans = mcom03_sdr_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_sdr_ucg_chans),
		.max_ucg_id = 0,  /* Other UCGs are not specified in ucgs yet */

		.gates = mcom03_sdr_gates,
		.nr_gates = ARRAY_SIZE(mcom03_sdr_gates),

		.nr_clocks = CLK_SDR_NR_CLOCKS,

		.init = mcom03_sdr_clk_init,
	},
	[MCOM03_SUBSYSTEM_MEDIA] = {
		.plls = mcom03_media_plls,
		.nr_plls = ARRAY_SIZE(mcom03_media_plls),

		.ucg_chans = mcom03_media_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_media_ucg_chans),
		.max_ucg_id = 3,

		.nr_clocks = CLK_MEDIA_NR_CLOCKS,
	},
	[MCOM03_SUBSYSTEM_DDR] = {
		.plls = mcom03_ddr_plls,
		.nr_plls = ARRAY_SIZE(mcom03_ddr_plls),

		.ucg_chans = mcom03_ddr_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_ddr_ucg_chans),
		.max_ucg_id = 1,

		.nr_clocks = CLK_DDR_NR_CLOCKS,
	},
	[MCOM03_SUBSYSTEM_HSPERIPH] = {
		.plls = mcom03_hsperiph_plls,
		.nr_plls = ARRAY_SIZE(mcom03_hsperiph_plls),

		.refmuxes = mcom03_hsperiph_remuxes,
		.nr_refmuxes = ARRAY_SIZE(mcom03_hsperiph_remuxes),

		.ucg_chans = mcom03_hsperiph_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_hsperiph_ucg_chans),
		.max_ucg_id = 3,

		.nr_clocks = CLK_HSP_NR_CLOCKS,
	},
	[MCOM03_SUBSYSTEM_LSPERIPH0] = {
		.plls = mcom03_lsperiph0_plls,
		.nr_plls = ARRAY_SIZE(mcom03_lsperiph0_plls),

		.ucg_chans = mcom03_lsperiph0_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_lsperiph0_ucg_chans),
		.max_ucg_id = 0,

		.nr_clocks = CLK_LSP0_NR_CLOCKS,
	},
	[MCOM03_SUBSYSTEM_LSPERIPH1] = {
		.plls = mcom03_lsperiph1_plls,
		.nr_plls = ARRAY_SIZE(mcom03_lsperiph1_plls),

		.refmuxes = mcom03_lsperiph1_remuxes,
		.nr_refmuxes = ARRAY_SIZE(mcom03_lsperiph1_remuxes),

		.ucg_chans = mcom03_lsperiph1_ucg_chans,
		.nr_ucg_chans = ARRAY_SIZE(mcom03_lsperiph1_ucg_chans),
		.max_ucg_id = 1,

		.nr_clocks = CLK_LSP1_NR_CLOCKS,
	},
};

static void __init mcom03_clk_provider_init(struct device_node *np,
					    struct mcom03_clk_provider *prov)
{
	u32 nr_clks = prov->sclk->nr_clocks;

	prov->clk_data = kzalloc(struct_size(prov->clk_data, hws, nr_clks),
				 GFP_KERNEL);

	prov->clk_data->num = nr_clks;
}

static void __init mcom03_init_plls(struct device_node *np,
				    struct mcom03_clk_provider *prov)
{
	struct mcom03_subsystem_clk *sclk = prov->sclk;
	const char *parent_name = of_clk_get_parent_name(np, 0);
	int ret;
	int i;

	for (i = 0; i < sclk->nr_plls; i++) {
		struct mcom03_pll *p = &sclk->plls[i];

		p->regmap = prov->urb;
		ret = mcom03_clk_pll_register(parent_name, p);
		if (ret) {
			pr_err("%pOFf: Failed to register clock %s (%d)\n",
			       np, p->name, ret);
			continue;
		}

		prov->clk_data->hws[p->clk_id] = &p->hw;
	}
}

static int __init mcom03_get_refmuxes_parents(struct device_node *np,
					      struct mcom03_clk_provider *prov,
					      int ucg_id,
					      const char **parent_names,
					      unsigned int *parent_count)
{
	unsigned int parent_clocks = of_clk_get_parent_count(np);

	switch (prov->subsystem) {
	case MCOM03_SUBSYSTEM_HSPERIPH:
		if (parent_clocks < 2) {
			pr_err("%pOFf: Expected 2 elements in clocks property\n",
			       np);
			return -EINVAL;
		}
		parent_names[0] = mcom03_hsperiph_plls[0].name;
		parent_names[1] = of_clk_get_parent_name(np, 1);
		*parent_count = 2;
		break;
	case MCOM03_SUBSYSTEM_LSPERIPH1:
		if (parent_clocks < 2) {
			pr_err("%pOFf: Expected 2 elements in clocks property\n",
			       np);
			return -EINVAL;
		}
		parent_names[0] = mcom03_lsperiph1_plls[0].name;
		parent_names[1] = of_clk_get_parent_name(np, 1);
		*parent_count = 2;
		break;
	default:
		pr_err("%pOFf: refmux for subsystem is not supported\n", np);
		return -EINVAL;
	}

	return 0;
}

static void __init mcom03_init_refmuxes(struct device_node *np,
					struct mcom03_clk_provider *prov)
{
	struct mcom03_subsystem_clk *sclk = prov->sclk;
	unsigned int parent_count;
	const char *parent_names[4];
	int ret;
	int i;

	for (i = 0; i < sclk->nr_refmuxes; i++) {
		struct mcom03_clk_refmux *p = &sclk->refmuxes[i];

		ret = mcom03_get_refmuxes_parents(np, prov, p->ucg_id,
						  parent_names, &parent_count);
		if (ret)
			return;

		p->regmap = prov->urb;
		ret = mcom03_clk_refmux_register(p, parent_names, parent_count);
		if (ret) {
			pr_err("%pOFf: Failed to register clock %s (%d)\n",
			       np, p->name, ret);
			continue;
		}

		prov->clk_data->hws[p->clk_id] = &p->hw;
	}
}

static void __init mcom03_init_ucgs(struct device_node *np,
				    struct mcom03_clk_provider *prov)
{
	int ret;
	int i;

	for (i = 0; i < prov->sclk->nr_ucg_chans; i++) {
		struct mcom03_ucg_chan *ucg_chan = &prov->sclk->ucg_chans[i];

		ret = mcom03_ucg_chan_register(ucg_chan);
		if (ret) {
			pr_err("%pOFf: Failed to register clock %s (%d)\n",
			       np, ucg_chan->name, ret);
			continue;
		}

		prov->clk_data->hws[ucg_chan->clk_id] = &ucg_chan->hw;
	}
}

static void __init mcom03_init_gates(struct device_node *np,
				     struct mcom03_clk_provider *prov)
{
	int ret;
	int i;

	for (i = 0; i < prov->sclk->nr_gates; i++) {
		struct mcom03_clk_gate *p = &prov->sclk->gates[i];

		p->sdr_urb = prov->urb;
		ret = mcom03_clk_gate_register(p);
		if (ret) {
			pr_err("%pOFf: Failed to register clock %s (%d)\n",
			       np, p->name, ret);
			continue;
		}

		prov->clk_data->hws[p->clk_id] = &p->hw;
	}
}

static bool __init is_valid_subsystem(int sub)
{
	return (sub >= 0 && sub < MCOM03_SUBSYSTEM_MAX);
}

static int mcom03_of_parse(struct device_node *np,
			   struct mcom03_clk_provider *prov)
{
	void __iomem *ucg_base[8];
	u32 expected_max_id;
	u32 ucg_id;
	u32 ucg_ids_mask;
	int subsystem;
	int ret;
	int nr_ucg_ids = 0;
	int i = 0;

	ret = of_property_read_u32(np, "elvees,subsystem", &subsystem);
	if (ret || !is_valid_subsystem(subsystem)) {
		pr_err("%pOFf: Failed to get subsystem id\n", np);
		if (!ret)
			ret = -EINVAL;

		return ret;
	}

	prov->subsystem = subsystem;
	prov->sclk = &mcom03_subsystems[subsystem];

	prov->urb = syscon_regmap_lookup_by_phandle(np, "elvees,urb");
	if (IS_ERR(prov->urb)) {
		pr_err("%pOFf: Failed to regmap URB region: %ld\n", np,
		       PTR_ERR(prov->urb));

		return PTR_ERR(prov->urb);
	}

	while (of_get_address(np, i++, NULL, NULL))
		nr_ucg_ids++;

	if (nr_ucg_ids == 0) {
		pr_err("%pOFf: Failed to get UCG address regions\n", np);
		ret = -ENOENT;
		goto free_urb;
	}

	expected_max_id = prov->sclk->max_ucg_id + 1;
	if (nr_ucg_ids != expected_max_id) {
		pr_err("%pOFf: Unexpected number of UCG regions, must be %d\n",
		       np, expected_max_id);
		ret = -EINVAL;
		goto free_urb;
	}

	if (unlikely(nr_ucg_ids > ARRAY_SIZE(ucg_base))) {
		WARN_ON(1);
		ret = -EINVAL;
		goto free_urb;
	}

	/* Prepare mapped addresses of UCGs */
	for (i = 0; i < nr_ucg_ids; i++)
		ucg_base[i] = of_iomap(np, i);

	/* Fill UCG base addresses for UCGs */
	for (i = 0; i < prov->sclk->nr_ucg_chans; i++)
		prov->sclk->ucg_chans[i].base = ucg_base[prov->sclk->ucg_chans[i].ucg_id];

	/* Fill UCG base addresses for refmuxes */
	if (prov->sclk->nr_refmuxes) {
		for (i = 0; i < prov->sclk->nr_refmuxes; i++) {
			ucg_id = prov->sclk->refmuxes[i].ucg_id;
			if (unlikely(ucg_id >= ARRAY_SIZE(ucg_base))) {
				WARN_ON(1);
				ret = -EINVAL;
				goto free_urb;
			}

			prov->sclk->refmuxes[i].base_ucg = ucg_base[ucg_id];
		}
	}

	/* Fill children UCGs for PLLs */
	for (i = 0; i < prov->sclk->nr_plls; i++) {
		struct mcom03_pll *pll = &prov->sclk->plls[i];
		u32 pos = 0;

		pll->sclk = prov->sclk;
		ucg_id = 0;
		ucg_ids_mask = prov->sclk->plls[i].ucg_ids_mask;
		pll->ucg_count = hweight32(ucg_ids_mask);
		if (!pll->ucg_count)
			continue;

		pll->ucg_ids = kcalloc(pll->ucg_count,
				       sizeof(*pll->ucg_ids),
				       GFP_KERNEL);
		pll->ucg_bypass = kcalloc(pll->ucg_count,
					  sizeof(*pll->ucg_bypass),
					  GFP_KERNEL);
		if (!pll->ucg_ids || !pll->ucg_bypass) {
			kfree(pll->ucg_ids);
			kfree(pll->ucg_bypass);
			pr_err("%pOFf: No memory\n", np);
			ret = -ENOMEM;
			goto free_urb;
		}
		while (ucg_ids_mask) {
			if (ucg_ids_mask & 0x1) {
				pll->ucg_ids[pos] = ucg_id;
				pos++;
			}
			ucg_id++;
			ucg_ids_mask >>= 1;
		}
	}

	return 0;

free_urb:
	regmap_exit(prov->urb);
	return ret;
}

static void __init mcom03_clk_init(struct device_node *np)
{
	struct mcom03_clk_provider *prov;
	int ret;

	prov = kzalloc(sizeof(*prov), GFP_KERNEL);
	if (!prov)
		return;

	ret = mcom03_of_parse(np, prov);
	if (ret)
		goto free_memory;

	if (prov->sclk->init)
		prov->sclk->init(prov);

	mcom03_clk_provider_init(np, prov);

	mcom03_init_plls(np, prov);
	mcom03_init_refmuxes(np, prov);
	mcom03_init_ucgs(np, prov);
	mcom03_init_gates(np, prov);

	of_clk_add_hw_provider(np, of_clk_hw_onecell_get, prov->clk_data);

	mcom03_of_clks_enable(np, prov->clk_data);

	return;

free_memory:
	kfree(prov);
}

CLK_OF_DECLARE(mcom03_clk, "elvees,mcom03-clk-pm", mcom03_clk_init);
