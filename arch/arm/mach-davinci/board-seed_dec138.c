/*
 * SEED_DEC138 board
 *
 * Copyright (C)
 *
 * Derived from: arch/arm/mach-davinci/board-da830-evm.c
 * Original Copyrights follow:
 *
 * 2007, 2009 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/platform_data/gpio-davinci.h>
#include <linux/platform_data/mtd-davinci.h>
#include <linux/platform_data/mtd-davinci-aemif.h>
#include <linux/platform_data/spi-davinci.h>
#include <linux/platform_data/uio_pruss.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/wl12xx.h>

#include <mach/common.h>
#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/mux.h>
#include <mach/sram.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/system_info.h>

#include <media/tvp514x.h>
#include <media/adv7343.h>

#define DA850_EVM_PHY_ID		"davinci_mdio-0:00"
#define DA850_LCD_PWR_PIN		GPIO_TO_PIN(2, 8)
#define DA850_LCD_BL_PIN		GPIO_TO_PIN(2, 15)

#define DA850_MMCSD_CD_PIN		GPIO_TO_PIN(4, 0)
#define DA850_MMCSD_WP_PIN		GPIO_TO_PIN(4, 1)

#define DA850_WLAN_EN			GPIO_TO_PIN(6, 9)
#define DA850_WLAN_IRQ			GPIO_TO_PIN(6, 10)

#define DA850_MII_MDIO_CLKEN_PIN	GPIO_TO_PIN(2, 6)

static struct spi_board_info da850evm_spi_info[] = {

};

static struct davinci_pm_config da850_pm_pdata = {
	.sleepcount = 128,
};

static struct platform_device da850_pm_device = {
	.name           = "pm-davinci",
	.dev = {
		.platform_data	= &da850_pm_pdata,
	},
	.id             = -1,
};

/* DA850/OMAP-L138 EVM includes a 512 MByte large-page NAND flash
 * (128K blocks). It may be used instead of the (default) SPI flash
 * to boot, using TI's tools to install the secondary boot loader
 * (UBL) and U-Boot.
 */
static struct mtd_partition da850_evm_nandflash_partition[] = {
	{
		.name		= "u-boot env",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	 },
	{
		.name		= "UBL",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "u-boot",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 4 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "kernel",
		.offset		= 0x300000,
		.size		= SZ_2M,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem",
		.offset		= 0x500000,
		.size		= SZ_32M,
		.mask_flags	= 0,
	},
	{
		.name		= "user",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	},
};

static struct davinci_aemif_timing da850_evm_nandflash_timing = {
	.wsetup		= 24,
	.wstrobe	= 21,
	.whold		= 14,
	.rsetup		= 19,
	.rstrobe	= 50,
	.rhold		= 0,
	.ta		= 20,
};

static struct davinci_nand_pdata da850_evm_nandflash_data = {
	.parts		= da850_evm_nandflash_partition,
	.nr_parts	= ARRAY_SIZE(da850_evm_nandflash_partition),
	.ecc_mode	= NAND_ECC_NONE,
	.ecc_bits	= 4,
	.bbt_options	= NAND_BBT_USE_FLASH,
	.timing		= &da850_evm_nandflash_timing,
};

static struct resource da850_evm_nandflash_resource[] = {
	{
		.start	= DA8XX_AEMIF_CS3_BASE,
		.end	= DA8XX_AEMIF_CS3_BASE + SZ_512K + 2 * SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= DA8XX_AEMIF_CTL_BASE,
		.end	= DA8XX_AEMIF_CTL_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device da850_evm_nandflash_device = {
	.name		= "davinci_nand",
	.id		= 1,
	.dev		= {
		.platform_data	= &da850_evm_nandflash_data,
	},
	.num_resources	= ARRAY_SIZE(da850_evm_nandflash_resource),
	.resource	= da850_evm_nandflash_resource,
};

static struct platform_device *da850_evm_devices[] = {
	&da850_evm_nandflash_device,
};

static const short da850_evm_nand_pins[] = {
	DA850_EMA_D_0, DA850_EMA_D_1, DA850_EMA_D_2, DA850_EMA_D_3,
	DA850_EMA_D_4, DA850_EMA_D_5, DA850_EMA_D_6, DA850_EMA_D_7,
	DA850_EMA_A_1, DA850_EMA_A_2, DA850_NEMA_CS_3, DA850_NEMA_CS_4,
	DA850_NEMA_WE, DA850_NEMA_OE,
	-1
};

#define HAS_MMC		IS_ENABLED(CONFIG_MMC_DAVINCI)

static inline void da850_evm_setup_nor_nand(void)
{
	int ret = 0;

	ret = davinci_cfg_reg_list(da850_evm_nand_pins);
	if (ret)
		pr_warn("%s: NAND mux setup failed: %d\n",
			__func__, ret);

	platform_add_devices(da850_evm_devices,
				ARRAY_SIZE(da850_evm_devices));

	if (davinci_aemif_setup(&da850_evm_nandflash_device))
		pr_warn("%s: Cannot configure AEMIF.\n", __func__);
}

#ifdef CONFIG_DA850_UI_RMII
static inline void da850_evm_setup_emac_rmii(int rmii_sel)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->rmii_en = 1;
	gpio_set_value_cansleep(rmii_sel, 0);
}
#else
static inline void da850_evm_setup_emac_rmii(int rmii_sel) { }
#endif

static struct i2c_board_info __initdata da850_evm_i2c_devices[] = {
};

static struct davinci_i2c_platform_data da850_evm_i2c_0_pdata = {
	.bus_freq	= 100,	/* kHz */
	.bus_delay	= 0,	/* usec */
};

static int da850_evm_mmc_get_ro(int index)
{
	return gpio_get_value(DA850_MMCSD_WP_PIN);
}

static int da850_evm_mmc_get_cd(int index)
{
	return !gpio_get_value(DA850_MMCSD_CD_PIN);
}

static struct davinci_mmc_config da850_mmc_config = {
	.get_ro		= da850_evm_mmc_get_ro,
	.get_cd		= da850_evm_mmc_get_cd,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
};

static const short da850_evm_mmcsd0_pins[] __initconst = {
	DA850_MMCSD0_DAT_0, DA850_MMCSD0_DAT_1, DA850_MMCSD0_DAT_2,
	DA850_MMCSD0_DAT_3, DA850_MMCSD0_CLK, DA850_MMCSD0_CMD,
	DA850_GPIO4_0, DA850_GPIO4_1,
	-1
};

static void da850_panel_power_ctrl(int val)
{
	/* lcd backlight */
	gpio_set_value(DA850_LCD_BL_PIN, val);

	/* lcd power */
	gpio_set_value(DA850_LCD_PWR_PIN, val);
}

static int da850_lcd_hw_init(void)
{
	int status;

	status = gpio_request(DA850_LCD_BL_PIN, "lcd bl\n");
	if (status < 0)
		return status;

	status = gpio_request(DA850_LCD_PWR_PIN, "lcd pwr\n");
	if (status < 0) {
		gpio_free(DA850_LCD_BL_PIN);
		return status;
	}

	gpio_direction_output(DA850_LCD_BL_PIN, 0);
	gpio_direction_output(DA850_LCD_PWR_PIN, 0);

	/* Switch off panel power and backlight */
	da850_panel_power_ctrl(0);

	/* Switch on panel power and backlight */
	da850_panel_power_ctrl(1);

	return 0;
}

/* Fixed regulator support */
static struct regulator_consumer_supply fixed_supplies[] = {
	/* Baseboard 3.3V: 5V -> TPS73701DCQ -> 3.3V */
	REGULATOR_SUPPLY("AVDD", "1-0018"),
	REGULATOR_SUPPLY("DRVDD", "1-0018"),

	/* Baseboard 1.8V: 5V -> TPS73701DCQ -> 1.8V */
	REGULATOR_SUPPLY("DVDD", "1-0018"),
};

static const short da850_evm_lcdc_pins[] = {
	DA850_GPIO2_8, DA850_GPIO2_15,
	-1
};

static const short da850_evm_mii_pins[] = {
	DA850_MII_TXEN, DA850_MII_TXCLK, DA850_MII_COL, DA850_MII_TXD_3,
	DA850_MII_TXD_2, DA850_MII_TXD_1, DA850_MII_TXD_0, DA850_MII_RXER,
	DA850_MII_CRS, DA850_MII_RXCLK, DA850_MII_RXDV, DA850_MII_RXD_3,
	DA850_MII_RXD_2, DA850_MII_RXD_1, DA850_MII_RXD_0, DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static const short da850_evm_rmii_pins[] = {
	DA850_RMII_TXD_0, DA850_RMII_TXD_1, DA850_RMII_TXEN,
	DA850_RMII_CRS_DV, DA850_RMII_RXD_0, DA850_RMII_RXD_1,
	DA850_RMII_RXER, DA850_RMII_MHZ_50_CLK, DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static int __init da850_evm_config_emac(void)
{
	void __iomem *cfg_chip3_base;
	int ret;
	u32 val;
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	u8 rmii_en = soc_info->emac_pdata->rmii_en;

	soc_info->emac_pdata->rmii_en = 1;
	rmii_en = soc_info->emac_pdata->rmii_en;

	if (!machine_is_davinci_da850_evm())
		return 0;

	cfg_chip3_base = DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG);

	val = __raw_readl(cfg_chip3_base);

	if (rmii_en) {
		val |= BIT(8);
		ret = davinci_cfg_reg_list(da850_evm_rmii_pins);
		pr_info("EMAC: RMII PHY configured, MII PHY will not be"
							" functional\n");
	} else {
		val &= ~BIT(8);
		ret = davinci_cfg_reg_list(da850_evm_mii_pins);
		pr_info("EMAC: MII PHY configured, RMII PHY will not be"
							" functional\n");
	}

	if (ret)
		pr_warn("%s: CPGMAC/RMII mux setup failed: %d\n",
			__func__, ret);

	/* configure the CFGCHIP3 register for RMII or MII */
	__raw_writel(val, cfg_chip3_base);

	ret = davinci_cfg_reg(DA850_GPIO2_6);
	if (ret)
		pr_warn("%s:GPIO(2,6) mux setup failed\n", __func__);

	ret = gpio_request(DA850_MII_MDIO_CLKEN_PIN, "mdio_clk_en");
	if (ret) {
		pr_warn("Cannot open GPIO %d\n", DA850_MII_MDIO_CLKEN_PIN);
		return ret;
	}

	/* Enable/Disable MII MDIO clock */
	gpio_direction_output(DA850_MII_MDIO_CLKEN_PIN, rmii_en);

	soc_info->emac_pdata->phy_id = DA850_EVM_PHY_ID;

	ret = da8xx_register_emac();
	if (ret)
		pr_warn("%s: EMAC registration failed: %d\n", __func__, ret);

	return 0;
}
device_initcall(da850_evm_config_emac);

/*
 * The following EDMA channels/slots are not being used by drivers (for
 * example: Timer, GPIO, UART events etc) on da850/omap-l138 EVM, hence
 * they are being reserved for codecs on the DSP side.
 */
static const s16 da850_dma0_rsv_chans[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma0_rsv_slots[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30, 50},
	{-1, -1}
};

static const s16 da850_dma1_rsv_chans[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma1_rsv_slots[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30, 90},
	{-1, -1}
};

static struct edma_rsv_info da850_edma_cc0_rsv = {
	.rsv_chans	= da850_dma0_rsv_chans,
	.rsv_slots	= da850_dma0_rsv_slots,
};

static struct edma_rsv_info da850_edma_cc1_rsv = {
	.rsv_chans	= da850_dma1_rsv_chans,
	.rsv_slots	= da850_dma1_rsv_slots,
};

static struct edma_rsv_info *da850_edma_rsv[2] = {
	&da850_edma_cc0_rsv,
	&da850_edma_cc1_rsv,
};

#ifdef CONFIG_CPU_FREQ
static __init int da850_evm_init_cpufreq(void)
{
	switch (system_rev & 0xF) {
	case 3:
		da850_max_speed = 456000;
		break;
	case 2:
		da850_max_speed = 408000;
		break;
	case 1:
		da850_max_speed = 372000;
		break;
	}

	return da850_register_cpufreq("pll0_sysclk3");
}
#else
static __init int da850_evm_init_cpufreq(void) { return 0; }
#endif

#if defined(CONFIG_DA850_UI_SD_VIDEO_PORT)

#define TVP5147_CH0		"tvp514x-0"
#define TVP5147_CH1		"tvp514x-1"

/* VPIF capture configuration */
static struct tvp514x_platform_data tvp5146_pdata = {
		.clk_polarity = 0,
		.hs_polarity  = 1,
		.vs_polarity  = 1,
};

#define TVP514X_STD_ALL (V4L2_STD_NTSC | V4L2_STD_PAL)

static const struct vpif_input da850_ch0_inputs[] = {
	{
		.input = {
			.index = 0,
			.name  = "Composite",
			.type  = V4L2_INPUT_TYPE_CAMERA,
			.capabilities = V4L2_IN_CAP_STD,
			.std   = TVP514X_STD_ALL,
		},
		.input_route = INPUT_CVBS_VI2B,
		.output_route = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.subdev_name = TVP5147_CH0,
	},
};

static const struct vpif_input da850_ch1_inputs[] = {
	{
		.input = {
			.index = 0,
			.name  = "S-Video",
			.type  = V4L2_INPUT_TYPE_CAMERA,
			.capabilities = V4L2_IN_CAP_STD,
			.std   = TVP514X_STD_ALL,
		},
		.input_route = INPUT_SVIDEO_VI2C_VI1C,
		.output_route = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.subdev_name = TVP5147_CH1,
	},
};

static struct vpif_subdev_info da850_vpif_capture_sdev_info[] = {
	{
		.name = TVP5147_CH0,
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5d),
			.platform_data = &tvp5146_pdata,
		},
	},
	{
		.name = TVP5147_CH1,
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5c),
			.platform_data = &tvp5146_pdata,
		},
	},
};

static struct vpif_capture_config da850_vpif_capture_config = {
	.subdev_info = da850_vpif_capture_sdev_info,
	.subdev_count = ARRAY_SIZE(da850_vpif_capture_sdev_info),
	.chan_config[0] = {
		.inputs = da850_ch0_inputs,
		.input_count = ARRAY_SIZE(da850_ch0_inputs),
		.vpif_if = {
			.if_type = VPIF_IF_BT656,
			.hd_pol  = 1,
			.vd_pol  = 1,
			.fid_pol = 0,
		},
	},
	.chan_config[1] = {
		.inputs = da850_ch1_inputs,
		.input_count = ARRAY_SIZE(da850_ch1_inputs),
		.vpif_if = {
			.if_type = VPIF_IF_BT656,
			.hd_pol  = 1,
			.vd_pol  = 1,
			.fid_pol = 0,
		},
	},
	.card_name = "DA850/OMAP-L138 Video Capture",
};

/* VPIF display configuration */

static struct adv7343_platform_data adv7343_pdata = {
	.mode_config = {
		.dac = { 1, 1, 1 },
	},
	.sd_config = {
		.sd_dac_out = { 1 },
	},
};

static struct vpif_subdev_info da850_vpif_subdev[] = {
	{
		.name = "adv7343",
		.board_info = {
			I2C_BOARD_INFO("adv7343", 0x2a),
			.platform_data = &adv7343_pdata,
		},
	},
};

static const struct vpif_output da850_ch0_outputs[] = {
	{
		.output = {
			.index = 0,
			.name = "Composite",
			.type = V4L2_OUTPUT_TYPE_ANALOG,
			.capabilities = V4L2_OUT_CAP_STD,
			.std = V4L2_STD_ALL,
		},
		.subdev_name = "adv7343",
		.output_route = ADV7343_COMPOSITE_ID,
	},
	{
		.output = {
			.index = 1,
			.name = "S-Video",
			.type = V4L2_OUTPUT_TYPE_ANALOG,
			.capabilities = V4L2_OUT_CAP_STD,
			.std = V4L2_STD_ALL,
		},
		.subdev_name = "adv7343",
		.output_route = ADV7343_SVIDEO_ID,
	},
};

static struct vpif_display_config da850_vpif_display_config = {
	.subdevinfo   = da850_vpif_subdev,
	.subdev_count = ARRAY_SIZE(da850_vpif_subdev),
	.chan_config[0] = {
		.outputs = da850_ch0_outputs,
		.output_count = ARRAY_SIZE(da850_ch0_outputs),
	},
	.card_name    = "DA850/OMAP-L138 Video Display",
};

static __init void da850_vpif_init(void)
{
	int ret;

	ret = da850_register_vpif();
	if (ret)
		pr_warn("da850_evm_init: VPIF setup failed: %d\n", ret);

	ret = davinci_cfg_reg_list(da850_vpif_capture_pins);
	if (ret)
		pr_warn("da850_evm_init: VPIF capture mux setup failed: %d\n",
			ret);

	ret = da850_register_vpif_capture(&da850_vpif_capture_config);
	if (ret)
		pr_warn("da850_evm_init: VPIF capture setup failed: %d\n", ret);

	ret = davinci_cfg_reg_list(da850_vpif_display_pins);
	if (ret)
		pr_warn("da850_evm_init: VPIF display mux setup failed: %d\n",
			ret);

	ret = da850_register_vpif_display(&da850_vpif_display_config);
	if (ret)
		pr_warn("da850_evm_init: VPIF display setup failed: %d\n", ret);
}

#else
static __init void da850_vpif_init(void) {}
#endif

#ifdef CONFIG_DA850_WL12XX

static void wl12xx_set_power(int index, bool power_on)
{
	static bool power_state;

	pr_debug("Powering %s wl12xx", power_on ? "on" : "off");

	if (power_on == power_state)
		return;
	power_state = power_on;

	if (power_on) {
		/* Power up sequence required for wl127x devices */
		gpio_set_value(DA850_WLAN_EN, 1);
		usleep_range(15000, 15000);
		gpio_set_value(DA850_WLAN_EN, 0);
		usleep_range(1000, 1000);
		gpio_set_value(DA850_WLAN_EN, 1);
		msleep(70);
	} else {
		gpio_set_value(DA850_WLAN_EN, 0);
	}
}

static struct davinci_mmc_config da850_wl12xx_mmc_config = {
	.set_power	= wl12xx_set_power,
	.wires		= 4,
	.max_freq	= 25000000,
	.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_NONREMOVABLE |
			  MMC_CAP_POWER_OFF_CARD,
};

static const short da850_wl12xx_pins[] __initconst = {
	DA850_MMCSD1_DAT_0, DA850_MMCSD1_DAT_1, DA850_MMCSD1_DAT_2,
	DA850_MMCSD1_DAT_3, DA850_MMCSD1_CLK, DA850_MMCSD1_CMD,
	DA850_GPIO6_9, DA850_GPIO6_10,
	-1
};

static struct wl12xx_platform_data da850_wl12xx_wlan_data __initdata = {
	.irq			= -1,
	.board_ref_clock	= WL12XX_REFCLOCK_38,
	.platform_quirks	= WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

static __init int da850_wl12xx_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850_wl12xx_pins);
	if (ret) {
		pr_err("wl12xx/mmc mux setup failed: %d\n", ret);
		goto exit;
	}

	ret = da850_register_mmcsd1(&da850_wl12xx_mmc_config);
	if (ret) {
		pr_err("wl12xx/mmc registration failed: %d\n", ret);
		goto exit;
	}

	ret = gpio_request_one(DA850_WLAN_EN, GPIOF_OUT_INIT_LOW, "wl12xx_en");
	if (ret) {
		pr_err("Could not request wl12xx enable gpio: %d\n", ret);
		goto exit;
	}

	ret = gpio_request_one(DA850_WLAN_IRQ, GPIOF_IN, "wl12xx_irq");
	if (ret) {
		pr_err("Could not request wl12xx irq gpio: %d\n", ret);
		goto free_wlan_en;
	}

	da850_wl12xx_wlan_data.irq = gpio_to_irq(DA850_WLAN_IRQ);

	ret = wl12xx_set_platform_data(&da850_wl12xx_wlan_data);
	if (ret) {
		pr_err("Could not set wl12xx data: %d\n", ret);
		goto free_wlan_irq;
	}

	return 0;

free_wlan_irq:
	gpio_free(DA850_WLAN_IRQ);

free_wlan_en:
	gpio_free(DA850_WLAN_EN);

exit:
	return ret;
}

#else /* CONFIG_DA850_WL12XX */

static __init int da850_wl12xx_init(void)
{
	return 0;
}

#endif /* CONFIG_DA850_WL12XX */

#define DA850EVM_SATA_REFCLKPN_RATE	(100 * 1000 * 1000)

static __init void da850_evm_init(void)
{
	int ret;

	ret = da850_register_gpio();
	if (ret)
		pr_warn("%s: GPIO init failed: %d\n", __func__, ret);

	regulator_register_fixed(0, fixed_supplies, ARRAY_SIZE(fixed_supplies));

	ret = da850_register_edma(da850_edma_rsv);
	if (ret)
		pr_warn("%s: EDMA registration failed: %d\n", __func__, ret);

	ret = davinci_cfg_reg_list(da850_i2c0_pins);
	if (ret)
		pr_warn("%s: I2C0 mux setup failed: %d\n", __func__, ret);

	ret = da8xx_register_i2c(0, &da850_evm_i2c_0_pdata);
	if (ret)
		pr_warn("%s: I2C0 registration failed: %d\n", __func__, ret);


	ret = da8xx_register_watchdog();
	if (ret)
		pr_warn("%s: watchdog registration failed: %d\n",
			__func__, ret);

	if (HAS_MMC) {
		ret = davinci_cfg_reg_list(da850_evm_mmcsd0_pins);
		if (ret)
			pr_warn("%s: MMCSD0 mux setup failed: %d\n",
				__func__, ret);

		ret = gpio_request(DA850_MMCSD_CD_PIN, "MMC CD\n");
		if (ret)
			pr_warn("%s: can not open GPIO %d\n",
				__func__, DA850_MMCSD_CD_PIN);
		gpio_direction_input(DA850_MMCSD_CD_PIN);

		ret = gpio_request(DA850_MMCSD_WP_PIN, "MMC WP\n");
		if (ret)
			pr_warn("%s: can not open GPIO %d\n",
				__func__, DA850_MMCSD_WP_PIN);
		gpio_direction_input(DA850_MMCSD_WP_PIN);

		ret = da8xx_register_mmcsd0(&da850_mmc_config);
		if (ret)
			pr_warn("%s: MMCSD0 registration failed: %d\n",
				__func__, ret);

		ret = da850_wl12xx_init();
		if (ret)
			pr_warn("%s: WL12xx initialization failed: %d\n",
				__func__, ret);
	}

	davinci_serial_init(da8xx_serial_device);

	i2c_register_board_info(1, da850_evm_i2c_devices,
			ARRAY_SIZE(da850_evm_i2c_devices));

	/*
	 * shut down uart 0 and 1; they are not used on the board and
	 * accessing them causes endless "too much work in irq53" messages
	 * with arago fs
	 */
	__raw_writel(0, IO_ADDRESS(DA8XX_UART1_BASE) + 0x30);
	__raw_writel(0, IO_ADDRESS(DA8XX_UART0_BASE) + 0x30);


	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret)
		pr_warn("%s: LCDC mux setup failed: %d\n", __func__, ret);

	ret = da8xx_register_uio_pruss();
	if (ret)
		pr_warn("da850_evm_init: pruss initialization failed: %d\n",
				ret);

	/* Handle board specific muxing for LCD here */
	ret = davinci_cfg_reg_list(da850_evm_lcdc_pins);
	if (ret)
		pr_warn("%s: EVM specific LCD mux setup failed: %d\n",
			__func__, ret);

	ret = da850_lcd_hw_init();
	if (ret)
		pr_warn("%s: LCD initialization failed: %d\n", __func__, ret);

	sharp_lk043t1dg01_pdata.panel_power_ctrl = da850_panel_power_ctrl,
	ret = da8xx_register_lcdc(&sharp_lk043t1dg01_pdata);
	if (ret)
		pr_warn("%s: LCDC registration failed: %d\n", __func__, ret);

	da850_evm_setup_nor_nand();

	ret = da8xx_register_rtc();
	if (ret)
		pr_warn("%s: RTC setup failed: %d\n", __func__, ret);

	ret = da850_evm_init_cpufreq();
	if (ret)
		pr_warn("%s: cpufreq registration failed: %d\n", __func__, ret);

	ret = da8xx_register_cpuidle();
	if (ret)
		pr_warn("%s: cpuidle registration failed: %d\n", __func__, ret);

	ret = da850_register_pm(&da850_pm_device);
	if (ret)
		pr_warn("%s: suspend registration failed: %d\n", __func__, ret);

	da850_vpif_init();

	ret = spi_register_board_info(da850evm_spi_info,
				      ARRAY_SIZE(da850evm_spi_info));
	if (ret)
		pr_warn("%s: spi info registration failed: %d\n", __func__,
			ret);

	ret = da8xx_register_spi_bus(1, ARRAY_SIZE(da850evm_spi_info));
	if (ret)
		pr_warn("%s: SPI 1 registration failed: %d\n", __func__, ret);

	ret = da850_register_sata(DA850EVM_SATA_REFCLKPN_RATE);
	if (ret)
		pr_warn("%s: SATA registration failed: %d\n", __func__, ret);

	ret = da8xx_register_rproc();
	if (ret)
		pr_warn("%s: dsp/rproc registration failed: %d\n",
			__func__, ret);
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init da850_evm_console_init(void)
{
	if (!machine_is_davinci_da850_evm())
		return 0;

	return add_preferred_console("ttyS", 2, "115200");
}
console_initcall(da850_evm_console_init);
#endif

static void __init da850_evm_map_io(void)
{
	da850_init();
}

MACHINE_START(DAVINCI_DA850_EVM, "DaVinci DA850/OMAP-L138/AM18x EVM")
	.atag_offset	= 0x100,
	.map_io		= da850_evm_map_io,
	.init_irq	= cp_intc_init,
	.init_time	= davinci_timer_init,
	.init_machine	= da850_evm_init,
	.init_late	= davinci_init_late,
	.dma_zone_size	= SZ_128M,
	.restart	= da8xx_restart,
	.reserve	= da8xx_rproc_reserve_cma,
MACHINE_END