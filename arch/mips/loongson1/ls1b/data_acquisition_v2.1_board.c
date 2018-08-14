/*
 * Copyright (c) 2015 Tang Haifeng <pengren.mcu@qq.com>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <platform.h>
#include <loongson1.h>

#ifdef CONFIG_DMA_LOONGSON1
#include <dma.h>
struct plat_ls1x_dma ls1x_dma_pdata = {
	.nr_channels	= 1,
};
#endif

#if defined(CONFIG_MTD_NAND_LOONGSON1) || defined(CONFIG_MTD_NAND_LS1X)
#include <nand.h>
struct plat_ls1x_nand ls1x_nand_pdata = {
	.hold_cycle	= 0x2,
	.wait_cycle	= 0xc,
};
#endif

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
#include <linux/spi/flash.h>
static struct mtd_partition ls1x_spi_flash_partitions[] = {
	{
		.name = "bootloader",
		.size = 512*1024,
		.offset = 0,
//		.mask_flags = MTD_CAP_ROM
	}
};

static struct flash_platform_data ls1x_spi_flash_data = {
	.name = "spi-flash",
	.parts = ls1x_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(ls1x_spi_flash_partitions),
	.type = "w25x40",
};
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
/* 开发板使用GPIO40(CAN1_RX)引脚作为MMC/SD卡的插拔探测引脚 */
#define DETECT_GPIO  29
static struct mmc_spi_platform_data mmc_spi __maybe_unused = {
	.flags = MMC_SPI_USE_CD_GPIO,
	.cd_gpio = DETECT_GPIO,
//	.caps = MMC_CAP_NEEDS_POLL,
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V only */
};
#endif  /* defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE) */

#ifdef CONFIG_SPI_LS1X_SPI0
#include <linux/spi/spi.h>
#include <linux/spi/spi_ls1x.h>
#if defined(CONFIG_SPI_CS_USED_GPIO)
static int spi0_gpios_cs[] = { 27, 28, 29, 30 };
#endif

struct ls1x_spi_platform_data ls1x_spi0_platdata = {
#if defined(CONFIG_SPI_CS_USED_GPIO)
	.gpio_cs_count = ARRAY_SIZE(spi0_gpios_cs),
	.gpio_cs = spi0_gpios_cs,
#elif defined(CONFIG_SPI_CS)
	.cs_count = SPI0_CS3 + 1,
#endif
};

static struct spi_board_info ls1x_spi0_devices[] = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		.modalias	= "m25p80",
		.bus_num 		= 0,
		.chip_select	= SPI0_CS0,
		.max_speed_hz	= 60000000,
		.platform_data	= &ls1x_spi_flash_data,
	},
#endif
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias		= "mmc_spi",
		.bus_num 		= 0,
		.chip_select	= SPI0_CS1,
		.max_speed_hz	= 25000000,
		.platform_data	= &mmc_spi,
	},
#endif
};
#endif

#ifdef CONFIG_SPI_LS1X_SPI1
#include <linux/spi/spi.h>
#include <linux/spi/spi_ls1x.h>
#if defined(CONFIG_SPI_CS_USED_GPIO)
static int spi1_gpios_cs[] = { 38, 0, 1 };
#endif

struct ls1x_spi_platform_data ls1x_spi1_platdata = {
#if defined(CONFIG_SPI_CS_USED_GPIO)
	.gpio_cs_count = ARRAY_SIZE(spi1_gpios_cs),
	.gpio_cs = spi1_gpios_cs,
#elif defined(CONFIG_SPI_CS)
	.cs_count = SPI1_CS2 + 1,
#endif
};

static struct spi_board_info ls1x_spi1_devices[] = {
};
#endif

#if defined(CONFIG_GPIO_PCA953X) || defined(CONFIG_GPIO_PCA953X_MODULE)
#include <linux/i2c.h>
#include <linux/platform_data/pca953x.h>
#define TCA6424_GPIO_BASE 188
#define TCA6424_IRQ_BASE 188
#define TCA6424_GPIO_IRQ 36

static int tca6424_setup(struct i2c_client *client, unsigned gpio,
				unsigned int ngpio, void *context)
{
	int i = 0;

	for (i=0; i<8; i++) {
		gpio_direction_output(gpio+i, 0);
	}

	gpio_request(34, "lv245a_oe");
	gpio_direction_output(34, 0);
	gpio_free(34);

#define USB_HUB_RESET (TCA6424_GPIO_BASE + 16)
#define USB_4G_RESET (TCA6424_GPIO_BASE + 19)
	//usb hub复位 4g模块复位
	gpio_request(USB_HUB_RESET, "usb_hub_reset");
	gpio_request(USB_4G_RESET, "usb_4g_reset");
	gpio_direction_output(USB_HUB_RESET, 0);
	gpio_direction_output(USB_4G_RESET, 1);
	msleep(50);
	gpio_set_value(USB_HUB_RESET, 1);
	gpio_set_value(USB_4G_RESET, 0);
	gpio_free(USB_HUB_RESET);
	gpio_free(USB_4G_RESET);

	return 0;
}

static struct pca953x_platform_data i2c_tca6424_platdata = {
	.gpio_base	= TCA6424_GPIO_BASE, /* Start directly after the CPU's GPIO */
	.irq_base = TCA6424_IRQ_BASE,
//	.invert		= 0, /* Do not invert */
	.setup		= tca6424_setup,
};
#endif /* defined(CONFIG_GPIO_PCA953X) || defined(CONFIG_GPIO_PCA953X_MODULE) */

#ifdef CONFIG_I2C_LS1X
#include <linux/i2c.h>
static struct i2c_board_info ls1x_i2c0_board_info[] = {
#ifdef CONFIG_RTC_DRV_SD2068
	{
		I2C_BOARD_INFO("sd2068", 0x32),
	},
#endif
#if defined(CONFIG_GPIO_PCA953X) || defined(CONFIG_GPIO_PCA953X_MODULE)
	{
		I2C_BOARD_INFO("tca6424", 0x22),
		.irq = LS1X_GPIO_FIRST_IRQ + TCA6424_GPIO_IRQ,
		.platform_data = &i2c_tca6424_platdata,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX
	{
		I2C_BOARD_INFO("GDIX1001:00", 0x14),
		.irq = LS1X_GPIO_FIRST_IRQ + 35,
	},
#endif
};

#include <linux/i2c-ls1x.h>
struct ls1x_i2c_platform_data ls1x_i2c0_data = {
	.bus_clock_hz = 100000, /* i2c bus clock in Hz */
	.devices	= ls1x_i2c0_board_info, /* optional table of devices */
	.num_devices	= ARRAY_SIZE(ls1x_i2c0_board_info), /* table size */
};

struct ls1x_i2c_platform_data ls1x_i2c1_data = {
	.bus_clock_hz = 100000, /* i2c bus clock in Hz */
//	.devices	= ls1x_i2c1_board_info, /* optional table of devices */
//	.num_devices	= ARRAY_SIZE(ls1x_i2c1_board_info), /* table size */
};

struct ls1x_i2c_platform_data ls1x_i2c2_data = {
	.bus_clock_hz = 100000, /* i2c bus clock in Hz */
//	.devices	= ls1x_i2c2_board_info, /* optional table of devices */
//	.num_devices	= ARRAY_SIZE(ls1x_i2c2_board_info), /* table size */
};
#endif

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
#include <linux/leds.h>
static struct gpio_led gpio_leds[] = {
	{
		.name			= "led_orange",
		.gpio			= 37,
		.active_low		= 0,
		.default_trigger	= NULL,//"heartbeat",
		.default_state	= LEDS_GPIO_DEFSTATE_ON,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	}
};
#endif //#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)

#ifdef CONFIG_BACKLIGHT_GPIO
#define GPIO_BACKLIGHT_CTRL	(TCA6424_GPIO_BASE + 20)
#include <linux/platform_data/gpio_backlight.h>
static struct gpio_backlight_platform_data gpio_backlight_data = {
	.fbdev = &ls1x_fb0_pdev.dev,
	.gpio = GPIO_BACKLIGHT_CTRL,
	.def_value = 1,
	.name = "backlight",
};

static struct platform_device ls1x_bl_pdev = {
	.name			= "gpio-backlight",
	.dev = {
		.platform_data	= &gpio_backlight_data,
	},
};
#endif //#ifdef CONFIG_BACKLIGHT_GPIO

#ifdef CONFIG_CAN_SJA1000_PLATFORM
#include <linux/can/platform/sja1000.h>
static void ls1x_can_setup(void)
{
	struct sja1000_platform_data *sja1000_pdata;
	struct clk *clk;
	u32 x;

	clk = clk_get(NULL, "apb_clk");
	if (IS_ERR(clk))
		panic("unable to get apb clock, err=%ld", PTR_ERR(clk));

	#ifdef CONFIG_LS1X_CAN0
	sja1000_pdata = &ls1x_sja1000_platform_data_0;
	sja1000_pdata->osc_freq = clk_get_rate(clk);
	#endif
	#ifdef CONFIG_LS1X_CAN1
	sja1000_pdata = &ls1x_sja1000_platform_data_1;
	sja1000_pdata->osc_freq = clk_get_rate(clk);
	#endif

	#ifdef CONFIG_LS1X_CAN0
	/* CAN0复用设置 */
/*	gpio_request(38, NULL);
	gpio_request(39, NULL);
	gpio_free(38);
	gpio_free(39);*/
	/* 清除与 SPI1 UART1_2 的复用  */
	x = __raw_readl(LS1X_MUX_CTRL1);
	x = x & (~SPI1_USE_CAN) & (~UART1_2_USE_CAN0);
	__raw_writel(x, LS1X_MUX_CTRL1);
	/* 清除与 I2C1 的复用  */
	x = __raw_readl(LS1X_MUX_CTRL0);
	x = x & (~I2C1_USE_CAN0);
	__raw_writel(x, LS1X_MUX_CTRL0);
	#endif
	#ifdef CONFIG_LS1X_CAN1
	/* CAN1复用设置 */
/*	gpio_request(40, NULL);
	gpio_request(41, NULL);
	gpio_free(40);
	gpio_free(41);*/
	/* 清除与 SPI1 UART1_3 的复用  */
	x = __raw_readl(LS1X_MUX_CTRL1);
	x = x & (~SPI1_USE_CAN) & (~UART1_3_USE_CAN1);
	__raw_writel(x, LS1X_MUX_CTRL1);
	/* 清除与 I2C2 的复用  */
	x = __raw_readl(LS1X_MUX_CTRL0);
	x = x & (~I2C2_USE_CAN1);
	__raw_writel(x, LS1X_MUX_CTRL0);
	#endif
}
#endif //#ifdef CONFIG_CAN_SJA1000_PLATFORM


static struct platform_device *ls1b_platform_devices[] __initdata = {
	&ls1x_uart_pdev,
#ifdef CONFIG_DMA_LOONGSON1
	&ls1x_dma_pdev,
#endif
#if defined(CONFIG_MTD_NAND_LOONGSON1) || defined(CONFIG_MTD_NAND_LS1X)
	&ls1x_nand_pdev,
#endif
#if defined(CONFIG_LS1X_GMAC0)
	&ls1x_eth0_pdev,
#endif
#if defined(CONFIG_LS1X_GMAC1)
	&ls1x_eth1_pdev,
#endif
#ifdef CONFIG_USB_OHCI_HCD_PLATFORM
	&ls1x_ohci_pdev,
#endif
#ifdef CONFIG_USB_EHCI_HCD_PLATFORM
	&ls1x_ehci_pdev,
#endif
#ifdef CONFIG_RTC_DRV_RTC_LOONGSON1
	&ls1x_rtc_pdev,
#endif
#ifdef CONFIG_RTC_DRV_TOY_LOONGSON1
	&ls1x_toy_pdev,
#endif
#ifdef CONFIG_LS1X_WDT
	&ls1x_wdt_pdev,
#endif
#ifdef CONFIG_SPI_LS1X_SPI0
	&ls1x_spi0_pdev,
#endif
#ifdef CONFIG_SPI_LS1X_SPI1
	&ls1x_spi1_pdev,
#endif
#ifdef CONFIG_I2C_LS1X
	&ls1x_i2c0_pdev,
#endif
#ifdef CONFIG_LS1X_FB0
	&ls1x_fb0_pdev,
#endif
#ifdef CONFIG_SND_LS1X_SOC_AC97
	&ls1x_ac97_pdev,
	&ls1x_stac_pdev,
#endif
#ifdef CONFIG_SND_LS1X_SOC
	&ls1x_pcm_pdev,
	&ls1x_audio_pdev,
#endif
#ifdef CONFIG_PWM_LS1X_PWM0
	&ls1x_pwm0_pdev,
#endif
#ifdef CONFIG_PWM_LS1X_PWM1
	&ls1x_pwm1_pdev,
#endif
#ifdef CONFIG_PWM_LS1X_PWM2
	&ls1x_pwm2_pdev,
#endif
#ifdef CONFIG_PWM_LS1X_PWM3
	&ls1x_pwm3_pdev,
#endif
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	&leds,
#endif
#ifdef CONFIG_BACKLIGHT_GPIO
	&ls1x_bl_pdev,
#endif
#ifdef CONFIG_CAN_SJA1000_PLATFORM
#ifdef CONFIG_LS1X_CAN0
	&ls1x_sja1000_0,
#endif
#ifdef CONFIG_LS1X_CAN1
	&ls1x_sja1000_1,
#endif
#endif
};

static int __init ls1b_platform_init(void)
{
	int err;

	ls1x_pin_mux();
	ls1x_serial_setup(&ls1x_uart_pdev);
#ifdef CONFIG_DMA_LOONGSON1
	ls1x_dma_set_platdata(&ls1x_dma_pdata);
#endif
#if defined(CONFIG_MTD_NAND_LOONGSON1) || defined(CONFIG_MTD_NAND_LS1X)
	ls1x_nand_set_platdata(&ls1x_nand_pdata);
#endif
#if defined(CONFIG_SPI_LS1X_SPI0)
	spi_register_board_info(ls1x_spi0_devices, ARRAY_SIZE(ls1x_spi0_devices));
#endif
#if defined(CONFIG_SPI_LS1X_SPI1)
	/* SPI1复用设置 */
	__raw_writel(__raw_readl(LS1X_MUX_CTRL1) | SPI1_USE_CAN | SPI1_CS_USE_PWM01, LS1X_MUX_CTRL1);
	spi_register_board_info(ls1x_spi1_devices, ARRAY_SIZE(ls1x_spi1_devices));
#endif
#ifdef CONFIG_CAN_SJA1000_PLATFORM
	ls1x_can_setup();
#endif

	/* 根据需要修改复用关系，gma0需要用到pwm01 gmac1需要用到pwm23，可能需要把pwm或gmac的驱动选项关闭 */
#if defined(CONFIG_PWM_LS1X_PWM0) || defined(CONFIG_PWM_LS1X_PWM1)
	{
	u32 x;
	x = __raw_readl(LS1X_MUX_CTRL0);
	x = x & (~UART0_USE_PWM01) & (~NAND3_USE_PWM01) & (~NAND2_USE_PWM01) & (~NAND1_USE_PWM01);
	__raw_writel(x, LS1X_MUX_CTRL0);
	x = __raw_readl(LS1X_MUX_CTRL1);
	x = x & (~SPI1_CS_USE_PWM01) & (~GMAC0_USE_PWM01);
	__raw_writel(x, LS1X_MUX_CTRL1);
	}
#endif
#if defined(CONFIG_PWM_LS1X_PWM2) || defined(CONFIG_PWM_LS1X_PWM3)
	{
	u32 x;
	x = __raw_readl(LS1X_MUX_CTRL0);
	x = x & (~UART0_USE_PWM23) & (~NAND3_USE_PWM23) & (~NAND2_USE_PWM23) & (~NAND1_USE_PWM23);
	__raw_writel(x, LS1X_MUX_CTRL0);
	x = __raw_readl(LS1X_MUX_CTRL1);
	x = x & (~GMAC1_USE_PWM23);
	__raw_writel(x, LS1X_MUX_CTRL1);
	}
#endif
#ifdef CONFIG_LEDS_PWM
	pwm_add_table(pwm_lookup, ARRAY_SIZE(pwm_lookup));
#endif

	err = platform_add_devices(ls1b_platform_devices,
				   ARRAY_SIZE(ls1b_platform_devices));
	return err;
}

arch_initcall(ls1b_platform_init);
