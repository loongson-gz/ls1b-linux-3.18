/*
 * Copyright (c) 2011 Zhang, Keguang <keguang.zhang@gmail.com>
 * Copyright (c) 2015 Tang Haifeng <tanghaifeng-gz@loongson.cn> or <pengren.mcu@qq.com>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/gpio.h>

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

#ifdef CONFIG_TOUCHSCREEN_ADS7846
#include <linux/spi/ads7846.h>
#define ADS7846_GPIO_IRQ 60 /* 开发板触摸屏使用的外部中断 */
static struct ads7846_platform_data ads_info __maybe_unused = {
	.model				= 7846,
	.vref_delay_usecs	= 1,
	.keep_vref_on		= 0,
	.settle_delay_usecs	= 20,
//	.x_plate_ohms		= 800,
	.pressure_min		= 0,
	.pressure_max		= 2048,
	.debounce_rep		= 3,
	.debounce_max		= 10,
	.debounce_tol		= 50,
//	.get_pendown_state	= ads7846_pendown_state,
	.get_pendown_state	= NULL,
	.gpio_pendown		= ADS7846_GPIO_IRQ,
	.filter_init		= NULL,
	.filter 			= NULL,
	.filter_cleanup 	= NULL,
};
#endif /* TOUCHSCREEN_ADS7846 */

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
/* 开发板使用GPIO40(CAN1_RX)引脚作为MMC/SD卡的插拔探测引脚 */
#define DETECT_GPIO  3
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
		.mode = SPI_MODE_3,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	{
		.modalias = "ads7846",
		.platform_data = &ads_info,
		.bus_num 		= 0,
		.chip_select 	= SPI0_CS1,
		.max_speed_hz 	= 2500000,
		.mode 			= SPI_MODE_1,
		.irq			= LS1X_GPIO_FIRST_IRQ + ADS7846_GPIO_IRQ,
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
#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
		.modalias		= "mmc_spi",
		.bus_num 		= 1,
		.chip_select	= SPI1_CS0,
		.max_speed_hz	= 20000000,
		.platform_data	= &mmc_spi,
		.mode = SPI_MODE_3,
	},
#endif
};
#endif

#ifdef CONFIG_I2C_LS1X
#include <linux/i2c.h>
static struct i2c_board_info ls1x_i2c0_board_info[] = {
#ifdef CONFIG_RTC_DRV_SD2068
	{
		I2C_BOARD_INFO("sd2068", 0x32),
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

#ifdef CONFIG_LEDS_PWM
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
static struct pwm_lookup pwm_lookup[] = {
	/* LEDB -> PMU_STAT */
	PWM_LOOKUP("ls1x-pwm.2", 0, "leds_pwm", "ls1x_pwm_led2",
			7812500, PWM_POLARITY_NORMAL),
/*	PWM_LOOKUP("ls1x-pwm.3", 0, "leds_pwm", "ls1x_pwm_led3",
			7812500, PWM_POLARITY_NORMAL),*/
};

static struct led_pwm ls1x_pwm_leds[] = {
	{
		.name		= "ls1x_pwm_led2",
		.max_brightness	= 255,
		.pwm_period_ns	= 7812500,
	},
/*	{
		.name		= "ls1x_pwm_led3",
		.max_brightness	= 255,
		.pwm_period_ns	= 7812500,
	},*/
};

static struct led_pwm_platform_data ls1x_pwm_data = {
	.num_leds	= ARRAY_SIZE(ls1x_pwm_leds),
	.leds		= ls1x_pwm_leds,
};

static struct platform_device ls1x_leds_pwm = {
	.name	= "leds_pwm",
	.id		= -1,
	.dev	= {
		.platform_data = &ls1x_pwm_data,
	},
};
#endif //#ifdef CONFIG_LEDS_PWM

#if defined(CONFIG_BACKLIGHT_PWM)
#include <linux/pwm_backlight.h>
static struct pwm_lookup pwm_backlight_lookup[] = {
	PWM_LOOKUP("ls1x-pwm.3", 0, "pwm-backlight", NULL,
			7812500, PWM_POLARITY_NORMAL),
};

static struct platform_pwm_backlight_data ls1x_backlight_data = {
	.pwm_id		= -1,	/* Superseded by pwm_lookup */
	.max_brightness	= 255,
	.dft_brightness	= 100,
	.pwm_period_ns	= 7812500,
};

static struct platform_device ls1x_pwm_backlight = {
	.name = "pwm-backlight",
	.id   = -1,
	.dev = {
//		.parent	= &ls1x_pwm3_pdev.dev,
		.platform_data = &ls1x_backlight_data,
	},
};
#endif

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
#ifdef CONFIG_LEDS_PWM
	&ls1x_leds_pwm,
#endif
#if defined(CONFIG_BACKLIGHT_PWM)
	&ls1x_pwm_backlight,
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
#if defined(CONFIG_BACKLIGHT_PWM)
	pwm_add_table(pwm_backlight_lookup, ARRAY_SIZE(pwm_backlight_lookup));
#endif

	err = platform_add_devices(ls1b_platform_devices,
				   ARRAY_SIZE(ls1b_platform_devices));
	return err;
}

arch_initcall(ls1b_platform_init);
