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

#include <platform.h>
#include <loongson1.h>

#ifdef CONFIG_MTD_NAND_LS1X
#include <ls1x_nand.h>
static struct mtd_partition ls1x_nand_partitions[] = {
	{
		.name	= "kernel",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 14*1024*1024,
	}, {
		.name	= "rootfs",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 100*1024*1024,
	}, {
		.name	= "data",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

struct ls1x_nand_platform_data ls1x_nand_parts = {
	.parts		= ls1x_nand_partitions,
	.nr_parts	= ARRAY_SIZE(ls1x_nand_partitions),
};
#endif

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
#include <linux/leds.h>
static struct gpio_led gpio_leds[] = {
	{
		.name			= "led_green0",
		.gpio			= 38,
		.active_low		= 1,
		.default_trigger	= "timer",
		.default_state	= LEDS_GPIO_DEFSTATE_ON,
	},
	{
		.name			= "led_green1",
		.gpio			= 39,
		.active_low		= 1,
		.default_trigger	= "heartbeat",
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

#ifdef CONFIG_INPUT_GPIO_BEEPER
#include <linux/gpio.h>
#include <linux/gpio/machine.h>
static struct gpiod_lookup_table buzzer_gpio_table = {
	.dev_id = "gpio-beeper",	/* 注意 该值与ls1x_gpio_beeper.name相同 */
	.table = {
		GPIO_LOOKUP("ls1x-gpio1", 8, NULL, 0),	/* 使用第1组gpio的第8个引脚 注意"ls1x-gpio1"与gpio的label值相同 */
		{ },
	},
};

struct platform_device ls1x_gpio_beeper = {
	.name = "gpio-beeper",
	.id = -1,
};
#endif

static struct platform_device *ls1b_platform_devices[] __initdata = {
	&ls1x_uart_pdev,
#ifdef CONFIG_MTD_NAND_LS1X
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
	&ls1x_rtc_pdev,
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	&leds,
#endif
#ifdef CONFIG_INPUT_GPIO_BEEPER
	&ls1x_gpio_beeper,
#endif
};

static int __init ls1b_platform_init(void)
{
	int err;

	ls1x_serial_setup(&ls1x_uart_pdev);

#ifdef CONFIG_INPUT_GPIO_BEEPER
	gpiod_add_lookup_table(&buzzer_gpio_table);
#endif

	err = platform_add_devices(ls1b_platform_devices,
				   ARRAY_SIZE(ls1b_platform_devices));
	return err;
}

arch_initcall(ls1b_platform_init);
