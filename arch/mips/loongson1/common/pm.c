/*
 * Copyright (c) 2016 Tang Haifeng <tanghaifeng-gz@loongson.cn> or <pengren.mcu@qq.com>
 *	Loongson1 SoC power management support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/gpio.h>
#include <asm/idle.h>

#include <loongson1.h>

static int ls1x_pm_enter(suspend_state_t state)
{
	int regval __maybe_unused;

	/* cpu时钟频率设置为晶振频率（降频） */
#if defined(CONFIG_LOONGSON1_LS1A)
#elif defined(CONFIG_LOONGSON1_LS1B)
	regval = __raw_readl(LS1X_CLK_PLL_DIV);
	regval |= 0x00000300;	//cpu_bypass 置1
	__raw_writel(regval, LS1X_CLK_PLL_DIV);
#elif defined(CONFIG_LOONGSON1_LS1C)
	regval = __raw_readl(LS1X_CLK_PLL_DIV);
	regval &= ~0x00000003;	//cpu_bypass 置1
	__raw_writel(regval, LS1X_CLK_PLL_DIV);
#endif

#if 1
	cpu_wait();
	cpu_wait();
#else
	/* 使用某个gpio作为唤醒 */
/*	regval = __raw_readl(LS1X_GPIO_CFG1);
	regval |= (0x1 << 8);
	__raw_writel(regval, LS1X_GPIO_CFG1);

	regval = __raw_readl(LS1X_GPIO_OE1);
	regval |= (0x1 << 8);
	__raw_writel(regval, LS1X_GPIO_OE1);

	while (1) {
		regval = __raw_readl(LS1X_GPIO_IN1);
		regval &= (0x1 << 8);
		if (regval == 0) {
			mdelay(4);
			regval = __raw_readl(LS1X_GPIO_IN1);
			regval &= (0x1 << 8);
			if (regval == 0)
				break;
		}
	}

	regval = __raw_readl(LS1X_GPIO_CFG1);
	regval &= ~(0x1 << 8);
	__raw_writel(regval, LS1X_GPIO_CFG1);*/

	#define PM_GPIO	29	// 假设使用gpio0
	gpio_request(PM_GPIO, "pm_suspend");
	gpio_direction_input(PM_GPIO);

	while (1) {
		regval = gpio_get_value(PM_GPIO);
		if (regval == 0) {
			mdelay(4);
			regval = gpio_get_value(PM_GPIO);
			if (regval == 0)
				break;
		}
	}
	gpio_free(PM_GPIO);
#endif

	/* 恢复cpu频率 */
#if defined(CONFIG_LOONGSON1_LS1A)
#elif defined(CONFIG_LOONGSON1_LS1B)
	regval = __raw_readl(LS1X_CLK_PLL_DIV);
	regval &= ~0x00000100;	//cpu_bypass 清0
	__raw_writel(regval, LS1X_CLK_PLL_DIV);
#elif defined(CONFIG_LOONGSON1_LS1C)
	regval = __raw_readl(LS1X_CLK_PLL_DIV);
	regval |= 0x00000003;	//cpu_bypass 清0
	__raw_writel(regval, LS1X_CLK_PLL_DIV);
#endif

	return 0;
}

static int ls1x_pm_valid_state(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;

	default:
		return 0;
	}
}

static const struct platform_suspend_ops ls1x_pm_ops = {
	.valid		= ls1x_pm_valid_state,
	.enter		= ls1x_pm_enter,
};

static int __init ls1x_pm_init(void)
{
	suspend_set_ops(&ls1x_pm_ops);
	return 0;

}
late_initcall(ls1x_pm_init);
