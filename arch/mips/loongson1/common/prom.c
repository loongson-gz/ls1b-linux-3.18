/*
 * Copyright (c) 2011 Zhang, Keguang <keguang.zhang@gmail.com>
 * Copyright (c) 2015 Tang Haifeng <tanghaifeng-gz@loongson.cn>
 *
 * Modified from arch/mips/pnx833x/common/prom.c.
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <asm/bootinfo.h>

#include <loongson1.h>
#include <prom.h>

int prom_argc;
char **prom_argv, **prom_envp;
unsigned long memsize, highmemsize;

char *prom_getenv(char *envname)
{
	char **env = prom_envp;
	int i;

	i = strlen(envname);

	while (*env) {
		if (strncmp(envname, *env, i) == 0 && *(*env+i) == '=')
			return *env + i + 1;
		env++;
	}

	return 0;
}

static inline unsigned long env_or_default(char *env, unsigned long dfl)
{
	char *str = prom_getenv(env);
	return str ? simple_strtol(str, 0, 0) : dfl;
}

void __init prom_init_cmdline(void)
{
	char *c = &(arcs_cmdline[0]);
	int i;

	for (i = 1; i < prom_argc; i++) {
		strcpy(c, prom_argv[i]);
		c += strlen(prom_argv[i]);
		if (i < prom_argc-1)
			*c++ = ' ';
	}
	*c = 0;
}

void __init prom_init(void)
{
	prom_argc = fw_arg0;
	prom_argv = (char **)fw_arg1;
	prom_envp = (char **)fw_arg2;

#if defined(CONFIG_LOONGSON1_LS1C)
	__raw_writel(__raw_readl(LS1X_MUX_CTRL0) & (~USBHOST_SHUT), LS1X_MUX_CTRL0);
	__raw_writel(__raw_readl(LS1X_MUX_CTRL1) & (~USBHOST_RSTN), LS1X_MUX_CTRL1);
	mdelay(60);
	/* reset stop */
	__raw_writel(__raw_readl(LS1X_MUX_CTRL1) | USBHOST_RSTN, LS1X_MUX_CTRL1);
#else
	/* 需要复位一次USB控制器，且复位时间要足够长，否则启动时莫名其妙的死机 */
	#if defined(CONFIG_LOONGSON1_LS1A)
	#define MUX_CTRL0 LS1X_MUX_CTRL0
	#define MUX_CTRL1 LS1X_MUX_CTRL1
	#elif defined(CONFIG_LOONGSON1_LS1B)
	#define MUX_CTRL0 LS1X_MUX_CTRL1
	#define MUX_CTRL1 LS1X_MUX_CTRL1
	#endif
//	__raw_writel(__raw_readl(MUX_CTRL0) | USB_SHUT, MUX_CTRL0);
//	__raw_writel(__raw_readl(MUX_CTRL1) & (~USB_RESET), MUX_CTRL1);
//	mdelay(10);
//	#if defined(CONFIG_USB_EHCI_HCD_LS1X) || defined(CONFIG_USB_OHCI_HCD_LS1X)
	/* USB controller enable and reset */
	__raw_writel(__raw_readl(MUX_CTRL0) & (~USB_SHUT), MUX_CTRL0);
	__raw_writel(__raw_readl(MUX_CTRL1) & (~USB_RESET), MUX_CTRL1);
	mdelay(60);
	/* reset stop */
	__raw_writel(__raw_readl(MUX_CTRL1) | USB_RESET, MUX_CTRL1);
//	#endif
#endif

	prom_init_cmdline();

	memsize = env_or_default("memsize", DEFAULT_MEMSIZE);
	highmemsize = env_or_default("highmemsize", 0x0);

	pr_info("memsize=%ldMB, highmemsize=%ldMB\n", memsize, highmemsize);
}

void __init prom_free_prom_memory(void)
{
}

void prom_putchar(char c)
{
	int timeout;

	timeout = 1024;

	while (((readb(PORT(UART_LSR)) & UART_LSR_THRE) == 0)
			&& (timeout-- > 0))
		;

	writeb(c, PORT(UART_TX));
}
