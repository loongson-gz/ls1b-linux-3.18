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
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/phy.h>
#include <linux/serial_8250.h>
#include <linux/stmmac.h>
#include <asm-generic/sizes.h>

#include <loongson1.h>

#define LS1X_UART(_id)						\
	{							\
		.mapbase	= LS1X_UART ## _id ## _BASE,	\
		.irq		= LS1X_UART ## _id ## _IRQ,	\
		.iotype		= UPIO_MEM,			\
		.flags		= UPF_IOREMAP | UPF_FIXED_TYPE, \
		.type		= PORT_16550A,			\
	}

static struct plat_serial8250_port ls1x_serial8250_port[] = {
	LS1X_UART(0),
	LS1X_UART(1),
	LS1X_UART(2),
	LS1X_UART(3),
	{},
};

struct platform_device ls1x_uart_pdev = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev		= {
		.platform_data = ls1x_serial8250_port,
	},
};

void __init ls1x_serial_setup(struct platform_device *pdev)
{
	struct clk *clk;
	struct plat_serial8250_port *p;

	clk = clk_get(NULL, pdev->name);
	if (IS_ERR(clk))
		panic("unable to get %s clock, err=%ld",
			pdev->name, PTR_ERR(clk));

	for (p = pdev->dev.platform_data; p->flags != 0; ++p)
		p->uartclk = clk_get_rate(clk);
}

/* Synopsys Ethernet GMAC */
#if defined(CONFIG_STMMAC_ETH)
static struct stmmac_mdio_bus_data ls1x_mdio_bus_data = {
	.phy_mask	= 0,
};

static struct stmmac_dma_cfg ls1x_eth_dma_cfg = {
	.pbl		= 1,
};

int ls1x_eth_mux_init(struct platform_device *pdev, void *priv)
{
	struct plat_stmmacenet_data *plat_dat = NULL;
	u32 val;

	val = __raw_readl(LS1X_MUX_CTRL1);

#if defined(CONFIG_LOONGSON1_LS1A)
	val = __raw_readl(LS1X_MUX_CTRL0);
	plat_dat = dev_get_platdata(&pdev->dev);
	if (plat_dat->bus_id) {
		val |= (GMAC1_USE_UART1 | GMAC1_USE_UART0);
		switch (plat_dat->interface) {
		case PHY_INTERFACE_MODE_RGMII:
			val = val & (~GMAC1_USE_TXCLK) & (~GMAC1_USE_PWM23);
			break;
		case PHY_INTERFACE_MODE_MII:
			val = val | GMAC1_USE_TXCLK | GMAC1_USE_PWM23;
			break;
		default:
			pr_err("unsupported mii mode %d\n",
			       plat_dat->interface);
			return -ENOTSUPP;
		}
		__raw_writel(val & (~GMAC1_SHUT), LS1X_MUX_CTRL0);
	} else {
		switch (plat_dat->interface) {
		case PHY_INTERFACE_MODE_RGMII:
			val = val & (~GMAC0_USE_TXCLK) & (~GMAC0_USE_PWM01);
			break;
		case PHY_INTERFACE_MODE_MII:
			val = val | GMAC0_USE_TXCLK | GMAC0_USE_PWM01;
			break;
		default:
			pr_err("unsupported mii mode %d\n",
			       plat_dat->interface);
			return -ENOTSUPP;
		}
		__raw_writel(val & (~GMAC0_SHUT), LS1X_MUX_CTRL0);
	}
#elif defined(CONFIG_LOONGSON1_LS1B)
	plat_dat = dev_get_platdata(&pdev->dev);
	if (plat_dat->bus_id) {
		__raw_writel(__raw_readl(LS1X_MUX_CTRL0) | GMAC1_USE_UART1 |
			     GMAC1_USE_UART0, LS1X_MUX_CTRL0);
		switch (plat_dat->interface) {
		case PHY_INTERFACE_MODE_RGMII:
			val &= ~(GMAC1_USE_TXCLK | GMAC1_USE_PWM23);
			break;
		case PHY_INTERFACE_MODE_MII:
			val |= (GMAC1_USE_TXCLK | GMAC1_USE_PWM23);
			break;
		default:
			pr_err("unsupported mii mode %d\n",
			       plat_dat->interface);
			return -ENOTSUPP;
		}
		val &= ~GMAC1_SHUT;
	} else {
		switch (plat_dat->interface) {
		case PHY_INTERFACE_MODE_RGMII:
			val &= ~(GMAC0_USE_TXCLK | GMAC0_USE_PWM01);
			break;
		case PHY_INTERFACE_MODE_MII:
			val |= (GMAC0_USE_TXCLK | GMAC0_USE_PWM01);
			break;
		default:
			pr_err("unsupported mii mode %d\n",
			       plat_dat->interface);
			return -ENOTSUPP;
		}
		val &= ~GMAC0_SHUT;
	}
	__raw_writel(val, LS1X_MUX_CTRL1);

#elif defined(CONFIG_LOONGSON1_LS1C)
	plat_dat = dev_get_platdata(&pdev->dev);

	val &= ~PHY_INTF_SELI;
	#if defined(CONFIG_LS1X_GMAC0_RMII)
	val |= 0x4 << PHY_INTF_SELI_SHIFT;
	#endif
	__raw_writel(val, LS1X_MUX_CTRL1);

	val = __raw_readl(LS1X_MUX_CTRL0);
	__raw_writel(val & (~GMAC_SHUT), LS1X_MUX_CTRL0);

	#if defined(CONFIG_LS1X_GMAC0_RMII)
	__raw_writel(0x400, (void __iomem *)KSEG1ADDR(LS1X_GMAC0_BASE + 0x14));
	#endif
	__raw_writel(0xe4b, (void __iomem *)KSEG1ADDR(LS1X_GMAC0_BASE + 0x10));
#endif

	return 0;
}
#endif

#if defined(CONFIG_LS1X_GMAC0)
static struct plat_stmmacenet_data ls1x_eth0_pdata = {
	.bus_id		= 0,
	.phy_addr	= -1,
#if defined(CONFIG_LOONGSON1_LS1A)
	.interface	= PHY_INTERFACE_MODE_RGMII,
#elif defined(CONFIG_LOONGSON1_LS1B)
	.interface	= PHY_INTERFACE_MODE_MII,
#elif defined(CONFIG_LOONGSON1_LS1C)
	.interface	= PHY_INTERFACE_MODE_RMII,
#endif
	.mdio_bus_data	= &ls1x_mdio_bus_data,
	.dma_cfg	= &ls1x_eth_dma_cfg,
	.has_gmac	= 1,
	.tx_coe		= 1,
	.init		= ls1x_eth_mux_init,
};

static struct resource ls1x_eth0_resources[] = {
	[0] = {
		.start	= LS1X_GMAC0_BASE,
		.end	= LS1X_GMAC0_BASE + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "macirq",
		.start	= LS1X_GMAC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device ls1x_eth0_pdev = {
	.name		= "stmmaceth",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(ls1x_eth0_resources),
	.resource	= ls1x_eth0_resources,
	.dev		= {
		.platform_data = &ls1x_eth0_pdata,
	},
};
#endif

#if defined(CONFIG_LS1X_GMAC1)
static struct plat_stmmacenet_data ls1x_eth1_pdata = {
	.bus_id		= 1,
	.phy_addr	= -1,
#if defined(CONFIG_LOONGSON1_LS1A)
	.interface	= PHY_INTERFACE_MODE_MII,
#elif defined(CONFIG_LOONGSON1_LS1B)
	.interface	= PHY_INTERFACE_MODE_MII,
#endif
	.mdio_bus_data	= &ls1x_mdio_bus_data,
	.dma_cfg	= &ls1x_eth_dma_cfg,
	.has_gmac	= 1,
	.tx_coe		= 1,
	.init		= ls1x_eth_mux_init,
};

static struct resource ls1x_eth1_resources[] = {
	[0] = {
		.start	= LS1X_GMAC1_BASE,
		.end	= LS1X_GMAC1_BASE + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "macirq",
		.start	= LS1X_GMAC1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device ls1x_eth1_pdev = {
	.name		= "stmmaceth",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(ls1x_eth1_resources),
	.resource	= ls1x_eth1_resources,
	.dev		= {
		.platform_data = &ls1x_eth1_pdata,
	},
};
#endif

/* USB OHCI */
#ifdef CONFIG_USB_OHCI_HCD_PLATFORM
#include <linux/usb/ohci_pdriver.h>
static u64 ls1x_ohci_dmamask = DMA_BIT_MASK(32);

static struct resource ls1x_ohci_resources[] = {
	[0] = {
		.start	= LS1X_OHCI_BASE,
		.end	= LS1X_OHCI_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= LS1X_OHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct usb_ohci_pdata ls1x_ohci_pdata = {
};

struct platform_device ls1x_ohci_pdev = {
	.name		= "ohci-platform",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ls1x_ohci_resources),
	.resource	= ls1x_ohci_resources,
	.dev		= {
		.dma_mask = &ls1x_ohci_dmamask,
		.platform_data = &ls1x_ohci_pdata,
	},
};
#endif

/* USB EHCI */
#ifdef CONFIG_USB_EHCI_HCD_PLATFORM
#include <linux/usb/ehci_pdriver.h>
static u64 ls1x_ehci_dmamask = DMA_BIT_MASK(32);

static struct resource ls1x_ehci_resources[] = {
	[0] = {
		.start	= LS1X_EHCI_BASE,
		.end	= LS1X_EHCI_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= LS1X_EHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct usb_ehci_pdata ls1x_ehci_pdata = {
};

struct platform_device ls1x_ehci_pdev = {
	.name		= "ehci-platform",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ls1x_ehci_resources),
	.resource	= ls1x_ehci_resources,
	.dev		= {
		.dma_mask = &ls1x_ehci_dmamask,
		.platform_data = &ls1x_ehci_pdata,
	},
};
#endif

/* Real Time Clock */
struct platform_device ls1x_rtc_pdev = {
	.name		= "ls1x-rtc",
	.id		= -1,
};

#ifdef CONFIG_MTD_NAND_LS1X
#include <ls1x_nand.h>
extern struct ls1x_nand_platform_data ls1x_nand_parts;
static struct resource ls1x_nand_resources[] = {
	[0] = {
		.start	= LS1X_NAND_BASE,
		.end	= LS1X_NAND_BASE + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= LS1X_DMA0_IRQ,
        .end	= LS1X_DMA0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device ls1x_nand_pdev = {
	.name	= "ls1x-nand",
	.id		= -1,
	.dev	= {
		.platform_data = &ls1x_nand_parts,
	},
	.num_resources	= ARRAY_SIZE(ls1x_nand_resources),
	.resource		= ls1x_nand_resources,
};
#endif //CONFIG_MTD_NAND_LS1X

#ifdef CONFIG_SPI_LS1X_SPI0
#include <linux/spi/spi_ls1x.h>
static struct resource ls1x_spi0_resource[] = {
	[0] = {
		.start	= LS1X_SPI0_BASE,
		.end	= LS1X_SPI0_BASE + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
#if defined(CONFIG_SPI_IRQ_MODE)
	[1] = {
		.start	= LS1X_SPI0_IRQ,
		.end	= LS1X_SPI0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#endif
};

#ifdef CONFIG_SPI_CS_USED_GPIO
static int spi0_gpios_cs[] =
	{ 27, 28, 29, 30 };
#endif

static struct ls1x_spi_platform_data ls1x_spi0_platdata = {
#ifdef CONFIG_SPI_CS_USED_GPIO
	.gpio_cs_count = ARRAY_SIZE(spi0_gpios_cs),
	.gpio_cs = spi0_gpios_cs,
#elif CONFIG_SPI_CS
	.cs_count = SPI0_CS3 + 1,
#endif
};

struct platform_device ls1x_spi0_pdev = {
	.name		= "spi_ls1x",
	.id 		= 0,
	.num_resources	= ARRAY_SIZE(ls1x_spi0_resource),
	.resource	= ls1x_spi0_resource,
	.dev		= {
		.platform_data	= &ls1x_spi0_platdata,
	},
};
#endif

#ifdef CONFIG_SPI_LS1X_SPI1
#include <linux/spi/spi_ls1x.h>
static struct resource ls1x_spi1_resource[] = {
	[0] = {
		.start	= LS1X_SPI1_BASE,
		.end	= LS1X_SPI1_BASE + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
#if defined(CONFIG_SPI_IRQ_MODE)
	[1] = {
		.start	= LS1X_SPI1_IRQ,
		.end	= LS1X_SPI1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#endif
};

#ifdef CONFIG_SPI_CS_USED_GPIO
static int spi1_gpios_cs[] =
#if defined(CONFIG_LOONGSON1_LS1A)
	{ 47, 69, 68 };
#elif defined(CONFIG_LOONGSON1_LS1B)
	{ 38, 0, 1 };
#endif
#endif

static struct ls1x_spi_platform_data ls1x_spi1_platdata = {
#ifdef CONFIG_SPI_CS_USED_GPIO
	.gpio_cs_count = ARRAY_SIZE(spi1_gpios_cs),
	.gpio_cs = spi1_gpios_cs,
#elif CONFIG_SPI_CS
	.cs_count = SPI1_CS2 + 1,
#endif
};

struct platform_device ls1x_spi1_pdev = {
	.name		= "spi_ls1x",
	.id 		= 1,
	.num_resources	= ARRAY_SIZE(ls1x_spi1_resource),
	.resource	= ls1x_spi1_resource,
	.dev		= {
		.platform_data	= &ls1x_spi1_platdata,
	},
};
#endif
