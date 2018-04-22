#include <linux/io.h>

#include <loongson1.h>
#include <platform.h>

void __init ls1x_pin_mux(void)
{
	void __iomem *pin_base __maybe_unused;
	int reg __maybe_unused;
#if defined(CONFIG_LOONGSON1_LS1A)
#elif defined(CONFIG_LOONGSON1_LS1B)
	pin_base = ioremap_nocache(LS1X_GPIO0_BASE, 0x0f);
	#ifdef CONFIG_SND_LS1X_SOC_AC97
	reg = __raw_readl(pin_base + 4);
	reg &= ~(0xf << 2);
	__raw_writel(reg, pin_base + 4);
	#endif
	iounmap(pin_base);
#elif defined(CONFIG_LOONGSON1_LS1C)
#endif
}
