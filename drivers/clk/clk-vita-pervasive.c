// SPDX-License-Identifier: GPL-2.0
/*
 * PS Vita pervasive clock gates.
 *
 * The pervasive block at 0xE3102000 is a flat array of 32-bit registers, one
 * per device, mirroring the reset block at 0xE3101000 (same register index).
 * Setting a bit enables that device's clock; clearing it disables the clock.
 *
 * Register index is offset/4, matching the reset controller's id convention
 * (gpio0 = 0x100/4 = 64, and so on).  Some devices span several bits, so each
 * gate carries its own mask.
 *
 * The three USB buses live at 0x090/0x094/0x098 (indices 36/37/38).  The USB
 * controller at 0xE4020000 is on bus 2 and needs the full 0xF mask -- SceUdcd
 * drives only bits 1 and 3, the other two belong to the host/PHY side, and the
 * block stays dead unless all four are set.
 *
 * USB host-mode integration (2026-08-19, lab/usb-re/HOSTMODE-2026-08-19.md):
 * VitaOS hands Linux the pervasive USB mode flag (0xE3100084 + bus*4) latched
 * to 1 = device mode.  While it stays 1 the EHCI register file decodes but
 * the host core domain stays dark (FRINDEX frozen at 0 forever).  Sony's
 * usbserv host activation (sub_81000E64, loc_81000E78) latches the flag to
 * host while the USB clock is parked, between clock-off and clock-on -- i.e.
 * exactly clk .prepare semantics:
 *
 *   reset[0xE3101090 + bus*4] |= 0xB        (assert holds)
 *   clk  [0xE3102090 + bus*4] &= ~0xB       (park; Sony reaches 0)
 *   flag [0xE3100084 + bus*4]  = 0          (HOST mode select, latched parked)
 *   clk  [0xE3102090 + bus*4] |= 9          (host clocks)
 *   reset[0xE3101090 + bus*4] &= ~9         (release; leaves 0x2)
 *   poll PHY ready (0xE3110F30 bit 2), 5 s budget
 *
 * This was proven on silicon by the cycle-10 userspace repro (hostmode.c):
 * afterwards FRINDEX advances, the +0x200 OHCI window decodes, and the PSTV's
 * internal Ethernet NIC enumerates into a working eth0.
 *
 * Bus-uniform (2026-08-19 cycle 12, lab/usb-re/HOSTBUS-2026-08-19.md):
 * usbserv carries per-bus near-clones of the host choreography (0x81001150
 * bus0, 0x810011f6 bus1, 0x81001b4e bus2) with identical constants, and the
 * SceUdcd cable dispatcher (0x81005b0e) decodes the PHY page as one shared
 * window at 0xE3110000: [F34] is a one-hot port-select strobe and [F30] bit
 * (1 << bus) is that port's ready bit -- there is no separate bus-0/1 PHY
 * page.  Proven on bus 1 by the cycle-12 userspace repro (hostbus.c 1):
 * end-state gate=9/reset=0x2/flag=0, PHY ready 0 ms, FRINDEX advancing,
 * OHCI +0x200 decoding.
 *
 * All-bus latch wedged the box at cycle 12b (dark, no SSH; buses 0+1+2
 * together at kernel-early time recreates the 08-17 trap).  The mask is
 * therefore opt-in per bus on the kernel command line, one bus at a time:
 *   vita_pervasive.buses=0x4   bus 2 only (default, 11b-proven safe)
 *   vita_pervasive.buses=0x6   bus 2 + bus 1 (cycle 12 userspace-proven)
 *   vita_pervasive.buses=0x7   all buses (wedged 2026-08-19, do not use)
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/vita-pervasive.h>

struct vita_pervasive_gate_def {
	const char *name;
	u16 idx;	/* register index (offset / 4) */
	u32 mask;
	bool hostmode;	/* carry the Sony host-mode latch in .prepare */
};

/*
 * Buses to carry the host latch, opt-in: bit N = bus N.  Default 0x4 (bus 2
 * only, cycle-11b-proven).  One new bus at a time on the cmdline; 0x7 wedged
 * the box (cycle 12b, 08-17 buses-together trap).
 */
static unsigned int buses_mask = 0x4;
module_param(buses_mask, uint, 0444);
MODULE_PARM_DESC(buses_mask, "USB buses carrying host-mode latch (bit N = bus N; default 0x4 = bus 2)");

static const struct vita_pervasive_gate_def vita_pervasive_gates[VITA_PCLK_NR] = {
	[VITA_PCLK_USB0] = { "usb0", 0x090 / 4, 0xf, true },
	[VITA_PCLK_USB1] = { "usb1", 0x094 / 4, 0xf, true },
	[VITA_PCLK_USB2] = { "usb2", 0x098 / 4, 0xf, true },
};

struct vita_pervasive_clk {
	struct clk_hw hw;
	void __iomem *reg;		/* clock gate register */
	void __iomem *reset_reg;	/* reset control register (hostmode) */
	void __iomem *flag_reg;		/* pervasive mode flag (hostmode) */
	void __iomem *phy_reg;		/* PHY ready register (hostmode) */
	void __iomem *vbus_reg;		/* GPIO1 block, VBUS drive (bus 1 only) */
	u32 mask;
	u32 phy_bit;		/* per-port ready bit in F30 (hostmode) */
	spinlock_t *lock;
};

#define to_vita_pervasive_clk(_hw) \
	container_of(_hw, struct vita_pervasive_clk, hw)

static int vita_pervasive_clk_prepare(struct clk_hw *hw)
{
	struct vita_pervasive_clk *gate = to_vita_pervasive_clk(hw);
	u32 reset_v, gate_v, flag_v;
	int i;

	if (!gate->reset_reg || !gate->flag_reg || !gate->phy_reg)
		return 0;	/* plain gate: nothing to latch */

	reset_v = readl(gate->reset_reg);
	gate_v = readl(gate->reg);
	flag_v = readl(gate->flag_reg);

	pr_info("vita pervasive %s prepare: reset=%#x gate=%#x mode_flag=%#x\n",
		clk_hw_get_name(hw), reset_v, gate_v, flag_v);

	if (!flag_v)
		return 0;	/* already host mode (warm path) */

	/* Sony usbserv host activation, park-window choreography. */
	writel(reset_v | 0xB, gate->reset_reg);		/* assert holds */
	msleep(100);
	writel(gate_v & ~0xB, gate->reg);		/* park */
	msleep(100);
	writel(0, gate->flag_reg);			/* HOST MODE SELECT */
	msleep(100);
	writel((gate_v & ~0xB) | 9, gate->reg);		/* host clocks */
	msleep(100);
	writel((reset_v | 0xB) & ~9, gate->reset_reg);	/* release, 0x2 */

	/* PHY ready: bit (1 << bus), 5 s budget (Sony polls up to 2000 tries). */
	for (i = 0; i < 250; i++) {
		if (readl(gate->phy_reg) & gate->phy_bit)
			break;
		msleep(20);
	}

	pr_info("vita pervasive %s: host mode latched, PHY %s after %d ms (F30=%#x)\n",
		clk_hw_get_name(hw),
		(readl(gate->phy_reg) & gate->phy_bit) ? "ready" : "NOT ready",
		i * 20, readl(gate->phy_reg));

	/*
	 * VBUS (bus 1 only, the Type-A port): Sony's host-start tail does
	 * ksceGpioPortSet(1, 3) after the core comes up.  Lowio disasm
	 * (ksceGpioPortSet @ 0x81002150): port 1 = SceGpio1Reg block at
	 * 0xE0100000, SET register +0x08, write (1 << pin).  Pin mode is
	 * configured once by usbserv module init (ksceGpioSetPortMode(1,3,0)
	 * @ 0x81001120) and is not re-touched here, matching Sony's sequence.
	 * The PSTV is a GPIO1-poor environment: this driver is the only user
	 * of the block on the PSTV bench, so the direct write is safe for the
	 * proof cycle; the eventual proper home is a gpio1@e0100000 DT node
	 * with a gpio-hog, consumed by the ehci1 node.
	 */
	if (gate->vbus_reg) {
		writel(BIT(3), gate->vbus_reg + 0x08);
		pr_info("vita pervasive %s: VBUS driven (GPIO1 pin3 SET, E0100008 <= 0x8)\n",
			clk_hw_get_name(hw));
	}

	return 0;
}

static int vita_pervasive_clk_enable(struct clk_hw *hw)
{
	struct vita_pervasive_clk *gate = to_vita_pervasive_clk(hw);
	unsigned long flags;

	spin_lock_irqsave(gate->lock, flags);
	writel(readl(gate->reg) | gate->mask, gate->reg);
	spin_unlock_irqrestore(gate->lock, flags);

	return 0;
}

static void vita_pervasive_clk_disable(struct clk_hw *hw)
{
	struct vita_pervasive_clk *gate = to_vita_pervasive_clk(hw);
	unsigned long flags;

	spin_lock_irqsave(gate->lock, flags);
	writel(readl(gate->reg) & ~gate->mask, gate->reg);
	spin_unlock_irqrestore(gate->lock, flags);
}

static int vita_pervasive_clk_is_enabled(struct clk_hw *hw)
{
	struct vita_pervasive_clk *gate = to_vita_pervasive_clk(hw);

	return (readl(gate->reg) & gate->mask) == gate->mask;
}

static const struct clk_ops vita_pervasive_clk_ops = {
	.prepare	= vita_pervasive_clk_prepare,
	.enable		= vita_pervasive_clk_enable,
	.disable	= vita_pervasive_clk_disable,
	.is_enabled	= vita_pervasive_clk_is_enabled,
};

struct vita_pervasive {
	void __iomem *flags_base;	/* 0xE3100000 */
	void __iomem *gate_base;	/* 0xE3102000 */
	void __iomem *phy_base;		/* 0xE3110000 */
	void __iomem *reset_base;	/* 0xE3101000 (unclaimed; reset-vita owns) */
	spinlock_t lock;
	struct vita_pervasive_clk gates[VITA_PCLK_NR];
	struct clk_hw_onecell_data *onecell;
};

static int vita_pervasive_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_pervasive *priv;
	struct resource *res;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->flags_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->flags_base))
		return PTR_ERR(priv->flags_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->gate_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->gate_base))
		return PTR_ERR(priv->gate_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	priv->phy_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->phy_base))
		return PTR_ERR(priv->phy_base);

	/* Reset page: claimed by reset-vita, so map without claiming. */
	priv->reset_base = devm_ioremap(dev, 0xE3101000, 0x1000);
	if (!priv->reset_base)
		return -ENOMEM;

	/*
	 * GPIO1 (SceGpio1Reg @ 0xE0100000, from lowio ksceGpioPortSet
	 * disasm): only the USB1 gate gets it, for the VBUS drive above.
	 * Mapped without claiming -- no DT node exists for the block yet.
	 */
	priv->gates[VITA_PCLK_USB1].vbus_reg = devm_ioremap(dev, 0xE0100000, 0x1000);
	if (IS_ERR(priv->gates[VITA_PCLK_USB1].vbus_reg))
		return PTR_ERR(priv->gates[VITA_PCLK_USB1].vbus_reg);
	if (!priv->gates[VITA_PCLK_USB1].vbus_reg)
		return -ENOMEM;

	priv->onecell = devm_kzalloc(dev,
				     struct_size(priv->onecell, hws, VITA_PCLK_NR),
				     GFP_KERNEL);
	if (!priv->onecell)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	priv->onecell->num = VITA_PCLK_NR;

	for (i = 0; i < VITA_PCLK_NR; i++) {
		const struct vita_pervasive_gate_def *def = &vita_pervasive_gates[i];
		struct clk_init_data init = {};

		init.name = def->name;
		init.ops = &vita_pervasive_clk_ops;
		init.flags = 0;
		init.num_parents = 0;

		priv->gates[i].reg = priv->gate_base + def->idx * 4;
		priv->gates[i].mask = def->mask;
		priv->gates[i].lock = &priv->lock;
		if (def->hostmode && (buses_mask & (1 << i))) {
			priv->gates[i].reset_reg = priv->reset_base + 0x090 + i * 4;
			priv->gates[i].flag_reg = priv->flags_base + 0x084 + i * 4;
			priv->gates[i].phy_reg = priv->phy_base + 0xF30;
			priv->gates[i].phy_bit = 1 << i;
		}
		priv->gates[i].hw.init = &init;

		ret = devm_clk_hw_register(dev, &priv->gates[i].hw);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to register clock %s\n",
					     def->name);

		priv->onecell->hws[i] = &priv->gates[i].hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   priv->onecell);
}

static const struct of_device_id vita_pervasive_of_match[] = {
	{ .compatible = "vita,pervasive-clk" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vita_pervasive_of_match);

static struct platform_driver vita_pervasive_driver = {
	.probe = vita_pervasive_probe,
	.driver = {
		.name = "vita-pervasive-clk",
		.of_match_table = vita_pervasive_of_match,
	},
};
module_platform_driver(vita_pervasive_driver);

MODULE_AUTHOR("vita-linux-r0-clean contributors");
MODULE_DESCRIPTION("PS Vita pervasive clock gates with USB host-mode latch");
MODULE_LICENSE("GPL");
