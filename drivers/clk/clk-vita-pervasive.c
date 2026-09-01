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
 *   poll PHY ready (0xE3110F30 bit bus), 5 s budget
 *
 * This was proven on silicon for bus 2 by the cycle-10 userspace repro
 * (hostmode.c): afterwards FRINDEX advances, the +0x200 OHCI window decodes,
 * and the PSTV's internal Ethernet NIC enumerates into a working eth0.
 * Offline usbserv/SceUdcd analysis then showed that all ports share the PHY
 * page at 0xE3110000: +0xF30 uses bit N for bus N.  Cycle 13c proved bus 1
 * kernel-early host latching safe and its EHCI core advancing; an idle root
 * hub merely runtime-suspends and freezes FRINDEX until resumed.
 *
 * Keep bus 2 as the default.  Boards opt into additional buses through the
 * clock provider's Device Tree node, keeping the selection reviewable and
 * independent of a mutable kernel command-line experiment.  The all-bus 0x7
 * case remains unvalidated and is rejected.
 *
 * The non-USB indices below are intentionally registration-only.  No DT
 * consumer is added here, and no newly exposed clock is enabled by default.
 * The static register resolution identifies DSI0/1, GPIO, SPI0, UART0 and
 * MSIF, but does not establish instance counts for the indexed SPI/UART/audio
 * families.  Only the instances already named by the in-tree Vita DT/driver
 * set are exposed; audio remains deferred until its masks and semantic names
 * are recovered.
 *
 * Boards explicitly list available non-USB indices in
 * vita,clock-gate-indices.  This prevents a future consumer from enabling the
 * handheld DSI0 gate on PSTV, where bus 0 is known to be inaccessible.  USB
 * IDs remain registered unconditionally to preserve the existing DT ABI.
 */

#include <linux/bits.h>
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

/* Bit N selects Sony USB bus N for the host-mode park-window sequence. */
#define VITA_USB_HOSTMODE_MASK_DEFAULT	BIT(2)
#define VITA_USB_HOSTMODE_MASK_ALL	GENMASK(2, 0)

static const struct vita_pervasive_gate_def vita_pervasive_gates[VITA_PCLK_NR] = {
	[VITA_PCLK_USB0] = { "usb0", 0x090 / 4, 0xf, true },
	[VITA_PCLK_USB1] = { "usb1", 0x094 / 4, 0xf, true },
	[VITA_PCLK_USB2] = { "usb2", 0x098 / 4, 0xf, true },
	[VITA_PCLK_DSI0] = { "dsi0", 0x080 / 4, 0xf, false },
	[VITA_PCLK_DSI1] = { "dsi1", 0x084 / 4, 0xf, false },
	[VITA_PCLK_GPIO] = { "gpio", 0x100 / 4, 1, false },
	[VITA_PCLK_SPI0] = { "spi0", 0x104 / 4, 1, false },
	[VITA_PCLK_UART0] = { "uart0", 0x120 / 4, 1, false },
	[VITA_PCLK_MSIF] = { "msif", 0x0b0 / 4, 1, false },
};

/* Preserve the existing USB DT ABI while making omissions fail at build time. */
static_assert(VITA_PCLK_USB0 == 0);
static_assert(VITA_PCLK_USB1 == 1);
static_assert(VITA_PCLK_USB2 == 2);
static_assert(VITA_PCLK_NR <= 32);

struct vita_pervasive_clk {
	struct clk_hw hw;
	void __iomem *reg;		/* clock gate register */
	void __iomem *reset_reg;	/* reset control register (hostmode) */
	void __iomem *flag_reg;		/* pervasive mode flag (hostmode) */
	void __iomem *phy_reg;		/* PHY ready register (hostmode) */
	u32 phy_bit;			/* ready bit in shared PHY register */
	u32 mask;
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

	/* PHY ready: bit N for bus N, 5 s budget. */
	for (i = 0; i < 250; i++) {
		if (readl(gate->phy_reg) & gate->phy_bit)
			break;
		msleep(20);
	}

	pr_info("vita pervasive %s: host mode latched, PHY %s after %d ms (F30=%#x)\n",
		clk_hw_get_name(hw),
		(readl(gate->phy_reg) & gate->phy_bit) ? "ready" : "NOT ready",
		i * 20, readl(gate->phy_reg));

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

static int vita_pervasive_parse_hostmode_mask(struct device *dev, u32 *mask)
{
	u32 value;
	int ret;

	*mask = VITA_USB_HOSTMODE_MASK_DEFAULT;
	ret = of_property_read_u32(dev->of_node, "vita,usb-hostmode-mask",
				   &value);
	if (ret == -EINVAL)
		return 0;
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read USB host-mode mask\n");
	if (value & ~VITA_USB_HOSTMODE_MASK_ALL)
		return dev_err_probe(dev, -EINVAL,
				     "USB host-mode mask %#x selects an unknown bus\n",
				     value);
	if (value == VITA_USB_HOSTMODE_MASK_ALL)
		return dev_err_probe(dev, -EINVAL,
				     "USB host-mode mask %#x enables unvalidated buses\n",
				     value);

	*mask = value;
	return 0;
}

static int vita_pervasive_parse_gate_mask(struct device *dev, u32 *mask)
{
	int count, i, ret;
	u32 id;

	/* Existing USB indices are always available for ABI compatibility. */
	*mask = GENMASK(VITA_PCLK_USB2, VITA_PCLK_USB0);
	if (!of_property_present(dev->of_node, "vita,clock-gate-indices"))
		return 0;

	count = of_property_count_u32_elems(dev->of_node,
					    "vita,clock-gate-indices");
	if (count < 0)
		return dev_err_probe(dev, count,
				     "failed to count clock-gate indices\n");
	if (!count)
		return dev_err_probe(dev, -EINVAL,
				     "clock-gate indices list is empty\n");
	if (count > VITA_PCLK_NR - VITA_PCLK_USB2 - 1)
		return dev_err_probe(dev, -EINVAL,
				     "too many non-USB clock-gate indices: %d\n",
				     count);

	for (i = 0; i < count; i++) {
		ret = of_property_read_u32_index(dev->of_node,
						 "vita,clock-gate-indices", i,
						 &id);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to read clock-gate index %d\n", i);
		if (id <= VITA_PCLK_USB2 || id >= VITA_PCLK_NR)
			return dev_err_probe(dev, -EINVAL,
					     "invalid non-USB clock-gate index %u\n",
					     id);
		if (*mask & BIT(id))
			return dev_err_probe(dev, -EINVAL,
					     "duplicate non-USB clock-gate index %u\n",
					     id);
		*mask |= BIT(id);
	}

	return 0;
}

static int vita_pervasive_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_pervasive *priv;
	struct resource *res;
	u32 hostmode_mask;
	u32 gate_mask;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = vita_pervasive_parse_hostmode_mask(dev, &hostmode_mask);
	if (ret)
		return ret;
	ret = vita_pervasive_parse_gate_mask(dev, &gate_mask);
	if (ret)
		return ret;

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

		if (!(gate_mask & BIT(i))) {
			priv->onecell->hws[i] = ERR_PTR(-ENOENT);
			continue;
		}

		init.name = def->name;
		init.ops = &vita_pervasive_clk_ops;
		/*
		 * Non-USB gates are registration-only until consumers exist.
		 * Preserve firmware-owned state during clk_disable_unused().
		 */
		init.flags = i > VITA_PCLK_USB2 ? CLK_IGNORE_UNUSED : 0;
		init.num_parents = 0;

		priv->gates[i].reg = priv->gate_base + def->idx * 4;
		priv->gates[i].mask = def->mask;
		priv->gates[i].lock = &priv->lock;
		if (def->hostmode && (hostmode_mask & BIT(i))) {
			priv->gates[i].reset_reg = priv->reset_base + 0x090 + i * 4;
			priv->gates[i].flag_reg = priv->flags_base + 0x084 + i * 4;
			priv->gates[i].phy_reg = priv->phy_base + 0xF30;
			priv->gates[i].phy_bit = 1U << i;
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
