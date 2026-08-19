// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/types.h>
#include <linux/of_device.h>

/*
 * Reset ids pack an optional bit-mask into the upper 16 bits:
 *
 *   id = register_index | (mask << 16)
 *
 * A plain index (mask == 0) means "bit 0", which keeps every existing
 * consumer (gpio0=64, spi0=65, i2c0/1=68/69, ...) working unchanged.
 * Some devices occupy several bits of one register -- e.g. USB bus 2 at
 * register index 38 (offset 0x98) uses mask 0xF.
 */
#define VITA_RESET_IDX(id)	((id) & 0xffff)
#define VITA_RESET_MASK(id)	((u32)((id) >> 16))

struct vita_reset {
	void __iomem *reg_base;
	unsigned int nr_regs;
	struct reset_controller_dev rcdev;
	spinlock_t lock;
};

static int vita_reset_level(struct reset_controller_dev *rcdev,
			    unsigned long id, bool assert)
{
	struct vita_reset *vreset =
		container_of(rcdev, struct vita_reset, rcdev);
	unsigned long idx = VITA_RESET_IDX(id);
	u32 mask = VITA_RESET_MASK(id);
	void __iomem *reg_addr = vreset->reg_base + idx * 4;
	unsigned long flags;
	u32 val;

	if (!mask)
		mask = 1;

	if (idx >= vreset->nr_regs)
		return -EINVAL;

	pr_info("%s: idx: %lu, mask: %#x, assert: %d\n",
		__func__, idx, mask, assert);

	spin_lock_irqsave(&vreset->lock, flags);

	val = readl(reg_addr);
	if (assert)
		writel(val | mask, reg_addr);
	else
		writel(val & ~mask, reg_addr);

	spin_unlock_irqrestore(&vreset->lock, flags);

	return 0;
}

static int vita_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	return vita_reset_level(rcdev, id, true);
}

static int vita_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return vita_reset_level(rcdev, id, false);
}

static const struct reset_control_ops vita_reset_ops = {
	.assert		= vita_reset_assert,
	.deassert	= vita_reset_deassert,
};

static int vita_reset_probe(struct platform_device *pdev)
{
	struct vita_reset *data;
	struct resource *res;

	dev_info(&pdev->dev, "probe\n");

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->reg_base))
		return PTR_ERR(data->reg_base);

	platform_set_drvdata(pdev, data);

	spin_lock_init(&data->lock);

	data->rcdev.owner = THIS_MODULE;
	data->nr_regs = resource_size(res) / 4;
	/* ids carry a mask in the upper bits, so the framework's bounds check
	 * cannot be the register count; vita_reset_level() validates the
	 * index.
	 */
	data->rcdev.nr_resets = 0xffffff;
	data->rcdev.ops = &vita_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;

	return devm_reset_controller_register(&pdev->dev, &data->rcdev);
}

static const struct of_device_id vita_reset_of_match[] = {
	{ .compatible = "vita,reset" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vita_reset_of_match);

static struct platform_driver vita_reset_driver = {
	.probe	= vita_reset_probe,
	.driver = {
		.name		= "vita-reset",
		.of_match_table	= vita_reset_of_match,
	},
};
module_platform_driver(vita_reset_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita reset driver");
MODULE_LICENSE("GPL");

