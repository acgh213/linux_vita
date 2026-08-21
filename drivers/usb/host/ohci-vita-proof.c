// SPDX-License-Identifier: GPL-2.0
/*
 * PS Vita OHCI safe-first-touch diagnostic.
 *
 * This is lab-only scaffolding.  It proves that the OHCI companion can be
 * accessed after its paired EHCI controller has completed probe and is
 * actively advancing frames.  It deliberately performs no OHCI writes and
 * does not register a host controller.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#define VITA_EHCI_FRINDEX	0x1c
#define VITA_OHCI_REVISION	0x00
#define VITA_OHCI_REV_10	0x10

static int vita_ohci_proof_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *ehci_pdev;
	struct device_node *ehci_np;
	struct usb_hcd *ehci_hcd;
	void __iomem *regs;
	u32 frame_before, frame_after, revision;
	int ret = 0;

	ehci_np = of_parse_phandle(dev->of_node, "sony,ehci", 0);
	if (!ehci_np)
		return dev_err_probe(dev, -EINVAL, "missing sony,ehci phandle\n");

	ehci_pdev = of_find_device_by_node(ehci_np);
	of_node_put(ehci_np);
	if (!ehci_pdev)
		return -EPROBE_DEFER;

	device_lock(&ehci_pdev->dev);
	if (!device_is_bound(&ehci_pdev->dev)) {
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	ehci_hcd = platform_get_drvdata(ehci_pdev);
	if (!ehci_hcd || !HC_IS_RUNNING(ehci_hcd->state)) {
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	frame_before = readl(ehci_hcd->regs + VITA_EHCI_FRINDEX);
	usleep_range(1000, 1500);
	frame_after = readl(ehci_hcd->regs + VITA_EHCI_FRINDEX);
	if (frame_before == frame_after) {
		ret = -EPROBE_DEFER;
		goto out_unlock;
	}

	dev_info(dev, "paired EHCI running, FRINDEX %#x -> %#x\n",
		 frame_before, frame_after);

out_unlock:
	device_unlock(&ehci_pdev->dev);
	put_device(&ehci_pdev->dev);
	if (ret)
		return ret;

	/* Sony SceUsbd waits 100 ms before its first OHCI register access. */
	msleep(100);

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	revision = readl(regs + VITA_OHCI_REVISION);
	if ((revision & 0xff) != VITA_OHCI_REV_10)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected HcRevision %#x\n", revision);

	dev_info(dev, "OHCI safe-first-touch green: HcRevision=%#x; no writes issued\n",
		 revision);
	return 0;
}

static const struct of_device_id vita_ohci_proof_of_match[] = {
	{ .compatible = "sony,vita-ohci-proof" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_ohci_proof_of_match);

static struct platform_driver vita_ohci_proof_driver = {
	.probe = vita_ohci_proof_probe,
	.driver = {
		.name = "vita-ohci-proof",
		.of_match_table = vita_ohci_proof_of_match,
	},
};
module_platform_driver(vita_ohci_proof_driver);

MODULE_DESCRIPTION("PS Vita OHCI safe-first-touch diagnostic");
MODULE_LICENSE("GPL");
