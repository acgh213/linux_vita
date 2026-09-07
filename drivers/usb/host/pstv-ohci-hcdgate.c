// SPDX-License-Identifier: GPL-2.0
/*
 * Opt-in post-boot OHCI HCD bring-up for the PSTV bus-zero companion.
 * Called by the diagnostic gate only after EHCI is bound, running, and
 * FRINDEX-advancing, with the OHCI window already mapped. No probe-time MMIO.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#ifdef __KERNEL__
#include "ohci.h"
#else
/* Host test build: ohci_init_driver/ohci_setup doubles come from the shim. */
#endif
#include "pstv-ohci-gate-core.h"
#include "pstv-ohci-hcdgate.h"

#define HCDGATE_BASE 0xe40e0200
#define HCDGATE_SIZE 0x100

static struct hc_driver gate_hc_driver;
static bool hcd_active;
static struct usb_hcd *active_hcd;
static struct usb_hcd *saved_ehci_hcd;
static struct device *saved_drvdata_dev;

/*
 * Sony's order differs from generic ohci_setup(): interrupts are masked and
 * HCR is asserted BEFORE Control is zeroed. Do that pre-reset first, then let
 * the generic OHCI core take over.
 */
static int gate_hcd_reset(struct usb_hcd *hcd)
{
	writel_relaxed(PSTV_OHCI_IRQ_MASK, hcd->regs + PSTV_OHCI_INTR_DISABLE);
	writel_relaxed(PSTV_OHCI_HCR, hcd->regs + PSTV_OHCI_CMD_STATUS);
	writel_relaxed(0, hcd->regs + PSTV_OHCI_CONTROL);
	return ohci_setup(hcd);
}

static const struct ohci_driver_overrides gate_overrides = {
	.product_desc = "PSTV OHCI (gated)",
	.reset = gate_hcd_reset,
};

/*
 * gate_hcd_up - register a real OHCI HCD on the proven gate window.
 * Caller must hold the EHCI device lock, root-hub lock, and PM hold, and
 * @pdev: EHCI platform device whose hardware owns the companion window
 * @regs: mapped OHCI companion register window
 * @irq: validated GIC virtual IRQ for the OHCI companion
 * @old_hcd: original EHCI HCD drvdata to restore after handoff
 *
 * must have the OHCI registers mapped and the IRQ mapped/requested.
 */
int gate_hcd_up(struct platform_device *pdev, void __iomem *regs,
		unsigned int irq, struct usb_hcd *old_hcd)
{
	struct usb_hcd *hcd;
	int ret;

	if (hcd_active)
		return -EBUSY;
	if (!old_hcd || !pdev)
		return -EINVAL;
	ohci_init_driver(&gate_hc_driver, &gate_overrides);
	saved_drvdata_dev = &pdev->dev;
	saved_ehci_hcd = old_hcd;
	hcd = usb_create_hcd(&gate_hc_driver, &pdev->dev, "pstv-ohci-hcd");
	if (!hcd) {
		saved_drvdata_dev = NULL;
		saved_ehci_hcd = NULL;
		return -ENOMEM;
	}
	/* usb_create_hcd() temporarily installs OHCI drvdata on this device. */
	dev_set_drvdata(saved_drvdata_dev, saved_ehci_hcd);
	hcd->rsrc_start = HCDGATE_BASE;
	hcd->rsrc_len = HCDGATE_SIZE;
	hcd->skip_phy_initialization = 1;
	hcd->speed = HCD_USB11;
	hcd->regs = regs;
	ret = usb_add_hcd(hcd, irq, 0);
	if (ret) {
		dev_set_drvdata(saved_drvdata_dev, saved_ehci_hcd);
		saved_drvdata_dev = NULL;
		saved_ehci_hcd = NULL;
		usb_put_hcd(hcd);
		return ret;
	}
	dev_set_drvdata(saved_drvdata_dev, saved_ehci_hcd);
	active_hcd = hcd;
	hcd_active = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gate_hcd_up);

int gate_hcd_set_companion(struct usb_hcd *ehci)
{
	if (!active_hcd || !ehci || !active_hcd->self.root_hub ||
	    !ehci->self.root_hub)
		return -EINVAL;
	active_hcd->self.hs_companion = &ehci->self;
	return 0;
}
EXPORT_SYMBOL_GPL(gate_hcd_set_companion);

void gate_hcd_down(void)
{
	if (!active_hcd)
		return;
	usb_remove_hcd(active_hcd);
	usb_put_hcd(active_hcd);
	if (saved_drvdata_dev)
		dev_set_drvdata(saved_drvdata_dev, saved_ehci_hcd);
	saved_drvdata_dev = NULL;
	saved_ehci_hcd = NULL;
	active_hcd = NULL;
	hcd_active = false;
}
EXPORT_SYMBOL_GPL(gate_hcd_down);

bool gate_hcd_active(void)
{
	return hcd_active;
}
EXPORT_SYMBOL_GPL(gate_hcd_active);
