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
 * must have the OHCI registers mapped and the IRQ mapped/requested.
 */
int gate_hcd_up(struct platform_device *pdev, void __iomem *regs,
		unsigned int irq)
{
	struct usb_hcd *hcd;
	int ret;

	if (hcd_active)
		return -EBUSY;
	ohci_init_driver(&gate_hc_driver, &gate_overrides);
	hcd = usb_create_hcd(&gate_hc_driver, &pdev->dev, "pstv-ohci-hcd");
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = HCDGATE_BASE;
	hcd->rsrc_len = HCDGATE_SIZE;
	hcd->regs = regs;
	ret = usb_add_hcd(hcd, irq, 0);
	if (ret) {
		usb_put_hcd(hcd);
		return ret;
	}
	hcd_active = true;
	return 0;
}
EXPORT_SYMBOL_GPL(gate_hcd_up);

bool gate_hcd_active(void)
{
	return hcd_active;
}
EXPORT_SYMBOL_GPL(gate_hcd_active);
