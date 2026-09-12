// SPDX-License-Identifier: GPL-2.0
/*
 * PlayStation TV bus-zero OHCI companion host controller.
 *
 * The Type-A socket's full/low-speed devices are routed to a one-port OHCI
 * 1.0 companion at +0x200 inside the paired EHCI page.  That window only
 * decodes once firmware's host-mode latch has been applied and the paired
 * EHCI core is running, so this driver owns a device of its own but refuses
 * to touch the window until the companion is demonstrably alive:
 *
 *   1. the paired EHCI platform device is bound and its HCD is running,
 *   2. its root hub is registered and runtime-resumed by us, and
 *   3. its frame index is advancing.
 *
 * A pre-EHCI read poisons the whole bus page until the next power cycle, so
 * every one of those checks runs before the first OHCI access, and probe
 * defers rather than guessing.  Reaching operational state was proven on
 * hardware on 2026-09-06, and a directly attached full-speed keyboard
 * enumerated through the EHCI companion hand-off on 2026-09-07.
 *
 * Sony's usbd resets the controller in a different order than the generic
 * OHCI core: interrupts are masked and HcCommandStatus.HCR is asserted
 * BEFORE HcControl is cleared.  That preamble is kept; everything after it
 * is the generic OHCI transfer engine.
 */

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reboot.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ohci.h"

#define DRIVER_DESC "PSTV bus-zero OHCI companion"

/* The only companion this driver claims; see the bus map in vita.dtsi. */
#define PSTV_OHCI_REG_BASE	0xe40e0200
#define PSTV_OHCI_REG_SIZE	0x100

/* OHCI operational registers touched by the reset preamble and admission. */
#define PSTV_OHCI_REVISION	0x00
#define PSTV_OHCI_CONTROL	0x04
#define PSTV_OHCI_CMD_STATUS	0x08
#define PSTV_OHCI_INTR_DISABLE	0x14

#define PSTV_OHCI_HCR		0x00000001
#define PSTV_OHCI_IRQ_MASK	0xc000007f
#define PSTV_OHCI_REV_MASK	0xff
#define PSTV_OHCI_REV_11	0x10

/* EHCI operational registers, relative to the paired core's CAPLENGTH. */
#define PSTV_EHCI_USBCMD	0x00
#define PSTV_EHCI_USBSTS	0x04
#define PSTV_EHCI_FRINDEX	0x0c

#define PSTV_EHCI_CMD_RUN	BIT(0)
#define PSTV_EHCI_STS_HALTED	BIT(12)
#define PSTV_EHCI_FRINDEX_MASK	0x3fff

/*
 * The Type-A rail is dropped by vita-dolce-usb-power's reboot notifier at
 * priority 256, which runs before device_shutdown().  Tear the controller
 * and its children down first so no USB work can be submitted into a window
 * that has already lost VBUS.  Higher priority runs earlier.
 */
#define PSTV_OHCI_REBOOT_PRIORITY	257

static struct hc_driver __read_mostly pstv_ohci_hc_driver;

struct pstv_ohci {
	struct usb_hcd *hcd;
	struct usb_hcd *ehci_hcd;
	struct usb_device *ehci_rh;
	struct notifier_block reboot_nb;
	struct mutex lock;	/* serialises teardown against reboot */
	bool live;
};

/*
 * Sony's reset order, then the generic OHCI initialisation.  Overriding
 * .reset replaces the generic hook outright, so ohci_setup() must be called
 * here or the controller is never initialised.
 */
static int pstv_ohci_reset(struct usb_hcd *hcd)
{
	writel_relaxed(PSTV_OHCI_IRQ_MASK, hcd->regs + PSTV_OHCI_INTR_DISABLE);
	writel_relaxed(PSTV_OHCI_HCR, hcd->regs + PSTV_OHCI_CMD_STATUS);
	writel_relaxed(0, hcd->regs + PSTV_OHCI_CONTROL);

	return ohci_setup(hcd);
}

static const struct ohci_driver_overrides pstv_ohci_overrides __initconst = {
	.product_desc	= DRIVER_DESC,
	.reset		= pstv_ohci_reset,
};

/*
 * Take the references that keep the paired EHCI HCD and its root hub alive.
 * The EHCI device lock is held only for the bound check and the reference
 * acquisition: driver-core unbind cannot race in between, and once the
 * references and the device link exist the lock is no longer what protects
 * us.  Never dereference the EHCI drvdata outside this window.
 */
static int pstv_ohci_grab_ehci(struct device *dev,
			       struct platform_device *ehci_pdev,
			       struct usb_hcd **ehci_hcd,
			       struct usb_device **ehci_rh)
{
	struct usb_hcd *hcd;
	int ret = 0;

	device_lock(&ehci_pdev->dev);

	if (!device_is_bound(&ehci_pdev->dev)) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}

	hcd = platform_get_drvdata(ehci_pdev);
	if (!hcd || !hcd->self.root_hub) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}

	usb_get_hcd(hcd);
	*ehci_rh = usb_get_dev(hcd->self.root_hub);
	*ehci_hcd = hcd;

unlock:
	device_unlock(&ehci_pdev->dev);

	if (ret)
		dev_dbg(dev, "paired EHCI not bound yet\n");

	return ret;
}

/*
 * The companion window only answers while the paired core is actually
 * running.  A frozen frame index means an idle root hub was allowed to
 * runtime-suspend, so this runs with our PM reference already held.
 */
static int pstv_ohci_ehci_running(struct device *dev, struct usb_hcd *ehci_hcd)
{
	void __iomem *op;
	u32 caplength, first, last;

	if (!HCD_HW_ACCESSIBLE(ehci_hcd) || !ehci_hcd->rh_registered ||
	    READ_ONCE(ehci_hcd->state) != HC_STATE_RUNNING) {
		dev_dbg(dev, "paired EHCI not running yet\n");
		return -EPROBE_DEFER;
	}

	caplength = readl(ehci_hcd->regs) & 0xff;
	if (caplength != 0x10 && caplength != 0x20) {
		dev_err(dev, "implausible EHCI CAPLENGTH %#x\n", caplength);
		return -ENODEV;
	}

	op = ehci_hcd->regs + caplength;
	if (!(readl(op + PSTV_EHCI_USBCMD) & PSTV_EHCI_CMD_RUN) ||
	    (readl(op + PSTV_EHCI_USBSTS) & PSTV_EHCI_STS_HALTED)) {
		dev_dbg(dev, "paired EHCI halted\n");
		return -EPROBE_DEFER;
	}

	first = readl(op + PSTV_EHCI_FRINDEX) & PSTV_EHCI_FRINDEX_MASK;
	usleep_range(2000, 3000);
	last = readl(op + PSTV_EHCI_FRINDEX) & PSTV_EHCI_FRINDEX_MASK;
	if (first == last) {
		dev_dbg(dev, "paired EHCI frame index frozen\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

static void pstv_ohci_teardown(struct pstv_ohci *pstv)
{
	guard(mutex)(&pstv->lock);

	if (!pstv->live)
		return;
	pstv->live = false;

	/*
	 * Disconnects children, cancels root-hub work, synchronises the IRQ
	 * and stops DMA.  It must complete before the rail goes away.
	 */
	usb_remove_hcd(pstv->hcd);
	pstv->hcd->self.hs_companion = NULL;

	pm_runtime_put_sync_autosuspend(&pstv->ehci_rh->dev);
	usb_put_dev(pstv->ehci_rh);
	usb_put_hcd(pstv->ehci_hcd);
	pstv->ehci_rh = NULL;
	pstv->ehci_hcd = NULL;
}

static int pstv_ohci_reboot(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	struct pstv_ohci *pstv = container_of(nb, struct pstv_ohci, reboot_nb);

	pstv_ohci_teardown(pstv);

	return NOTIFY_DONE;
}

static int pstv_ohci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *ehci_pdev;
	struct usb_hcd *hcd, *ehci_hcd = NULL;
	struct usb_device *ehci_rh = NULL;
	struct pstv_ohci *pstv;
	struct device_node *np;
	struct resource *res;
	void __iomem *regs;
	u32 revision;
	int irq, ret;

	if (usb_disabled())
		return -ENODEV;

	pstv = devm_kzalloc(dev, sizeof(*pstv), GFP_KERNEL);
	if (!pstv)
		return -ENOMEM;
	mutex_init(&pstv->lock);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "no 32-bit DMA mask\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no register window\n");
	/*
	 * Identity check only: the mapping below still uses the described
	 * resource.  A node pointing anywhere else is not this companion,
	 * and probing the wrong window is the one-way poisoning case.
	 */
	if (res->start != PSTV_OHCI_REG_BASE ||
	    resource_size(res) != PSTV_OHCI_REG_SIZE)
		return dev_err_probe(dev, -EINVAL,
				     "unexpected window %pR\n", res);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	np = of_parse_phandle(dev->of_node, "vita,ehci", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no vita,ehci phandle\n");
	ehci_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!ehci_pdev)
		return -EPROBE_DEFER;

	ret = pstv_ohci_grab_ehci(dev, ehci_pdev, &ehci_hcd, &ehci_rh);
	if (ret)
		goto put_ehci_pdev;

	/*
	 * Make the dependency real before anything else: the driver core now
	 * unbinds this driver ahead of the EHCI one, so the references taken
	 * above cannot outlive their supplier.
	 *
	 * The link is ordering-only.  ehci-platform never enables runtime PM
	 * on its platform device, so it reports "unsupported" and asking for
	 * DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE makes device_link_add()
	 * fail its internal pm_runtime_get_sync() with -EACCES and return
	 * NULL (observed on hardware 2026-09-11).  What actually has to stay
	 * powered is the EHCI *root hub*, and that is held below with its own
	 * runtime-PM reference.
	 */
	if (!device_link_add(dev, &ehci_pdev->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = dev_err_probe(dev, -EINVAL, "no device link to EHCI\n");
		goto put_ehci;
	}

	/* The root-hub usage count, not a held lock, is what keeps it up. */
	ret = pm_runtime_resume_and_get(&ehci_rh->dev);
	if (ret < 0) {
		dev_err_probe(dev, ret, "cannot resume EHCI root hub\n");
		goto put_ehci;
	}

	usb_lock_device(ehci_rh);
	ret = pstv_ohci_ehci_running(dev, ehci_hcd);
	if (ret)
		goto unlock_root_hub;

	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		goto unlock_root_hub;
	}

	/* First companion access of the whole driver. */
	revision = readl(regs + PSTV_OHCI_REVISION);
	if ((revision & PSTV_OHCI_REV_MASK) != PSTV_OHCI_REV_11) {
		ret = dev_err_probe(dev, -ENODEV,
				    "no OHCI 1.0 companion (revision %#x)\n",
				    revision);
		goto unlock_root_hub;
	}

	hcd = usb_create_hcd(&pstv_ohci_hc_driver, dev, dev_name(dev));
	if (!hcd) {
		ret = -ENOMEM;
		goto unlock_root_hub;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = regs;
	hcd->speed = HCD_USB11;
	hcd->skip_phy_initialization = 1;
	hcd->self.hs_companion = &ehci_hcd->self;

	ret = usb_add_hcd(hcd, irq, 0);
	if (ret) {
		dev_err_probe(dev, ret, "cannot register companion HCD\n");
		hcd->self.hs_companion = NULL;
		usb_put_hcd(hcd);
		goto unlock_root_hub;
	}

	/*
	 * The hand-off is complete, so the root-hub lock has done its job:
	 * it only had to keep hub work out of the window between validating
	 * the companion and publishing it.  Lifetime from here on belongs to
	 * the references, the device link and the runtime-PM count.
	 */
	usb_unlock_device(ehci_rh);

	pstv->hcd = hcd;
	pstv->ehci_hcd = ehci_hcd;
	pstv->ehci_rh = ehci_rh;
	pstv->live = true;

	pstv->reboot_nb.notifier_call = pstv_ohci_reboot;
	pstv->reboot_nb.priority = PSTV_OHCI_REBOOT_PRIORITY;
	ret = register_reboot_notifier(&pstv->reboot_nb);
	if (ret) {
		dev_err_probe(dev, ret, "no reboot notifier\n");
		pstv->live = false;
		usb_remove_hcd(hcd);
		hcd->self.hs_companion = NULL;
		usb_put_hcd(hcd);
		goto put_pm;
	}

	platform_set_drvdata(pdev, pstv);
	/* The EHCI/root-hub references and PM hold remain live until remove. */
	put_device(&ehci_pdev->dev);

	return 0;

unlock_root_hub:
	usb_unlock_device(ehci_rh);
put_pm:
	pm_runtime_put_sync_autosuspend(&ehci_rh->dev);
put_ehci:
	usb_put_dev(ehci_rh);
	usb_put_hcd(ehci_hcd);
put_ehci_pdev:
	put_device(&ehci_pdev->dev);

	return ret;
}

static void pstv_ohci_remove(struct platform_device *pdev)
{
	struct pstv_ohci *pstv = platform_get_drvdata(pdev);

	if (!pstv)
		return;

	unregister_reboot_notifier(&pstv->reboot_nb);
	pstv_ohci_teardown(pstv);
	platform_set_drvdata(pdev, NULL);
	usb_put_hcd(pstv->hcd);
}

static void pstv_ohci_shutdown(struct platform_device *pdev)
{
	struct pstv_ohci *pstv = platform_get_drvdata(pdev);

	/* Idempotent: the reboot notifier has usually run already. */
	if (pstv)
		pstv_ohci_teardown(pstv);
}

/*
 * System sleep is deliberately unimplemented.  Resuming this companion needs
 * the firmware host-mode latch and the Type-A rail to be re-established in
 * an order that has not been observed on hardware, and a wrong guess costs a
 * one-way poisoned bus page.  Refuse the transition instead of replaying an
 * unproven reset; the device link keeps the paired EHCI core up with us.
 */
static int pstv_ohci_suspend(struct device *dev)
{
	dev_warn(dev, "system sleep is not supported by this companion\n");

	return -EBUSY;
}

static const struct dev_pm_ops pstv_ohci_pm_ops = {
	.suspend = pstv_ohci_suspend,
	.freeze = pstv_ohci_suspend,
	.poweroff = pstv_ohci_suspend,
};

static const struct of_device_id pstv_ohci_of_match[] = {
	{ .compatible = "vita,pstv-ohci" },
	{ }
};
MODULE_DEVICE_TABLE(of, pstv_ohci_of_match);

static struct platform_driver pstv_ohci_driver = {
	.probe = pstv_ohci_probe,
	.remove = pstv_ohci_remove,
	.shutdown = pstv_ohci_shutdown,
	.driver = {
		.name = "pstv-ohci",
		.of_match_table = pstv_ohci_of_match,
		.pm = &pstv_ohci_pm_ops,
		/*
		 * Rebinding would have to redo the whole EHCI-first
		 * admission from userspace's timing, which is not a
		 * supported entry point yet.
		 */
		.suppress_bind_attrs = true,
	},
};

static int __init pstv_ohci_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ohci_init_driver(&pstv_ohci_hc_driver, &pstv_ohci_overrides);

	return platform_driver_register(&pstv_ohci_driver);
}
module_init(pstv_ohci_init);

static void __exit pstv_ohci_cleanup(void)
{
	platform_driver_unregister(&pstv_ohci_driver);
}
module_exit(pstv_ohci_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
