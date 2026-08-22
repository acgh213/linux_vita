// SPDX-License-Identifier: GPL-2.0
/*
 * PSTV (Dolce) external Type-A USB power sequencer.
 *
 * The PlayStation TV's external USB-A socket needs two independent things
 * before a device can be seen, and neither is done by the host controller:
 *
 *   1. Ernie's Dolce USB 5 V rail (syscon short command 0x8c5), and
 *   2. GPIO1 pin 3, which must be switched to output *before* it is
 *      asserted -- setting the pin while it is still an input silently
 *      does nothing (2026-08-20, cycle 22).
 *
 * VitaOS performs the same pair as ksceSysconCtrlDolceUsbPower() followed
 * by ksceGpioSetPortMode(1, 3, 0) and ksceGpioPortSet(1, 3).
 *
 * The socket is wired to Sony USB bus 0 (EHCI at 0xE40E0000, Linux usb1),
 * so this driver waits for that controller to finish probing before it
 * powers the port: powering a socket whose controller is already up is the
 * proven hotplug path, whereas powering it first would leave any attached
 * device unenumerated until something else disturbed the bus.
 *
 * Note that a bound EHCI controller does not by itself mean the SoC core
 * behind it is in host mode: clk-vita-pervasive only performs the host-mode
 * latch for buses named in its buses_mask, and bus 0 is not in the default
 * mask.  Powering the socket on a bus left in device mode is harmless -- the
 * port is simply dead -- but it is why the board must select bus 0.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include <linux/mfd/vita-syscon.h>

/*
 * Must outrank the syscon driver's own reboot notifier (priority 255).
 * That one switches the rail off, and if it ran first the pin would be
 * left asserted above a rail that had already gone away.
 */
#define VITA_DOLCE_USB_REBOOT_PRIORITY	256

struct vita_dolce_usb_power {
	struct vita_syscon *syscon;
	struct gpio_desc *enable_gpio;
	struct notifier_block reboot_nb;
	bool powered;
};

static struct vita_syscon *vita_dolce_usb_get_syscon(struct device *dev)
{
	struct device_node *syscon_np;
	struct device *syscon_dev;
	struct vita_syscon *syscon = NULL;

	syscon_np = of_parse_phandle(dev->of_node, "vita,syscon", 0);
	if (!syscon_np)
		return ERR_PTR(-EINVAL);

	syscon_dev = bus_find_device_by_of_node(&spi_bus_type, syscon_np);
	of_node_put(syscon_np);
	if (!syscon_dev)
		return ERR_PTR(-EPROBE_DEFER);

	/*
	 * Only trust the driver data of a device that is actually bound.
	 * vita_syscon_probe() publishes it early and can still fail
	 * afterwards, at which point devres frees the object without
	 * clearing the pointer.
	 */
	device_lock(syscon_dev);
	if (device_is_bound(syscon_dev))
		syscon = spi_get_drvdata(to_spi_device(syscon_dev));
	device_unlock(syscon_dev);
	put_device(syscon_dev);

	if (!syscon)
		return ERR_PTR(-EPROBE_DEFER);

	return syscon;
}

/*
 * Wait for the paired EHCI controller to finish probing, then take a device
 * link on it so the ordering holds for the rest of this device's life.
 *
 * usb_add_hcd() starts the controller and registers the root hub before the
 * platform bind completes, so "bound with a non-halted HCD" means the host
 * side is ready for a connect.  The link makes this driver unbind, dropping
 * VBUS, if the controller ever goes away, and fixes the shutdown order
 * instead of leaving it to depend on device-tree node order.
 *
 * Deliberately *not* checked: whether FRINDEX is advancing.  The cycle-25
 * OHCI diagnostic required that because reading the companion's register
 * window before the bus is genuinely alive poisons the page.  This driver
 * touches no OHCI window, and requiring live frames here would deadlock a
 * cold boot with an empty port: an idle root hub autosuspends and freezes
 * FRINDEX (2026-08-19, cycle 13c), frames only resume once a device
 * attaches, and no device can attach until this driver applies VBUS.
 */
static int vita_dolce_usb_link_ehci(struct device *dev)
{
	struct platform_device *ehci_pdev;
	struct device_node *ehci_np;
	struct usb_hcd *hcd;
	int ret = 0;

	ehci_np = of_parse_phandle(dev->of_node, "vita,ehci", 0);
	if (!ehci_np)
		return -EINVAL;

	ehci_pdev = of_find_device_by_node(ehci_np);
	of_node_put(ehci_np);
	if (!ehci_pdev)
		return -EPROBE_DEFER;

	device_lock(&ehci_pdev->dev);
	if (!device_is_bound(&ehci_pdev->dev)) {
		ret = -EPROBE_DEFER;
		goto out;
	}

	hcd = platform_get_drvdata(ehci_pdev);
	if (!hcd || hcd->state == HC_STATE_HALT)
		ret = -EPROBE_DEFER;

out:
	device_unlock(&ehci_pdev->dev);

	if (!ret && !device_link_add(dev, &ehci_pdev->dev,
				     DL_FLAG_AUTOREMOVE_CONSUMER))
		ret = -EINVAL;

	put_device(&ehci_pdev->dev);

	return ret;
}

/* Rail first, then the pin: the pin gates a rail that must already exist. */
static int vita_dolce_usb_power_on(struct vita_dolce_usb_power *vbus)
{
	int ret;

	ret = vita_syscon_dolce_usb_power_set(vbus->syscon, true);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(vbus->enable_gpio, 1);
	vbus->powered = true;

	return 0;
}

/* Exact reverse: drop the pin before the rail it gates disappears. */
static void vita_dolce_usb_power_off(struct vita_dolce_usb_power *vbus)
{
	if (!vbus->powered)
		return;

	gpiod_set_value_cansleep(vbus->enable_gpio, 0);
	vita_syscon_dolce_usb_power_set(vbus->syscon, false);
	vbus->powered = false;
}

/*
 * Reboot and poweroff run the notifier chain before device_shutdown(), so
 * depowering here -- ahead of the syscon notifier -- is the only way to keep
 * the pin-then-rail order on those paths.  .shutdown then finds nothing to do.
 */
static int vita_dolce_usb_power_reboot(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	struct vita_dolce_usb_power *vbus =
		container_of(nb, struct vita_dolce_usb_power, reboot_nb);

	vita_dolce_usb_power_off(vbus);

	return NOTIFY_DONE;
}

static int vita_dolce_usb_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_dolce_usb_power *vbus;
	int ret;

	vbus = devm_kzalloc(dev, sizeof(*vbus), GFP_KERNEL);
	if (!vbus)
		return -ENOMEM;

	vbus->syscon = vita_dolce_usb_get_syscon(dev);
	if (IS_ERR(vbus->syscon))
		return dev_err_probe(dev, PTR_ERR(vbus->syscon),
				     "no vita,syscon\n");

	ret = vita_dolce_usb_link_ehci(dev);
	if (ret)
		return dev_err_probe(dev, ret, "paired EHCI not ready\n");

	/*
	 * GPIOD_OUT_LOW establishes the direction as part of the request, so
	 * the pin is a driven output before it is ever asserted.
	 */
	vbus->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(vbus->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(vbus->enable_gpio),
				     "no enable-gpios\n");

	vbus->reboot_nb.notifier_call = vita_dolce_usb_power_reboot;
	vbus->reboot_nb.priority = VITA_DOLCE_USB_REBOOT_PRIORITY;
	ret = devm_register_reboot_notifier(dev, &vbus->reboot_nb);
	if (ret)
		return dev_err_probe(dev, ret, "no reboot notifier\n");

	ret = vita_dolce_usb_power_on(vbus);
	if (ret) {
		/* Defensive: nothing is asserted yet if the rail refused. */
		vita_dolce_usb_power_off(vbus);
		return dev_err_probe(dev, ret, "failed to power the port\n");
	}

	platform_set_drvdata(pdev, vbus);
	dev_info(dev, "Type-A port powered: Dolce rail on, enable GPIO asserted\n");

	return 0;
}

static void vita_dolce_usb_power_remove(struct platform_device *pdev)
{
	vita_dolce_usb_power_off(platform_get_drvdata(pdev));
}

static void vita_dolce_usb_power_shutdown(struct platform_device *pdev)
{
	vita_dolce_usb_power_off(platform_get_drvdata(pdev));
}

static const struct of_device_id vita_dolce_usb_power_of_match[] = {
	{ .compatible = "vita,dolce-usb-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, vita_dolce_usb_power_of_match);

static struct platform_driver vita_dolce_usb_power_driver = {
	.probe = vita_dolce_usb_power_probe,
	.remove = vita_dolce_usb_power_remove,
	.shutdown = vita_dolce_usb_power_shutdown,
	.driver = {
		.name = "vita-dolce-usb-power",
		.of_match_table = vita_dolce_usb_power_of_match,
	},
};
module_platform_driver(vita_dolce_usb_power_driver);

MODULE_DESCRIPTION("PSTV external Type-A USB power sequencer");
MODULE_LICENSE("GPL");
