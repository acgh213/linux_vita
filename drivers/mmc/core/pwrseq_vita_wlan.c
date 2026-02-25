// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pwrseq_vita_wlan.c - mmc-pwrseq driver for the PS Vita's SD8787 WiFi module
 *
 * The Vita's SD8787 SDIO WiFi chip requires a multi-step power-on sequence
 * that goes through the Ernie syscon (SPI) and a clockgen chip (I2C),
 * followed by a full SDHCI controller re-init at the pervasive level.
 * This doesn't fit the standard pwrseq_sd8787 driver which expects
 * direct GPIO control of powerdown/reset pins.
 *
 * Power-on sequence:
 *   pre_power_on:
 *     1. Disable SDHCI interrupts (prevent premature MMC card detect)
 *     2. Enable 27MHz WlanBt reference clock (clockgen I2C)
 *     3. Power on wireless via Ernie syscon (cmd 0x88A)
 *     4. De-assert WLANBT reset via Ernie syscon (cmd 0x88F)
 *   post_power_on:
 *     5. Full SDHCI controller re-init (pervasive reset, 1.8V I/O, clocks)
 *
 * The MMC core handles card detection (CMD5 SDIO enum) naturally after
 * mmc_power_up() completes — no manual rescan trigger needed.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>

#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/mfd/vita-syscon.h>

#include "pwrseq.h"

struct mmc_pwrseq_vita_wlan {
	struct mmc_pwrseq pwrseq;
	struct vita_syscon *syscon;
	bool power_on_ok;
};

#define to_pwrseq_vita_wlan(p) container_of(p, struct mmc_pwrseq_vita_wlan, pwrseq)

static int mmc_pwrseq_vita_wlan_get_bus_index(struct mmc_host *host)
{
	u32 bus_index;

	if (of_property_read_u32(mmc_dev(host)->of_node,
				 "vita,bus-index", &bus_index))
		return -EINVAL;
	return bus_index;
}

static void mmc_pwrseq_vita_wlan_pre_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_vita_wlan *pwrseq = to_pwrseq_vita_wlan(host->pwrseq);
	int bus_index, ret;

	bus_index = mmc_pwrseq_vita_wlan_get_bus_index(host);
	if (bus_index < 0)
		return;

	/* Disable SDHCI interrupts to prevent premature card detect */
	sdhci_vita_suppress_irqs(bus_index);

	/* Power on: clockgen enable, Ernie power, Ernie reset deassert */
	ret = vita_syscon_wlan_power_on(pwrseq->syscon);
	if (ret) {
		dev_err(pwrseq->pwrseq.dev,
			"WLAN power-on sequence failed: %d\n", ret);
		pwrseq->power_on_ok = false;
		return;
	}

	pwrseq->power_on_ok = true;
}

static void mmc_pwrseq_vita_wlan_post_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_vita_wlan *pwrseq = to_pwrseq_vita_wlan(host->pwrseq);
	int bus_index;

	if (!pwrseq->power_on_ok)
		return;

	bus_index = mmc_pwrseq_vita_wlan_get_bus_index(host);
	if (bus_index < 0)
		return;

	/*
	 * Full SDHCI controller re-init: pervasive gate/reset cycle,
	 * 1.8V I/O voltage, SDHCI software reset, interrupt setup,
	 * bus voltage select, clock configuration.
	 *
	 * This must happen after the SD8787 is powered but before the
	 * MMC core tries to communicate with the card.
	 */
	sdhci_vita_reinit_host(bus_index);
}

static void mmc_pwrseq_vita_wlan_power_off(struct mmc_host *host)
{
	struct mmc_pwrseq_vita_wlan *pwrseq = to_pwrseq_vita_wlan(host->pwrseq);
	int ret;

	ret = vita_syscon_wlan_power_off(pwrseq->syscon);
	if (ret)
		dev_err(pwrseq->pwrseq.dev,
			"WLAN power-off sequence failed: %d\n", ret);
}

static const struct mmc_pwrseq_ops mmc_pwrseq_vita_wlan_ops = {
	.pre_power_on = mmc_pwrseq_vita_wlan_pre_power_on,
	.post_power_on = mmc_pwrseq_vita_wlan_post_power_on,
	.power_off = mmc_pwrseq_vita_wlan_power_off,
};

static const struct of_device_id mmc_pwrseq_vita_wlan_of_match[] = {
	{ .compatible = "vita,pwrseq-wlan" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mmc_pwrseq_vita_wlan_of_match);

static int mmc_pwrseq_vita_wlan_probe(struct platform_device *pdev)
{
	struct mmc_pwrseq_vita_wlan *pwrseq;
	struct device *dev = &pdev->dev;
	struct device_node *syscon_np;
	struct device *syscon_dev;
	struct vita_syscon *syscon;

	pwrseq = devm_kzalloc(dev, sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return -ENOMEM;

	/*
	 * Look up the vita,syscon phandle to get the syscon struct.
	 * The syscon driver stores it as SPI driver data.
	 */
	syscon_np = of_parse_phandle(dev->of_node, "vita,syscon", 0);
	if (!syscon_np) {
		dev_err(dev, "missing vita,syscon phandle\n");
		return -EINVAL;
	}

	syscon_dev = bus_find_device_by_of_node(&spi_bus_type, syscon_np);
	of_node_put(syscon_np);
	if (!syscon_dev) {
		dev_dbg(dev, "syscon device not ready, deferring\n");
		return -EPROBE_DEFER;
	}

	syscon = spi_get_drvdata(to_spi_device(syscon_dev));
	put_device(syscon_dev);
	if (!syscon) {
		dev_dbg(dev, "syscon not probed yet, deferring\n");
		return -EPROBE_DEFER;
	}

	pwrseq->syscon = syscon;
	pwrseq->pwrseq.dev = dev;
	pwrseq->pwrseq.ops = &mmc_pwrseq_vita_wlan_ops;
	pwrseq->pwrseq.owner = THIS_MODULE;
	platform_set_drvdata(pdev, pwrseq);

	dev_info(dev, "Vita WLAN power sequencer registered\n");

	return mmc_pwrseq_register(&pwrseq->pwrseq);
}

static void mmc_pwrseq_vita_wlan_remove(struct platform_device *pdev)
{
	struct mmc_pwrseq_vita_wlan *pwrseq = platform_get_drvdata(pdev);

	mmc_pwrseq_unregister(&pwrseq->pwrseq);
}

static struct platform_driver mmc_pwrseq_vita_wlan_driver = {
	.probe = mmc_pwrseq_vita_wlan_probe,
	.remove_new = mmc_pwrseq_vita_wlan_remove,
	.driver = {
		.name = "pwrseq_vita_wlan",
		.of_match_table = mmc_pwrseq_vita_wlan_of_match,
	},
};

module_platform_driver(mmc_pwrseq_vita_wlan_driver);
MODULE_DESCRIPTION("Power sequence support for PS Vita SD8787 WiFi module");
MODULE_LICENSE("GPL v2");
