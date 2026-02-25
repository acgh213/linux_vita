// SPDX-License-Identifier: GPL-2.0
/*
 * I2C bus controller driver for the PlayStation Vita
 *
 * The Vita SoC has two I2C buses:
 *   I2C0 (0xE0500000) - audio codec (WM1803E), clockgen (P1P40167), cameras
 *   I2C1 (0xE0510000) - motion sensor, HDMI (ADV7533), LCD brightness
 *
 * Register map (byte offsets):
 *   0x00: Write FIFO
 *   0x04: Read FIFO
 *   0x08: Unknown (set to 1 during init)
 *   0x0C: Unknown (set to 1 during init)
 *   0x10: Device address (7-bit)
 *   0x14: Flags (transfer control)
 *   0x18: Speed
 *   0x1C: Busy
 *   0x28: IRQ Status
 *   0x2C: IRQ Control
 *
 * Protocol derived from vita-libbaremetal and the HENkaku wiki.
 *
 * Copyright (C) 2021 Sergi Granell
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* Register offsets (byte-addressed) */
#define VITA_I2C_WRITE_FIFO	0x00
#define VITA_I2C_READ_FIFO	0x04
#define VITA_I2C_REG08		0x08	/* unknown, set 1 at init */
#define VITA_I2C_REG0C		0x0C	/* unknown, set 1 at init */
#define VITA_I2C_DEVICE_ADDR	0x10	/* 7-bit address */
#define VITA_I2C_FLAGS		0x14
#define VITA_I2C_SPEED		0x18
#define VITA_I2C_BUSY		0x1C
#define VITA_I2C_IRQ_STATUS	0x28
#define VITA_I2C_IRQ_CONTROL	0x2C

/* Flags register values */
#define VITA_I2C_FLAGS_START	0x02	/* start + transmit */
#define VITA_I2C_FLAGS_STOP	0x04	/* stop condition */
#define VITA_I2C_FLAGS_RSTART	0x05	/* repeated start */
#define VITA_I2C_FLAGS_RESET	0x07	/* bus reset */
#define VITA_I2C_FLAGS_READ	0x13	/* read transfer */
#define VITA_I2C_FLAGS_LEN(n)	((n) << 8)

/* IRQ status bits (discovered via register probing) */
#define VITA_I2C_IRQ_NACK	BIT(15)	/* slave NACK'd the address/data */

/* IRQ control values from vita-libbaremetal */
#define VITA_I2C_IRQ_INIT	0x100F70F
#define VITA_I2C_IRQ_RUNNING	0x1000000

/* Pervasive clock gate (no clock driver exists yet) */
#define VITA_PERVASIVE_GATE_BASE	0xE3102000
#define VITA_PERVASIVE_I2C_OFFSET	0x110

/*
 * The flags register encodes the transfer length in bits [15:8].
 * Maximum supported length per message is 255 bytes.
 */
#define VITA_I2C_MAX_XFER_LEN	255

/* Timeouts */
#define VITA_I2C_TIMEOUT_US	100000
#define VITA_I2C_POLL_US	1

struct vita_i2c {
	void __iomem *base;
	struct i2c_adapter adap;
	struct device *dev;
	int bus_index;
};

static int vita_i2c_wait_busy(struct vita_i2c *i2c)
{
	u32 val;

	return readl_poll_timeout(i2c->base + VITA_I2C_BUSY, val,
				  !val, VITA_I2C_POLL_US, VITA_I2C_TIMEOUT_US);
}

/* Clear all pending IRQ status bits (write-1-to-clear) */
static void vita_i2c_clear_status(struct vita_i2c *i2c)
{
	writel(readl(i2c->base + VITA_I2C_IRQ_STATUS),
	       i2c->base + VITA_I2C_IRQ_STATUS);
}

/* Check the IRQ status register for NACK — returns true if slave NACK'd */
static bool vita_i2c_got_nack(struct vita_i2c *i2c)
{
	return readl(i2c->base + VITA_I2C_IRQ_STATUS) & VITA_I2C_IRQ_NACK;
}

static int vita_i2c_hw_init(struct vita_i2c *i2c)
{
	int ret;

	/* Initialize I2C bus (sequence from vita-libbaremetal) */
	writel(VITA_I2C_IRQ_INIT, i2c->base + VITA_I2C_IRQ_CONTROL);
	writel(1, i2c->base + VITA_I2C_REG08);
	writel(1, i2c->base + VITA_I2C_REG0C);

	/* Bus reset */
	writel(VITA_I2C_FLAGS_RESET, i2c->base + VITA_I2C_FLAGS);
	mb();
	ret = vita_i2c_wait_busy(i2c);
	if (ret) {
		dev_err(i2c->dev, "hw init: bus reset timeout\n");
		return ret;
	}

	/* Clear pending IRQs */
	writel(readl(i2c->base + VITA_I2C_IRQ_STATUS),
	       i2c->base + VITA_I2C_IRQ_STATUS);
	writel(VITA_I2C_IRQ_RUNNING, i2c->base + VITA_I2C_IRQ_CONTROL);

	/* Set bus speed */
	writel(4, i2c->base + VITA_I2C_SPEED);

	return 0;
}

static void vita_i2c_clock_gate_enable(struct vita_i2c *i2c)
{
	void __iomem *gate;
	u32 offset = VITA_PERVASIVE_I2C_OFFSET + i2c->bus_index * 4;

	gate = ioremap(VITA_PERVASIVE_GATE_BASE + offset, 4);
	if (gate) {
		writel(readl(gate) | 1, gate);
		iounmap(gate);
	}
	udelay(100);
}

static int vita_i2c_xfer_write(struct vita_i2c *i2c, struct i2c_msg *msg,
			       bool stop)
{
	int i, ret;

	writel(1, i2c->base + VITA_I2C_REG08);
	writel(1, i2c->base + VITA_I2C_REG0C);
	writel(msg->addr, i2c->base + VITA_I2C_DEVICE_ADDR);

	for (i = 0; i < msg->len; i++)
		writel(msg->buf[i], i2c->base + VITA_I2C_WRITE_FIFO);

	writel(VITA_I2C_FLAGS_LEN(msg->len) | VITA_I2C_FLAGS_START,
	       i2c->base + VITA_I2C_FLAGS);

	ret = vita_i2c_wait_busy(i2c);
	if (ret) {
		dev_err(i2c->dev, "write: busy timeout\n");
		return ret;
	}

	if (stop) {
		writel(VITA_I2C_FLAGS_STOP, i2c->base + VITA_I2C_FLAGS);
		ret = vita_i2c_wait_busy(i2c);
		if (ret) {
			dev_err(i2c->dev, "write stop: busy timeout\n");
			return ret;
		}
	}

	return 0;
}

static int vita_i2c_xfer_read(struct vita_i2c *i2c, struct i2c_msg *msg,
			      bool repeated_start, bool stop)
{
	int i, ret;

	if (!repeated_start) {
		/* Pure read: set up the bus first */
		writel(1, i2c->base + VITA_I2C_REG08);
		writel(1, i2c->base + VITA_I2C_REG0C);
	}

	/* Always program device address (may differ between messages) */
	writel(msg->addr, i2c->base + VITA_I2C_DEVICE_ADDR);

	if (repeated_start) {
		writel(VITA_I2C_FLAGS_RSTART, i2c->base + VITA_I2C_FLAGS);
		ret = vita_i2c_wait_busy(i2c);
		if (ret) {
			dev_err(i2c->dev, "read rstart: busy timeout\n");
			return ret;
		}
	}

	writel(VITA_I2C_FLAGS_LEN(msg->len) | VITA_I2C_FLAGS_READ,
	       i2c->base + VITA_I2C_FLAGS);

	ret = vita_i2c_wait_busy(i2c);
	if (ret) {
		dev_err(i2c->dev, "read: busy timeout\n");
		return ret;
	}

	for (i = 0; i < msg->len; i++)
		msg->buf[i] = readl(i2c->base + VITA_I2C_READ_FIFO);

	if (stop) {
		writel(VITA_I2C_FLAGS_STOP, i2c->base + VITA_I2C_FLAGS);
		ret = vita_i2c_wait_busy(i2c);
		if (ret) {
			dev_err(i2c->dev, "read stop: busy timeout\n");
			return ret;
		}
	}

	return 0;
}

static int vita_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			 int num)
{
	struct vita_i2c *i2c = i2c_get_adapdata(adap);
	int idx, ret;

	/* Clear stale IRQ status before starting the transfer */
	vita_i2c_clear_status(i2c);

	for (idx = 0; idx < num; idx++) {
		bool is_last = (idx == num - 1);
		bool is_read = msgs[idx].flags & I2C_M_RD;
		bool next_is_read = !is_last &&
				    (msgs[idx + 1].flags & I2C_M_RD);

		if (is_read) {
			/*
			 * If preceded by a write (combined xfer), the write
			 * left the bus without a stop — issue repeated start.
			 */
			ret = vita_i2c_xfer_read(i2c, &msgs[idx],
						 idx > 0, is_last);
		} else {
			/*
			 * Only suppress stop if the next message is a read
			 * (combined write-then-read transaction).  For
			 * write-then-write, issue stop between messages
			 * since the controller has only been validated for
			 * write→read repeated-start sequences.
			 */
			bool stop = is_last || !next_is_read;

			ret = vita_i2c_xfer_write(i2c, &msgs[idx], stop);
		}

		if (ret)
			return ret;
	}

	/*
	 * Check for NACK after the entire transfer completes.
	 * The Vita I2C controller accumulates NACK status in
	 * IRQ_STATUS bit 15 across all phases of a transaction.
	 */
	if (vita_i2c_got_nack(i2c))
		return -ENXIO;

	return num;
}

static u32 vita_i2c_functionality(struct i2c_adapter *adap)
{
	/*
	 * Exclude I2C_FUNC_SMBUS_QUICK: the Vita I2C controller does not
	 * reliably generate NACK for zero-length transfers (quick write).
	 * i2cdetect must use read mode (-r) to get correct results.
	 */
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_adapter_quirks vita_i2c_quirks = {
	.max_read_len	= VITA_I2C_MAX_XFER_LEN,
	.max_write_len	= VITA_I2C_MAX_XFER_LEN,
};

static const struct i2c_algorithm vita_i2c_algo = {
	.xfer		= vita_i2c_xfer,
	.functionality	= vita_i2c_functionality,
};

static int vita_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vita_i2c *i2c;
	struct reset_control *rst;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->dev = dev;

	ret = of_property_read_u32(dev->of_node, "vita,bus-index",
				   &i2c->bus_index);
	if (ret) {
		dev_err(dev, "missing vita,bus-index property\n");
		return ret;
	}

	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	/* Enable clock gate (no clock driver — raw MMIO like sdhci-vita) */
	vita_i2c_clock_gate_enable(i2c);

	/* Deassert reset via reset controller */
	rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rst)) {
		dev_err(dev, "failed to get reset control: %ld\n",
			PTR_ERR(rst));
		return PTR_ERR(rst);
	}
	ret = reset_control_deassert(rst);
	if (ret) {
		dev_err(dev, "failed to deassert reset: %d\n", ret);
		return ret;
	}

	/* Initialize hardware */
	ret = vita_i2c_hw_init(i2c);
	if (ret)
		return ret;

	/* Register I2C adapter */
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &vita_i2c_algo;
	i2c->adap.quirks = &vita_i2c_quirks;
	i2c->adap.dev.parent = dev;
	i2c->adap.dev.of_node = dev->of_node;
	i2c->adap.timeout = msecs_to_jiffies(1000);
	strscpy(i2c->adap.name, "Vita I2C", sizeof(i2c->adap.name));
	i2c_set_adapdata(&i2c->adap, i2c);
	platform_set_drvdata(pdev, i2c);

	ret = i2c_add_adapter(&i2c->adap);
	if (ret) {
		dev_err(dev, "failed to add I2C adapter: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Vita I2C%d adapter registered\n", i2c->bus_index);
	return 0;
}

static void vita_i2c_remove(struct platform_device *pdev)
{
	struct vita_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);
}

static const struct of_device_id vita_i2c_of_match[] = {
	{ .compatible = "vita,i2c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vita_i2c_of_match);

static struct platform_driver vita_i2c_driver = {
	.probe		= vita_i2c_probe,
	.remove_new	= vita_i2c_remove,
	.driver = {
		.name		= "i2c-vita",
		.of_match_table	= vita_i2c_of_match,
	},
};
module_platform_driver(vita_i2c_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita I2C bus controller driver");
MODULE_LICENSE("GPL");
