// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 Sergi Granell

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/vita-syscon.h>
#include <linux/reboot.h>

/* From sdhci-vita.c -- full SDHCI re-init after WiFi power change */
void __weak sdhci_vita_reinit_host(int bus_index)
{
	pr_warn_once("vita-syscon: sdhci_vita_reinit_host not available\n");
}

/* From sdhci-vita.c -- trigger MMC card detect rescan */
void __weak sdhci_vita_trigger_rescan(int bus_index)
{
	pr_warn_once("vita-syscon: sdhci_vita_trigger_rescan not available\n");
}

/*
 * WiFi (SD8787 "Robin") power control via Ernie syscon commands.
 *
 * VitaOS controls the SD8787's power through Ernie rather than direct GPIO:
 *   - cmd 0x88A: wireless power (data: 0=off, 1=on)
 *   - cmd 0x88F: device reset (data: device_mask | mode<<8)
 *     device_mask 0x10 = WLANBT, mode: 0=assert reset, 1=de-assert
 *
 * The SD8787 also requires a 27MHz reference clock from the P1P40167
 * clockgen chip, which is connected via I2C bus 0 (address 0x69).
 *
 * Power-up: enable clock, power on, de-assert reset.
 * Power-down is the reverse: assert reset, power off, disable clock.
 */
#define SYSCON_CMD_WIRELESS_POWER	0x88A
#define SYSCON_CMD_DEVICE_RESET		0x88F
#define SYSCON_DEVICE_RESET_WLANBT	0x10

/*
 * P1P40167 clockgen -- raw MMIO I2C access.
 *
 * The clockgen is a CY27040-compatible clock generator on I2C bus 0.
 * It has 3 registers (0=revision, 1=clock control, 2=spread spectrum).
 * The CY27040 write protocol uses command bytes: reg N -> cmd byte (N - 128),
 * i.e. register 1 -> 0x81, not 0x01. This is why earlier raw I2C attempts
 * with register index 0x01 as the command byte failed silently.
 *
 * Register 1 bit assignments (Vita-specific, from PSP uofw + RE):
 *   bit 0: audio frequency select (0=44100, 1=48000)
 *   bit 2: MotionClk enable (confirmed by henkaku wiki)
 *   bit 3: WlanBtClk enable (27MHz buffered oscillator -- inferred from
 *           register reads showing 0xF7 after VitaOS disables WlanBt)
 *   bit 4: AudioClk enable
 */
#define VITA_I2C0_BASE			0xE0500000
#define VITA_I2C0_SIZE			0x1000
#define VITA_PERVASIVE_GATE_BASE	0xE3102000
#define VITA_PERVASIVE_RESET_BASE	0xE3101000
#define VITA_PERVASIVE_I2C0_OFF		0x110

#define CLOCKGEN_I2C_ADDR_7BIT		0x69	/* 8-bit: 0xD2 */
#define CLOCKGEN_REG_CLOCK		1
#define CLOCKGEN_CMD_REG(n)		((u8)((n) - 128))  /* reg 1 -> 0x81 */
#define CLOCKGEN_WLANBT_BIT		BIT(3)

/* I2C register offsets (indexed as u32 words) */
#define I2C_REG_WRITE_FIFO		0	/* [0] */
#define I2C_REG_READ_FIFO		1	/* [1] */
#define I2C_REG_UNK2			2	/* [2] */
#define I2C_REG_UNK3			3	/* [3] */
#define I2C_REG_DEVICE_ADDR		4	/* [4] */
#define I2C_REG_FLAGS			5	/* [5] */
#define I2C_REG_SPEED			6	/* [6] */
#define I2C_REG_BUSY			7	/* [7] */
#define I2C_REG_IRQ_STATUS		0xA	/* [0xA] */
#define I2C_REG_IRQ_CONTROL		0xB	/* [0xB] */

static void __iomem *i2c0_base;

static void vita_i2c0_wait_busy(void)
{
	int timeout = 10000;

	while (readl(i2c0_base + I2C_REG_BUSY * 4) && --timeout > 0)
		udelay(1);
	if (!timeout)
		pr_warn("vita-clockgen: I2C0 busy timeout\n");
}

static void vita_i2c0_init(void)
{
	void __iomem *gate, *reset;

	/* Enable I2C0 clock and take out of reset via pervasive registers */
	gate = ioremap(VITA_PERVASIVE_GATE_BASE + VITA_PERVASIVE_I2C0_OFF, 4);
	if (gate) {
		writel(readl(gate) | 1, gate);
		iounmap(gate);
	}
	reset = ioremap(VITA_PERVASIVE_RESET_BASE + VITA_PERVASIVE_I2C0_OFF, 4);
	if (reset) {
		writel(readl(reset) & ~1, reset);
		iounmap(reset);
	}
	udelay(100);

	i2c0_base = ioremap(VITA_I2C0_BASE, VITA_I2C0_SIZE);
	if (!i2c0_base) {
		pr_err("vita-clockgen: failed to ioremap I2C0\n");
		return;
	}

	/* Initialize I2C bus 0 (from vita-libbaremetal reference) */
	writel(0x100F70F, i2c0_base + I2C_REG_IRQ_CONTROL * 4);
	writel(1, i2c0_base + I2C_REG_UNK2 * 4);
	writel(1, i2c0_base + I2C_REG_UNK3 * 4);
	writel(7, i2c0_base + I2C_REG_FLAGS * 4);  /* reset bus */
	mb();

	vita_i2c0_wait_busy();

	/* Clear pending IRQs */
	writel(readl(i2c0_base + I2C_REG_IRQ_STATUS * 4),
	       i2c0_base + I2C_REG_IRQ_STATUS * 4);
	writel(0x1000000, i2c0_base + I2C_REG_IRQ_CONTROL * 4);

	writel(4, i2c0_base + I2C_REG_SPEED * 4);
}

static int vita_i2c0_write(u8 addr_7bit, const u8 *buf, int len)
{
	int i;

	if (!i2c0_base)
		return -ENODEV;

	writel(1, i2c0_base + I2C_REG_UNK2 * 4);
	writel(1, i2c0_base + I2C_REG_UNK3 * 4);
	writel(addr_7bit, i2c0_base + I2C_REG_DEVICE_ADDR * 4);

	for (i = 0; i < len; i++)
		writel(buf[i], i2c0_base + I2C_REG_WRITE_FIFO * 4);

	writel((len << 8) | 2, i2c0_base + I2C_REG_FLAGS * 4);
	vita_i2c0_wait_busy();

	writel(4, i2c0_base + I2C_REG_FLAGS * 4);  /* stop */
	vita_i2c0_wait_busy();

	return 0;
}

static int vita_i2c0_write_read(u8 addr_7bit, const u8 *wbuf, int wlen,
				u8 *rbuf, int rlen)
{
	int i;

	if (!i2c0_base)
		return -ENODEV;

	/* Write phase */
	writel(1, i2c0_base + I2C_REG_UNK2 * 4);
	writel(1, i2c0_base + I2C_REG_UNK3 * 4);
	writel(addr_7bit, i2c0_base + I2C_REG_DEVICE_ADDR * 4);

	for (i = 0; i < wlen; i++)
		writel(wbuf[i], i2c0_base + I2C_REG_WRITE_FIFO * 4);

	writel((wlen << 8) | 2, i2c0_base + I2C_REG_FLAGS * 4);
	vita_i2c0_wait_busy();

	/* Repeated start for read */
	writel(5, i2c0_base + I2C_REG_FLAGS * 4);
	vita_i2c0_wait_busy();

	writel((rlen << 8) | 0x13, i2c0_base + I2C_REG_FLAGS * 4);
	vita_i2c0_wait_busy();

	for (i = 0; i < rlen; i++)
		rbuf[i] = readl(i2c0_base + I2C_REG_READ_FIFO * 4);

	writel(4, i2c0_base + I2C_REG_FLAGS * 4);  /* stop */
	vita_i2c0_wait_busy();

	return 0;
}

/*
 * Read a single clockgen register using the CY27040 protocol.
 * Send cmd byte, then read 1 byte back.
 */
static int vita_clockgen_read_reg(u8 reg, u8 *val)
{
	u8 cmd = CLOCKGEN_CMD_REG(reg);

	return vita_i2c0_write_read(CLOCKGEN_I2C_ADDR_7BIT, &cmd, 1, val, 1);
}

/*
 * Write a single clockgen register using the CY27040 protocol.
 * Send: [cmd_byte, value] where cmd_byte = (reg_index - 128).
 */
static int vita_clockgen_write_reg(u8 reg, u8 val)
{
	u8 buf[2] = { CLOCKGEN_CMD_REG(reg), val };

	return vita_i2c0_write(CLOCKGEN_I2C_ADDR_7BIT, buf, 2);
}

static int vita_clockgen_wlanbt_enable(struct device *dev)
{
	u8 reg_val;
	int ret;

	ret = vita_clockgen_read_reg(CLOCKGEN_REG_CLOCK, &reg_val);
	if (ret)
		return ret;

	reg_val |= CLOCKGEN_WLANBT_BIT;

	ret = vita_clockgen_write_reg(CLOCKGEN_REG_CLOCK, reg_val);
	if (ret)
		return ret;

	/* Verify the write took effect */
	ret = vita_clockgen_read_reg(CLOCKGEN_REG_CLOCK, &reg_val);
	if (ret)
		return ret;

	if (!(reg_val & CLOCKGEN_WLANBT_BIT)) {
		dev_err(dev, "clockgen: WlanBt bit did not stick!\n");
		return -EIO;
	}

	return 0;
}

static int vita_clockgen_wlanbt_disable(struct device *dev)
{
	u8 reg_val;
	int ret;

	ret = vita_clockgen_read_reg(CLOCKGEN_REG_CLOCK, &reg_val);
	if (ret)
		return ret;

	reg_val &= ~CLOCKGEN_WLANBT_BIT;

	return vita_clockgen_write_reg(CLOCKGEN_REG_CLOCK, reg_val);
}

static ssize_t wlan_power_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct vita_syscon *syscon = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", syscon->wlan_power);
}

static ssize_t wlan_power_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct vita_syscon *syscon = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	val = !!val;

	if (val == syscon->wlan_power)
		return count;

	if (val) {
		/* Power on sequence (matches VitaOS boot order):
		 * 1. Disable SDIF2 interrupts to prevent premature MMC detect
		 * 2. Enable 27MHz WlanBt clock from clockgen
		 * 3. Power on wireless via Ernie
		 * 4. De-assert WLANBT reset via Ernie
		 * 5. SDHCI re-init (pervasive teardown/rebuild, sets I/O voltage)
		 * 6. Trigger MMC rescan (CMD5 etc will be sent by MMC core)
		 *
		 * We must disable SDHCI interrupts BEFORE powering on to prevent
		 * the MMC core from racing to detect the card at the wrong voltage.
		 */

		/* Step 1: Disable SDIF2 interrupts to prevent premature detect */
		{
			void __iomem *sdif2 = ioremap(0xE0C10000, 0x100);
			if (sdif2) {
				writel(0, sdif2 + 0x34);  /* SDHCI_INT_ENABLE */
				writel(0, sdif2 + 0x38);  /* SDHCI_SIGNAL_ENABLE */
				writel(0xFFFFFFFF, sdif2 + 0x30);  /* Clear pending */
				iounmap(sdif2);
			}
		}

		/* Step 2: Enable 27MHz WlanBt clock from clockgen */
		ret = vita_clockgen_wlanbt_enable(dev);
		if (ret)
			dev_warn(dev, "wlan: clockgen enable failed: %d\n", ret);
		msleep(10);

		/* Step 3: Power on wireless via Ernie */
		ret = syscon->short_command_write(syscon,
			SYSCON_CMD_WIRELESS_POWER, 1, 2);
		if (ret) {
			dev_err(dev, "wlan: power on failed: %d\n", ret);
			return ret;
		}
		msleep(50);

		/* Step 4: De-assert WLANBT reset via Ernie
		 * vita-libbaremetal packs: mode | (device << 8)
		 * mode=1 (de-assert), device=0x10 (WLANBT)
		 */
		ret = syscon->short_command_write(syscon,
			SYSCON_CMD_DEVICE_RESET,
			1 | (SYSCON_DEVICE_RESET_WLANBT << 8), 3);
		if (ret) {
			dev_err(dev, "wlan: reset de-assert failed: %d\n", ret);
			return ret;
		}
		msleep(20);

		/* Step 5: SDHCI re-init (pervasive teardown/rebuild, 1.8V I/O) */
		sdhci_vita_reinit_host(2);

		/* Step 6: Trigger MMC rescan -- MMC core sends CMD5 etc */
		sdhci_vita_trigger_rescan(2);
	} else {
		/* Power off sequence (matches VitaOS shutdown order):
		 * 1. Assert WLANBT reset
		 * 2. Power off wireless
		 * 3. Disable 27MHz clock
		 */
		ret = syscon->short_command_write(syscon,
			SYSCON_CMD_DEVICE_RESET,
			0 | (SYSCON_DEVICE_RESET_WLANBT << 8), 3);
		if (ret) {
			dev_err(dev, "wlan: reset assert failed: %d\n", ret);
			return ret;
		}
		msleep(100);

		ret = syscon->short_command_write(syscon,
			SYSCON_CMD_WIRELESS_POWER, 0, 2);
		if (ret) {
			dev_err(dev, "wlan: power off failed: %d\n", ret);
			return ret;
		}

		vita_clockgen_wlanbt_disable(dev);
	}

	syscon->wlan_power = val;

	return count;
}

static DEVICE_ATTR_RW(wlan_power);

static struct attribute *vita_syscon_attrs[] = {
	&dev_attr_wlan_power.attr,
	NULL,
};
ATTRIBUTE_GROUPS(vita_syscon);

static const struct mfd_cell vita_syscon_devs[] = {
	{
		.name = "vita-syscon-ts",
		.of_compatible = "vita,syscon-ts"
	},
	{
		.name = "vita-syscon-buttons",
		.of_compatible = "vita,syscon-buttons"
	},
	{
		.name = "vita-syscon-rtc",
		.of_compatible = "vita,syscon-rtc"
	},
};

static inline void syscon_set_tx_gpio(struct vita_syscon *syscon, int is_on)
{
	gpiod_set_value_cansleep(syscon->tx_gpio, is_on);
}

static inline void syscon_calc_checksum(u8 *tx, int data_size)
{
	int i;
	u32 hash = 0;

	for (i = 0; i < data_size; i++)
		hash += tx[i];

	tx[data_size] = ~hash;
}

static int vita_syscon_transfer(struct vita_syscon *syscon, u8 *tx, void *rx, int rx_size)
{
	struct spi_message msg;
	struct spi_transfer tx_xfer, rx_xfer;
	struct spi_device *spi = syscon->spi;
	unsigned int timeout;
	int ret = 0;
	int tx_size = SYSCON_TX_HEADER_SIZE + tx[SYSCON_TX_LENGTH];
	u8 res;

	memset(&tx_xfer, 0, sizeof(tx_xfer));
	memset(&rx_xfer, 0, sizeof(rx_xfer));

	tx_xfer.tx_buf = tx;
	tx_xfer.len = tx_size;
	rx_xfer.rx_buf = rx;
	rx_xfer.len = rx_size;

	syscon_calc_checksum(tx, tx_size - 1);

	spi_bus_lock(spi->controller);

	do {
		reinit_completion(&syscon->rx_irq);

		syscon_set_tx_gpio(syscon, true);

		/* Send data */
		spi_message_init(&msg);
		spi_message_add_tail(&tx_xfer, &msg);
		ret = spi_sync_locked(spi, &msg);
		if (ret < 0) {
			syscon_set_tx_gpio(syscon, false);
			goto out;
		}

		syscon_set_tx_gpio(syscon, false);

		/* Wait RX interrupt */
		timeout = wait_for_completion_timeout(&syscon->rx_irq,
						      msecs_to_jiffies(250));
		if (!timeout) {
			dev_warn(&spi->dev, "%s: RX IRQ timeout", dev_name(&spi->dev));
			ret = -ETIMEDOUT;
			goto out;
		}

		/* Receive data */
		spi_message_init(&msg);
		spi_message_add_tail(&rx_xfer, &msg);
		ret = spi_sync_locked(spi, &msg);
		if (ret < 0)
			goto out;
		res = ((u8 *)rx)[SYSCON_RX_RESULT];
	} while (res == 0x80 || res == 0x81);

out:
	spi_bus_unlock(spi->controller);

	return ret;
}

static int vita_syscon_command_read(struct vita_syscon *syscon, u16 cmd, void *rx, int rx_size)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + 1];

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = 1;

	return vita_syscon_transfer(syscon, tx, rx, rx_size);
}

static int vita_syscon_short_command_write(struct vita_syscon *syscon, u16 cmd, u32 data, int cmd_len)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + sizeof(u32) + 1];
	u8 rx[32];

	tx[SYSCON_TX_CMD_LO] = cmd & 0xFF;
	tx[SYSCON_TX_CMD_HI] = (cmd >> 8) & 0xFF;
	tx[SYSCON_TX_LENGTH] = cmd_len;

	tx[SYSCON_TX_DATA(0)] = data & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (data >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = (data >> 16) & 0xFF;
	tx[SYSCON_TX_DATA(3)] = (data >> 24) & 0xFF;

	return vita_syscon_transfer(syscon, tx, rx, sizeof(rx));
}

static int vita_syscon_scratchpad_read(struct vita_syscon *syscon, u16 offset, void *buffer, int size)
{
	u8 tx[SYSCON_TX_HEADER_SIZE + 3 + 1];
	u8 rx[32];
	int ret;

	if ((size - 1 >= 0x18) || ((size + offset) >= 0x101))
		return -EINVAL;

	tx[SYSCON_TX_CMD_LO] = 0x90;
	tx[SYSCON_TX_CMD_HI] = 0;
	tx[SYSCON_TX_LENGTH] = 4;

	tx[SYSCON_TX_DATA(0)] = offset & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (offset >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = size;

	ret = vita_syscon_transfer(syscon, tx, rx, sizeof(rx));
	if (ret < 0)
		return ret;

	memcpy(buffer, &rx[SYSCON_RX_DATA], size);

	return 0;
}

static int vita_syscon_scratchpad_write(struct vita_syscon *syscon, u16 offset, const void *buffer, int size)
{
	u8 tx[32];
	u8 rx[32];

	if ((size - 1 >= 0x18) || ((size + offset) >= 0x101))
		return -EINVAL;

	tx[SYSCON_TX_CMD_LO] = 0x91;
	tx[SYSCON_TX_CMD_HI] = 0;
	tx[SYSCON_TX_LENGTH] = size + 4;

	tx[SYSCON_TX_DATA(0)] = offset & 0xFF;
	tx[SYSCON_TX_DATA(1)] = (offset >> 8) & 0xFF;
	tx[SYSCON_TX_DATA(2)] = size;
	memcpy(&tx[SYSCON_TX_DATA(3)], buffer, size);

	return vita_syscon_transfer(syscon, tx, rx, sizeof(rx));
}

/*
 * Power management via direct SPI to Ernie.
 *
 * The TrustZone Secure Monitor (SMC 0x11A) normally handles these commands,
 * but after Linux reconfigures the GIC and SPI controller, the Monitor can
 * no longer function.  We replicate the TZ handler's SPI packets directly.
 *
 * From RE of VitaOS SceSyscon and henkaku wiki sceSysconSetPowerModeForDriver:
 *
 *   type=0 (power off):  cmd=0x00C0, data={type, ~mode_lo, ~mode_hi, mode_upper}
 *   type=1 (suspend):    cmd=0x00C0, data={type, ~mode_lo, ~mode_hi, mode_upper}
 *   type=2 (cold reset): cmd=0x0801, data={0x00}
 *   type=3 (ext boot):   cmd=0x00C1, data={0x00}
 *   type=4 (update):     cmd=0x00C1, data={0x01}
 *   type=5 (hibernate):  cmd=0x00C2, data={0x5A}
 *
 * For cmd 0x00C0 the mode bytes are bit-inverted (~mode) in the payload.
 * Mode 0x2 = software-initiated, 0x8002 = UDC/BT driver initiated.
 */
static int vita_syscon_set_power_mode(struct vita_syscon *syscon,
				      u8 type, u32 mode)
{
	u32 data;

	data = (u32)type |
	       ((~mode & 0xFF) << 8) |
	       (((~mode >> 8) & 0xFF) << 16) |
	       (((mode >> 16) & 0xFF) << 24);

	return vita_syscon_short_command_write(syscon, 0x00C0, data, 5);
}

static int vita_syscon_reboot_notify(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct vita_syscon *syscon = container_of(nb, struct vita_syscon,
						  reboot_nb);
	int ret;

	if (action != SYS_RESTART && action != SYS_POWER_OFF &&
	    action != SYS_HALT)
		return NOTIFY_DONE;

	/*
	 * Power off peripherals so VitaOS (on reboot) or Ernie (on poweroff)
	 * finds controllers in a clean state.  Without this, the Sony memory
	 * card (MSIF) stays half-initialized and VitaOS can't mount it.
	 */
	vita_syscon_short_command_write(syscon, 0x89B, 0, 2);  /* MSIF */
	vita_syscon_short_command_write(syscon, 0x888, 0, 2);  /* game card */

	if (syscon->wlan_power) {
		vita_syscon_short_command_write(syscon, SYSCON_CMD_DEVICE_RESET,
			0 | (SYSCON_DEVICE_RESET_WLANBT << 8), 3);
		vita_syscon_short_command_write(syscon, SYSCON_CMD_WIRELESS_POWER,
			0, 2);
		vita_clockgen_wlanbt_disable(&syscon->spi->dev);
	}

	if (action == SYS_POWER_OFF || action == SYS_HALT) {
		/* Power off: type=0, mode=0x2 (software-initiated) */
		ret = vita_syscon_set_power_mode(syscon, 0, 0x2);
		if (ret)
			pr_emerg("vita-syscon: poweroff failed: %d\n", ret);
		return NOTIFY_DONE;
	}

	/* Cold reset */
	ret = vita_syscon_short_command_write(syscon, 0x0801, 0x00, 2);
	if (ret)
		pr_emerg("vita-syscon: cold reset failed: %d\n", ret);

	mdelay(5000);

	return NOTIFY_DONE;
}

static irqreturn_t vita_syscon_rx_gpio_irq_handler(int irq, void *dev_id)
{
	struct vita_syscon *syscon = dev_id;

	complete(&syscon->rx_irq);

	return IRQ_HANDLED;
}

static int vita_syscon_probe(struct spi_device *spi)
{
	struct vita_syscon *syscon;
	u8 baryon_version[SYSCON_RX_HEADER_SIZE + sizeof(syscon->baryon_version)];
	u8 hw_info[SYSCON_RX_HEADER_SIZE + sizeof(syscon->hardware_info)];
	u8 hw_flags[SYSCON_RX_HEADER_SIZE + sizeof(syscon->hardware_flags)];
	int ret, irq;

	pr_info("vita_syscon_probe\n");

	syscon = devm_kzalloc(&spi->dev, sizeof(struct vita_syscon),
				GFP_KERNEL);
	if (!syscon)
		return -ENOMEM;

	irq = of_irq_get(spi->dev.of_node, 0);
	if (irq <= 0)
		return irq ? irq : -ENODEV;

	ret = devm_request_irq(&spi->dev, irq, vita_syscon_rx_gpio_irq_handler, 0,
			       "vita-syscon-gpio-rx", syscon);
	if (ret) {
		dev_err(&spi->dev, "could not request IRQ: %d\n", ret);
		return ret;
	}

	syscon->tx_gpio = devm_gpiod_get(&spi->dev, "tx", GPIOD_OUT_LOW);
	if (IS_ERR(syscon->tx_gpio))
		return PTR_ERR(syscon->tx_gpio);

	init_completion(&syscon->rx_irq);

	spi_set_drvdata(spi, syscon);
	syscon->dev = &spi->dev;
	syscon->spi = spi;
	syscon->transfer = vita_syscon_transfer;
	syscon->command_read = vita_syscon_command_read;
	syscon->short_command_write = vita_syscon_short_command_write;
	syscon->scratchpad_read = vita_syscon_scratchpad_read;
	syscon->scratchpad_write = vita_syscon_scratchpad_write;

	ret = vita_syscon_command_read(syscon, 1, baryon_version, sizeof(baryon_version));
	if (ret < 0) {
		return ret;
	}
	memcpy(&syscon->baryon_version, &baryon_version[SYSCON_RX_DATA],
	       sizeof(syscon->baryon_version));

	pr_info("Vita Syscon Baryon version: 0x%X\n", syscon->baryon_version);

	if (syscon->baryon_version > 0x1000003)
		ret = vita_syscon_short_command_write(syscon, 0x80, 0x12, 3);
	else if (syscon->baryon_version > 0x70501)
		ret = vita_syscon_short_command_write(syscon, 0x80, 2, 3);
	if (ret < 0) {
		return ret;
	}

	ret = vita_syscon_command_read(syscon, 5, hw_info, sizeof(hw_info));
	if (ret < 0) {
		return ret;
	}
	memcpy(&syscon->hardware_info, &hw_info[SYSCON_RX_DATA], sizeof(syscon->hardware_info));

	pr_info("Vita Syscon HW info: 0x%X\n", syscon->hardware_info);

	ret = vita_syscon_command_read(syscon, 6, hw_flags, sizeof(hw_flags));
	if (ret < 0) {
		return ret;
	}
	memcpy(syscon->hardware_flags, &hw_flags[SYSCON_RX_DATA], sizeof(syscon->hardware_flags));

	/* Initialize raw I2C bus 0 for clockgen access */
	vita_i2c0_init();

	syscon->reboot_nb.notifier_call = vita_syscon_reboot_notify;
	syscon->reboot_nb.priority = 255;
	ret = register_reboot_notifier(&syscon->reboot_nb);
	if (ret)
		dev_warn(&spi->dev, "failed to register reboot notifier: %d\n", ret);

	return devm_mfd_add_devices(syscon->dev, PLATFORM_DEVID_NONE,
				    vita_syscon_devs, ARRAY_SIZE(vita_syscon_devs),
				    NULL, 0, NULL);
}

static const struct of_device_id vita_syscon_of_match[] = {
	{ .compatible = "vita,syscon" },
	{}
};

MODULE_DEVICE_TABLE(of, vita_syscon_of_match);

static struct spi_driver vita_syscon_driver = {
	.driver = {
		.name   = "vita-syscon",
		.of_match_table = vita_syscon_of_match,
		.dev_groups = vita_syscon_groups,
	},
	.probe = vita_syscon_probe,
};

module_spi_driver(vita_syscon_driver);

MODULE_AUTHOR("Sergi Granell");
MODULE_DESCRIPTION("PlayStation Vita's Syscon (Ernie) driver");
MODULE_LICENSE("GPL");
