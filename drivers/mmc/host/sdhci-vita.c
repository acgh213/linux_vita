// SPDX-License-Identifier: GPL-2.0-only
/*
 * SDHCI driver for the PlayStation Vita (SDIF controllers)
 *
 * The Vita SoC has 4 SDIF hosts using standard SDHCI registers:
 *   SDIF0 (0xE0B00000) - eMMC
 *   SDIF1 (0xE0C00000) - Game card
 *   SDIF2 (0xE0C10000) - WLAN/BT (Marvell SD8787 SDIO)
 *   SDIF3 (0xE0C20000) - microSD (SD2Vita mod)
 *
 * Clock gating and reset are managed via the "pervasive" registers:
 *   Gate:  0xE3102000 + 0xA0 + bus*4  (set bit 0 to enable)
 *   Reset: 0xE3101000 + 0xA0 + bus*4  (clear bit 0 to deassert)
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>


#include "sdhci.h"

/*
 * Keep a reference to each SDHCI host so other Vita drivers (e.g. syscon)
 * can trigger card detect changes after WiFi power-on.
 */
static struct sdhci_host *vita_sdif_hosts[4];

/* Exported to vita-syscon for WiFi power sequencing and SDIF1 diagnosis */
void sdhci_vita_reinit_host(int bus_index);
void sdhci_vita_trigger_rescan(int bus_index);
int sdhci_vita_read_present_state(int bus_index, u32 *state);
bool sdhci_vita_host_ready(int bus_index);

/* From vita-syscon.c -- raise the game-card rail before the host registers */
int vita_syscon_gamecard_power_on_boot(void);

int __weak vita_syscon_gamecard_power_on_boot(void)
{
	return -ENODEV;
}

#define PERVASIVE_GATE_BASE	0xE3102000
#define PERVASIVE_RESET_BASE	0xE3101000
#define PERVASIVE_MISC_BASE	0xE3100000
#define PERVASIVE_SDIF_OFFSET	0xA0

/*
 * SDIF bus indices:
 *   0 = eMMC (1.8V)
 *   1 = Game card (3.3V)
 *   2 = WLAN/BT (1.8V)
 *   3 = microSD (3.3V)
 *
 * The pervasive misc register at 0xE3100124 controls I/O voltage:
 *   bit N = 1 -> SDIF N uses 1.8V, bit N = 0 -> 3.3V
 *
 * Register 0xE3100310 has 3-bit fields per bus (bus*8 bit offset):
 *   WLAN/BT (bus 2) needs value 3 (bits 16-18 = 0x30000)
 *
 * These must be set BEFORE the SDHCI controller sends commands,
 * otherwise the I/O voltage mismatch causes all commands to time out.
 */
static void sdhci_vita_pervasive_init(struct device *dev, u32 bus_index)
{
	void __iomem *gate_reg, *reset_reg;
	u32 val;

	gate_reg = devm_ioremap(dev,
		PERVASIVE_GATE_BASE + PERVASIVE_SDIF_OFFSET + bus_index * 4, 4);
	reset_reg = devm_ioremap(dev,
		PERVASIVE_RESET_BASE + PERVASIVE_SDIF_OFFSET + bus_index * 4, 4);
	if (!gate_reg || !reset_reg) {
		dev_warn(dev, "SDIF%u: could not map pervasive regs\n", bus_index);
		return;
	}

	/*
	 * HW reset sequence from vita-libbaremetal:
	 * 1. Assert reset (set bit 0 in GATE register -- yes, GATE, not RESET)
	 * 2. Deassert reset (clear bit 0 in RESET register)
	 * 3. Enable clock gate (set bit 0 in GATE register)
	 */
	val = readl(gate_reg);
	val |= 1;
	writel(val, gate_reg);	/* assert reset via gate reg */
	udelay(100);

	val = readl(reset_reg);
	val &= ~1;
	writel(val, reset_reg);	/* deassert reset */

	val = readl(gate_reg);
	val |= 1;
	writel(val, gate_reg);	/* enable clock */
}

/**
 * sdhci_vita_reinit_host - full HW+SW reinit of an SDIF host after card power change
 * @bus_index: SDIF bus number (0-3), use 2 for WLAN
 *
 * Performs the exact sequence from vita-libbaremetal's sdif_reset(host, HW|SW)
 * followed by sdif_bus_voltage_select(). This is required after powering on
 * the SD8787 WiFi chip, because the SDHCI controller needs to be fully
 * re-initialized to communicate with the newly-powered card.
 *
 * For WLAN (bus 2) on SoC revision 0 (non-Dolce), the sequence is:
 *   1. Pervasive: assert reset, deassert reset, enable clock
 *   2. Pervasive: set 0x124 bit 2 = 1 (1.8V I/O for SDIF2)
 *   3. SDHCI: software reset all
 *   4. SDHCI: set timeout, clear interrupts, enable interrupt mask (including CARD_INT)
 *   5. SDHCI: bus voltage select (1.8V power, clock div 128, card clock enable)
 *   6. Trigger MMC core rescan
 */
/**
 * sdhci_vita_suppress_irqs - disable all SDHCI interrupts and clear pending
 * @bus_index: SDIF bus number (0-3)
 *
 * Called before WiFi power-on to prevent premature card-detect interrupts
 * at the wrong I/O voltage.  Uses the host's already-mapped ioaddr.
 */
void sdhci_vita_suppress_irqs(int bus_index)
{
	struct sdhci_host *host;

	if (bus_index < 0 || bus_index > 3 || !vita_sdif_hosts[bus_index])
		return;

	host = vita_sdif_hosts[bus_index];
	sdhci_writel(host, 0, SDHCI_INT_ENABLE);
	sdhci_writel(host, 0, SDHCI_SIGNAL_ENABLE);
	sdhci_writel(host, 0xFFFFFFFF, SDHCI_INT_STATUS);
}
EXPORT_SYMBOL_GPL(sdhci_vita_suppress_irqs);

/**
 * sdhci_vita_read_present_state - read SDHCI_PRESENT_STATE for SDIF1 diagnosis
 * @bus_index: SDIF bus number (0-3)
 * @state: receives the raw register value
 *
 * Bit 16 (card present) is the pivot of the SDIF1 bring-up discriminator table:
 * sdhci_vita_reinit_host() gates its whole power/voltage/clock-enable step on
 * it, so whether that bit ever asserts decides which failure branch we are in.
 * The running rootfs gives no reliable way to peek a controller register from
 * userspace (no guaranteed devmem applet, no debugfs node), so expose it here.
 *
 * Safe once the host is registered: sdhci_vita_pervasive_init() enables the
 * SDIF module clock at probe, independently of the card rail state.
 *
 * Returns 0, or -ENODEV if no host is registered for that bus.
 */
int sdhci_vita_read_present_state(int bus_index, u32 *state)
{
	struct sdhci_host *host;

	if (bus_index < 0 || bus_index > 3 || !vita_sdif_hosts[bus_index])
		return -ENODEV;

	host = vita_sdif_hosts[bus_index];
	*state = sdhci_readl(host, SDHCI_PRESENT_STATE);

	return 0;
}
EXPORT_SYMBOL_GPL(sdhci_vita_read_present_state);

/**
 * sdhci_vita_host_ready - is an SDIF host registered for this bus?
 * @bus_index: SDIF bus number (0-3)
 *
 * Callers that want to reinit or rescan a bus can ask first, instead of
 * calling the helpers and relying on their no-op path, which warns.
 *
 * Returns true if a host is registered.
 */
bool sdhci_vita_host_ready(int bus_index)
{
	if (bus_index < 0 || bus_index > 3)
		return false;

	return vita_sdif_hosts[bus_index] != NULL;
}
EXPORT_SYMBOL_GPL(sdhci_vita_host_ready);

void sdhci_vita_reinit_host(int bus_index)
{
	struct sdhci_host *host;
	void __iomem *ioaddr;
	void __iomem *gate_reg, *reset_reg, *misc_base;
	u32 enable, val;
	u16 clockctrl;
	u8 tmp8;
	int timeout;
	const u16 sdclk_freq = 128;

	if (bus_index < 0 || bus_index > 3 || !vita_sdif_hosts[bus_index]) {
		pr_warn("sdhci-vita: reinit_host: bus %d not available\n", bus_index);
		return;
	}

	host = vita_sdif_hosts[bus_index];
	ioaddr = host->ioaddr;

	/*
	 * Step 1: Pervasive HW reset -- matches vita-libbaremetal sdif_reset() HW path
	 * For SoC rev 0, non-Dolce, host >= WLAN_BT: just enter+exit reset, enable clock.
	 * No 0x110/0x310 writes needed for this SoC revision.
	 */
	gate_reg = ioremap(PERVASIVE_GATE_BASE + PERVASIVE_SDIF_OFFSET + bus_index * 4, 4);
	reset_reg = ioremap(PERVASIVE_RESET_BASE + PERVASIVE_SDIF_OFFSET + bus_index * 4, 4);
	if (gate_reg && reset_reg) {
		/*
		 * Pervasive reset sequence:
		 * First, do a full clock disable + assert reset to ensure clean state,
		 * then follow vita-libbaremetal's sequence.
		 *
		 * The SDIF2 controller was already init'd during probe, so we need
		 * to fully tear it down and bring it back up.
		 */

		/* First: disable clock gate (tear down) */
		val = readl(gate_reg);
		writel(val & ~1, gate_reg);
		mb();
		readl(gate_reg);
		dsb(sy);

		/* Assert reset */
		val = readl(reset_reg);
		writel(val | 1, reset_reg);
		mb();
		readl(reset_reg);
		dsb(sy);
		udelay(200);

		/* pervasive_reset_enter_sdif: set bit 0 in gate register */
		val = readl(gate_reg);
		writel(val | 1, gate_reg);
		mb();
		readl(gate_reg);
		dsb(sy);
		udelay(100);

		/* pervasive_reset_exit_sdif: clear bit 0 in reset register */
		val = readl(reset_reg);
		writel(val & ~1, reset_reg);
		mb();
		readl(reset_reg);
		dsb(sy);

		/* pervasive_clock_enable_sdif: set bit 0 in gate register */
		val = readl(gate_reg);
		writel(val | 1, gate_reg);
		mb();
		readl(gate_reg);
		dsb(sy);
	}
	if (gate_reg) iounmap(gate_reg);
	if (reset_reg) iounmap(reset_reg);

	/*
	 * Step 2: Set pervasive I/O voltage -- 0x124 bit N = 1 means 1.8V
	 * VitaOS/vita-libbaremetal sets WLAN_BT (bus 2) to 1.8V:
	 *   pervasive_sdif_misc_0x124(2, 1);
	 */
	misc_base = ioremap(PERVASIVE_MISC_BASE, 0x400);
	if (misc_base) {
		val = readl(misc_base + 0x124);
		if (bus_index == 2) {
			/* Set bit 2 = 1 for 1.8V */
			val |= BIT(2);
			writel(val, misc_base + 0x124);
			mb();
		}
		iounmap(misc_base);
	}

	/*
	 * Step 3: SDHCI software reset -- matches vita-libbaremetal sdif_reset() SW path
	 */
	writeb(SDHCI_RESET_ALL, ioaddr + SDHCI_SOFTWARE_RESET);
	timeout = 1000;
	while ((readb(ioaddr + SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) && --timeout > 0)
		udelay(10);
	if (!timeout)
		pr_warn("sdhci-vita: SDIF%d: SW reset timeout!\n", bus_index);

	/* Set timeout control = 14 (matches vita-libbaremetal) */
	writeb(14, ioaddr + SDHCI_TIMEOUT_CONTROL);

	/* Wait for card state stable -- SDHCI needs time after SW reset to
	 * detect the card. vita-libbaremetal uses a tight loop of 102 reads
	 * but we add a delay to ensure the debounce logic settles. */
	timeout = 200;
	do {
		if (readl(ioaddr + SDHCI_PRESENT_STATE) & BIT(17))  /* CARD_STATE_STABLE */
			break;
		udelay(100);
	} while (--timeout > 0);

	/* Additional wait for card presence to be detected by the SDHCI
	 * controller after reset. The SD8787 needs time to respond. */
	if (!(readl(ioaddr + SDHCI_PRESENT_STATE) & BIT(16))) {
		int cd_wait = 50;  /* up to 50ms */
		while (cd_wait-- > 0) {
			if (readl(ioaddr + SDHCI_PRESENT_STATE) & BIT(16))
				break;
			mdelay(1);
		}
	}

	/* Clear all interrupts, disable everything, then re-enable */
	writel(0, ioaddr + SDHCI_INT_ENABLE);
	writel(0, ioaddr + SDHCI_SIGNAL_ENABLE);
	writel(readl(ioaddr + SDHCI_INT_STATUS), ioaddr + SDHCI_INT_STATUS);

	timeout = 100;
	while (readl(ioaddr + SDHCI_INT_STATUS) != 0 && --timeout > 0)
		udelay(10);

	/* Enable interrupts -- same mask as vita-libbaremetal, including CARD_INT for WLAN */
	enable = SDHCI_INT_RESPONSE | SDHCI_INT_DATA_END |
		 SDHCI_INT_DMA_END | SDHCI_INT_SPACE_AVAIL |
		 SDHCI_INT_DATA_AVAIL | SDHCI_INT_CARD_INSERT |
		 SDHCI_INT_CARD_REMOVE;
	if (bus_index == 2)  /* WLAN_BT */
		enable |= SDHCI_INT_CARD_INT;
	enable |= SDHCI_INT_TIMEOUT | SDHCI_INT_CRC |
		  SDHCI_INT_END_BIT | SDHCI_INT_INDEX |
		  SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC |
		  SDHCI_INT_DATA_END_BIT | SDHCI_INT_AUTO_CMD_ERR |
		  SDHCI_INT_ADMA_ERROR;

	writel(enable, ioaddr + SDHCI_INT_ENABLE);
	/*
	 * Enable signal interrupts for command/data completion, but suppress
	 * card-insert/remove signals.  Without this, the pending card-insert
	 * status fires a GIC interrupt immediately and the MMC core races to
	 * detect the card before we've finished setting up bus voltage.
	 * Our caller will trigger mmc_detect_change() explicitly.
	 */
	writel(enable & ~(SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE),
	       ioaddr + SDHCI_SIGNAL_ENABLE);

	/*
	 * Step 4: Bus voltage select -- matches vita-libbaremetal sdif_bus_voltage_select()
	 * For WLAN: voltage = SDHCI_POWER_180 (0x0A), divisor 128 (375kHz)
	 */
	val = readl(ioaddr + SDHCI_PRESENT_STATE);

	if (val & BIT(16)) {  /* SDHCI_CARD_PRESENT */
		/* Clear power-on bit */
		tmp8 = readb(ioaddr + SDHCI_POWER_CONTROL);
		tmp8 &= ~SDHCI_POWER_ON;
		writeb(tmp8, ioaddr + SDHCI_POWER_CONTROL);

		/* Set voltage bits (1.8V for WLAN, 3.3V for others) */
		tmp8 = readb(ioaddr + SDHCI_POWER_CONTROL);
		if (bus_index == 2)
			tmp8 |= SDHCI_POWER_180;
		else
			tmp8 |= SDHCI_POWER_330;
		writeb(tmp8, ioaddr + SDHCI_POWER_CONTROL);

		/* Set power-on */
		tmp8 = readb(ioaddr + SDHCI_POWER_CONTROL);
		tmp8 |= SDHCI_POWER_ON;
		writeb(tmp8, ioaddr + SDHCI_POWER_CONTROL);

		/* Reset clock */
		writew(0, ioaddr + SDHCI_CLOCK_CONTROL);

		/* Set clock: divisor 128 + internal clock enable */
		clockctrl = SDHCI_CLOCK_INT_EN |
			    ((sdclk_freq & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT) |
			    (((sdclk_freq & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
			     << SDHCI_DIVIDER_HI_SHIFT);
		writew(clockctrl, ioaddr + SDHCI_CLOCK_CONTROL);

		/* Wait for internal clock stable */
		timeout = 10000;
		while (!(readw(ioaddr + SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_INT_STABLE) &&
		       --timeout > 0)
			udelay(10);
		if (!timeout)
			pr_warn("sdhci-vita: SDIF%d: internal clock never stabilised\n",
				bus_index);

		/* Enable card clock (vita-libbaremetal always enables for WLAN) */
		clockctrl = readw(ioaddr + SDHCI_CLOCK_CONTROL);
		clockctrl |= SDHCI_CLOCK_CARD_EN;
		writew(clockctrl, ioaddr + SDHCI_CLOCK_CONTROL);

		/* Set host control = 0 (1-bit bus, low speed) -- matches vita-libbaremetal */
		writeb(0, ioaddr + SDHCI_HOST_CONTROL);

		/* Let the card see at least 74 clock cycles after power+clock stable
		 * SDIO spec requires this before first command.
		 * At 375kHz, 74 clocks = ~200us. Use 1ms for safety. */
		mdelay(1);
	}

	/*
	 * The controller is now set up with 1.8V bus power, clock running at
	 * 375kHz (div 128), and interrupts enabled.  The MMC core will handle
	 * the actual SDIO enumeration (CMD5 etc.) when we trigger a rescan.
	 *
	 * Note: we don't send manual CMD0/CMD5 here -- that would race with
	 * the MMC core's own detect path which runs as soon as interrupts
	 * are re-enabled.
	 */
}
EXPORT_SYMBOL_GPL(sdhci_vita_reinit_host);

/**
 * sdhci_vita_trigger_rescan - trigger MMC card detection on a given SDIF bus
 * @bus_index: SDIF bus number (0-3), use 2 for WLAN
 *
 * Simple rescan trigger without full re-init. Use sdhci_vita_reinit_host()
 * for post-power-on initialization.
 */
void sdhci_vita_trigger_rescan(int bus_index)
{
	struct sdhci_host *host;

	if (bus_index < 0 || bus_index > 3 || !vita_sdif_hosts[bus_index])
		return;

	host = vita_sdif_hosts[bus_index];
	mmc_detect_change(host->mmc, msecs_to_jiffies(200));
}
EXPORT_SYMBOL_GPL(sdhci_vita_trigger_rescan);

static unsigned int sdhci_vita_get_max_clock(struct sdhci_host *host)
{
	return 48000000;
}

/*
 * Custom set_power for Vita SDHCI hosts.
 *
 * For SDIF2 (WLAN/BT), the card advertises 3.3V support in its OCR but the
 * Vita hardware runs the bus at 1.8V (controlled by pervasive register 0x124).
 * The MMC framework negotiates 3.3V and tries to set SDHCI_POWER_330 in the
 * POWER_CONTROL register.  We intercept this and always use SDHCI_POWER_180
 * for SDIF2, matching the actual hardware configuration.
 *
 * For other buses we defer to the standard SDHCI power setting.
 */
static void sdhci_vita_set_power(struct sdhci_host *host, unsigned char mode,
				 unsigned short vdd)
{
	int bus_index;

	/* Find our bus index */
	for (bus_index = 0; bus_index < 4; bus_index++) {
		if (vita_sdif_hosts[bus_index] == host)
			break;
	}

	if (bus_index == 2 && mode != MMC_POWER_OFF) {
		/* WLAN/BT: always use 1.8V regardless of negotiated voltage.
		 * MMC_VDD_165_195 = 0x80 = bit 7, so vdd=7 -> SDHCI_POWER_180.
		 */
		sdhci_set_power_noreg(host, mode, 7);
	} else {
		sdhci_set_power_noreg(host, mode, vdd);
	}
}

static const struct sdhci_ops sdhci_vita_ops = {
	.set_clock = sdhci_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.get_max_clock = sdhci_vita_get_max_clock,
	.set_power = sdhci_vita_set_power,
};

static int sdhci_vita_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct resource *iomem;
	void __iomem *ioaddr;
	u32 bus_index;
	u16 version;
	u32 caps, present;
	int irq, ret;

	ret = of_property_read_u32(pdev->dev.of_node, "vita,bus-index", &bus_index);
	if (ret) {
		dev_err(&pdev->dev, "missing vita,bus-index property\n");
		return ret;
	}
	if (bus_index > 3) {
		dev_err(&pdev->dev, "invalid bus-index %u\n", bus_index);
		return -EINVAL;
	}

	/*
	 * Raise the game-card rail before anything else, so the initialisation
	 * the MMC core performs after sdhci_add_host() meets a powered card.
	 * Previously the rail came up from the syscon driver's probe, which runs
	 * after this probe: the core's first command went out against a dead slot
	 * and burned sdhci's flat 10 s software timeout before the retry worked.
	 *
	 * -EPROBE_DEFER means the syscon is not probed yet; returning it orders
	 * this probe after the syscon, which is exactly the dependency we need.
	 */
	if (of_property_read_bool(pdev->dev.of_node,
				  "vita,gamecard-power-on-boot")) {
		ret = vita_syscon_gamecard_power_on_boot();
		if (ret == -EPROBE_DEFER)
			return ret;
		if (ret)
			dev_warn(&pdev->dev,
				 "game-card rail on at boot failed: %d\n", ret);
		else
			dev_info(&pdev->dev, "game-card rail raised before init\n");
	}

	/* Enable clock and deassert reset before touching SDHCI registers */
	sdhci_vita_pervasive_init(&pdev->dev, bus_index);

	/* Map SDHCI registers to read diagnostics regardless of IRQ */
	ioaddr = devm_platform_get_and_ioremap_resource(pdev, 0, &iomem);
	if (IS_ERR(ioaddr))
		return PTR_ERR(ioaddr);

	version = readw(ioaddr + SDHCI_HOST_VERSION);
	caps = readl(ioaddr + SDHCI_CAPABILITIES);
	present = readl(ioaddr + SDHCI_PRESENT_STATE);

	dev_info(&pdev->dev,
		 "SDIF%u @ 0x%08x: version 0x%04x, caps 0x%08x, caps1 0x%08x, present 0x%08x%s\n",
		 bus_index, (u32)iomem->start, version, caps,
		 readl(ioaddr + SDHCI_CAPABILITIES_1),
		 present,
		 (present & (1 << 16)) ? " [card present]" : " [no card]");
	dev_info(&pdev->dev,
		 "SDIF%u: base_clk=%uMHz timeout_clk=%u%s max_blk=%u%s%s%s%s%s\n",
		 bus_index,
		 (caps >> 8) & 0x3f,
		 caps & 0x3f,
		 (caps & (1 << 7)) ? "MHz" : "kHz",
		 512 << ((caps >> 16) & 0x3),
		 (caps & (1 << 21)) ? " HS" : "",
		 (caps & (1 << 22)) ? " SDMA" : "",
		 (caps & (1 << 19)) ? " ADMA2" : "",
		 (caps & (1 << 24)) ? " 3.3V" : "",
		 (caps & (1 << 26)) ? " 1.8V" : "");

	/* Check if we have an interrupt -- full SDHCI needs one */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0) {
		dev_info(&pdev->dev,
			 "SDIF%u: no IRQ assigned, hardware probe only\n",
			 bus_index);
		return 0;
	}

	/* Full SDHCI host registration */
	host = sdhci_alloc_host(&pdev->dev, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);

	host->ioaddr = ioaddr;
	host->irq = irq;
	host->ops = &sdhci_vita_ops;
	host->quirks = SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER |
		       SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		       SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		       SDHCI_QUIRK_BROKEN_CARD_DETECTION;

	/*
	 * Restrict OCR per bus to match Vita hardware voltage requirements.
	 * vita-libbaremetal sets:
	 *   SDIF0 (eMMC):   MMC_VDD_165_195  (1.8V)
	 *   SDIF1 (GC):     MMC_VDD_32_33 | MMC_VDD_33_34  (3.3V)
	 *   SDIF2 (WLAN):   MMC_VDD_165_195  (1.8V)
	 *   SDIF3 (microSD): MMC_VDD_32_33 | MMC_VDD_33_34  (3.3V)
	 *
	 * The SDHCI capabilities report both 3.3V and 1.8V for all buses,
	 * but the pervasive I/O voltage register (0x124) controls the actual
	 * pad voltage. Without this, the MMC core picks 3.3V for WLAN which
	 * causes all SDIO commands to time out.
	 */
	switch (bus_index) {
	case 0: /* eMMC -- 1.8V only */
		host->ocr_mask = MMC_VDD_165_195;
		break;
	case 2: /* WLAN/BT -- hardware I/O is 1.8V (pervasive 0x124 bit 2),
		 * but the SD8787 reports OCR voltage range 0xFF8000 (2.7V-3.6V)
		 * and does NOT advertise 1.65-1.95V support.  On the Vita, the
		 * pad voltage is forced by the pervasive register, not negotiated.
		 * We must advertise a voltage the card supports so the SDIO OCR
		 * handshake succeeds, but we always set SDHCI_POWER_180 in the
		 * POWER_CONTROL register (handled by our set_power override).
		 */
		host->ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34;
		break;
	case 1: /* Game card */
	case 3: /* microSD */
		host->ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34;
		break;
	}

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_free;

	/*
	 * Register in the global array BEFORE sdhci_add_host(), because
	 * sdhci_add_host() triggers mmc_power_up() which calls the pwrseq
	 * post_power_on callback, and that needs sdhci_vita_reinit_host()
	 * to find this host in vita_sdif_hosts[].
	 */
	vita_sdif_hosts[bus_index] = host;

	ret = sdhci_add_host(host);
	if (ret) {
		vita_sdif_hosts[bus_index] = NULL;
		goto err_free;
	}

	platform_set_drvdata(pdev, host);
	return 0;

err_free:
	sdhci_free_host(host);
	return ret;
}

static void sdhci_vita_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	int i;

	if (host) {
		/* Clear our reference */
		for (i = 0; i < 4; i++) {
			if (vita_sdif_hosts[i] == host)
				vita_sdif_hosts[i] = NULL;
		}
		sdhci_remove_host(host, 0);
		sdhci_free_host(host);
	}
}

static const struct of_device_id sdhci_vita_of_match[] = {
	{ .compatible = "vita,sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_vita_of_match);

static struct platform_driver sdhci_vita_driver = {
	.driver = {
		.name = "sdhci-vita",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_vita_of_match,
	},
	.probe = sdhci_vita_probe,
	.remove_new = sdhci_vita_remove,
};
module_platform_driver(sdhci_vita_driver);

MODULE_DESCRIPTION("SDHCI driver for PlayStation Vita");
MODULE_LICENSE("GPL v2");
