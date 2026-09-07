// SPDX-License-Identifier: GPL-2.0
/* Opt-in post-boot experiment, not an OHCI HCD. No probe-time MMIO. */
#include <linux/capability.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "pstv-ohci-gate-core.h"
#include "pstv-ohci-hcdgate.h"

#define GATE_BASE 0xe40e0200
#define GATE_SIZE 0x100
#define EHCI_BASE 0xe40e0000
#define GATE_SF BIT(2)
#define GATE_UE BIT(4)

struct gate_hcca {
	__le32 table[32];
	__le16 frame;
	__le16 pad;
	__le32 done;
	u8 reserved[120];
};

struct gate_session {
	struct platform_device *pdev;
	struct usb_hcd *hcd;
	struct usb_device *hub;
	bool controller_locked;
	bool hub_locked;
	bool pm_held;
	bool region;
	bool irq_requested;
	void __iomem *regs;
	void __iomem *ehci;
	struct gate_hcca *hcca;
	dma_addr_t dma;
	unsigned int irq;
	struct completion sof;
	atomic_t irq_count;
	atomic_t irq_errors;
	u32 fr_first;
	u32 fr_last;
	u32 control_stopped;
	u32 interrupts_stopped;
};

static DEFINE_MUTEX(gate_lock);
static bool poisoned;
static bool attempted;
static unsigned int last_stage;
static struct pstv_ohci_gate_result last_result;
static u32 last_fr_first, last_fr_last, last_control, last_interrupts;

static int gate_ehci_progress(struct gate_session *g)
{
	u32 command, status;

	if (!HCD_HW_ACCESSIBLE(g->hcd) ||
	    READ_ONCE(g->hcd->state) != HC_STATE_RUNNING)
		return -EHOSTDOWN;
	command = readl(g->ehci);
	status = readl(g->ehci + 4);
	if (!(command & BIT(0)) || (status & BIT(12)))
		return -EHOSTDOWN;
	g->fr_first = readl(g->ehci + 0x0c) & 0x3fff;
	usleep_range(2000, 3000);
	g->fr_last = readl(g->ehci + 0x0c) & 0x3fff;
	return g->fr_first == g->fr_last ? -ETIMEDOUT : 0;
}

static int gate_acquire(void *ctx)
{
	struct gate_session *g = ctx;
	struct device_node *power, *np;
	struct resource *res;
	u32 caplength;
	int ret, port;

	/* Only the PSTV DT has this node and its bus-zero phandle. */
	power = of_find_compatible_node(NULL, NULL, "vita,dolce-usb-power");
	if (!power)
		return -ENODEV;
	np = of_device_is_available(power) ?
		of_parse_phandle(power, "vita,ehci", 0) : NULL;
	of_node_put(power);
	if (!np)
		return -ENODEV;
	g->pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!g->pdev)
		return -ENODEV;

	device_lock(&g->pdev->dev);
	g->controller_locked = true;
	if (!g->pdev->dev.driver ||
	    strcmp(g->pdev->dev.driver->name, "ehci-platform"))
		return -ENODEV;
	res = platform_get_resource(g->pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != EHCI_BASE || resource_size(res) != 0x100)
		return -EINVAL;
	g->hcd = platform_get_drvdata(g->pdev);
	if (!g->hcd)
		return -ENODEV;
	usb_get_hcd(g->hcd);
	g->hub = usb_get_dev(g->hcd->self.root_hub);
	if (!g->hub)
		return -ENODEV;
	/* Parent before child; both remain locked through stop and DMA cleanup. */
	usb_lock_device(g->hub);
	g->hub_locked = true;
	if (g->hub->state == USB_STATE_NOTATTACHED || !g->hub->dev.driver)
		return -ENODEV;
	/* Controller-only experiment: do not disturb enumerated children. */
	for (port = 1; port <= g->hub->maxchild; port++)
		if (usb_hub_find_child(g->hub, port))
			return -EBUSY;
	/* EHCI platform PM is disabled here; USB root-hub PM is the real gate. */
	ret = pm_runtime_resume_and_get(&g->hub->dev);
	if (ret < 0)
		return ret;
	g->pm_held = true;
	caplength = readl(g->hcd->regs) & 0xff;
	if (caplength != 0x10 && caplength != 0x20)
		return -EINVAL;
	g->ehci = g->hcd->regs + caplength;
	ret = gate_ehci_progress(g);
	if (ret)
		return ret;
	msleep(100);
	ret = gate_ehci_progress(g);
	if (ret)
		return ret;
	if (!request_mem_region(GATE_BASE, GATE_SIZE, "pstv-ohci-gate"))
		return -EBUSY;
	g->region = true;
	g->regs = ioremap(GATE_BASE, GATE_SIZE);
	return g->regs ? 0 : -ENOMEM;
}

static int gate_read(void *ctx, unsigned int reg, u32 *value)
{
	struct gate_session *g = ctx;

	*value = readl(g->regs + reg);
	return *value == U32_MAX ? -EIO : 0;
}

static int gate_write(void *ctx, unsigned int reg, u32 value)
{
	struct gate_session *g = ctx;

	writel(value, g->regs + reg);
	return 0;
}

static int gate_sleep(void *ctx, unsigned int milliseconds)
{
	usleep_range(milliseconds * 1000, milliseconds * 1000 + 500);
	return 0;
}

static irqreturn_t gate_irq(int irq, void *data)
{
	struct gate_session *g = data;
	u32 status = readl(g->regs + PSTV_OHCI_INTR_STATUS);
	u32 enabled = readl(g->regs + PSTV_OHCI_INTR_ENABLE);

	if (!(enabled & BIT(31)) || !(status & enabled & 0x4000007f))
		return IRQ_NONE;
	writel(status & 0x4000007f, g->regs + PSTV_OHCI_INTR_STATUS);
	if (status & GATE_UE)
		atomic_inc(&g->irq_errors);
	if (status & GATE_SF) {
		atomic_inc(&g->irq_count);
		complete(&g->sof);
	}
	return IRQ_HANDLED;
}

/*
 * Map and validate the bus-zero OHCI interrupt. The frame stage and the HCD
 * hand-off share this; only the frame stage additionally requests it.
 */
static int gate_map_irq(struct gate_session *g)
{
	struct of_phandle_args args;
	struct irq_domain *domain;
	struct irq_data *irq_data;
	int ret;

	ret = of_irq_parse_one(g->pdev->dev.of_node, 0, &args);
	if (ret)
		return ret;
	/* Paired EHCI is SPI114; Sony's bus-zero OHCI is SPI113 / GIC145. */
	if (!of_device_is_compatible(args.np, "arm,cortex-a9-gic") ||
	    args.args_count != 3 || args.args[0] != 0 ||
	    args.args[1] != 114 || args.args[2] != IRQ_TYPE_LEVEL_HIGH) {
		ret = -EINVAL;
		goto out_node;
	}
	domain = irq_find_host(args.np);
	if (!domain) {
		ret = -ENODEV;
		goto out_node;
	}
	args.args[1] = 113;
	g->irq = irq_create_of_mapping(&args);
	if (!g->irq) {
		ret = -EINVAL;
		goto out_node;
	}
	irq_data = irq_get_irq_data(g->irq);
	if (!irq_data || irq_data->domain != domain || irq_data->hwirq != 145 ||
	    irq_get_trigger_type(g->irq) != IRQ_TYPE_LEVEL_HIGH) {
		ret = -EINVAL;
		goto out_node;
	}
 out_node:
	of_node_put(args.np);
	return ret;
}

static int gate_prepare_frame(void *ctx, u32 *dma)
{
	struct gate_session *g = ctx;
	int ret;

	static_assert(sizeof(struct gate_hcca) == 256);
	g->hcca = dma_alloc_coherent(&g->pdev->dev, sizeof(*g->hcca),
				     &g->dma, GFP_KERNEL);
	if (!g->hcca)
		return -ENOMEM;
	if (!g->dma || upper_32_bits(g->dma) || (g->dma & 0xff))
		return -EINVAL;
	memset(g->hcca, 0, sizeof(*g->hcca));
	dma_wmb();
	ret = gate_map_irq(g);
	if (ret)
		return ret;
	init_completion(&g->sof);
	atomic_set(&g->irq_count, 0);
	atomic_set(&g->irq_errors, 0);
	ret = request_irq(g->irq, gate_irq, 0, "pstv-ohci-gate", g);
	if (!ret)
		g->irq_requested = true;
	*dma = lower_32_bits(g->dma);
	return ret;
}

static int gate_wait_sof(void *ctx, unsigned int timeout_ms)
{
	struct gate_session *g = ctx;

	return wait_for_completion_timeout(&g->sof, msecs_to_jiffies(timeout_ms)) ?
		0 : -ETIMEDOUT;
}

static int gate_frame_progress(void *ctx, u32 *first, u32 *last)
{
	struct gate_session *g = ctx;
	u32 hardware_first, hardware_last;

	dma_rmb();
	*first = le16_to_cpu(READ_ONCE(g->hcca->frame));
	hardware_first = readl(g->regs + PSTV_OHCI_FRAME_NUMBER) & 0xffff;
	msleep(20);
	dma_rmb();
	*last = le16_to_cpu(READ_ONCE(g->hcca->frame));
	hardware_last = readl(g->regs + PSTV_OHCI_FRAME_NUMBER) & 0xffff;
	return hardware_first == hardware_last ? -ETIMEDOUT : 0;
}

static int gate_irq_count(void *ctx, u32 *count)
{
	struct gate_session *g = ctx;

	*count = atomic_read(&g->irq_count);
	return atomic_read(&g->irq_errors) ? -EIO : 0;
}

static int gate_quiescent(void *ctx)
{
	struct gate_session *g = ctx;
	u32 control, cmd, enabled, before = 0, after = 0;

	control = readl(g->regs + PSTV_OHCI_CONTROL);
	cmd = readl(g->regs + PSTV_OHCI_CMD_STATUS);
	enabled = readl(g->regs + PSTV_OHCI_INTR_ENABLE);
	g->control_stopped = control;
	g->interrupts_stopped = enabled;
	if (readl(g->regs + PSTV_OHCI_REVISION) != 0x10 ||
	    (control & 0x3c) || (control & 0xc0) == 0x80 ||
	    (control & 0xc0) == 0x40 || (cmd & PSTV_OHCI_HCR) ||
	    (enabled & PSTV_OHCI_IRQ_MASK))
		return -EBUSY;
	if (g->irq_requested)
		synchronize_irq(g->irq);
	/* Only after confirmed Reset/Suspend may we sever the DMA pointer. */
	writel(0, g->regs + PSTV_OHCI_HCCA);
	if (readl(g->regs + PSTV_OHCI_HCCA))
		return -EIO;
	if (g->hcca)
		before = le16_to_cpu(READ_ONCE(g->hcca->frame));
	msleep(20);
	dma_rmb();
	if (g->hcca)
		after = le16_to_cpu(READ_ONCE(g->hcca->frame));
	if (before != after)
		return -EBUSY;
	return gate_ehci_progress(g);
}

static void gate_release(void *ctx)
{
	struct gate_session *g = ctx;

	if (g->irq_requested) {
		free_irq(g->irq, g);
		g->irq_requested = false;
	}
	/*
	 * OF mappings are shared boot-lifetime bookkeeping. The creation helper
	 * may return another caller's mapping; only our exclusive IRQ action is
	 * owned here. Subsequent gates reuse this single GIC mapping.
	 */
	g->irq = 0;
	if (g->hcca) {
		dma_free_coherent(&g->pdev->dev, sizeof(*g->hcca), g->hcca, g->dma);
		g->hcca = NULL;
	}
}

static void gate_finish(void *ctx, int retain)
{
	struct gate_session *g = ctx;

	last_fr_first = g->fr_first;
	last_fr_last = g->fr_last;
	last_control = g->control_stopped;
	last_interrupts = g->interrupts_stopped;
	if (retain) {
		/*
		 * No devm allocation: unbind cannot free hardware-referenced DMA.
		 * Keep the region, mapping, device/HCD/USB references and PM count.
		 * Disable the CPU IRQ too, so later power changes cannot cause MMIO.
		 * This builtin gate cannot unload; all later commands are rejected.
		 */
		poisoned = true;
		if (g->irq_requested)
			disable_irq(g->irq);
		pr_err("pstv-ohci-gate: unproven stop; resources quarantined, physical recovery required\n");
	} else {
		gate_release(g);
		if (g->regs)
			iounmap(g->regs);
		if (g->region)
			release_mem_region(GATE_BASE, GATE_SIZE);
		if (g->pm_held) {
			pm_runtime_mark_last_busy(&g->hub->dev);
			pm_runtime_put_sync_autosuspend(&g->hub->dev);
		}
	}
	if (!retain) {
		if (g->hub_locked)
			usb_unlock_device(g->hub);
		if (g->controller_locked)
			device_unlock(&g->pdev->dev);
		usb_put_dev(g->hub);
		if (g->hcd)
			usb_put_hcd(g->hcd);
		if (g->pdev)
			put_device(&g->pdev->dev);
	}
}

static const struct pstv_ohci_gate_ops gate_ops = {
	.acquire = gate_acquire,
	.finish = gate_finish,
	.read = gate_read,
	.write = gate_write,
	.sleep_ms = gate_sleep,
	.prepare_frame = gate_prepare_frame,
	.wait_sof = gate_wait_sof,
	.frame_progress = gate_frame_progress,
	.irq_count = gate_irq_count,
	.quiescent = gate_quiescent,
	.release = gate_release,
};

/*
 * HCD hand-off: bring the controller through the same read/reset admission,
 * then register a real OHCI HCD on the mapped window. On success the HCD owns
 * the controller; the gate keeps the PM hold, region, mapping and references
 * alive until "hcd-down" removes it.
 */
static struct gate_session *hcd_gate_session;
static bool hcd_transition;

static bool gate_hcd_live(void)
{
	return hcd_gate_session || hcd_transition;
}

static int gate_hcd_bringup(struct gate_session *g)
{
	int ret;

	ret = gate_acquire(g);
	if (ret)
		return ret;
	ret = pstv_ohci_read_stage(&last_result, &gate_ops, g);
	if (ret)
		return ret;
	/* Do not take over an active controller or firmware-owned schedules. */
	if (last_result.control & (0x100 | 0x3c) ||
	    (last_result.control & 0xc0) == 0x80 ||
	    (last_result.control & 0xc0) == 0x40) {
		ret = -EBUSY;
		return ret;
	}
	ret = pstv_ohci_reset_stage(&last_result, &gate_ops, g);
	if (ret)
		return ret;
	ret = gate_map_irq(g);
	if (ret)
		return ret;
	ret = gate_hcd_up(g->pdev, g->regs, g->irq, g->hcd);
	if (ret)
		return ret;
	ret = gate_hcd_set_companion(g->hcd);
	if (ret) {
		gate_hcd_down();
		return ret;
	}
	/* usb_remove_hcd() must lock/disconnect root hubs at hcd-down. */
	usb_unlock_device(g->hub);
	g->hub_locked = false;
	return 0;
}

static void gate_hcd_cleanup(struct gate_session *g, int retain)
{
	last_result.cleanup_status = retain;
	last_result.quarantined = !!retain;
	gate_finish(g, retain);
}

static int gate_hcd_down_trigger(void)
{
	struct gate_session *g = hcd_gate_session;

	if (!g)
		return -EINVAL;
	hcd_transition = true;
	gate_hcd_down();
	/* hcd-down leaves the controller stopped by ohci_stop; release gate. */
	gate_finish(g, 0);
	kfree(g);
	hcd_gate_session = NULL;
	hcd_transition = false;
	attempted = true;
	last_stage = 4;
	last_result.status = 0;
	return 0;
}

static ssize_t gate_trigger(struct file *file, const char __user *buffer,
			    size_t count, loff_t *pos)
{
	struct gate_session *g;
	char command[16];
	unsigned int stage, sleep_flags;
	int ret;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (!count || count >= sizeof(command))
		return -EINVAL;
	if (copy_from_user(command, buffer, count))
		return -EFAULT;
	if (memchr(command, '\0', count))
		return -EINVAL;
	command[count] = '\0';
	if (sysfs_streq(command, "read"))
		stage = 0;
	else if (sysfs_streq(command, "reset"))
		stage = 1;
	else if (sysfs_streq(command, "frame"))
		stage = 2;
	else if (sysfs_streq(command, "hcd"))
		stage = 3;
	else if (sysfs_streq(command, "hcd-down"))
		stage = 4;
	else
		return -EINVAL;
	sleep_flags = lock_system_sleep();
	if (!mutex_trylock(&gate_lock)) {
		unlock_system_sleep(sleep_flags);
		return -EBUSY;
	}
	if (poisoned) {
		ret = -EIO;
		goto unlock;
	}
	if (stage == 4) {
		ret = gate_hcd_down_trigger();
		goto unlock;
	}
	if (stage == 3) {
		if (hcd_gate_session || gate_hcd_active()) {
			ret = -EBUSY;
			goto unlock;
		}
		g = kzalloc(sizeof(*g), GFP_KERNEL);
		if (!g) {
			ret = -ENOMEM;
			goto unlock;
		}
		hcd_transition = true;
		last_stage = 3;
		attempted = true;
		pr_info("pstv-ohci-gate: hcd bring-up begin\n");
		ret = gate_hcd_bringup(g);
		if (!ret) {
			hcd_gate_session = g;
			hcd_transition = false;
			last_result.status = 0;
			pr_info("pstv-ohci-gate: hcd live (controller handed to OHCI core)\n");
			goto unlock;
		}
		pr_info("pstv-ohci-gate: hcd bring-up failed rc=%d\n", ret);
		/*
		 * Mirror transaction cleanup: quarantine only when the stop
		 * itself is unprovable; a controller that was never started
		 * can release normally.
		 */
		if (last_result.phase >= PSTV_OHCI_PHASE_RESET) {
			int stop = pstv_ohci_stop_stage(&last_result, &gate_ops, g);

			gate_hcd_cleanup(g, !!stop);
		} else {
			gate_hcd_cleanup(g, 0);
		}
		if (!last_result.quarantined)
			kfree(g);
		hcd_transition = false;
		goto unlock;
	}
	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g) {
		ret = -ENOMEM;
		goto unlock;
	}
	last_stage = stage;
	attempted = true;
	pr_info("pstv-ohci-gate: stage=%u begin (post-boot opt-in)\n", stage);
	ret = pstv_ohci_transaction(&last_result, &gate_ops, g, stage);
	pr_info("pstv-ohci-gate: stage=%u rc=%d stop=%d poison=%u rev=%x irq=%u frame=%u->%u\n",
		stage, ret, last_result.cleanup_status, poisoned,
		last_result.revision, last_result.irq_count,
		last_result.frame_first, last_result.frame_last);
	if (!poisoned)
		kfree(g);
 unlock:
	mutex_unlock(&gate_lock);
	unlock_system_sleep(sleep_flags);
	if (ret)
		return ret;
	return count;
}

static int gate_result_show(struct seq_file *s, void *unused)
{
	mutex_lock(&gate_lock);
	seq_printf(s, "attempted=%u\nstage=%u\nstatus=%d\ncleanup_status=%d\npoisoned=%u\nphase=%u\n",
		   attempted, last_stage, last_result.status,
		   last_result.cleanup_status, poisoned, last_result.phase);
	seq_printf(s, "revision=%08x\ncontrol_before=%08x\ninterrupts_before=%08x\nfm_interval_before=%08x\nhcca_before=%08x\n",
		   last_result.revision, last_result.control,
		   last_result.intr_enable, last_result.fm_interval, last_result.hcca);
	seq_printf(s, "hcca_dma=%08x\nirq_count=%u\nframe_first=%u\nframe_last=%u\nehci_first=%u\nehci_last=%u\ncontrol_stopped=%08x\ninterrupts_stopped=%08x\n",
		   last_result.hcca_dma, last_result.irq_count,
		   last_result.frame_first, last_result.frame_last,
		   last_fr_first, last_fr_last, last_control, last_interrupts);
	mutex_unlock(&gate_lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gate_result);

static const struct file_operations gate_trigger_fops = {
	.owner = THIS_MODULE,
	.write = gate_trigger,
};

static int gate_pm_notify(struct notifier_block *nb, unsigned long event, void *p)
{
	int ret = NOTIFY_OK;

	if (event != PM_SUSPEND_PREPARE && event != PM_HIBERNATION_PREPARE &&
	    event != PM_RESTORE_PREPARE)
		return NOTIFY_DONE;
	if (!mutex_trylock(&gate_lock))
		return NOTIFY_BAD;
	if (poisoned || gate_hcd_live())
		ret = NOTIFY_BAD;
	mutex_unlock(&gate_lock);
	return ret;
}

static struct notifier_block gate_pm_nb = {
	.notifier_call = gate_pm_notify,
};

static int __init gate_init(void)
{
	struct dentry *dir;
	int ret;

	ret = register_pm_notifier(&gate_pm_nb);
	if (ret)
		return ret;
	dir = debugfs_create_dir("pstv-ohci-gate", NULL);
	if (IS_ERR(dir)) {
		unregister_pm_notifier(&gate_pm_nb);
		return PTR_ERR(dir);
	}
	debugfs_create_file("trigger", 0200, dir, NULL, &gate_trigger_fops);
	debugfs_create_file("result", 0400, dir, NULL, &gate_result_fops);
	return 0;
}
late_initcall(gate_init);
MODULE_LICENSE("GPL");
