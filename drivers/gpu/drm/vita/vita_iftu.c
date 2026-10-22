// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fixed inherited-mode IFTU scanout, with CPU shadow copies into two CDRAM
 * buffers. Only the vblank ISR switches plane A's selector. The loader still
 * owns clocks, timing, HDMI and the DSI interrupt mask; this is not cold modeset.
 */

#include <linux/atomic.h>
#include <linux/dma-fence.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/moduleparam.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_file.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_panic.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#define DRIVER_NAME	"vita-iftu"
#define DRIVER_DESC	"DRM driver for the PS Vita/PS TV IFTU display controller"
#define DRIVER_DATE	"20260830"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

#define VITA_IFTU_TRACE_LIMIT	64

/* Supervised M3-E only; ordinary boots retain the no-client baseline. */
static bool enable_fbdev;
module_param_named(fbdev, enable_fbdev, bool, 0400);
MODULE_PARM_DESC(fbdev, "Opt in to experimental IFTU framebuffer-console takeover");

/* IFTU control-block offsets, verified on PSTV bus 1 (0xE5032000). */
#define IFTU_CREG_CONTROL		0x00
#define IFTU_CREG_CONTROL_ENABLE	BIT(0)
#define IFTU_CREG_CONTROL2		0x04
#define IFTU_CREG_PLANE_SEL(plane)	(0x10 + (plane) * 0x08)

#define IFTU_PREG_SCANLINE		0x0008
#define IFTU_PREG_SCANOUT_BYTE		0x0030

/* DSI bus-1 interrupt block (relative to the "dsi" reg resource at
 * 0xE5060000). Hardware-verified 2026-08-30: status/ack +0x50 = 0xE
 * (write-to-clear), pending +0x48 = 0x1, enable +0x54 = 0x2 (vblank,
 * armed by the loader). See lab/gpu-re/IFTU-VBLANK-FINDING-2026-08-30.md.
 */
#define IFTU_DSI_INTR_PENDING		0x48
#define IFTU_DSI_INTR_STATUS		0x50
#define IFTU_DSI_INTR_ENABLE		0x54
#define IFTU_DSI_INTR_VBLANK		BIT(1)

/* Plane-config blocks, relative to the "planes" reg resource.
 * Two configs per plane at +0x200 + cfg*0x100; the control block's
 * PLANE_SEL selects which is scanned out. Hardware-verified live on
 * PSTV bus 1: config 0 @ 0xE5030200 (FB_PADDR=0x20000000,
 * SRC_PIXELFMT=0x10, SRC_FB_W=0x500, SRC_FB_H=0x2D0, DST_W/H same,
 * scale 0x10000) and config 1 @ 0xE5030300 (initially mirrors config 0).
 */
#define IFTU_PLANE_A_CFG(plane)		(0x200 + (plane) * 0x100)
#define IFTU_PCFG_FB_PADDR		0x00
#define IFTU_PCFG_SRC_X			0x24
#define IFTU_PCFG_SRC_Y			0x28
#define IFTU_PCFG_SRC_PIXELFMT		0x40
#define IFTU_PCFG_SRC_FB_W		0x44
#define IFTU_PCFG_SRC_FB_H		0x48
#define IFTU_PCFG_DST_W			0xA4
#define IFTU_PCFG_DST_H			0xA8
#define IFTU_PCFG_SRC_W			0xC0
#define IFTU_PCFG_SRC_H			0xC4
#define IFTU_PCFG_DST_X			0xC8
#define IFTU_PCFG_DST_Y			0xCC
#define IFTU_PIXFMT_A8B8G8R8		0x10
#define IFTU_SCALE_1X			0x10000

/* Second page-flip buffer: immediately after the inherited framebuffer
 * (1280x720x4 = 0x384000 bytes), page-aligned, inside CDRAM. ~0.1% of
 * the 128 MiB region. See DRM-M2-DESIGN-2026-09-01.md.
 */
#define VITA_IFTU_SECOND_FB_OFFSET	0x384000

struct vita_iftu_device {
	struct drm_device dev;

	/* Only the devicetree-selected bus is mapped; the other may be gated. */
	void __iomem *plane_regs;
	void __iomem *control_regs;
	void __iomem *dsi_regs;
	int irq;

	/* Vblank IRQ count, bumped in the ISR and reported via debugfs. */
	atomic64_t vblank_count;

	/* Inherited mode + framebuffer, exactly like simpledrm's
	 * system-memory path: fixed at probe time from devicetree, never
	 * changed by this driver (no modeset in M1).
	 */
	struct drm_display_mode mode;
	const struct drm_format_info *format;
	unsigned int pitch;
	struct iosys_map screen_base;

	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	/* Native ABGR8888 plus legacy XRGB8888 conversion for fbdev.
	 * Both must be explicit: the generic list helper removes native alpha.
	 */
	uint32_t formats[2];
	size_t nformats;

	/* Bounded callback trace for the supervised first-commit test. */
	atomic_t trace_count;

	/* All publication/ISR/lifecycle state is protected by flip_lock.
	 * Atomic helpers run in process context. Copy pixels outside the lock,
	 * then publish one complete target/event/ref record in atomic_flush.
	 * First-enable flush precedes atomic_enable in the default helper tail.
	 */
	spinlock_t flip_lock;
	unsigned int live_cfg;
	bool enabled;
	struct {
		unsigned int target;
		bool valid;
		bool ready;
		bool ref;
		struct drm_pending_vblank_event *event;
	} pending;
	/* Commit-local preparation; helper dependency waits serialize this CRTC. */
	unsigned int copy_target;
	bool copy_ok;
	atomic64_t copy_count;
	atomic64_t copy_total_ns;
	atomic64_t copy_last_ns;
	atomic64_t present_count;
	atomic64_t wait_total_ns;
	u64 ready_ns;
};

static struct vita_iftu_device *vita_iftu_device_of_dev(struct drm_device *dev)
{
	return container_of(dev, struct vita_iftu_device, dev);
}

static bool vita_iftu_trace_take(struct vita_iftu_device *idev,
				 unsigned int *seq)
{
	int count = atomic_inc_return(&idev->trace_count);

	if (count > VITA_IFTU_TRACE_LIMIT)
		return false;

	*seq = count;
	return true;
}

/*
 * Devicetree parsing
 */

static int vita_iftu_get_u32(struct drm_device *dev, struct device_node *np,
			      const char *name, u32 *out)
{
	int ret = of_property_read_u32(np, name, out);

	if (ret)
		drm_err(dev, "vita-iftu: cannot parse \"%s\": %d\n", name, ret);
	return ret;
}

static const struct drm_format_info *
vita_iftu_get_format(struct drm_device *dev, struct device_node *np)
{
	const char *name;
	u32 fourcc;
	int ret;

	ret = of_property_read_string(np, "format", &name);
	if (ret) {
		drm_err(dev, "vita-iftu: cannot parse \"format\": %d\n", ret);
		return ERR_PTR(ret);
	}

	/* Only the format the loader actually configures is supported in
	 * M1. IFTU pixelformat 0x10 == DT "a8b8g8r8" == DRM_FORMAT_ABGR8888
	 * byte order on this little-endian target -- verified against the
	 * live SRC_PIXELFMT register readback in
	 * lab/gpu-re/IFTU-VBLANK-FINDING-2026-08-30.md table 1.
	 */
	if (strcmp(name, "a8b8g8r8") != 0) {
		drm_err(dev, "vita-iftu: unsupported format \"%s\" (only a8b8g8r8 verified)\n",
			name);
		return ERR_PTR(-EINVAL);
	}

	fourcc = DRM_FORMAT_ABGR8888;
	return drm_format_info(fourcc);
}

/*
 * Modesetting -- fixed pipe, no hardware modeset (M1). Structure mirrors
 * drivers/gpu/drm/tiny/simpledrm.c's primary-plane/crtc/encoder/connector
 * split, which is the documented right template for "take over an
 * already-configured framebuffer" (DISPLAY-DRM-RESEARCH-2026-08-30.md §4).
 */

static const uint64_t vita_iftu_primary_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int vita_iftu_primary_plane_helper_atomic_check(struct drm_plane *plane,
							struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *new_shadow_plane_state =
		to_drm_shadow_plane_state(new_plane_state);
	struct drm_framebuffer *new_fb = new_plane_state->fb;
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;
	struct drm_device *dev = plane->dev;
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(dev);
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state,
											  plane);
	unsigned int trace_seq;
	int ret;

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	ret = drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, false);
	if (ret)
		return ret;
	else if (!new_plane_state->visible)
		return 0;

	/* No partial positioning/scaling: every copied target is one full mode. */
	if (new_plane_state->dst.x1 || new_plane_state->dst.y1 ||
	    drm_rect_width(&new_plane_state->dst) != 1280 ||
	    drm_rect_height(&new_plane_state->dst) != 720 ||
	    (new_plane_state->src.x1 & 0xffff) ||
	    (new_plane_state->src.y1 & 0xffff))
		return -EINVAL;

	if (vita_iftu_trace_take(idev, &trace_seq))
		drm_dbg(dev, "vita-iftu trace[%u] check old(crtc=%d fb=%d) "
			 "new(crtc=%d fb=%d visible=%d) src=%d,%d-%d,%d "
			 "dst=%d,%d-%d,%d\n", trace_seq,
			 !!old_plane_state->crtc, !!old_plane_state->fb,
			 !!new_plane_state->crtc, !!new_plane_state->fb,
			 new_plane_state->visible,
			 new_plane_state->src.x1, new_plane_state->src.y1,
			 new_plane_state->src.x2, new_plane_state->src.y2,
			 new_plane_state->dst.x1, new_plane_state->dst.y1,
			 new_plane_state->dst.x2, new_plane_state->dst.y2);

	if (new_fb->format != idev->format) {
		void *buf;

		buf = drm_format_conv_state_reserve(&new_shadow_plane_state->fmtcnv_state,
						     idev->pitch, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
	}

	return 0;
}

static void vita_iftu_primary_plane_helper_atomic_update(struct drm_plane *plane,
						  struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(ps);
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(plane->dev);
	struct drm_rect clip = ps->src;
	struct iosys_map dst = idev->screen_base;
	unsigned long flags;
	u64 copy_start, copy_ns;
	int ret, idx;

	if (!ps->visible)
		return;

	idev->copy_ok = false;
	spin_lock_irqsave(&idev->flip_lock, flags);
	if (idev->pending.valid) {
		/* Dependency timeouts must not let us overwrite a queued target. */
		spin_unlock_irqrestore(&idev->flip_lock, flags);
		drm_err(plane->dev, "previous presentation still pending\n");
		return;
	}
	idev->copy_target = idev->live_cfg ^ 1;
	spin_unlock_irqrestore(&idev->flip_lock, flags);

	ret = drm_gem_fb_begin_cpu_access(ps->fb, DMA_FROM_DEVICE);
	if (ret)
		goto out_error;
	if (!drm_dev_enter(plane->dev, &idx)) {
		ret = -ENODEV;
		goto out_access;
	}

	/* Ping-pong buffers are not damage-history coherent. Always copy the
	 * complete visible frame, never read the slow CDRAM scanout back.
	 */
	drm_rect_fp_to_int(&clip, &clip);
	iosys_map_incr(&dst, idev->copy_target * VITA_IFTU_SECOND_FB_OFFSET);
	copy_start = ktime_get_ns();
	ret = drm_fb_blit(&dst, &idev->pitch, idev->format->format,
			  shadow->data, ps->fb, &clip, &shadow->fmtcnv_state);
	/* Drain WC pixel stores on the producing CPU BEFORE publishing a
	 * selector request to an IRQ that may execute on another CPU.
	 * Include that drain in the observed copy duration.
	 */
	wmb();
	copy_ns = ktime_get_ns() - copy_start;
	atomic64_set(&idev->copy_last_ns, copy_ns);
	atomic64_add(copy_ns, &idev->copy_total_ns);
	atomic64_inc(&idev->copy_count);
	idev->copy_ok = !ret;
	drm_dev_exit(idx);
out_access:
	drm_gem_fb_end_cpu_access(ps->fb, DMA_FROM_DEVICE);
out_error:
	if (ret)
		drm_err(plane->dev, "frame copy failed: %d; scanout unchanged\n", ret);
}

static void vita_iftu_primary_plane_helper_atomic_disable(struct drm_plane *plane,
							   struct drm_atomic_state *state)
{
	struct drm_device *dev = plane->dev;
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(dev);
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state,
											  plane);
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state,
											  plane);
	unsigned int trace_seq;
	int idx;

	/* CRTC atomic_disable drains pending events before plane teardown. */

	if (!drm_dev_enter(dev, &idx))
		return;

	/*
	 * Do not clear the inherited CDRAM scanout here. It is live display
	 * memory, not a disposable shadow buffer; clearing it makes any disable
	 * destructive and can strand the display black if the following update
	 * fails. B2 observes this callback without modifying scanout contents.
	 */
	if (vita_iftu_trace_take(idev, &trace_seq))
		drm_dbg(dev, "vita-iftu trace[%u] disable old(crtc=%d fb=%d) "
			 "new(crtc=%d fb=%d) -- scanout clear suppressed\n", trace_seq,
			!!old_plane_state->crtc, !!old_plane_state->fb,
			!!new_plane_state->crtc, !!new_plane_state->fb);

	drm_dev_exit(idx);
}

static int vita_iftu_primary_plane_helper_get_scanout_buffer(struct drm_plane *plane,
							      struct drm_scanout_buffer *sb)
{
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(plane->dev);

	sb->width = idev->mode.hdisplay;
	sb->height = idev->mode.vdisplay;
	sb->format = idev->format;
	sb->pitch[0] = idev->pitch;
	sb->map[0] = idev->screen_base;
	iosys_map_incr(&sb->map[0], READ_ONCE(idev->live_cfg) *
		       VITA_IFTU_SECOND_FB_OFFSET);

	return 0;
}

static const struct drm_plane_helper_funcs vita_iftu_primary_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = vita_iftu_primary_plane_helper_atomic_check,
	.atomic_update = vita_iftu_primary_plane_helper_atomic_update,
	.atomic_disable = vita_iftu_primary_plane_helper_atomic_disable,
	.get_scanout_buffer = vita_iftu_primary_plane_helper_get_scanout_buffer,
};

static const struct drm_plane_funcs vita_iftu_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static enum drm_mode_status vita_iftu_crtc_helper_mode_valid(struct drm_crtc *crtc,
							      const struct drm_display_mode *mode)
{
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(crtc->dev);

	return drm_crtc_helper_mode_valid_fixed(crtc, mode, &idev->mode);
}

static int vita_iftu_crtc_helper_atomic_check(struct drm_crtc *crtc,
					       struct drm_atomic_state *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	int ret = drm_crtc_helper_atomic_check(crtc, state);

	cs->no_vblank = false;
	return ret;
}

/* Lock order: flip_lock -> DRM event_lock -> DRM vblank locks. */
static void vita_iftu_send_event(struct drm_crtc *crtc,
			       struct drm_pending_vblank_event *event)
{
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

/* Cancel a failed presentation, without delivering a success event. Helpers
 * attach flip_done even to internal events: cancellation must release that
 * completion/reference as well, or all subsequent commits will time out.
 * The event has already been detached from driver/state under flip_lock.
 */
static void vita_iftu_cancel_event(struct drm_device *dev,
				 struct drm_pending_vblank_event *event)
{
	if (!event)
		return;
	if (event->base.completion) {
		complete_all(event->base.completion);
		event->base.completion_release(event->base.completion);
		event->base.completion = NULL;
	}
	if (event->base.fence) {
		dma_fence_set_error(event->base.fence, -EIO);
		dma_fence_signal(event->base.fence);
	}
	drm_event_cancel_free(dev, &event->base);
}

/* flip_lock is held; acquire one vblank ref even for an internal/no-flip
 * event. A first-enable record stays staged until vblank_on has run.
 */
static bool vita_iftu_publish(struct vita_iftu_device *idev)
{
	if (drm_crtc_vblank_get(&idev->crtc))
		return false;
	idev->pending.ref = true;
	idev->ready_ns = ktime_get_ns();
	idev->pending.ready = true;
	return true;
}

static void vita_iftu_crtc_helper_atomic_begin(struct drm_crtc *crtc,
					    struct drm_atomic_state *state)
{
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(crtc->dev);

	unsigned long flags;

	/* A no-plane-change commit still needs a real vblank completion.
	 * Reject timed-out dependencies, even if their ISR finishes before flush.
	 */
	spin_lock_irqsave(&idev->flip_lock, flags);
	idev->copy_ok = !idev->pending.valid;
	idev->copy_target = idev->live_cfg;
	spin_unlock_irqrestore(&idev->flip_lock, flags);
}

static void vita_iftu_crtc_helper_atomic_flush(struct drm_crtc *crtc,
					      struct drm_atomic_state *state)
{
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(crtc->dev);
	struct drm_pending_vblank_event *cancel = NULL;
	unsigned long flags;

	spin_lock_irqsave(&idev->flip_lock, flags);
	if (!cs->active) {
		/* Disable has no future vblank; complete its helper/user event. */
		if (cs->event)
			vita_iftu_send_event(crtc, cs->event);
		cs->event = NULL;
	} else if (!idev->copy_ok || WARN_ON(idev->pending.valid)) {
		cancel = cs->event;
		cs->event = NULL;
	} else {
		idev->pending.target = idev->copy_target;
		idev->pending.event = cs->event;
		idev->pending.valid = true;
		cs->event = NULL;
		if (idev->enabled && !vita_iftu_publish(idev)) {
			cancel = idev->pending.event;
			memset(&idev->pending, 0, sizeof(idev->pending));
		}
	}
	spin_unlock_irqrestore(&idev->flip_lock, flags);
	vita_iftu_cancel_event(crtc->dev, cancel);
}

static void vita_iftu_crtc_helper_atomic_enable(struct drm_crtc *crtc,
					       struct drm_atomic_state *state)
{
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(crtc->dev);
	struct drm_pending_vblank_event *cancel = NULL;
	unsigned long flags;

	drm_crtc_vblank_on(crtc);
	spin_lock_irqsave(&idev->flip_lock, flags);
	idev->enabled = true;
	if (idev->pending.valid && !vita_iftu_publish(idev)) {
		cancel = idev->pending.event;
		memset(&idev->pending, 0, sizeof(idev->pending));
	}
	spin_unlock_irqrestore(&idev->flip_lock, flags);
	vita_iftu_cancel_event(crtc->dev, cancel);
}

static void vita_iftu_crtc_helper_atomic_disable(struct drm_crtc *crtc,
						struct drm_atomic_state *state)
{
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(crtc->dev);
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_pending_vblank_event *cancel;
	unsigned long flags;

	spin_lock_irqsave(&idev->flip_lock, flags);
	idev->enabled = false;
	cancel = idev->pending.event;
	if (idev->pending.ref)
		drm_crtc_vblank_put(crtc);
	memset(&idev->pending, 0, sizeof(idev->pending));
	if (!cs->active && cs->event) {
		vita_iftu_send_event(crtc, cs->event);
		cs->event = NULL;
	}
	spin_unlock_irqrestore(&idev->flip_lock, flags);
	vita_iftu_cancel_event(crtc->dev, cancel);
	/* vblank_off takes event_lock itself. Never call it under that lock. */
	drm_crtc_vblank_off(crtc);
}

static const struct drm_crtc_helper_funcs vita_iftu_crtc_helper_funcs = {
	.mode_valid = vita_iftu_crtc_helper_mode_valid,
	.atomic_check = vita_iftu_crtc_helper_atomic_check,
	.atomic_begin = vita_iftu_crtc_helper_atomic_begin,
	.atomic_flush = vita_iftu_crtc_helper_atomic_flush,
	.atomic_enable = vita_iftu_crtc_helper_atomic_enable,
	.atomic_disable = vita_iftu_crtc_helper_atomic_disable,
};

/*
 * M2 Gate A vblank IRQ. The loader already arms the DSI-side vblank mask
 * (intr enable +0x54 = 2); Linux claims the GIC line (SPI 178, hardware
 * IRQ 210) and acks via the write-to-clear status register +0x50.
 */
static irqreturn_t vita_iftu_irq_handler(int irq, void *data)
{
	struct vita_iftu_device *idev = data;
	unsigned long flags;
	u32 status = readl(idev->dsi_regs + IFTU_DSI_INTR_STATUS);

	if (!(status & IFTU_DSI_INTR_VBLANK))
		return IRQ_NONE;
	writel(status, idev->dsi_regs + IFTU_DSI_INTR_STATUS);
	spin_lock_irqsave(&idev->flip_lock, flags);
	atomic64_inc(&idev->vblank_count);
	if (idev->enabled && idev->pending.ready) {
		writel(idev->pending.target,
		       idev->control_regs + IFTU_CREG_PLANE_SEL(0));
		/* Flush the posted selector write before reporting completion. */
		readl(idev->control_regs + IFTU_CREG_PLANE_SEL(0));
		WRITE_ONCE(idev->live_cfg, idev->pending.target);
		atomic64_inc(&idev->present_count);
		atomic64_add(ktime_get_ns() - idev->ready_ns, &idev->wait_total_ns);
	}
	/* Core updates its counter/events before our flip completion is sent. */
	if (idev->enabled)
		drm_crtc_handle_vblank(&idev->crtc);
	if (idev->enabled && idev->pending.ready) {
		if (idev->pending.event)
			vita_iftu_send_event(&idev->crtc, idev->pending.event);
		if (idev->pending.ref)
			drm_crtc_vblank_put(&idev->crtc);
		memset(&idev->pending, 0, sizeof(idev->pending));
	}
	spin_unlock_irqrestore(&idev->flip_lock, flags);
	return IRQ_HANDLED;
}

static int vita_iftu_enable_vblank(struct drm_crtc *crtc)
{
	/* The loader keeps the DSI vblank mask armed; nothing to enable. */
	return 0;
}

static void vita_iftu_disable_vblank(struct drm_crtc *crtc)
{
	/* Keep the line armed; the loader owns the mask. */
}

static const struct drm_crtc_funcs vita_iftu_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = vita_iftu_enable_vblank,
	.disable_vblank = vita_iftu_disable_vblank,
};

static const struct drm_encoder_funcs vita_iftu_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int vita_iftu_connector_helper_get_modes(struct drm_connector *connector)
{
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &idev->mode);
}

static const struct drm_connector_helper_funcs vita_iftu_connector_helper_funcs = {
	.get_modes = vita_iftu_connector_helper_get_modes,
};

static const struct drm_connector_funcs vita_iftu_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs vita_iftu_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/*
 * Init / probe
 */

/*
 * Describe the inherited VIC-4 mode; do not program clocks or timing.
 * The old 64473 kHz/52 Hz inference predated the A9 timer correction.
 * With periphclk=165667500, 3600 IRQs over about 60.058 s agree with
 * 59.94 Hz and the loader's nominal 74.1758 MHz (rounded to kHz below).
 * This is metadata only. Timer/PLL registers are deliberately untouched.
 * Evidence: umbrella lab/gates/2026-10-21-m3-corrected-boot/cadence-60s.json.
 */
static struct drm_display_mode vita_iftu_pstv_hdmi_mode(void)
{
	struct drm_display_mode mode = {
		DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74176,
			 1280, 1390, 1430, 1650, 0,
			 720, 725, 730, 750, 0,
			 DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
	};

	mode.width_mm = DRM_MODE_RES_MM(1280, 96ul);
	mode.height_mm = DRM_MODE_RES_MM(720, 96ul);

	return mode;
}

static void vita_iftu_log_control_regs(struct drm_device *dev,
					struct vita_iftu_device *idev)
{
	u32 control, control2;

	if (!idev->control_regs)
		return;

	/* Read-only sanity check against the values hardware-verified in
	 * IFTU-VBLANK-FINDING-2026-08-30.md table "Control regs at
	 * 0xE5032000". Mismatch here means the loader state changed under
	 * us and M2's flip assumptions need re-verifying before use.
	 */
	control = readl(idev->control_regs + IFTU_CREG_CONTROL);
	control2 = readl(idev->control_regs + IFTU_CREG_CONTROL2);

	drm_dbg(dev, "vita-iftu: control=0x%08x control2=0x%08x (expect 0x1/0x1)\n",
		control, control2);
	if (!(control & IFTU_CREG_CONTROL_ENABLE))
		drm_warn(dev, "vita-iftu: IFTU bus control ENABLE bit is clear -- "
			      "the loader's mode may not actually be live\n");
}

/*
 * M2 Gate B: program plane A config 1 (the inactive config) to a second
 * CDRAM buffer, leaving the selected config 0 untouched. The loader
 * initialized both configs identically; we copy the live config 0 block
 * to config 1 so it inherits the exact geometry, then repoint FB_PADDR.
 * No selector write happens here -- PLANE_A_SEL stays on config 0, so the
 * active scanout must not change. Gate B verifies that on hardware.
 */
static void vita_iftu_program_second_config(struct drm_device *dev,
					    struct vita_iftu_device *idev,
					    u32 second_fb_paddr)
{
	u32 src = IFTU_PLANE_A_CFG(0);
	u32 dst = IFTU_PLANE_A_CFG(1);
	u32 off;

	for (off = 0; off < 0x100; off += 4)
		writel(readl(idev->plane_regs + src + off),
		       idev->plane_regs + dst + off);

	writel(second_fb_paddr, idev->plane_regs + dst + IFTU_PCFG_FB_PADDR);

	drm_info(dev,
		 "vita-iftu: plane A config 1 -> second buffer 0x%08x, config 0 active\n",
		 second_fb_paddr);
}

/*
 * M3-B2 evidence. The userspace probe cannot read the IFTU register window
 * itself without /dev/mem, so the driver reports what the hardware is doing.
 * Both the driver's belief (live_cfg) and the actual register read are printed:
 * if those two ever disagree, the belief is the thing that is wrong, and this
 * file is how that gets caught instead of being assumed.
 */
static int vita_iftu_debugfs_state_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(entry->dev);
	unsigned long flags;
	u32 selector, live, pending, scanline, byte;
	u64 count;
	int idx;

	if (!drm_dev_enter(entry->dev, &idx))
		return -ENODEV;
	spin_lock_irqsave(&idev->flip_lock, flags);
	selector = readl(idev->control_regs + IFTU_CREG_PLANE_SEL(0));
	live = idev->live_cfg;
	pending = idev->pending.valid;
	scanline = readl(idev->plane_regs + IFTU_PREG_SCANLINE) >> 16;
	byte = readl(idev->plane_regs + IFTU_PREG_SCANOUT_BYTE);
	count = atomic64_read(&idev->vblank_count);
	spin_unlock_irqrestore(&idev->flip_lock, flags);
	drm_dev_exit(idx);
	seq_printf(m, "selector %u\nlive_cfg %u\nflip_pending %u\n"
		   "scanline %u\nscanout_byte %u\nvblank_count %llu\n",
		   selector, live, pending, scanline, byte, count);
	seq_printf(m, "copy_count %lld\ncopy_total_ns %lld\ncopy_last_ns %lld\n"
		   "present_count %lld\nwait_total_ns %lld\n",
		   atomic64_read(&idev->copy_count),
		   atomic64_read(&idev->copy_total_ns),
		   atomic64_read(&idev->copy_last_ns),
		   atomic64_read(&idev->present_count),
		   atomic64_read(&idev->wait_total_ns));
	return 0;
}

static struct vita_iftu_device *vita_iftu_device_create(const struct drm_driver *drv,
							 struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct vita_iftu_device *idev;
	struct drm_device *dev;
	struct resource *mem;
	struct resource fb_mem;
	struct drm_plane *primary_plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	unsigned long max_width, max_height;
	void *screen_base;
	int width, height, stride;
	int ret;

	idev = devm_drm_dev_alloc(&pdev->dev, drv, struct vita_iftu_device, dev);
	if (IS_ERR(idev))
		return ERR_CAST(idev);
	dev = &idev->dev;
	platform_set_drvdata(pdev, idev);
	spin_lock_init(&idev->flip_lock);

	ret = vita_iftu_get_u32(dev, of_node, "width", &width);
	if (ret)
		return ERR_PTR(ret);
	ret = vita_iftu_get_u32(dev, of_node, "height", &height);
	if (ret)
		return ERR_PTR(ret);
	ret = vita_iftu_get_u32(dev, of_node, "stride", &stride);
	if (ret)
		return ERR_PTR(ret);

	idev->format = vita_iftu_get_format(dev, of_node);
	if (IS_ERR(idev->format))
		return ERR_CAST(idev->format);

	idev->mode = vita_iftu_pstv_hdmi_mode();
	if (width != 1280 || height != 720 || stride != 5120) {
		drm_err(dev, "only inherited 1280x720 stride 5120 is verified\n");
		return ERR_PTR(-EINVAL);
	}
	idev->pitch = stride;

	/* Map only the loader-configured bus from devicetree. */
	idev->plane_regs = devm_platform_ioremap_resource_byname(pdev, "planes");
	if (IS_ERR(idev->plane_regs)) {
		drm_err(dev, "vita-iftu: failed to map \"planes\" reg: %ld\n",
			PTR_ERR(idev->plane_regs));
		return ERR_CAST(idev->plane_regs);
	}
	idev->control_regs = devm_platform_ioremap_resource_byname(pdev, "control");
	if (IS_ERR(idev->control_regs)) {
		drm_err(dev, "vita-iftu: failed to map \"control\" reg: %ld\n",
			PTR_ERR(idev->control_regs));
		return ERR_CAST(idev->control_regs);
	}
	vita_iftu_log_control_regs(dev, idev);

	/* DSI bus-1 interrupt block (M2 Gate A). The loader already armed
	 * the vblank mask; we map the window so the ISR can ack +0x50.
	 */
	idev->dsi_regs = devm_platform_ioremap_resource_byname(pdev, "dsi");
	if (IS_ERR(idev->dsi_regs)) {
		drm_err(dev, "vita-iftu: failed to map \"dsi\" reg: %ld\n",
			PTR_ERR(idev->dsi_regs));
		return ERR_CAST(idev->dsi_regs);
	}

	/*
	 * Framebuffer memory: the "memory-region" points at the
	 * reserved-memory CDRAM node (see pstv.dts). This is normal
	 * physical RAM the loader already lit up, not an MMIO aperture --
	 * same handling as simpledrm's "mem" branch (devm_memremap with
	 * MEMREMAP_WC), not the MMIO-resource branch.
	 */
	{
		struct device_node *mem_node = of_parse_phandle(of_node, "memory-region", 0);

		if (!mem_node) {
			drm_err(dev, "vita-iftu: no memory-region\n");
			return ERR_PTR(-EINVAL);
		}
		ret = of_address_to_resource(mem_node, 0, &fb_mem);
		of_node_put(mem_node);
		if (ret)
			return ERR_PTR(ret);
		mem = &fb_mem;
	}
	if (mem->start != 0x20000000 || resource_size(mem) != 0x08000000) {
		drm_err(dev, "vita-iftu: unexpected framebuffer resource %pr\n", mem);
		return ERR_PTR(-EINVAL);
	}
	if ((u64)idev->pitch * height != VITA_IFTU_SECOND_FB_OFFSET ||
	    VITA_IFTU_SECOND_FB_OFFSET + (u64)idev->pitch * height >
	    resource_size(mem)) {
		drm_err(dev, "vita-iftu: framebuffer %ux%u exceeds resource %pr\n",
			width, height, mem);
		return ERR_PTR(-EINVAL);
	}

	/* Reject an unexpected inherited state before ANY register writes.
	 * Config 0 geometry/address below matches the verified loader capture.
	 */
	if (readl(idev->control_regs + IFTU_CREG_PLANE_SEL(0)) != 0 ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_FB_PADDR) != mem->start ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_SRC_X) != 0 ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_SRC_Y) != 0 ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_SRC_PIXELFMT) !=
		IFTU_PIXFMT_A8B8G8R8 ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_SRC_FB_W) != width ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_SRC_FB_H) != height ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_DST_W) != width ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_DST_H) != height ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_SRC_W) !=
		IFTU_SCALE_1X ||
	    readl(idev->plane_regs + IFTU_PLANE_A_CFG(0) + IFTU_PCFG_SRC_H) !=
		IFTU_SCALE_1X) {
		drm_err(dev, "unverified inherited plane A state; refusing takeover\n");
		return ERR_PTR(-EINVAL);
	}

	ret = devm_aperture_acquire_from_firmware(dev, mem->start, resource_size(mem));
	if (ret) {
		drm_err(dev, "vita-iftu: could not acquire memory range %pr: %d\n", mem, ret);
		return ERR_PTR(ret);
	}

	screen_base = devm_memremap(dev->dev, mem->start, resource_size(mem), MEMREMAP_WC);
	if (IS_ERR(screen_base))
		return ERR_CAST(screen_base);
	iosys_map_set_vaddr(&idev->screen_base, screen_base);
	vita_iftu_program_second_config(dev, idev,
					mem->start + VITA_IFTU_SECOND_FB_OFFSET);

	drm_dbg(dev, "vita-iftu: framebuffer format=%p4cc, size=%dx%d, stride=%d, mode="
		     DRM_MODE_FMT "\n",
		&idev->format->format, width, height, stride, DRM_MODE_ARG(&idev->mode));

	/*
	 * Mode config
	 */
	ret = drmm_mode_config_init(dev);
	if (ret)
		return ERR_PTR(ret);

	max_width = max_t(unsigned long, width, DRM_SHADOW_PLANE_MAX_WIDTH);
	max_height = max_t(unsigned long, height, DRM_SHADOW_PLANE_MAX_HEIGHT);

	dev->mode_config.min_width = idev->mode.hdisplay;
	dev->mode_config.max_width = max_width;
	dev->mode_config.min_height = idev->mode.vdisplay;
	dev->mode_config.max_height = max_height;
	dev->mode_config.prefer_shadow = false;
	dev->mode_config.preferred_depth = idev->format->depth;
	dev->mode_config.funcs = &vita_iftu_mode_config_funcs;

	max_width = idev->mode.hdisplay;
	max_height = idev->mode.vdisplay;

	/* Advertise only paths drm_fb_blit() can execute for ABGR scanout.
	 * The generic list helper replaces ABGR with XBGR, which the blit
	 * helper cannot convert back to ABGR and hides the native copy path.
	 */
	idev->formats[0] = idev->format->format;
	idev->formats[1] = DRM_FORMAT_XRGB8888;
	idev->nformats = 2;

	primary_plane = &idev->primary_plane;
	ret = drm_universal_plane_init(dev, primary_plane, 0,
					&vita_iftu_primary_plane_funcs,
					idev->formats, idev->nformats,
					vita_iftu_primary_plane_format_modifiers,
					DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ERR_PTR(ret);
	drm_plane_helper_add(primary_plane, &vita_iftu_primary_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(primary_plane);

	crtc = &idev->crtc;
	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
					 &vita_iftu_crtc_funcs, NULL);
	if (ret)
		return ERR_PTR(ret);
	drm_crtc_helper_add(crtc, &vita_iftu_crtc_helper_funcs);

	/* M2 Gate A: one vblank-capable CRTC. */
	ret = drm_vblank_init(dev, 1);
	if (ret)
		return ERR_PTR(ret);
	atomic64_set(&idev->vblank_count, 0);
	drm_crtc_vblank_reset(crtc);

	encoder = &idev->encoder;
	ret = drm_encoder_init(dev, encoder, &vita_iftu_encoder_funcs,
				DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ERR_PTR(ret);
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	connector = &idev->connector;
	ret = drm_connector_init(dev, connector, &vita_iftu_connector_funcs,
				  DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ERR_PTR(ret);
	drm_connector_helper_add(connector, &vita_iftu_connector_helper_funcs);
	/* The loader owns HDMI detection and the inherited mode is fixed. */
	connector->status = connector_status_connected;
	drm_connector_set_panel_orientation(connector, DRM_MODE_PANEL_ORIENTATION_NORMAL);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ERR_PTR(ret);

	drm_mode_config_reset(dev);

	return idev;
}

DEFINE_DRM_GEM_FOPS(vita_iftu_fops);

static const struct drm_driver vita_iftu_driver = {
	DRM_GEM_SHMEM_DRIVER_OPS,
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops			= &vita_iftu_fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
};

static int vita_iftu_probe(struct platform_device *pdev)
{
	struct vita_iftu_device *idev;
	struct drm_device *dev;
	int ret;

	idev = vita_iftu_device_create(&vita_iftu_driver, pdev);
	if (IS_ERR(idev))
		return PTR_ERR(idev);
	dev = &idev->dev;
	atomic_set(&idev->trace_count, 0);

	/* CRTC/vblank state is initialized before requesting the IRQ. Claim
	 * it before registration can expose commits to userspace.
	 */
	idev->irq = platform_get_irq(pdev, 0);
	if (idev->irq < 0) {
		drm_err(dev, "vita-iftu: no vblank interrupt in devicetree: %d\n",
			idev->irq);
		return idev->irq;
	}
	ret = devm_request_irq(&pdev->dev, idev->irq, vita_iftu_irq_handler,
			       IRQF_TRIGGER_HIGH, DRIVER_NAME, idev);
	if (ret) {
		drm_err(dev, "vita-iftu: request_irq(%d) failed: %d\n",
			idev->irq, ret);
		return ret;
	}
	drm_info(dev, "vita-iftu: vblank IRQ %d claimed\n", idev->irq);

	ret = drm_dev_register(dev, 0);
	if (ret)
		return ret;

	drm_debugfs_add_file(dev, "iftu_state", vita_iftu_debugfs_state_show, NULL);

	/*
	 * fbdev-console emulation: historical failure record and M3-E opt-in.
	 *
	 * drm_fbdev_shmem_setup() drives DRM's client helper through a real
	 * first atomic commit onto the primary plane -- the actual first
	 * hardware exercise of vita_iftu_primary_plane_helper_atomic_update()
	 * / _atomic_disable() on real silicon. The first deploy attempt
	 * (before the drm_fb_build_fourcc_list() fix below) never reached
	 * this path: the format lookup failed, fbdev setup bailed out with
	 * a logged warning, and the DRM device sat inert -- which is why
	 * that build was hardware-clean (4/4 cores, /dev/dri/card0 present,
	 * no faults) despite never having painted anything.
	 *
	 * Fixing the format-list bug (see idev->formats/nformats above) let
	 * this path actually run for the first time, and the very next
	 * deploy attempt produced a black HDMI picture that did not
	 * recover on its own -- twice. VitaOS's own network stack survived
	 * both times (port 1338 came back / was still reachable), which
	 * points at the display atomic-commit path hanging or corrupting
	 * scanout, not a full kernel panic.
	 *
	 * RESOLVED (2026-09-01): the B2 failures were root-caused and fixed
	 * on hardware, and the corrected first atomic commit PASSED.
	 *   - The B2 Oops was the driver memremap'ing the plane-register
	 *     window (0xe5030000) instead of the DT memory-region (CDRAM
	 *     0x20000000); the hard-gated mapping fix (mem->start ==
	 *     0x20000000 && resource_size == 0x08000000) landed first.
	 *   - atomic_disable() no longer memsets the live scanout buffer
	 *     (see the disable callback above: trace-only, clear
	 *     suppressed), so no transient disable can strand the display.
	 *   - The M1 corrected trace confirmed the normal first enable is
	 *     atomic_check -> begin_fb_access -> atomic_update ->
	 *     end_fb_access: no initial atomic_disable, begin_fb_access
	 *     returned 0, and the full-frame blit succeeded. The disable
	 *     callback was entered only during teardown (after
	 *     DRM_IOCTL_MODE_RMFB), with the clear suppressed.
	 *   - M2 Gates A-D (vblank IRQ, inactive config, first vblank-synced
	 *     page flip, bounded flips) all passed on hardware.
	 * Records: lab/gpu-re/IFTU-B2-CORRECTED-RESULT-2026-09-01.md and
	 * lab/gpu-re/IFTU-M2-GATE{A,B,C,D}-RESULT-2026-09-01.md (research
	 * commits 5d47f54, 5f0fd77..4459066).
	 *
	 * Automatic fbdev-console takeover remains OFF by default. M3-B2
	 * passed on 2026-10-21; M3-C throughput did not meet refresh cadence.
	 * The opt-in parameter allows a separate, supervised M3-E console
	 * usability gate, not a claim that sustained full-rate flips passed.
	 */
	if (enable_fbdev)
		drm_fbdev_shmem_setup(dev, 32);

	return 0;
}

static void vita_iftu_remove(struct platform_device *pdev)
{
	struct vita_iftu_device *idev = platform_get_drvdata(pdev);

	drm_dev_unplug(&idev->dev);
	drm_atomic_helper_shutdown(&idev->dev);
}

static void vita_iftu_shutdown(struct platform_device *pdev)
{
	struct vita_iftu_device *idev = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&idev->dev);
}

static const struct of_device_id vita_iftu_of_match[] = {
	{ .compatible = "vita,iftu" },
	{ },
};
MODULE_DEVICE_TABLE(of, vita_iftu_of_match);

static struct platform_driver vita_iftu_platform_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = vita_iftu_of_match,
	},
	.probe = vita_iftu_probe,
	.remove = vita_iftu_remove,
	.shutdown = vita_iftu_shutdown,
};
module_platform_driver(vita_iftu_platform_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
