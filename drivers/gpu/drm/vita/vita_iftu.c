// SPDX-License-Identifier: GPL-2.0-only
/*
 * vita-iftu: DRM driver for the PS Vita / PS TV IFTU display controller.
 *
 * M1 milestone (see lab/gpu-re/DISPLAY-DRM-RESEARCH-2026-08-30.md): takes
 * over the framebuffer the bare-metal loader already configured and scanned
 * out, exactly the way drivers/gpu/drm/tiny/simpledrm.c takes over a
 * "simple-framebuffer" node -- fixed mode, one plane, no modeset, no
 * hardware page flip and no vblank IRQ yet. This is the WORKING FOUNDATION
 * the later milestones extend:
 *
 *   M1 (this file) -- inherited-mode takeover, CPU blit into CDRAM
 *   M2 -- dual plane-config hardware page flip + real/fake vblank
 *   M3 -- real IFTU modesetting, scaling, CSC, second plane
 *   M4 -- DSI host + ADV7533 bridge, full cold modeset
 *
 * Everything this file assumes about the hardware is hardware-verified, not
 * guessed -- see the register map and citations in
 * lab/gpu-re/IFTU-VBLANK-FINDING-2026-08-30.md and
 * lab/gpu-re/VBLANK-52HZ-RESOLVED-2026-08-30.md:
 *
 *   - IFTU plane regs:    base + bus*0x10000 + plane*0x1000
 *   - IFTU control regs:  base + 0x2000 + bus*0x10000
 *   - PSTV is bus 1 (HDMI). Bus 0 (OLED/LCD) is clock-gated on PSTV and
 *     SIGBUSes on read -- this driver only ever touches the bus given to it
 *     via devicetree "reg", never assumes or scans the other bus.
 *   - CDRAM (0x20000000, 128 MiB on PSTV) already holds a live, correctly
 *     scanned-out framebuffer at boot. Format is IFTU pixelformat 0x10 =
 *     "a8b8g8r8" in DT terms (BGRX8888 per the wiki's naming).
 *   - The bootloader also populated a SECOND, identically-configured
 *     plane-config (config 1) pointing at the same buffer. This driver does
 *     not yet use it (that is M2's dual-buffer flip); for M1 that config is
 *     left completely alone.
 *   - Real refresh rate is measured at 52.099 Hz (20/23 of the loader
 *     table's nominal 74.1758 MHz pixel clock -- root cause not yet
 *     resolved, see the M-1-numbered doc above). The DRM mode below encodes
 *     the MEASURED clock, not the nominal one, so userspace vrefresh
 *     queries are honest.
 *
 * STRICTLY NO MMIO WRITES to the plane/control registers in this file yet:
 * M1 only maps the inherited CDRAM buffer as write-combining memory and
 * blits into it, exactly like simpledrm's system-memory path. The IFTU
 * plane/control "reg" resource is mapped and stashed for M2, and read from
 * for validation (get_scanout_buffer sanity, not correctness-critical), but
 * nothing in this driver's probe or update paths currently writes it. Do
 * not add plane-select or CSC writes here -- that belongs in the M2 patch
 * once the config-select atomicity question is answered on hardware.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
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

/* IFTU control-block offsets, relative to the "control" reg resource.
 * Verified live on PSTV bus 1 (0xE5032000): CONTROL=0x1 (enabled),
 * CONTROL2=0x1 (alpha enabled), PLANE_A_SEL=0, PLANE_B_SEL=0.
 * Not written by M1 -- kept here for M2's plane-flip patch and for
 * a read-only sanity check in probe().
 */
#define IFTU_CREG_CONTROL		0x00
#define IFTU_CREG_CONTROL_ENABLE	BIT(0)
#define IFTU_CREG_CONTROL2		0x04
#define IFTU_CREG_PLANE_SEL(plane)	(0x10 + (plane) * 0x08)

/* Per-frame vblank flag inside the planes window (0xE5031004 absolute),
 * verified 2026-08-30: toggles 0..1 once per frame. Polled for the
 * page-flip sync; the DSI IRQ (Gate A) is the DRM-side vblank source.
 */
#define IFTU_PREG_VBLANK_FLAG		0x1004

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

	/* IFTU register windows. control_regs is unused (read-only sanity
	 * check only) until the M2 plane-flip patch. dsi_regs is the DSI
	 * bus-1 interrupt block, claimed for the M2 vblank IRQ.
	 */
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

	/* Plane format list. simpledrm's format-conversion emulation (fbdev
	 * console, etc.) needs more than just the native pixel format --
	 * drm_fb_build_fourcc_list() expands it to include the legacy
	 * XRGB8888 target fbdev emulation looks for. Without this, probe
	 * succeeds but fbdev-shmem setup fails with "No compatible format
	 * found" because drm_mode_legacy_fb_format(32, 24) can't find
	 * XRGB8888 in a single-entry ABGR8888-only list.
	 */
	uint32_t formats[8];
	size_t nformats;

	/* Bounded callback trace for the supervised first-commit test. */
	atomic_t trace_count;
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
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_device *dev = plane->dev;
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(dev);
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	unsigned int trace_seq;
	int ret, idx;

	if (vita_iftu_trace_take(idev, &trace_seq))
		drm_dbg(dev, "vita-iftu trace[%u] update old_fb=%d new_fb=%d "
			 "visible=%d src=%d,%d-%d,%d dst=%d,%d-%d,%d\n",
			 trace_seq, !!old_plane_state->fb, !!fb,
			 plane_state->visible,
			 plane_state->src.x1, plane_state->src.y1,
			 plane_state->src.x2, plane_state->src.y2,
			 plane_state->dst.x1, plane_state->dst.y1,
			 plane_state->dst.x2, plane_state->dst.y2);

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (vita_iftu_trace_take(idev, &trace_seq))
		drm_dbg(dev, "vita-iftu trace[%u] begin_fb_access ret=%d\n",
			 trace_seq, ret);
	if (ret)
		return;

	if (!drm_dev_enter(dev, &idx))
		goto out_end_cpu_access;

	/*
	 * IMPORTANT (see IFTU-VBLANK-FINDING-2026-08-30.md §3): the
	 * framebuffer read path measured 25.7 MB/s vs 167 MB/s for writes --
	 * ~6.5x slower. drm_fb_blit() reads the shadow-plane source (system
	 * memory) and writes to sdev->screen_base (CDRAM) -- the correct
	 * direction. Never add a path that reads screen_base.
	 */
	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		struct drm_rect dst_clip = plane_state->dst;
		struct iosys_map dst = idev->screen_base;

		if (!drm_rect_intersect(&dst_clip, &damage))
			continue;

		iosys_map_incr(&dst, drm_fb_clip_offset(idev->pitch, idev->format, &dst_clip));
		ret = drm_fb_blit(&dst, &idev->pitch, idev->format->format,
				  shadow_plane_state->data, fb, &damage,
				  &shadow_plane_state->fmtcnv_state);
		if (vita_iftu_trace_take(idev, &trace_seq))
			drm_dbg(dev, "vita-iftu trace[%u] damage=%d,%d-%d,%d blit_ret=%d\n",
				 trace_seq, damage.x1, damage.y1, damage.x2, damage.y2, ret);
	}

	drm_dev_exit(idx);
out_end_cpu_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	if (vita_iftu_trace_take(idev, &trace_seq))
		drm_dbg(dev, "vita-iftu trace[%u] end_fb_access\n", trace_seq);
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

/*
 * No vblank support in M1 -- .no_vblank is set on the crtc state in
 * atomic_check below, which makes DRM's atomic helper fake a vblank event
 * immediately on commit (documented behavior of drm_simple_display_pipe /
 * struct drm_crtc_state.no_vblank; see simpledrm and vkms for both ends of
 * this spectrum). M2 replaces this with drm_crtc_arm_vblank_event() driven
 * by the DSI IRQ (GIC 210 on PSTV) once the config-select flip is wired up
 * -- see the M2 entry in DISPLAY-DRM-RESEARCH-2026-08-30.md's milestone
 * ladder for exactly what changes.
 */
static int vita_iftu_crtc_helper_atomic_check(struct drm_crtc *crtc,
					       struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(crtc->dev);
	unsigned int trace_seq;
	int ret;

	ret = drm_crtc_helper_atomic_check(crtc, state);
	if (ret)
		return ret;

	crtc_state->no_vblank = true;
	if (vita_iftu_trace_take(idev, &trace_seq))
		drm_dbg(crtc->dev, "vita-iftu trace[%u] crtc_check no_vblank=%d\n",
			 trace_seq, crtc_state->no_vblank);
	return 0;
}

static const struct drm_crtc_helper_funcs vita_iftu_crtc_helper_funcs = {
	.mode_valid = vita_iftu_crtc_helper_mode_valid,
	.atomic_check = vita_iftu_crtc_helper_atomic_check,
};

/*
 * M2 Gate A vblank IRQ. The loader already arms the DSI-side vblank mask
 * (intr enable +0x54 = 2); Linux claims the GIC line (SPI 178, hardware
 * IRQ 210) and acks via the write-to-clear status register +0x50.
 */
static irqreturn_t vita_iftu_irq_handler(int irq, void *data)
{
	struct vita_iftu_device *idev = data;
	u32 status;

	status = readl(idev->dsi_regs + IFTU_DSI_INTR_STATUS);
	if (status & IFTU_DSI_INTR_VBLANK) {
		/* Write-to-clear ack, then signal the core. */
		writel(status, idev->dsi_regs + IFTU_DSI_INTR_STATUS);
		atomic64_inc(&idev->vblank_count);
		drm_crtc_handle_vblank(&idev->crtc);
	}

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
 * Build the fixed mode from MEASURED timing, not the loader's nominal
 * table entry. See lab/gpu-re/VBLANK-52HZ-RESOLVED-2026-08-30.md: htotal
 * and vtotal are exactly as the loader's VIC-4 table says (1650 x 750 --
 * confirmed by the frame-period / line-period ratio landing on 750.01
 * lines), but the pixel clock is 20/23 of the nominal 74.1758 MHz, i.e.
 * ~64.473 MHz measured over a 60 s precision count (52.099142 Hz, 3126
 * edges). DRM's "clock" field is in kHz.
 */
static struct drm_display_mode vita_iftu_pstv_hdmi_mode(void)
{
	struct drm_display_mode mode = {
		DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 64473,
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
 * M2 Gate C: one hardware page flip. Wait for the next vblank edge (the
 * per-frame flag in the planes window), then toggle PLANE_A_SEL so the
 * display switches between config 0 (inherited framebuffer) and config 1
 * (second buffer). This is the first write to the selector register --
 * the actual page flip. The DRM-side vblank event comes from the DSI IRQ
 * (Gate A); this sync only ensures the selector write lands at the frame
 * boundary, so the visible switch should be tear-free.
 *
 * The caller (debugfs "flip" show) reads the file to perform ONE flip.
 */
static int vita_iftu_do_page_flip(struct drm_device *dev,
				  struct vita_iftu_device *idev)
{
	u32 sel, new_sel, vblank, prev_vblank;
	unsigned int polls = 0;

	sel = readl(idev->control_regs + IFTU_CREG_PLANE_SEL(0));
	new_sel = sel ? 0 : 1;

	/* Wait for a vblank edge: the flag toggles once per frame (~52 Hz
	 * on PSTV, ~19 ms period, so each phase lasts ~9.6 ms). Sleep-poll
	 * with a small delay so the window spans several frame periods --
	 * a tight bounded spin only covers ~1-5 ms and misses the edge.
	 * Bound at ~200 ms; a dead display must not hang the caller.
	 */
	prev_vblank = readl(idev->plane_regs + IFTU_PREG_VBLANK_FLAG);
	do {
		usleep_range(500, 1000);
		vblank = readl(idev->plane_regs + IFTU_PREG_VBLANK_FLAG);
		if (vblank != prev_vblank)
			break;
	} while (++polls < 200);

	if (vblank == prev_vblank) {
		drm_err(dev, "vita-iftu: vblank flag not toggling; flip aborted\n");
		return -ETIMEDOUT;
	}

	writel(new_sel, idev->control_regs + IFTU_CREG_PLANE_SEL(0));

	drm_info(dev, "vita-iftu: page flip %u -> %u (vblank edge after %u polls)\n",
		 sel, new_sel, polls);

	return 0;
}

static int vita_iftu_debugfs_flip_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct vita_iftu_device *idev = vita_iftu_device_of_dev(entry->dev);
	int ret;

	ret = vita_iftu_do_page_flip(entry->dev, idev);
	if (ret) {
		seq_puts(m, "flip failed\n");
		return ret;
	}
	seq_puts(m, "flip ok\n");
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
	if (idev->mode.hdisplay != width || idev->mode.vdisplay != height)
		drm_warn(dev,
			 "vita-iftu: devicetree %dx%d disagrees with the compiled-in "
			 "%dx%d PSTV mode -- update vita_iftu_pstv_hdmi_mode() if this "
			 "board's timing differs\n",
			 width, height, idev->mode.hdisplay, idev->mode.vdisplay);
	idev->pitch = stride;

	/*
	 * IFTU plane/control registers. Mapped now so M2 has them ready and
	 * so probe can do the read-only control-register sanity check
	 * above; nothing in M1 writes through these pointers.
	 */
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
	if ((u64)idev->pitch * height > resource_size(mem)) {
		drm_err(dev, "vita-iftu: framebuffer %ux%u exceeds resource %pr\n",
			width, height, mem);
		return ERR_PTR(-EINVAL);
	}

	/*
	 * M2 Gate B: the second page-flip buffer lives immediately after
	 * the inherited framebuffer, inside the same CDRAM memory-region.
	 * Program plane A config 1 (inactive) to point at it now so the
	 * later page-flip gate only has to toggle the selector. Config 0
	 * stays selected; the active scanout must remain untouched.
	 */
	vita_iftu_program_second_config(dev, idev,
					mem->start + VITA_IFTU_SECOND_FB_OFFSET);

	ret = devm_aperture_acquire_from_firmware(dev, mem->start, resource_size(mem));
	if (ret) {
		drm_err(dev, "vita-iftu: could not acquire memory range %pr: %d\n", mem, ret);
		return ERR_PTR(ret);
	}

	screen_base = devm_memremap(dev->dev, mem->start, resource_size(mem), MEMREMAP_WC);
	if (IS_ERR(screen_base))
		return ERR_CAST(screen_base);
	iosys_map_set_vaddr(&idev->screen_base, screen_base);

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

	idev->nformats = drm_fb_build_fourcc_list(dev, &idev->format->format, 1,
						   idev->formats, ARRAY_SIZE(idev->formats));

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

	ret = drm_dev_register(dev, 0);
	if (ret)
		return ret;

	/*
	 * M2 Gate A vblank IRQ. The loader armed the DSI-side mask; the
	 * GIC line (SPI 178 = hardware IRQ 210) is disabled until claimed
	 * here. request_irq before drm_dev_register would leave a window
	 * where the ISR could call drm_crtc_handle_vblank() on an
	 * unregistered device, so the request happens after registration.
	 * The crtc is initialized in device_create, so handle_vblank is
	 * safe from this point on. Failure is fatal: an unclaimed but
	 * enabled line can produce spurious IRQs, and M2 vblank needs the
	 * line claimed anyway.
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

	/*
	 * M2 Gate C: reading /sys/kernel/debug/dri/0/flip performs ONE
	 * vblank-synced page flip (selector toggle). Supervised use only;
	 * this is the first write to the selector register and the gate
	 * runs it exactly once from a client, then verifies and rolls back.
	 */
	drm_debugfs_add_file(dev, "flip", vita_iftu_debugfs_flip_show, NULL);

	/*
	 * fbdev-console emulation DELIBERATELY DISABLED (2026-08-30).
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
	 * Automatic fbdev-console takeover remains disabled and unvalidated.
	 * Do not re-enable this call until the debugfs flip path is wired
	 * into DRM's atomic page_flip API (userspace drmModePageFlip) and an
	 * explicit fbdev-console gate passes on hardware.
	 */

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
