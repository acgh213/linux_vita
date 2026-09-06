/* SPDX-License-Identifier: GPL-2.0 */
#ifndef PSTV_OHCI_GATE_BACKEND_SHIM_H
#define PSTV_OHCI_GATE_BACKEND_SHIM_H

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <stdarg.h>
#include <sys/types.h>

typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
typedef uint16_t __le16; typedef uint32_t __le32; typedef uint64_t __le64;
typedef uint64_t dma_addr_t;
typedef unsigned int gfp_t; typedef int pm_message_t;
#define __iomem
#define __user
#define __init
#define __force
#define __packed __attribute__((packed))
#define __must_check
#define __printf(a,b) __attribute__((format(printf,a,b)))
#define __initconst
#define __maybe_unused __attribute__((unused))
#ifndef __always_inline
#define __always_inline inline
#endif
#define KERN_ERR ""
#define BIT(n) (1U << (n))
#define U32_MAX UINT32_MAX
#define GFP_KERNEL 0
#define IORESOURCE_MEM 0x00000200
#define IRQ_TYPE_LEVEL_HIGH 4
#define PSTV_OHCI_CMD_STATUS 0x08
#define PSTV_OHCI_HCR 0x00000001
#define IRQ_NONE 0
#define IRQ_HANDLED 1
#define IRQF_SHARED 0x80
#define CAP_SYS_RAWIO 17
#define HC_STATE_RUNNING 1
#define HCD_USB11 1
#define USB_STATE_NOTATTACHED 0
#define THIS_MODULE NULL
#define NOTIFY_OK 0
#define NOTIFY_DONE 1
#define NOTIFY_BAD 2
#define IS_ERR(p) (false)
#define PTR_ERR(p) (-EINVAL)
#define lower_32_bits(x) ((u32)((x) & UINT32_MAX))
#define upper_32_bits(x) ((u32)(((u64)(x)) >> 32))
#define static_assert _Static_assert
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x) = (v))
#define le16_to_cpu(x) (x)
#define cpu_to_le16(x) (x)
#define HCD_HW_ACCESSIBLE(h) ((h)->hw_accessible)
#define resource_size(r) ((r)->end - (r)->start + 1)
#define late_initcall(fn)
#define MODULE_LICENSE(x)
#define DEFINE_MUTEX(name) struct mutex name = { 0 }
#define PM_SUSPEND_PREPARE 1
#define PM_HIBERNATION_PREPARE 2
#define DEFINE_SHOW_ATTRIBUTE(name) static const struct file_operations name##_fops = { 0 }
#define container_of(ptr,type,member) ((type *)((char *)(ptr) - offsetof(type, member)))
#define writel_relaxed(v, addr) writel((v), (addr))
#define EXPORT_SYMBOL_GPL(sym)

typedef struct { int counter; } atomic_t;
struct mutex { int locked; };
struct completion { unsigned done; };
struct device_node { int available; struct device_node *phandle; };
struct device_driver { const char *name; };
struct device { struct device_driver *driver; struct device_node *of_node; int locked; int refs; };
struct resource { unsigned long start, end; };
struct usb_device;
struct usb_bus { struct usb_device *root_hub; struct device *controller; };
struct usb_device { struct device dev; int state; struct usb_device *children[8]; int refs; int locked; int maxchild; };
struct usb_hcd { struct usb_bus self; void __iomem *regs; int state; int hw_accessible; int refs;
	struct hc_driver *driver; unsigned long rsrc_start, rsrc_len; unsigned int irq;
	int skip_phy_initialization; int speed; };
struct hc_driver { const char *description; const char *product_desc; size_t hcd_priv_size;
	int (*reset)(struct usb_hcd *); int (*start)(struct usb_hcd *); void (*stop)(struct usb_hcd *);
	int (*hub_control)(struct usb_hcd *, u16, u16, u16, char *, u16); unsigned long flags; };
struct ohci_driver_overrides { const char *product_desc; size_t extra_priv_size;
	int (*reset)(struct usb_hcd *); };
struct platform_device { struct device dev; struct resource resource; struct usb_hcd *drvdata; int refs; };
struct irq_domain { int tag; };
struct of_phandle_args { struct device_node *np; int args_count; u32 args[8]; };
struct file { int unused; }; struct seq_file { int unused; };
struct dentry { int unused; };
struct inode { int unused; };
struct notifier_block { int (*notifier_call)(struct notifier_block *, unsigned long, void *); };
struct file_operations { void *owner; ssize_t (*write)(struct file *, const char __user *, size_t, loff_t *); };

struct shim_state {
	struct device_node power, ehci, irq_node;
	struct device_driver ehci_driver;
	struct platform_device pdev;
	struct usb_hcd hcd;
	struct usb_device hub;
	struct irq_domain domain;
	u32 ehci_regs[0x100 / 4];
	u32 gate_regs[0x100 / 4];
	int have_power, have_ehci_device, driver_bound, hub_attached, hub_driver;
	int pm_result, request_region_result, ioremap_result;
	int dma_result, dma_bad_alignment;
	int irq_parse_result, wrong_domain, wrong_hwirq, wrong_trigger;
	unsigned int mapped_virq, existing_mapping, create_mapping_result;
	int request_irq_result, mapping_preexisting;
	int region_held, ioremap_count, iounmap_count, readl_count;
	int hcd_gets, hcd_puts, hub_gets, hub_puts, device_locks, device_unlocks;
	int hub_locks, hub_unlocks, pm_gets, pm_puts, irq_requests, irq_frees;
	int irq_disposes, irq_disables, dma_allocs, dma_frees;
	int synchronize_irqs, node_puts, pdev_lookups;
	int sleep_calls, ehci_frame_reads, event_seq, free_irq_seq, dma_free_seq, irq_dispose_seq;
	int mapped_hwirq, domain_is_gic;
	int pm_live, frame_frozen;
	int hcd_creations, hcd_adds, hcd_removes, ohci_init_driver_calls, ohci_setup_calls;
	int hcd_create_result, hcd_add_result, ohci_setup_result;
	struct usb_hcd *created_hcd;
	void *last_drvdata;
};
static struct shim_state shim;

static void shim_reset(void)
{
	memset(&shim, 0, sizeof(shim));
	shim.have_power = shim.have_ehci_device = shim.driver_bound = 1;
	shim.hub_attached = shim.hub_driver = 1;
	shim.pm_result = 0; shim.request_region_result = 1;
	shim.ioremap_result = 1; shim.domain.tag = 1; shim.domain_is_gic = 1; shim.mapped_hwirq = 145;
	shim.ehci_driver.name = "ehci-platform";
	shim.pdev.dev.driver = &shim.ehci_driver;
	shim.pdev.dev.of_node = &shim.ehci;
	shim.pdev.resource.start = 0xe40e0000;
	shim.pdev.resource.end = 0xe40e00ff;
	shim.pdev.drvdata = &shim.hcd;
	shim.hcd.self.root_hub = &shim.hub;
	shim.hcd.regs = shim.ehci_regs; shim.hcd.state = HC_STATE_RUNNING;
	shim.hcd.hw_accessible = 1;
	shim.hub.state = 1; shim.hub.dev.driver = &shim.ehci_driver; shim.hub.maxchild = 1;
	shim.power.available = 1; shim.power.phandle = &shim.ehci;
	shim.ehci.phandle = &shim.irq_node;
	shim.ehci_regs[0] = 0x10; shim.ehci_regs[4] = 1;
	shim.ehci_regs[7] = 100; shim.irq_node.available = 1;
	shim.gate_regs[0] = 0x10;
}

static inline u32 readl(const void *addr)
{
	const unsigned char *p = addr;
	const unsigned char *e = (const unsigned char *)shim.ehci_regs;
	shim.readl_count++;
	if (p >= e && p < e + sizeof(shim.ehci_regs)) {
		assert(shim.pdev.dev.locked && shim.hub.locked && shim.pm_live == 1);
		if (p == e + 7 * sizeof(u32)) return *(const u32 *)p + (shim.frame_frozen ? 0 : shim.ehci_frame_reads++);
		return *(const u32 *)p;
	}
	p = addr; e = (const unsigned char *)shim.gate_regs;
	if (p >= e && p < e + sizeof(shim.gate_regs)) return *(const u32 *)p;
	return 0;
}
static inline void writel(u32 v, void *addr)
{
	unsigned char *p = addr, *e = (unsigned char *)shim.ehci_regs;
	if (p >= e && p < e + sizeof(shim.ehci_regs)) { *(u32 *)p = v; return; }
	p = addr; e = (unsigned char *)shim.gate_regs;
	if (p >= e && p < e + sizeof(shim.gate_regs)) {
		/* Model reset completion: HCR self-clears once the core runs. */
		if (p == e + PSTV_OHCI_CMD_STATUS * 4 / sizeof(u32))
			v &= ~PSTV_OHCI_HCR;
		*(u32 *)p = v;
	}
}
static inline void *ioremap(unsigned long addr, size_t size)
{ (void)addr; (void)size; shim.ioremap_count++; return shim.ioremap_result ? shim.gate_regs : NULL; }
static inline void iounmap(void *addr) { (void)addr; shim.iounmap_count++; }
static inline void dma_wmb(void) { __sync_synchronize(); }
static inline void dma_rmb(void) { __sync_synchronize(); }
static inline void usleep_range(unsigned long a, unsigned long b) { (void)a; (void)b; shim.sleep_calls++; }
static inline void msleep(unsigned int ms) { (void)ms; shim.sleep_calls++; }
static inline unsigned long msecs_to_jiffies(unsigned int ms) { return ms; }

static inline struct device_node *of_find_compatible_node(void *a, void *b, const char *c)
{ (void)a; (void)b; (void)c; return shim.have_power ? &shim.power : NULL; }
static inline int of_device_is_available(const struct device_node *n) { return n && n->available; }
static inline struct device_node *of_parse_phandle(const struct device_node *n, const char *p, int i)
{ (void)p; (void)i; return n ? n->phandle : NULL; }
static inline struct platform_device *of_find_device_by_node(struct device_node *n)
{ (void)n; shim.pdev_lookups++; if (!shim.have_ehci_device) return NULL; shim.pdev.dev.refs++; return &shim.pdev; }
static inline void of_node_put(struct device_node *n) { if (n) shim.node_puts++; }
static inline struct resource *platform_get_resource(struct platform_device *p, unsigned t, unsigned i)
{ (void)t; (void)i; return p ? &p->resource : NULL; }
static inline void *platform_get_drvdata(struct platform_device *p) { return p ? p->drvdata : NULL; }
static inline void dev_set_drvdata(struct device *d, void *data) { if (d == &shim.pdev.dev) { shim.pdev.drvdata = data; shim.last_drvdata = data; } }
static inline int of_irq_parse_one(struct device_node *n, int i, struct of_phandle_args *a)
{ (void)n; (void)i; if (shim.irq_parse_result) return shim.irq_parse_result; a->np = &shim.irq_node; a->args_count=3; a->args[0]=0; a->args[1]=114; a->args[2]=shim.wrong_trigger ? 1 : IRQ_TYPE_LEVEL_HIGH; return 0; }
static inline struct irq_domain *irq_find_host(struct device_node *n)
{ (void)n; return &shim.domain; }
static inline int of_device_is_compatible(const struct device_node *n, const char *c)
{ (void)n; return shim.domain_is_gic && !strcmp(c, "arm,cortex-a9-gic"); }
struct irq_data { unsigned long hwirq; struct irq_domain *domain; };
static inline struct irq_data *irq_get_irq_data(unsigned int virq)
{ static struct irq_data d; (void)virq; d.hwirq=shim.mapped_hwirq; d.domain=&shim.domain; return &d; }
static inline struct irq_data *irq_domain_get_irq_data(struct irq_domain *d, unsigned int virq)
{ struct irq_data *x=irq_get_irq_data(virq); x->domain=d; return x; }
static inline unsigned int irq_find_mapping(struct irq_domain *d, unsigned long hwirq)
{ (void)d; return hwirq == 145 ? shim.existing_mapping : 0; }
static inline unsigned int irq_create_of_mapping(struct of_phandle_args *a)
{ (void)a; shim.mapped_virq = shim.create_mapping_result ? shim.create_mapping_result : 77; return shim.mapped_virq; }
typedef int irqreturn_t;
static inline int request_irq(unsigned int irq, irqreturn_t (*fn)(int, void *), unsigned long f, const char *n, void *d)
{ (void)irq; (void)fn; (void)f; (void)n; (void)d; shim.irq_requests++; return shim.request_irq_result; }
static inline void free_irq(unsigned int irq, void *d) { (void)irq; (void)d; shim.irq_frees++; shim.free_irq_seq=++shim.event_seq; }
static inline void irq_dispose_mapping(unsigned int irq) { (void)irq; shim.irq_disposes++; shim.irq_dispose_seq=++shim.event_seq; }
static inline void disable_irq(unsigned int irq) { (void)irq; shim.irq_disables++; }
static inline void synchronize_irq(unsigned int irq) { (void)irq; shim.synchronize_irqs++; }
static inline int request_mem_region(unsigned long s, unsigned long z, const char *n)
{ (void)s; (void)z; (void)n; if (!shim.request_region_result) return 0; shim.region_held++; return 1; }
static inline void release_mem_region(unsigned long s, unsigned long z)
{ (void)s; (void)z; if (shim.region_held > 0) shim.region_held--; }
static inline int pm_runtime_resume_and_get(struct device *d)
{ (void)d; assert(shim.pdev.dev.locked && shim.hub.locked); shim.pm_gets++; if (shim.pm_result >= 0) shim.pm_live++; return shim.pm_result; }
static inline void pm_runtime_mark_last_busy(struct device *d) { (void)d; }
static inline int pm_runtime_put_sync_autosuspend(struct device *d)
{ (void)d; if (shim.pm_live) shim.pm_live--; shim.pm_puts++; return 0; }
static inline void device_lock(struct device *d) { assert(!d->locked); d->locked=1; shim.device_locks++; }
static inline void device_unlock(struct device *d) { assert(d->locked); d->locked=0; shim.device_unlocks++; }
static inline void usb_lock_device(struct usb_device *d) { assert(!d->locked && shim.pdev.dev.locked); d->locked=1; shim.hub_locks++; }
static inline struct usb_device *usb_hub_find_child(struct usb_device *d, int port)
{ assert(d->locked); return d->children[port - 1]; }
static inline unsigned int irq_get_trigger_type(unsigned int irq)
{ (void)irq; return shim.wrong_trigger ? 1 : IRQ_TYPE_LEVEL_HIGH; }
static inline void usb_unlock_device(struct usb_device *d) { assert(d->locked && shim.pdev.dev.locked); d->locked=0; shim.hub_unlocks++; }
static inline void usb_get_hcd(struct usb_hcd *h) { h->refs++; shim.hcd_gets++; }
static inline void usb_put_hcd(struct usb_hcd *h)
{
	if (!h)
		return;
	shim.hcd_puts++;
	if (h == shim.created_hcd) {
		free(h);
		shim.created_hcd = NULL;
		return;
	}
	assert(h->refs > 0);
	h->refs--;
}
static inline struct usb_device *usb_get_dev(struct usb_device *d) { if (!d) return NULL; d->refs++; shim.hub_gets++; return d; }
static inline void usb_put_dev(struct usb_device *d) { if (d) { assert(d->refs > 0); d->refs--; shim.hub_puts++; } }
static inline void put_device(struct device *d) { assert(d->refs > 0); d->refs--; }

static inline void *dma_alloc_coherent(struct device *d, size_t n, dma_addr_t *dma, gfp_t f)
{ void *p; (void)d; (void)f; shim.dma_allocs++; if (shim.dma_result) return NULL; p=calloc(1,n); *dma=shim.dma_bad_alignment ? 0x12340040 : 0x12340000; return p; }
static inline void dma_free_coherent(struct device *d, size_t n, void *p, dma_addr_t dma)
{ (void)d; (void)n; shim.dma_frees++; shim.dma_free_seq=++shim.event_seq; free(p); }

static inline void init_completion(struct completion *c) { c->done=0; }
static inline void complete(struct completion *c) { c->done=1; }
static inline unsigned long wait_for_completion_timeout(struct completion *c, unsigned long t)
{ (void)t; return c->done ? 1 : 0; }
static inline void atomic_set(atomic_t *a, int v) { a->counter=v; }
static inline void atomic_inc(atomic_t *a) { a->counter++; }
static inline int atomic_read(const atomic_t *a) { return a->counter; }
static inline unsigned int lock_system_sleep(void) { return 0; }
static inline void unlock_system_sleep(unsigned int f) { (void)f; }
static inline int mutex_trylock(struct mutex *m) { if (m->locked) return 0; m->locked=1; return 1; }
static inline void mutex_lock(struct mutex *m) { m->locked=1; }
static inline void mutex_unlock(struct mutex *m) { m->locked=0; }
static inline int capable(int c) { (void)c; return 1; }
static inline int copy_from_user(void *d, const void *s, size_t n) { memcpy(d,s,n); return 0; }
static inline int sysfs_streq(const char *a, const char *b) { size_t n=strlen(a); while(n && (a[n-1]=='\n'||a[n-1]=='\r')) n--; return strlen(b)==n && !strncmp(a,b,n); }
static inline void *kzalloc(size_t n, gfp_t f) { (void)f; return calloc(1,n); }
static inline void kfree(void *p) { free(p); }
static inline int usb_disabled(void) { return 0; }
static inline struct usb_hcd *usb_create_hcd(const struct hc_driver *drv, struct device *dev, const char *n)
{ (void)n; shim.hcd_creations++; if (!shim.hcd_create_result) return NULL; shim.created_hcd = calloc(1, sizeof(*shim.created_hcd)); shim.created_hcd->driver = (struct hc_driver *)drv; shim.created_hcd->self.controller = dev; return shim.created_hcd; }
static inline int usb_add_hcd(struct usb_hcd *h, unsigned int irq, unsigned long f)
{ (void)f; shim.hcd_adds++; h->irq = irq; return shim.hcd_add_result; }
static inline void usb_remove_hcd(struct usb_hcd *h)
{ (void)h; shim.hcd_removes++; if (shim.created_hcd) shim.pdev.drvdata = &shim.hcd; }
static inline void ohci_init_driver(struct hc_driver *drv, const struct ohci_driver_overrides *over)
{ shim.ohci_init_driver_calls++; memset(drv, 0, sizeof(*drv)); drv->description = "ohci_hcd";
	if (over) { drv->product_desc = over->product_desc; drv->hcd_priv_size = 96 + over->extra_priv_size; drv->reset = over->reset; } }
static inline int ohci_setup(struct usb_hcd *h) { shim.ohci_setup_calls++; return shim.ohci_setup_result; }
static inline int ohci_init(struct usb_hcd *h) { return ohci_setup(h); }
static inline void pr_info(const char *f, ...) { (void)f; }
static inline void pr_err(const char *f, ...) { (void)f; }
static inline void seq_printf(struct seq_file *s, const char *f, ...) { (void)s; (void)f; }
static inline int register_pm_notifier(struct notifier_block *n) { (void)n; return 0; }
static inline int unregister_pm_notifier(struct notifier_block *n) { (void)n; return 0; }
static inline struct dentry *debugfs_create_dir(const char *n, struct dentry *p) { (void)n; (void)p; return (struct dentry *)1; }
static inline struct dentry *debugfs_create_file(const char *n, unsigned m, struct dentry *p, void *d, const struct file_operations *f) { (void)n;(void)m;(void)p;(void)d;(void)f;return (struct dentry *)1; }

#endif
