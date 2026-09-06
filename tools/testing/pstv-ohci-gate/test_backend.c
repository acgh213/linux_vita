/* SPDX-License-Identifier: GPL-2.0 */
/* Host tests include and exercise the actual backend, never a copy. */
#include "backend-shim.h"
#include "../../../drivers/usb/host/pstv-ohci-gate.c"
#include "../../../drivers/usb/host/pstv-ohci-hcdgate.c"

static int failures;
#define CHECK(c, m) do { if (!(c)) { fprintf(stderr, "FAIL: %s\n", m); failures++; } } while (0)

static void fresh(void)
{
	if (gate_hcd_active()) {
		gate_hcd_down();
		if (hcd_gate_session) {
			gate_finish(hcd_gate_session, 0);
			kfree(hcd_gate_session);
			hcd_gate_session = NULL;
		}
		hcd_active = false;
	}
	shim_reset();
	if (!gate_hcd_active())
		hcd_gate_session = NULL;
	poisoned = false; attempted = false; last_stage = 0; last_result = (struct pstv_ohci_gate_result){0};
}

static struct gate_session *acquired(void)
{
	static struct gate_session g;
	memset(&g, 0, sizeof(g));
	int ret = gate_acquire(&g);
	CHECK(ret == 0, "normal acquire succeeds");
	return &g;
}

static void clean_release(struct gate_session *g)
{
	gate_finish(g, 0);
	CHECK(shim.pdev.dev.refs == 0 && shim.hcd.refs == 0 && shim.hub.refs == 0,
	      "every non-quarantined unwind balances all device references");
	CHECK(shim.pm_live == 0 && shim.region_held == 0 &&
	      !shim.pdev.dev.locked && !shim.hub.locked,
	      "every non-quarantined unwind balances PM/region/locks");
}

static void test_acquire_failures(void)
{
	struct gate_session g;
	int *flags[] = { &shim.have_power, &shim.have_ehci_device };
	const char *names[] = { "missing power node", "missing EHCI device" };
	unsigned i;
	for (i=0; i<2; i++) {
		fresh(); *flags[i]=0; memset(&g,0,sizeof(g));
		CHECK(gate_acquire(&g) < 0, names[i]);
		clean_release(&g);
		CHECK(shim.device_locks == shim.device_unlocks && shim.hub_locks == shim.hub_unlocks, "early acquire locks balance");
	}
	fresh(); shim.driver_bound=0; shim.pdev.dev.driver=NULL; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-ENODEV, "unbound driver rejected"); clean_release(&g);
	fresh(); shim.pdev.resource.start=0; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-EINVAL, "bad resource rejected"); clean_release(&g);
	fresh(); shim.pdev.drvdata=NULL; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-ENODEV, "null HCD rejected"); clean_release(&g);
	fresh(); shim.hub_attached=0; shim.hub.state=USB_STATE_NOTATTACHED; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-ENODEV, "unattached hub rejected"); clean_release(&g);
	fresh(); shim.hub_driver=0; shim.hub.dev.driver=NULL; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-ENODEV, "hub without driver rejected"); clean_release(&g);
	fresh(); shim.pm_result=-EIO; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-EIO, "PM failure propagated"); clean_release(&g);
	fresh(); shim.ehci_regs[0]=0x11; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-EINVAL, "EHCI progress preflight failure propagated"); clean_release(&g);
	fresh(); shim.request_region_result=0; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-EBUSY, "region conflict propagated"); clean_release(&g);
	fresh(); shim.ioremap_result=0; memset(&g,0,sizeof(g));
	CHECK(gate_acquire(&g)==-ENOMEM, "ioremap failure propagated"); clean_release(&g);
	CHECK(shim.region_held == 0, "failed acquires do not leave region held");
}

static void test_normal_acquire_balances(void)
{
	struct gate_session *g;
	fresh(); g=acquired(); clean_release(g);
	CHECK(shim.device_locks==shim.device_unlocks && shim.hub_locks==shim.hub_unlocks, "normal locks balance");
	CHECK(shim.hcd_gets==shim.hcd_puts && shim.hub_gets==shim.hub_puts, "normal refs balance");
	CHECK(shim.pm_gets==shim.pm_puts && shim.region_held==0, "normal PM and region balance");
}

static void test_child_hub_is_exclusive(void)
{
	struct usb_device child={0}; struct gate_session g={0}; int before;
	fresh(); shim.hub.children[0]=&child; before=shim.readl_count;
	CHECK(gate_acquire(&g)==-EBUSY, "existing USB hub child rejects acquisition");
	CHECK(shim.readl_count==before, "child rejection performs no EHCI access");
	clean_release(&g);
	CHECK(shim.hcd_gets==shim.hcd_puts && shim.hub_gets==shim.hub_puts, "child rejection balances refs");
}

static void prepare_case(int wrong_domain, int wrong_hwirq, int wrong_trigger)
{
	struct gate_session *g; u32 dma=0; int ret;
	fresh(); g=acquired(); shim.wrong_domain=wrong_domain; shim.domain_is_gic=!wrong_domain;
	shim.wrong_hwirq=wrong_hwirq; shim.mapped_hwirq=wrong_hwirq ? 144 : 145;
	shim.wrong_trigger=wrong_trigger;
	ret=gate_prepare_frame(g,&dma);
	CHECK(ret < 0, "invalid IRQ description rejected");
	CHECK(shim.irq_requests==0 && shim.dma_frees==0, "invalid IRQ description has no IRQ action");
	clean_release(g);
}

static void test_irq_description_validation(void)
{
	prepare_case(1,0,0); prepare_case(0,1,0); prepare_case(0,0,1);
}

static void test_mapping_race_does_not_dispose_shared(void)
{
	struct gate_session *g; u32 dma=0; int ret;
	fresh(); g=acquired(); shim.create_mapping_result=77; shim.request_irq_result=-EBUSY; shim.mapping_preexisting=1;
	ret=gate_prepare_frame(g,&dma);
	CHECK(ret==-EBUSY, "request IRQ failure is propagated");
	CHECK(g->irq==77 && !g->irq_requested, "failed request retains mapping identity for release");
	gate_release(g);
	CHECK(shim.irq_disposes==0, "shared mapping is not disposed after request race");
	CHECK(shim.dma_frees==1 && shim.free_irq_seq==0, "mapping race still frees DMA without fake action");
	clean_release(g);
}

static void test_release_order_and_poison(void)
{
	struct gate_session *g; u32 dma=0; int ret;
	fresh(); g=acquired(); shim.create_mapping_result=77; ret=gate_prepare_frame(g,&dma);
	CHECK(ret==0, "valid frame preparation succeeds"); g->irq_requested=true;
	gate_release(g);
	CHECK(shim.irq_frees==1 && shim.irq_disposes==0 && shim.dma_frees==1, "release frees action before DMA without disposing persistent mapping");
	CHECK(shim.free_irq_seq < shim.dma_free_seq, "free_irq precedes DMA free"); clean_release(g);
	fresh(); g=acquired(); g->irq=77; g->irq_requested=true; g->hcca=calloc(1,sizeof(*g->hcca)); g->dma=0x12340000; g->region=true; g->pm_held=true;
	gate_finish(g,1);
	CHECK(poisoned && shim.irq_disables==1, "poison disables owned IRQ");
	CHECK(shim.irq_frees==0 && shim.dma_frees==0 && shim.region_held==1 && shim.pm_puts==0, "poison retains hardware resources");
	CHECK(shim.device_unlocks==0 && shim.hub_unlocks==0 && shim.pdev.dev.locked && shim.hub.locked,
	      "poison retains device and hub locks to block teardown");
	CHECK(shim.pdev.dev.refs == 1 && shim.hcd.refs == 1 && shim.hub.refs == 1 && shim.pm_live == 1,
	      "poison retains all device references and the runtime PM hold");
	free(g->hcca); g->hcca=NULL; iounmap(g->regs); shim.region_held=0;
}

static void test_trigger_validation(void)
{
	struct file f={0}; loff_t pos=0;
	const char *bad[]={"", "bogus", "read\0x", "0123456789012345"};
	const size_t lengths[]={0, 5, 6, 16}; unsigned i;
	fresh(); for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) CHECK(gate_trigger(&f,bad[i],lengths[i],&pos)<0,"invalid trigger rejected");
	CHECK(shim.readl_count==0 && shim.irq_requests==0, "invalid trigger has no hardware actions");
}

static void test_hcd_trigger_registers_and_refuses(void)
{
	struct file f={0}; loff_t pos=0;
	const char *cmd="hcd";
	size_t n=3;

	/* Existing child: admission must refuse before any HCD work. */
	fresh();
	{
		static struct usb_device child;

		shim.hub.children[0] = &child;
	}
	CHECK(gate_trigger(&f, cmd, n, &pos) < 0, "hcd trigger refused with child attached");
	CHECK(shim.hcd_creations == 0, "no hcd created on refused admission");

	/* Clean bus: registration goes through the real gate_hcd_up path. */
	fresh(); shim.hcd_create_result=1; shim.hcd_add_result=0;
	shim.gate_regs[0]=0x10; /* revision */
	{
		ssize_t ret = gate_trigger(&f, cmd, n, &pos);
		if (ret != (ssize_t)n)
			fprintf(stderr, "hcd trigger ret=%zd phase=%u status=%d cleanup=%d creates=%d adds=%d pm=%d\\n",
				ret, last_result.phase, last_result.status,
				last_result.cleanup_status, shim.hcd_creations,
				shim.hcd_adds, shim.pm_live);
		CHECK(ret == (ssize_t)n, "hcd trigger accepted");
	}
	CHECK(shim.hcd_creations == 1 && shim.hcd_adds == 1, "hcd registered through trigger");
	CHECK(shim.irq_requests == 0, "gate does not request the IRQ in hcd mode");
}

static void test_hcddown_trigger(void)
{
	struct file f={0}; loff_t pos=0;

	fresh(); shim.hcd_create_result=1; shim.hcd_add_result=0;
	shim.gate_regs[0]=0x10;
	CHECK(gate_trigger(&f, "hcd", 3, &pos) == 3, "hcd trigger accepted");
	CHECK(gate_hcd_active(), "hcd active after trigger");
	CHECK(gate_trigger(&f, "hcd-down", 8, &pos) == 8, "hcd-down accepted");
	CHECK(!gate_hcd_active(), "hcd inactive after hcd-down");
	CHECK(shim.hcd_removes == 1, "usb_remove_hcd called once");
}

int main(void)
{
	test_acquire_failures(); test_normal_acquire_balances(); test_child_hub_is_exclusive();
	test_irq_description_validation(); test_mapping_race_does_not_dispose_shared();
	test_release_order_and_poison(); test_trigger_validation();
	test_hcd_trigger_registers_and_refuses();
	test_hcddown_trigger();
	if (failures) { fprintf(stderr, "backend tests: %d failure(s)\n", failures); return 1; }
	puts("backend tests: PASS"); return 0;
}
