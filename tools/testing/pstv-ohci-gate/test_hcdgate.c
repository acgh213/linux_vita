// SPDX-License-Identifier: GPL-2.0
/* Host harness includes the actual HCD-gate production file, never a copy. */
#include "backend-shim.h"
#include "../../../drivers/usb/host/pstv-ohci-hcdgate.c"

static int failures;
#define CHECK(c, m) do { if (!(c)) { fprintf(stderr, "FAIL: %s\n", m); failures++; } } while (0)

static void fresh(void)
{
	shim_reset();
	hcd_active = false;
}

static void test_hcd_up_and_flags(void)
{
	fresh();
	shim.hcd_create_result = 1; shim.hcd_add_result = 0;
	CHECK(gate_hcd_up(&shim.pdev, shim.gate_regs, 42) == 0,
	      "hcd registration succeeds on live gate");
	CHECK(shim.hcd_creations == 1 && shim.hcd_adds == 1,
	      "one hcd created and added");
	CHECK(shim.ohci_init_driver_calls == 1,
	      "ohci driver table initialized once");
	CHECK(shim.created_hcd && shim.created_hcd->driver == &gate_hc_driver,
	      "hcd carries the vita hc_driver");
	CHECK(shim.created_hcd->rsrc_start == HCDGATE_BASE &&
	      shim.created_hcd->rsrc_len == HCDGATE_SIZE,
	      "hcd resource window matches OHCI companion");
	CHECK(shim.created_hcd->irq == 42, "hcd uses the provided IRQ");
	CHECK(gate_hcd_active(), "gate reports active hcd");
	CHECK(gate_hcd_up(&shim.pdev, shim.gate_regs, 42) == -EBUSY,
	      "second registration refused while active");
	usb_remove_hcd(shim.created_hcd);
	usb_put_hcd(shim.created_hcd);
	hcd_active = false;
	CHECK(shim.created_hcd == NULL, "teardown releases the hcd allocation");
}

static void test_hcd_create_failure(void)
{
	fresh();
	shim.hcd_create_result = 0;
	CHECK(gate_hcd_up(&shim.pdev, shim.gate_regs, 42) == -ENOMEM,
	      "usb_create_hcd failure returns -ENOMEM");
	CHECK(shim.hcd_adds == 0 && !gate_hcd_active(),
	      "no hcd added or marked active on create failure");
}

static void test_hcd_add_failure(void)
{
	fresh();
	shim.hcd_create_result = 1; shim.hcd_add_result = -EINVAL;
	CHECK(gate_hcd_up(&shim.pdev, shim.gate_regs, 42) == -EINVAL,
	      "usb_add_hcd failure propagated");
	CHECK(!gate_hcd_active(), "failed add does not mark hcd active");
	CHECK(shim.created_hcd == NULL, "failed add released the hcd allocation");
}

static void test_reset_override_writes_sony_order(void)
{
	u32 before_disable, before_cmd, before_ctrl;
	struct usb_hcd h = {0};

	fresh();
	h.regs = shim.gate_regs;
	before_disable = shim.gate_regs[PSTV_OHCI_INTR_DISABLE / 4];
	before_cmd = shim.gate_regs[PSTV_OHCI_CMD_STATUS / 4];
	before_ctrl = shim.gate_regs[PSTV_OHCI_CONTROL / 4];
	shim.gate_regs[PSTV_OHCI_CONTROL / 4] = 0xc0;
	before_ctrl = 0xc0;
	CHECK(gate_hcd_reset(&h) == 0, "reset override succeeds");
	CHECK(shim.gate_regs[PSTV_OHCI_INTR_DISABLE / 4] == PSTV_OHCI_IRQ_MASK &&
	      shim.gate_regs[PSTV_OHCI_INTR_DISABLE / 4] != before_disable,
	      "reset override masks interrupts");
	CHECK(shim.gate_regs[PSTV_OHCI_CMD_STATUS / 4] == PSTV_OHCI_HCR &&
	      shim.gate_regs[PSTV_OHCI_CMD_STATUS / 4] != before_cmd,
	      "reset override asserts HCR");
	CHECK(shim.gate_regs[PSTV_OHCI_CONTROL / 4] == 0 &&
	      shim.gate_regs[PSTV_OHCI_CONTROL / 4] != before_ctrl,
	      "reset override zeroes control");
}

int main(void)
{
	test_hcd_up_and_flags();
	test_hcd_create_failure();
	test_hcd_add_failure();
	test_reset_override_writes_sony_order();
	if (failures) { fprintf(stderr, "hcdgate tests: %d failure(s)\n", failures); return 1; }
	puts("hcdgate tests: PASS");
	return 0;
}
