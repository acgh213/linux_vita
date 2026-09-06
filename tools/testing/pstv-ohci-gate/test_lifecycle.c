// SPDX-License-Identifier: GPL-2.0
/* Exercise the same transaction wrapper used by the debugfs backend. */
#define main core_tests_main
#include "test_core.c"
#undef main

static int acquire_error, acquired, finished, retained;

static int acquire(void *ctx)
{
	(void)ctx;
	acquired++;
	return acquire_error;
}

static void finish(void *ctx, int keep)
{
	(void)ctx;
	finished++;
	retained = keep;
}

static int lifecycle_test(unsigned int stage, int start_error,
			  int stuck, int stop_error, int expected,
			  int keep)
{
	struct fake f = {
		.regs[PSTV_OHCI_REVISION / 4] = 0x10,
		.irq_count = 1,
		.hcr_stuck = stuck,
		.quiescent_result = stop_error,
		.fail_write_reg = -1,
	};
	struct pstv_ohci_gate_ops ops = fake_ops;
	struct pstv_ohci_gate_result result = { 0 };
	int ret;

	ops.acquire = acquire;
	ops.finish = finish;
	acquire_error = start_error;
	acquired = finished = retained = 0;
	ret = pstv_ohci_transaction(&result, &ops, &f, stage);
	return check(ret == expected, "transaction preserves failure") ||
	       check(acquired == 1 && finished == 1,
		     "every acquisition attempt has one finish") ||
	       check(retained == keep, "only unproven stops retain resources") ||
	       check(!(start_error && f.log_len), "no I/O after preflight failure") ||
	       check(!(keep && f.release_calls), "poison never releases DMA") ||
	       check(stage != 0 || !strpbrk(f.log, "ihoew"),
		     "read transaction never writes");
}

int main(void)
{
	int ret = core_tests_main();

	ret |= lifecycle_test(0, 0, 0, 0, 0, 0);
	ret |= lifecycle_test(1, 0, 0, 0, 0, 0);
	ret |= lifecycle_test(2, 0, 0, 0, 0, 0);
	ret |= lifecycle_test(0, -ENODEV, 0, 0, -ENODEV, 0);
	ret |= lifecycle_test(2, -ENOMEM, 0, 0, -ENOMEM, 0);
	ret |= lifecycle_test(1, 0, 1, 0, -ETIMEDOUT, 1);
	ret |= lifecycle_test(2, 0, 0, -EBUSY, -EBUSY, 1);
	if (!ret)
		puts("core and lifecycle tests PASS");
	return ret;
}
