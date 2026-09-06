// SPDX-License-Identifier: GPL-2.0
/* Fault each fallible sequencing callback in the actual transaction. */
#define main core_tests_main
#include "test_core.c"
#undef main

static unsigned int call, fail_at, finishes, retained, writes;
#define FAIL_POINT() do { if (++call == fail_at) return -EIO; } while (0)

static int acquire(void *ctx)
{
	(void)ctx;
	FAIL_POINT();
	return 0;
}

static void finish(void *ctx, int keep)
{
	(void)ctx;
	finishes++;
	retained = keep;
}

static int read_reg(void *ctx, unsigned int reg, uint32_t *value)
{
	FAIL_POINT();
	return fake_read(ctx, reg, value);
}

static int write_reg(void *ctx, unsigned int reg, uint32_t value)
{
	writes++;
	FAIL_POINT();
	return fake_write(ctx, reg, value);
}

static int sleep_ms(void *ctx, unsigned int ms)
{
	FAIL_POINT();
	return fake_sleep(ctx, ms);
}

static int prepare(void *ctx, uint32_t *dma)
{
	FAIL_POINT();
	return fake_prepare(ctx, dma);
}

static int wait_sof(void *ctx, unsigned int ms)
{
	FAIL_POINT();
	return fake_wait_sof(ctx, ms);
}

static int progress(void *ctx, uint32_t *first, uint32_t *last)
{
	FAIL_POINT();
	return fake_progress(ctx, first, last);
}

static int count_irq(void *ctx, uint32_t *count)
{
	FAIL_POINT();
	return fake_irq_count(ctx, count);
}

static int quiescent(void *ctx)
{
	FAIL_POINT();
	return fake_quiescent(ctx);
}

static const struct pstv_ohci_gate_ops ops = {
	.acquire = acquire, .finish = finish,
	.read = read_reg, .write = write_reg, .sleep_ms = sleep_ms,
	.prepare_frame = prepare, .wait_sof = wait_sof,
	.frame_progress = progress, .irq_count = count_irq,
	.quiescent = quiescent, .release = fake_release,
};

static int run(unsigned int stage, unsigned int fault, uint32_t control)
{
	struct fake f = {
		.regs[PSTV_OHCI_REVISION / 4] = 0x10,
		.regs[PSTV_OHCI_CONTROL / 4] = control,
		.irq_count = 1, .fail_write_reg = -1,
	};
	struct pstv_ohci_gate_result result = { 0 };
	int expected = stage > 2 ? -EINVAL : fault ? -EIO :
		(stage && control) ? -EBUSY : 0;
	int ret;

	call = finishes = retained = writes = 0;
	fail_at = fault;
	ret = pstv_ohci_transaction(&result, &ops, &f, stage);
	if (check(ret == expected, "correct transaction errno") ||
	    check(finishes == (stage <= 2), "balanced finish count") ||
	    check(!retained || !f.release_calls, "quarantine never frees DMA") ||
	    check(f.release_calls <= 1, "no double resource release") ||
	    check(stage != 0 || writes == 0, "read mode has zero writes") ||
	    check(!control || !writes, "existing owner is not modified")) {
		fprintf(stderr, "stage=%u fault=%u control=%x rc=%d\n",
			stage, fault, control, ret);
		return 1;
	}
	return 0;
}

int main(void)
{
	static const uint32_t owned[] = { 0x100, 0x80, 0x40, 0x04, 0x08, 0x10, 0x20 };
	unsigned int stage, point, limit, i, cases = 0;

	for (stage = 0; stage <= 2; stage++) {
		if (run(stage, 0, 0))
			return 1;
		cases++;
		limit = call;
		for (point = 1; point <= limit; point++) {
			if (run(stage, point, 0))
				return 1;
			cases++;
		}
	}
	for (stage = 1; stage <= 2; stage++)
		for (i = 0; i < sizeof(owned) / sizeof(owned[0]); i++) {
			if (run(stage, 0, owned[i]))
				return 1;
			cases++;
		}
	if (run(3, 0, 0))
		return 1;
	printf("transaction fault/ownership matrix PASS cases=%u\n", cases + 1);
	return 0;
}
