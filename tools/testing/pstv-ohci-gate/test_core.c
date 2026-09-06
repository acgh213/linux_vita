/* SPDX-License-Identifier: GPL-2.0 */
/* Host behavior tests for the PSTV OHCI gate sequencing core. */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../../drivers/usb/host/pstv-ohci-gate-core.h"

struct fake {
	uint32_t regs[0x100 / 4];
	char log[2048];
	size_t log_len;
	unsigned sleeps;
	unsigned wait_sof_calls;
	unsigned progress_calls;
	unsigned irq_count;
	unsigned release_calls;
	int fail_read;
	int fail_write_reg;
	int hcr_stuck;
	int sof_result;
	int progress_result;
	int quiescent_result;
	uint32_t prepared_dma;
};

static void event(struct fake *f, char c)
{
	if (f->log_len + 1 < sizeof(f->log))
		f->log[f->log_len++] = c;
}

static int fake_read(void *ctx, unsigned reg, uint32_t *value)
{
	struct fake *f = ctx;

	event(f, reg == PSTV_OHCI_CMD_STATUS ? 'c' : 'r');
	if (f->fail_read)
		return f->fail_read;
	*value = f->regs[reg / 4];
	if (reg == PSTV_OHCI_CMD_STATUS && !f->hcr_stuck)
		*value &= ~PSTV_OHCI_HCR;
	return 0;
}

static int fake_write(void *ctx, unsigned reg, uint32_t value)
{
	struct fake *f = ctx;

	event(f, reg == PSTV_OHCI_INTR_DISABLE ? 'i' :
	      reg == PSTV_OHCI_CMD_STATUS ? 'h' :
	      reg == PSTV_OHCI_CONTROL ? 'o' :
	      reg == PSTV_OHCI_INTR_ENABLE ? 'e' : 'w');
	if (f->fail_write_reg == (int)reg)
		return -EIO;
	f->regs[reg / 4] = value;
	return 0;
}

static int fake_sleep(void *ctx, unsigned ms)
{
	struct fake *f = ctx;

	(void)ms;
	f->sleeps++;
	event(f, 's');
	return 0;
}

static int fake_prepare(void *ctx, uint32_t *hcca_dma)
{
	struct fake *f = ctx;

	event(f, 'a');
	*hcca_dma = f->prepared_dma ? f->prepared_dma : 0x12340000;
	return 0;
}

static int fake_wait_sof(void *ctx, unsigned timeout_ms)
{
	struct fake *f = ctx;

	(void)timeout_ms;
	f->wait_sof_calls++;
	event(f, 'f');
	return f->sof_result;
}

static int fake_progress(void *ctx, uint32_t *first, uint32_t *last)
{
	struct fake *f = ctx;

	f->progress_calls++;
	event(f, 'p');
	*first = 0x20;
	*last = 0x21;
	return f->progress_result;
}

static int fake_irq_count(void *ctx, uint32_t *count)
{
	struct fake *f = ctx;

	*count = f->irq_count;
	event(f, 'n');
	return 0;
}

static int fake_quiescent(void *ctx)
{
	struct fake *f = ctx;

	event(f, 'q');
	return f->quiescent_result;
}

static void fake_release(void *ctx)
{
	struct fake *f = ctx;

	f->release_calls++;
	event(f, 'x');
}

static const struct pstv_ohci_gate_ops fake_ops = {
	.read = fake_read,
	.write = fake_write,
	.sleep_ms = fake_sleep,
	.prepare_frame = fake_prepare,
	.wait_sof = fake_wait_sof,
	.frame_progress = fake_progress,
	.irq_count = fake_irq_count,
	.quiescent = fake_quiescent,
	.release = fake_release,
};

static int check(int condition, const char *message)
{
	if (!condition) {
		fprintf(stderr, "FAIL: %s\n", message);
		return 1;
	}
	return 0;
}

static int test_successful_sequence(void)
{
	struct fake f = { .regs[PSTV_OHCI_REVISION / 4] = 0x10 };
	struct pstv_ohci_gate_result result = { 0 };
	int ret;

	ret = pstv_ohci_read_stage(&result, &fake_ops, &f);
	if (check(ret == 0, "read stage succeeds"))
		return 1;
	ret = pstv_ohci_reset_stage(&result, &fake_ops, &f);
	if (check(ret == 0, "reset stage succeeds"))
		return 1;
	f.irq_count = 1;
	ret = pstv_ohci_frame_stage(&result, &fake_ops, &f);
	if (check(ret == 0, "frame stage succeeds"))
		return 1;
	ret = pstv_ohci_stop_stage(&result, &fake_ops, &f);
	if (check(ret == 0, "stop stage succeeds"))
		return 1;

	if (check(result.phase == PSTV_OHCI_PHASE_STOPPED, "final phase is stopped") ||
	    check(result.revision == 0x10, "revision captured") ||
	    check(result.hcca_dma == 0x12340000, "HCCA DMA captured") ||
	    check(result.frame_first == 0x20 && result.frame_last == 0x21,
		  "frame progress captured") ||
	    check(f.regs[PSTV_OHCI_FM_INTERVAL_REG / 4] == 0x27782edf,
		  "standard OHCI FI/FSMP value programmed") ||
	    check(f.regs[PSTV_OHCI_PERIODIC_START / 4] == 0x2a2f,
		  "standard OHCI periodic start value programmed") ||
	    check(result.irq_count == 1, "IRQ count captured") ||
	    check(f.release_calls == 1, "resources released once"))
		return 1;
	{
		char *first_reset = strchr(f.log, 'i');
		char *prepare = strchr(f.log, 'a');

		return check(first_reset != NULL && prepare != NULL &&
			     first_reset < prepare,
			     "frame preparation follows reset");
	}
}

static int test_reset_timeout_is_bounded(void)
{
	struct fake f = { .hcr_stuck = 1 };
	struct pstv_ohci_gate_result result = { .phase = PSTV_OHCI_PHASE_READ };
	int ret = pstv_ohci_reset_stage(&result, &fake_ops, &f);

	return check(ret == -ETIMEDOUT, "reset timeout is reported") ||
	       check(f.sleeps == PSTV_OHCI_RESET_TIMEOUT_MS,
		     "reset timeout has exactly the bounded sleep budget") ||
	       check(result.phase == PSTV_OHCI_PHASE_RESET_FAILED,
		     "reset timeout records failed phase");
}

static int test_read_failure_does_not_write(void)
{
	struct fake f = { .fail_read = -EIO };
	struct pstv_ohci_gate_result result = { 0 };
	int ret = pstv_ohci_read_stage(&result, &fake_ops, &f);

	return check(ret == -EIO, "read error is propagated") ||
	       check(strchr(f.log, 'w') == NULL && strchr(f.log, 'h') == NULL,
		     "read failure performs no write");
}

static int test_revision_mismatch_stops_after_first_read(void)
{
	struct fake f = { .regs[PSTV_OHCI_REVISION / 4] = 0x11 };
	struct pstv_ohci_gate_result result = { 0 };
	int ret = pstv_ohci_read_stage(&result, &fake_ops, &f);

	return check(ret == -ENODEV, "revision mismatch is reported") ||
	       check(f.log_len == 1 && f.log[0] == 'r',
		     "revision mismatch performs only the first read");
}

static int test_unaligned_hcca_is_rejected(void)
{
	struct fake f = { .prepared_dma = 0x12340040 };
	struct pstv_ohci_gate_result result = { .phase = PSTV_OHCI_PHASE_RESET };
	int ret = pstv_ohci_frame_stage(&result, &fake_ops, &f);

	return check(ret == -EINVAL, "unaligned HCCA DMA is rejected") ||
	       check(strchr(f.log, 'w') == NULL,
		     "unaligned HCCA is not published to hardware");
}

static int test_zero_irq_count_is_rejected(void)
{
	struct fake f = { 0 };
	struct pstv_ohci_gate_result result = { .phase = PSTV_OHCI_PHASE_RESET };
	int ret = pstv_ohci_frame_stage(&result, &fake_ops, &f);

	return check(ret == -EIO, "zero IRQ count is rejected") ||
	       check(result.phase == PSTV_OHCI_PHASE_FRAME_FAILED,
		     "zero IRQ count records failed phase");
}

static int test_frame_timeout_can_be_stopped_without_release_on_failure(void)
{
	struct fake f = { .sof_result = -ETIMEDOUT };
	struct pstv_ohci_gate_result result = { .phase = PSTV_OHCI_PHASE_RESET };
	int ret;

	ret = pstv_ohci_frame_stage(&result, &fake_ops, &f);
	if (check(ret == -ETIMEDOUT, "SOF timeout is reported"))
		return 1;
	if (check(result.phase == PSTV_OHCI_PHASE_FRAME_FAILED,
		  "SOF timeout records failed phase"))
		return 1;
	f.quiescent_result = -EBUSY;
	ret = pstv_ohci_stop_stage(&result, &fake_ops, &f);
	return check(ret == -EBUSY, "unverified stop is reported") ||
	       check(f.release_calls == 0, "resources stay pinned when stop is unverified");
}

static int test_stage_preconditions(void)
{
	struct fake f = { 0 };
	struct pstv_ohci_gate_result result = { 0 };
	int ret = pstv_ohci_frame_stage(&result, &fake_ops, &f);

	return check(ret == -EINVAL, "frame rejects missing reset") ||
	       check(f.log_len == 0, "precondition failure performs no I/O");
}

int main(void)
{
	return test_successful_sequence() ||
	       test_reset_timeout_is_bounded() ||
	       test_read_failure_does_not_write() ||
	       test_revision_mismatch_stops_after_first_read() ||
	       test_unaligned_hcca_is_rejected() ||
	       test_zero_irq_count_is_rejected() ||
	       test_frame_timeout_can_be_stopped_without_release_on_failure() ||
	       test_stage_preconditions();
}
