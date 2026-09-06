// SPDX-License-Identifier: GPL-2.0
#include "pstv-ohci-gate-core.h"

#ifdef __KERNEL__
#include <linux/build_bug.h>
#include <linux/array_size.h>
#include <linux/errno.h>
#else
#include <errno.h>
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

static int gate_error(struct pstv_ohci_gate_result *result,
		      enum pstv_ohci_gate_phase phase, int error)
{
	result->phase = phase;
	result->status = error;
	return error;
}

static int gate_valid(const struct pstv_ohci_gate_result *result,
		      const struct pstv_ohci_gate_ops *ops)
{
	if (!result || !ops || !ops->read || !ops->write || !ops->sleep_ms)
		return -EINVAL;
	return 0;
}

int pstv_ohci_read_stage(struct pstv_ohci_gate_result *result,
			 const struct pstv_ohci_gate_ops *ops, void *ctx)
{
	static const unsigned int registers[] = {
		PSTV_OHCI_REVISION,
		PSTV_OHCI_CONTROL,
		PSTV_OHCI_CMD_STATUS,
		PSTV_OHCI_INTR_STATUS,
		PSTV_OHCI_INTR_ENABLE,
		PSTV_OHCI_HCCA,
		PSTV_OHCI_FM_INTERVAL_REG,
		PSTV_OHCI_PERIODIC_START,
		PSTV_OHCI_LS_THRESHOLD,
		PSTV_OHCI_FRAME_NUMBER,
	};
	pstv_ohci_u32 value;
	unsigned int i;
	int ret;

	ret = gate_valid(result, ops);
	if (ret)
		return ret;
	if (result->phase != PSTV_OHCI_PHASE_IDLE &&
	    result->phase != PSTV_OHCI_PHASE_READ_FAILED)
		return gate_error(result, PSTV_OHCI_PHASE_READ_FAILED, -EINVAL);

	/*
	 * Revision is the read-only admission check; do not touch the rest of
	 * the window when this first read disagrees. EHCI must already be awake.
	 */
	ret = ops->read(ctx, PSTV_OHCI_REVISION, &value);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_READ_FAILED, ret);
	result->revision = value;
	if ((value & 0xff) != 0x10)
		return gate_error(result, PSTV_OHCI_PHASE_READ_FAILED, -ENODEV);

	for (i = 1; i < ARRAY_SIZE(registers); i++) {
		ret = ops->read(ctx, registers[i], &value);
		if (ret)
			return gate_error(result, PSTV_OHCI_PHASE_READ_FAILED, ret);
		switch (registers[i]) {
		case PSTV_OHCI_REVISION:
			result->revision = value;
			break;
		case PSTV_OHCI_CONTROL:
			result->control = value;
			break;
		case PSTV_OHCI_CMD_STATUS:
			result->cmd_status = value;
			break;
		case PSTV_OHCI_INTR_STATUS:
			result->intr_status = value;
			break;
		case PSTV_OHCI_INTR_ENABLE:
			result->intr_enable = value;
			break;
		case PSTV_OHCI_HCCA:
			result->hcca = value;
			break;
		case PSTV_OHCI_FM_INTERVAL_REG:
			result->fm_interval = value;
			break;
		case PSTV_OHCI_PERIODIC_START:
			result->periodic_start = value;
			break;
		case PSTV_OHCI_LS_THRESHOLD:
			result->ls_threshold = value;
			break;
		default:
			break;
		}
	}

	result->phase = PSTV_OHCI_PHASE_READ;
	result->status = 0;
	return 0;
}

static int gate_reset_hw(struct pstv_ohci_gate_result *result,
			 const struct pstv_ohci_gate_ops *ops, void *ctx)
{
	pstv_ohci_u32 value;
	unsigned int i;
	int ret;

	/* Sony's order is intentional: HCR is asserted before Control=0. */
	ret = ops->write(ctx, PSTV_OHCI_INTR_DISABLE, PSTV_OHCI_IRQ_MASK);
	if (ret)
		return ret;
	ret = ops->write(ctx, PSTV_OHCI_CMD_STATUS, PSTV_OHCI_HCR);
	if (ret)
		return ret;
	ret = ops->write(ctx, PSTV_OHCI_CONTROL, 0);
	if (ret)
		return ret;

	for (i = 0; i < PSTV_OHCI_RESET_TIMEOUT_MS; i++) {
		ret = ops->read(ctx, PSTV_OHCI_CMD_STATUS, &value);
		if (ret)
			return ret;
		result->cmd_status = value;
		if (!(value & PSTV_OHCI_HCR))
			return 0;
		ret = ops->sleep_ms(ctx, 1);
		if (ret)
			return ret;
	}
	return -ETIMEDOUT;
}

int pstv_ohci_reset_stage(struct pstv_ohci_gate_result *result,
			  const struct pstv_ohci_gate_ops *ops, void *ctx)
{
	int ret;

	ret = gate_valid(result, ops);
	if (ret)
		return ret;
	if (result->phase != PSTV_OHCI_PHASE_READ &&
	    result->phase != PSTV_OHCI_PHASE_RESET_FAILED)
		return gate_error(result, PSTV_OHCI_PHASE_RESET_FAILED, -EINVAL);

	ret = gate_reset_hw(result, ops, ctx);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_RESET_FAILED, ret);
	result->phase = PSTV_OHCI_PHASE_RESET;
	result->status = 0;
	return 0;
}

int pstv_ohci_frame_stage(struct pstv_ohci_gate_result *result,
			  const struct pstv_ohci_gate_ops *ops, void *ctx)
{
	pstv_ohci_u32 irq_count;
	int ret;

	ret = gate_valid(result, ops);
	if (ret)
		return ret;
	if (!ops->prepare_frame || !ops->wait_sof || !ops->frame_progress ||
	    !ops->irq_count)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, -EINVAL);
	if (result->phase != PSTV_OHCI_PHASE_RESET)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, -EINVAL);

	ret = ops->prepare_frame(ctx, &result->hcca_dma);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	if (!result->hcca_dma || (result->hcca_dma & 0xff))
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, -EINVAL);
	ret = ops->write(ctx, PSTV_OHCI_HCCA, result->hcca_dma);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_PERIOD_CURRENT_ED, 0);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_CONTROL_HEAD_ED, 0);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_BULK_HEAD_ED, 0);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_FM_INTERVAL_REG,
			PSTV_OHCI_FM_INTERVAL_VALUE);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_PERIODIC_START, PSTV_OHCI_PERIODIC);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_LS_THRESHOLD, PSTV_OHCI_LST);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_INTR_DISABLE, PSTV_OHCI_IRQ_MASK);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_INTR_STATUS, PSTV_OHCI_IRQ_MASK);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_INTR_ENABLE, PSTV_OHCI_IRQ_ENABLE);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_CONTROL, PSTV_OHCI_OPERATIONAL);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->wait_sof(ctx, PSTV_OHCI_SOF_TIMEOUT_MS);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	ret = ops->frame_progress(ctx, &result->frame_first,
				  &result->frame_last);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	if (result->frame_first == result->frame_last)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, -ETIMEDOUT);
	ret = ops->irq_count(ctx, &irq_count);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, ret);
	if (!irq_count)
		return gate_error(result, PSTV_OHCI_PHASE_FRAME_FAILED, -EIO);
	result->irq_count = irq_count;
	result->phase = PSTV_OHCI_PHASE_FRAME;
	result->status = 0;
	return 0;
}

int pstv_ohci_stop_stage(struct pstv_ohci_gate_result *result,
			 const struct pstv_ohci_gate_ops *ops, void *ctx)
{
	pstv_ohci_u32 value;
	unsigned int i;
	int ret;

	ret = gate_valid(result, ops);
	if (ret)
		return ret;
	if (!ops->quiescent || !ops->release)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, -EINVAL);
	if (result->phase != PSTV_OHCI_PHASE_RESET &&
	    result->phase != PSTV_OHCI_PHASE_RESET_FAILED &&
	    result->phase != PSTV_OHCI_PHASE_FRAME &&
	    result->phase != PSTV_OHCI_PHASE_FRAME_FAILED &&
	    result->phase != PSTV_OHCI_PHASE_STOP_FAILED)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, -EINVAL);

	ret = ops->write(ctx, PSTV_OHCI_INTR_DISABLE, PSTV_OHCI_IRQ_MASK);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_INTR_STATUS, PSTV_OHCI_IRQ_MASK);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_CMD_STATUS, PSTV_OHCI_HCR);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, ret);
	ret = ops->write(ctx, PSTV_OHCI_CONTROL, 0);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, ret);

	for (i = 0; i < PSTV_OHCI_RESET_TIMEOUT_MS; i++) {
		ret = ops->read(ctx, PSTV_OHCI_CMD_STATUS, &value);
		if (ret)
			return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, ret);
		result->cmd_status = value;
		if (!(value & PSTV_OHCI_HCR))
			break;
		ret = ops->sleep_ms(ctx, 1);
		if (ret)
			return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, ret);
	}
	if (i == PSTV_OHCI_RESET_TIMEOUT_MS)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, -ETIMEDOUT);
	ret = ops->quiescent(ctx);
	if (ret)
		return gate_error(result, PSTV_OHCI_PHASE_STOP_FAILED, ret);
	ops->release(ctx);
	result->phase = PSTV_OHCI_PHASE_STOPPED;
	result->status = 0;
	return 0;
}

int pstv_ohci_transaction(struct pstv_ohci_gate_result *result,
			  const struct pstv_ohci_gate_ops *ops, void *ctx,
			  unsigned int stage)
{
	int ret, stop = 0;

	if (gate_valid(result, ops) || !ops->acquire || !ops->finish || stage > 2)
		return -EINVAL;
	*result = (struct pstv_ohci_gate_result) { 0 };
	ret = ops->acquire(ctx);
	if (ret)
		goto out;
	ret = pstv_ohci_read_stage(result, ops, ctx);
	if (ret || !stage)
		goto out;
	/* Do not take over an active controller or firmware-owned schedules. */
	if (result->control & (0x100 | 0x3c) ||
	    (result->control & 0xc0) == 0x80 ||
	    (result->control & 0xc0) == 0x40) {
		ret = -EBUSY;
		goto out;
	}
	ret = pstv_ohci_reset_stage(result, ops, ctx);
	if (!ret && stage == 2)
		ret = pstv_ohci_frame_stage(result, ops, ctx);
	stop = pstv_ohci_stop_stage(result, ops, ctx);
out:
	result->cleanup_status = stop;
	result->quarantined = !!stop;
	result->status = ret ? ret : stop;
	ops->finish(ctx, !!stop);
	return result->status;
}
