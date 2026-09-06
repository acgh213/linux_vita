/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _PSTV_OHCI_GATE_CORE_H
#define _PSTV_OHCI_GATE_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
typedef u32 pstv_ohci_u32;
#else
#include <stdint.h>
typedef uint32_t pstv_ohci_u32;
#endif

/* OHCI operational register offsets used by the diagnostic gate. */
#define PSTV_OHCI_REVISION		0x00
#define PSTV_OHCI_CONTROL		0x04
#define PSTV_OHCI_CMD_STATUS		0x08
#define PSTV_OHCI_INTR_STATUS		0x0c
#define PSTV_OHCI_INTR_ENABLE		0x10
#define PSTV_OHCI_INTR_DISABLE		0x14
#define PSTV_OHCI_HCCA			0x18
#define PSTV_OHCI_PERIOD_CURRENT_ED	0x1c
#define PSTV_OHCI_CONTROL_HEAD_ED	0x20
#define PSTV_OHCI_BULK_HEAD_ED		0x28
#define PSTV_OHCI_FRAME_NUMBER		0x3c
#define PSTV_OHCI_FM_INTERVAL_REG	0x34
#define PSTV_OHCI_PERIODIC_START	0x40
#define PSTV_OHCI_LS_THRESHOLD		0x44

#define PSTV_OHCI_HCR			0x00000001
#define PSTV_OHCI_OPERATIONAL		0x00000080
#define PSTV_OHCI_IRQ_MASK		0xc000007f
#define PSTV_OHCI_IRQ_ENABLE		0x80000004
#define PSTV_OHCI_FI			11999
#define PSTV_OHCI_FM_INTERVAL_VALUE	0x27782edf
#define PSTV_OHCI_PERIODIC		0x2a2f
#define PSTV_OHCI_LST			0x628
#define PSTV_OHCI_RESET_TIMEOUT_MS	1000
#define PSTV_OHCI_SOF_TIMEOUT_MS	100

enum pstv_ohci_gate_phase {
	PSTV_OHCI_PHASE_IDLE,
	PSTV_OHCI_PHASE_READ,
	PSTV_OHCI_PHASE_READ_FAILED,
	PSTV_OHCI_PHASE_RESET,
	PSTV_OHCI_PHASE_RESET_FAILED,
	PSTV_OHCI_PHASE_FRAME,
	PSTV_OHCI_PHASE_FRAME_FAILED,
	PSTV_OHCI_PHASE_STOPPED,
	PSTV_OHCI_PHASE_STOP_FAILED,
};

struct pstv_ohci_gate_result {
	enum pstv_ohci_gate_phase phase;
	int status;
	int cleanup_status;
	int quarantined;
	pstv_ohci_u32 revision;
	pstv_ohci_u32 control;
	pstv_ohci_u32 cmd_status;
	pstv_ohci_u32 intr_status;
	pstv_ohci_u32 intr_enable;
	pstv_ohci_u32 hcca;
	pstv_ohci_u32 fm_interval;
	pstv_ohci_u32 periodic_start;
	pstv_ohci_u32 ls_threshold;
	pstv_ohci_u32 hcca_dma;
	pstv_ohci_u32 frame_first;
	pstv_ohci_u32 frame_last;
	pstv_ohci_u32 irq_count;
};

/*
 * The core deliberately knows nothing about MMIO, DMA, IRQ domains, or PM.
 * The kernel wrapper supplies these operations, while host tests inject a
 * traceable implementation.  Operations are called in the documented order.
 */
struct pstv_ohci_gate_ops {
	int (*acquire)(void *ctx);
	void (*finish)(void *ctx, int retain);
	int (*read)(void *ctx, unsigned int reg, pstv_ohci_u32 *value);
	int (*write)(void *ctx, unsigned int reg, pstv_ohci_u32 value);
	int (*sleep_ms)(void *ctx, unsigned int milliseconds);
	int (*prepare_frame)(void *ctx, pstv_ohci_u32 *hcca_dma);
	int (*wait_sof)(void *ctx, unsigned int timeout_ms);
	int (*frame_progress)(void *ctx, pstv_ohci_u32 *first,
				      pstv_ohci_u32 *last);
	int (*irq_count)(void *ctx, pstv_ohci_u32 *count);
	int (*quiescent)(void *ctx);
	void (*release)(void *ctx);
};

int pstv_ohci_read_stage(struct pstv_ohci_gate_result *result,
			 const struct pstv_ohci_gate_ops *ops, void *ctx);
int pstv_ohci_reset_stage(struct pstv_ohci_gate_result *result,
			  const struct pstv_ohci_gate_ops *ops, void *ctx);
int pstv_ohci_frame_stage(struct pstv_ohci_gate_result *result,
			  const struct pstv_ohci_gate_ops *ops, void *ctx);
int pstv_ohci_stop_stage(struct pstv_ohci_gate_result *result,
			 const struct pstv_ohci_gate_ops *ops, void *ctx);

/* stage: 0 = read, 1 = reset and stop, 2 = frame and stop. */
int pstv_ohci_transaction(struct pstv_ohci_gate_result *result,
			  const struct pstv_ohci_gate_ops *ops, void *ctx,
			  unsigned int stage);

#endif
