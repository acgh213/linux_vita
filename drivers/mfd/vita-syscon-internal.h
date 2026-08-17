/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __VITA_SYSCON_INTERNAL_H
#define __VITA_SYSCON_INTERNAL_H

#include <linux/errno.h>
#include <linux/types.h>

#define SYSCON_RX_HEADER_SIZE	4
#define SYSCON_RX_LENGTH	2
#define SYSCON_RX_RESULT	3

#define SYSCON_RESULT_SUCCESS	0x00
#define SYSCON_RESULT_BUSY	0x80
#define SYSCON_RESULT_BUSY_ALT	0x81
#define SYSCON_RESULT_BUSY_ALT2	0x82

#define SYSCON_MAX_ATTEMPTS	16

/* A positive decision tells the transport to retry the command. */
enum syscon_result_decision {
	SYSCON_RESULT_RETRY = 1,
};

static inline int syscon_validate_rx_frame(const u8 *rx, size_t rx_capacity,
					   size_t *payload_len)
{
	size_t frame_size;
	size_t i;
	u8 checksum = 0;
	u8 declared_len;

	if (rx_capacity < SYSCON_RX_HEADER_SIZE)
		return -EMSGSIZE;

	declared_len = rx[SYSCON_RX_LENGTH];
	if (declared_len < 2)
		return -EPROTO;

	frame_size = SYSCON_RX_HEADER_SIZE - 1 + declared_len;
	if (frame_size > rx_capacity)
		return -EMSGSIZE;

	for (i = 0; i < frame_size; i++)
		checksum += rx[i];
	if (checksum != 0xff)
		return -EBADMSG;

	*payload_len = declared_len - 2;

	return 0;
}

static inline int syscon_result_policy(u8 result, unsigned int attempt)
{
	if (result == SYSCON_RESULT_SUCCESS)
		return 0;

	if (result == SYSCON_RESULT_BUSY || result == SYSCON_RESULT_BUSY_ALT ||
	    result == SYSCON_RESULT_BUSY_ALT2)
		return attempt < SYSCON_MAX_ATTEMPTS ? SYSCON_RESULT_RETRY :
			-EBUSY;

	return -EREMOTEIO;
}

#endif /* __VITA_SYSCON_INTERNAL_H */
