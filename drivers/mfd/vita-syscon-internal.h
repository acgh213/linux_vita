/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __VITA_SYSCON_INTERNAL_H
#define __VITA_SYSCON_INTERNAL_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>

#define SYSCON_RX_HEADER_SIZE	4
#define SYSCON_RX_LENGTH	2
#define SYSCON_RX_RESULT	3

#define SYSCON_RESULT_SUCCESS	0x00
#define SYSCON_RESULT_BUSY_FLAG	0x80	/* bit 7 set => command still busy */

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
	/*
	 * The result byte is a status flag: bit 7 is the busy flag.  When
	 * set (0x80..0xFF) the command is still processing and must be
	 * retried.  When clear (0x00..0x7F) the command is complete; the low
	 * 7 bits are a command-specific status.  0x00 is the common "success",
	 * but other values are legitimate (e.g. the hardware-flags read,
	 * cmd 6, returns 0x3f on the Vita 1000; the touchpanel info read
	 * returns 0x07) and MUST NOT be treated as errors.  A previous policy
	 * returned -EREMOTEIO for any unmapped byte, which aborted the syscon
	 * probe — and with it the whole MFD cell tree, including the WiFi
	 * power sequencer — on the Vita 1000 (2026-08-17).
	 */
	if (result & SYSCON_RESULT_BUSY_FLAG)
		return attempt < SYSCON_MAX_ATTEMPTS ? SYSCON_RESULT_RETRY :
			-EBUSY;

	return 0;
}

static inline int syscon_rx_payload(const u8 *rx, size_t rx_capacity,
				    void *dest, size_t dest_capacity)
{
	size_t payload_len;
	int ret;

	ret = syscon_validate_rx_frame(rx, rx_capacity, &payload_len);
	if (ret)
		return ret;

	ret = syscon_result_policy(rx[SYSCON_RX_RESULT], SYSCON_MAX_ATTEMPTS);
	if (ret)
		return ret;

	if (payload_len > dest_capacity)
		return -EMSGSIZE;

	memcpy(dest, &rx[SYSCON_RX_HEADER_SIZE], payload_len);

	return payload_len;
}

#endif /* __VITA_SYSCON_INTERNAL_H */
