// SPDX-License-Identifier: GPL-2.0
#include <kunit/test.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/spi/spi.h>

#include <linux/mfd/vita-syscon.h>

#include "vita-syscon-internal.h"

static void syscon_henkaku_version_frame_test(struct kunit *test)
{
	const u8 rx[] = { 0x04, 0x00, 0x06, 0x00, 0x0d, 0x06, 0x00, 0x01,
			0xe1 };
	size_t payload_len = SIZE_MAX;

	KUNIT_EXPECT_EQ(test,
			syscon_validate_rx_frame(rx, sizeof(rx), &payload_len), 0);
	KUNIT_EXPECT_EQ(test, payload_len, (size_t)4);
}

static void syscon_busy_frame_test(struct kunit *test)
{
	const u8 rx[] = { 0x04, 0x00, 0x02, 0x80, 0x79 };
	size_t payload_len = SIZE_MAX;

	KUNIT_ASSERT_EQ(test,
			syscon_validate_rx_frame(rx, sizeof(rx), &payload_len), 0);
	KUNIT_EXPECT_EQ(test, payload_len, (size_t)0);
	KUNIT_EXPECT_EQ(test, syscon_result_policy(rx[SYSCON_RX_RESULT], 1),
			SYSCON_RESULT_RETRY);
}

static void syscon_no_payload_success_frame_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x00, 0xd9 };
	size_t payload_len = SIZE_MAX;

	KUNIT_EXPECT_EQ(test,
			syscon_validate_rx_frame(rx, sizeof(rx), &payload_len), 0);
	KUNIT_EXPECT_EQ(test, payload_len, (size_t)0);
}

static void syscon_short_rx_capacity_test(struct kunit *test)
{
	const u8 rx[] = { 0x04, 0x00, 0x06, 0x00, 0x0d, 0x06, 0x00, 0x01,
			0xe1 };
	size_t payload_len;
	size_t capacity;

	for (capacity = 0; capacity < SYSCON_RX_HEADER_SIZE; capacity++)
		KUNIT_EXPECT_EQ(test,
				syscon_validate_rx_frame(rx, capacity,
							 &payload_len),
				-EMSGSIZE);
}

static void syscon_short_declared_length_test(struct kunit *test)
{
	u8 rx[] = { 0x24, 0x00, 0x00, 0x00 };
	size_t payload_len;
	u8 declared_len;

	for (declared_len = 0; declared_len < 2; declared_len++) {
		rx[SYSCON_RX_LENGTH] = declared_len;
		KUNIT_EXPECT_EQ(test,
				syscon_validate_rx_frame(rx, sizeof(rx),
							 &payload_len),
				-EPROTO);
	}
}

static void syscon_declared_frame_beyond_capacity_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x00, 0xd9 };
	size_t payload_len;

	KUNIT_EXPECT_EQ(test,
			syscon_validate_rx_frame(rx, sizeof(rx) - 1, &payload_len),
			-EMSGSIZE);
}

static void syscon_bad_checksum_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x00, 0xd8 };
	size_t payload_len;

	KUNIT_EXPECT_EQ(test,
			syscon_validate_rx_frame(rx, sizeof(rx), &payload_len),
			-EBADMSG);
}

static void syscon_trailing_bytes_ignored_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x00, 0xd9, 0xaa, 0x55 };
	size_t payload_len = SIZE_MAX;

	KUNIT_EXPECT_EQ(test,
			syscon_validate_rx_frame(rx, sizeof(rx), &payload_len), 0);
	KUNIT_EXPECT_EQ(test, payload_len, (size_t)0);
}

static void syscon_success_result_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, syscon_result_policy(0x00, 1), 0);
}

static void syscon_busy_result_retries_below_max_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, syscon_result_policy(0x80, 1),
			SYSCON_RESULT_RETRY);
	KUNIT_EXPECT_EQ(test,
			syscon_result_policy(0x81, SYSCON_MAX_ATTEMPTS - 1),
			SYSCON_RESULT_RETRY);
	KUNIT_EXPECT_EQ(test,
			syscon_result_policy(0x82, SYSCON_MAX_ATTEMPTS - 1),
			SYSCON_RESULT_RETRY);
}

static void syscon_busy_result_stops_at_max_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			syscon_result_policy(0x80, SYSCON_MAX_ATTEMPTS), -EBUSY);
	KUNIT_EXPECT_EQ(test,
			syscon_result_policy(0x81, SYSCON_MAX_ATTEMPTS), -EBUSY);
	KUNIT_EXPECT_EQ(test,
			syscon_result_policy(0x82, SYSCON_MAX_ATTEMPTS), -EBUSY);
}

static void syscon_status_result_done_test(struct kunit *test)
{
	/* Non-zero status with the busy flag clear is "done", not an
	 * error.  cmd 6 (hw-flags) returns 0x3f on the Vita 1000.
	 */
	KUNIT_EXPECT_EQ(test, syscon_result_policy(0x01, 1), 0);
	KUNIT_EXPECT_EQ(test, syscon_result_policy(0x3f, 1), 0);
	KUNIT_EXPECT_EQ(test, syscon_result_policy(0x07, 1), 0);
}

static void syscon_payload_roundtrip_test(struct kunit *test)
{
	const u8 rx[] = { 0x04, 0x00, 0x06, 0x00, 0x0d, 0x06, 0x00, 0x01,
			  0xe1 };
	u8 dest[4] = { 0xa5, 0xa5, 0xa5, 0xa5 };

	KUNIT_EXPECT_EQ(test,
			syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
			4);
	KUNIT_EXPECT_EQ(test, dest[0], 0x0d);
	KUNIT_EXPECT_EQ(test, dest[1], 0x06);
	KUNIT_EXPECT_EQ(test, dest[2], 0x00);
	KUNIT_EXPECT_EQ(test, dest[3], 0x01);
}

static void syscon_payload_larger_than_dest_test(struct kunit *test)
{
	const u8 rx[] = { 0x04, 0x00, 0x06, 0x00, 0x0d, 0x06, 0x00, 0x01,
			  0xe1 };
	u8 dest[2] = { 0xa5, 0xa5 };

	KUNIT_EXPECT_EQ(test,
			syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
			-EMSGSIZE);
	KUNIT_EXPECT_EQ(test, dest[0], 0xa5);
	KUNIT_EXPECT_EQ(test, dest[1], 0xa5);
}

static void syscon_payload_smaller_than_dest_test(struct kunit *test)
{
	const u8 rx[] = { 0x04, 0x00, 0x06, 0x00, 0x0d, 0x06, 0x00, 0x01,
			  0xe1 };
	u8 dest[8];
	int i;

	for (i = 0; i < 8; i++)
		dest[i] = 0xa5;

	KUNIT_EXPECT_EQ(test,
			syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
			4);
	for (i = 4; i < 8; i++)
		KUNIT_EXPECT_EQ(test, dest[i], 0xa5);
}

static void syscon_payload_zero_length_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x00, 0xd9 };
	u8 dest[4] = { 0xa5, 0xa5, 0xa5, 0xa5 };

	KUNIT_EXPECT_EQ(test,
			syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
			0);
	KUNIT_EXPECT_EQ(test, dest[0], 0xa5);
	KUNIT_EXPECT_EQ(test, dest[1], 0xa5);
}

static void syscon_payload_trailing_bytes_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x00, 0xd9, 0xaa, 0x55 };
	u8 dest[4] = { 0xa5, 0xa5, 0xa5, 0xa5 };

	KUNIT_EXPECT_EQ(test,
			syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
			0);
	KUNIT_EXPECT_EQ(test, dest[0], 0xa5);
}

static void syscon_payload_propagates_validation_error_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x00, 0xd8 }; /* bad checksum */
	u8 dest[4] = { 0xa5, 0xa5, 0xa5, 0xa5 };

	KUNIT_EXPECT_EQ(test,
			syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
			-EBADMSG);
	KUNIT_EXPECT_EQ(test, dest[0], 0xa5);
}

static void syscon_payload_busy_result_test(struct kunit *test)
{
	const u8 rx[] = { 0x04, 0x00, 0x02, 0x80, 0x79 };
	u8 dest[4] = { 0xa5, 0xa5, 0xa5, 0xa5 };

	KUNIT_EXPECT_EQ(test,
			syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
			-EBUSY);
	KUNIT_EXPECT_EQ(test, dest[0], 0xa5);
}

static void syscon_payload_status_result_test(struct kunit *test)
{
	const u8 rx[] = { 0x24, 0x00, 0x02, 0x01, 0xd8 };
	u8 dest[4] = { 0xa5, 0xa5, 0xa5, 0xa5 };

	/* result 0x01 (busy flag clear) is a status, not an error; the
	 * zero-length payload yields 0 and leaves dest untouched.
	 */
	KUNIT_EXPECT_EQ(test,
		syscon_rx_payload(rx, sizeof(rx), dest, sizeof(dest)),
		0);
	KUNIT_EXPECT_EQ(test, dest[0], 0xa5);
}

/*
 * PSTV (Dolce) external USB 5 V rail helper.
 *
 * The wire format is not negotiable: VitaOS's ksceSysconCtrlDolceUsbPower()
 * is short command 0x8c5 with a one-byte boolean payload and a total request
 * length of 2.  These tests pin the command, the payload, the length, and the
 * idempotency the sequencer relies on, using a stub transport in place of the
 * SPI link to Ernie.
 */
struct syscon_rail_stub {
	unsigned int calls;
	u16 last_cmd;
	u32 last_data;
	int last_cmd_len;
	int result;
};

static struct syscon_rail_stub rail_stub;

static int syscon_rail_stub_write(struct vita_syscon *syscon, u16 cmd, u32 data,
				  int cmd_len)
{
	rail_stub.calls++;
	rail_stub.last_cmd = cmd;
	rail_stub.last_data = data;
	rail_stub.last_cmd_len = cmd_len;

	return rail_stub.result;
}

static void syscon_rail_init(struct vita_syscon *syscon)
{
	memset(&rail_stub, 0, sizeof(rail_stub));
	memset(syscon, 0, sizeof(*syscon));
	mutex_init(&syscon->dolce_usb_mutex);
	syscon->short_command_write = syscon_rail_stub_write;
}

static void syscon_dolce_rail_on_uses_exact_wire_format_test(struct kunit *test)
{
	struct vita_syscon syscon;

	syscon_rail_init(&syscon);

	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true), 0);
	KUNIT_EXPECT_EQ(test, rail_stub.calls, 1u);
	KUNIT_EXPECT_EQ(test, rail_stub.last_cmd, (u16)0x8c5);
	KUNIT_EXPECT_EQ(test, rail_stub.last_data, 1u);
	KUNIT_EXPECT_EQ(test, rail_stub.last_cmd_len, 2);
	KUNIT_EXPECT_EQ(test, syscon.dolce_usb_power, 1);
	mutex_destroy(&syscon.dolce_usb_mutex);
}

static void syscon_dolce_rail_off_sends_zero_payload_test(struct kunit *test)
{
	struct vita_syscon syscon;

	syscon_rail_init(&syscon);
	KUNIT_ASSERT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true), 0);

	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, false), 0);
	KUNIT_EXPECT_EQ(test, rail_stub.calls, 2u);
	KUNIT_EXPECT_EQ(test, rail_stub.last_cmd, (u16)0x8c5);
	KUNIT_EXPECT_EQ(test, rail_stub.last_data, 0u);
	KUNIT_EXPECT_EQ(test, rail_stub.last_cmd_len, 2);
	KUNIT_EXPECT_EQ(test, syscon.dolce_usb_power, 0);
	mutex_destroy(&syscon.dolce_usb_mutex);
}

static void syscon_dolce_rail_repeat_is_idempotent_test(struct kunit *test)
{
	struct vita_syscon syscon;

	syscon_rail_init(&syscon);
	KUNIT_ASSERT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true), 0);

	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true), 0);
	KUNIT_EXPECT_EQ(test, rail_stub.calls, 1u);
	mutex_destroy(&syscon.dolce_usb_mutex);
}

static void syscon_dolce_rail_off_when_already_off_is_idempotent_test(struct kunit *test)
{
	struct vita_syscon syscon;

	syscon_rail_init(&syscon);

	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, false), 0);
	KUNIT_EXPECT_EQ(test, rail_stub.calls, 0u);
	mutex_destroy(&syscon.dolce_usb_mutex);
}

static void syscon_dolce_rail_transport_error_keeps_state_test(struct kunit *test)
{
	struct vita_syscon syscon;

	syscon_rail_init(&syscon);
	rail_stub.result = -EBUSY;

	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true),
			-EBUSY);
	KUNIT_EXPECT_EQ(test, syscon.dolce_usb_power, 0);

	/*
	 * A failed rail-on must not latch the cached state: the next attempt
	 * has to reach the hardware again, not be swallowed as a no-op.
	 */
	rail_stub.result = 0;
	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true), 0);
	KUNIT_EXPECT_EQ(test, rail_stub.calls, 2u);
	KUNIT_EXPECT_EQ(test, syscon.dolce_usb_power, 1);
	mutex_destroy(&syscon.dolce_usb_mutex);
}

static void syscon_dolce_rail_off_failure_keeps_state_on_test(struct kunit *test)
{
	struct vita_syscon syscon;

	syscon_rail_init(&syscon);
	KUNIT_ASSERT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true), 0);

	/*
	 * Rail-off is the shutdown path.  If a failure there cleared the
	 * cached state, the retry would be swallowed as a no-op and the rail
	 * would be left on.
	 */
	rail_stub.result = -EBUSY;
	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, false),
			-EBUSY);
	KUNIT_EXPECT_EQ(test, syscon.dolce_usb_power, 1);

	rail_stub.result = 0;
	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, false), 0);
	KUNIT_EXPECT_EQ(test, rail_stub.calls, 3u);
	KUNIT_EXPECT_EQ(test, syscon.dolce_usb_power, 0);
	mutex_destroy(&syscon.dolce_usb_mutex);
}

static void syscon_dolce_rail_missing_transport_test(struct kunit *test)
{
	struct vita_syscon syscon;

	syscon_rail_init(&syscon);
	syscon.short_command_write = NULL;

	KUNIT_EXPECT_EQ(test, vita_syscon_dolce_usb_power_set(&syscon, true),
			-ENODEV);
	KUNIT_EXPECT_EQ(test, syscon.dolce_usb_power, 0);
	mutex_destroy(&syscon.dolce_usb_mutex);
}

static struct kunit_case syscon_policy_test_cases[] = {
	KUNIT_CASE(syscon_henkaku_version_frame_test),
	KUNIT_CASE(syscon_busy_frame_test),
	KUNIT_CASE(syscon_no_payload_success_frame_test),
	KUNIT_CASE(syscon_short_rx_capacity_test),
	KUNIT_CASE(syscon_short_declared_length_test),
	KUNIT_CASE(syscon_declared_frame_beyond_capacity_test),
	KUNIT_CASE(syscon_bad_checksum_test),
	KUNIT_CASE(syscon_trailing_bytes_ignored_test),
	KUNIT_CASE(syscon_success_result_test),
	KUNIT_CASE(syscon_busy_result_retries_below_max_test),
	KUNIT_CASE(syscon_busy_result_stops_at_max_test),
	KUNIT_CASE(syscon_status_result_done_test),
	KUNIT_CASE(syscon_payload_roundtrip_test),
	KUNIT_CASE(syscon_payload_larger_than_dest_test),
	KUNIT_CASE(syscon_payload_smaller_than_dest_test),
	KUNIT_CASE(syscon_payload_zero_length_test),
	KUNIT_CASE(syscon_payload_trailing_bytes_test),
	KUNIT_CASE(syscon_payload_propagates_validation_error_test),
	KUNIT_CASE(syscon_payload_busy_result_test),
	KUNIT_CASE(syscon_payload_status_result_test),
	KUNIT_CASE(syscon_dolce_rail_on_uses_exact_wire_format_test),
	KUNIT_CASE(syscon_dolce_rail_off_sends_zero_payload_test),
	KUNIT_CASE(syscon_dolce_rail_repeat_is_idempotent_test),
	KUNIT_CASE(syscon_dolce_rail_off_when_already_off_is_idempotent_test),
	KUNIT_CASE(syscon_dolce_rail_transport_error_keeps_state_test),
	KUNIT_CASE(syscon_dolce_rail_off_failure_keeps_state_on_test),
	KUNIT_CASE(syscon_dolce_rail_missing_transport_test),
	{}
};

static struct kunit_suite syscon_policy_test_suite = {
	.name = "vita-syscon-policy",
	.test_cases = syscon_policy_test_cases,
};

kunit_test_suite(syscon_policy_test_suite);

MODULE_DESCRIPTION("KUnit tests for PlayStation Vita Syscon policy");
MODULE_LICENSE("GPL");
