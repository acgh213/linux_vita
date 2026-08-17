// SPDX-License-Identifier: GPL-2.0
#include <kunit/test.h>
#include <linux/errno.h>

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
	{}
};

static struct kunit_suite syscon_policy_test_suite = {
	.name = "vita-syscon-policy",
	.test_cases = syscon_policy_test_cases,
};

kunit_test_suite(syscon_policy_test_suite);

MODULE_DESCRIPTION("KUnit tests for PlayStation Vita Syscon policy");
MODULE_LICENSE("GPL");
