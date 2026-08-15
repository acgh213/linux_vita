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
}

static void syscon_busy_result_stops_at_max_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			syscon_result_policy(0x80, SYSCON_MAX_ATTEMPTS), -EBUSY);
	KUNIT_EXPECT_EQ(test,
			syscon_result_policy(0x81, SYSCON_MAX_ATTEMPTS), -EBUSY);
}

static void syscon_unknown_result_fails_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, syscon_result_policy(0x01, 1), -EREMOTEIO);
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
	KUNIT_CASE(syscon_unknown_result_fails_test),
	{}
};

static struct kunit_suite syscon_policy_test_suite = {
	.name = "vita-syscon-policy",
	.test_cases = syscon_policy_test_cases,
};

kunit_test_suite(syscon_policy_test_suite);

MODULE_DESCRIPTION("KUnit tests for PlayStation Vita Syscon policy");
MODULE_LICENSE("GPL");
