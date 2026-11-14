/* Host tests for the range-test payload. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "range_test.h"

static int failures;
static int checks;

#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        checks++;                                                          \
        if (!(cond)) {                                                     \
            failures++;                                                    \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                  \
            printf(__VA_ARGS__);                                           \
            printf("\n");                                                  \
        }                                                                  \
    } while (0)

static void test_pattern(void)
{
    printf("PRBS-9 pattern: period 511 bits, balanced, different per hop\n");
    uint8_t a[128], b[128];
    range_pattern(0, a, sizeof(a));
    range_pattern(1, b, sizeof(b));
    CHECK(memcmp(a, b, sizeof(a)) != 0, "hops 0 and 1 give the same pattern");
    range_pattern(0, b, sizeof(b));
    CHECK(memcmp(a, b, sizeof(a)) == 0, "pattern not repeatable");

    /* period: bit i equals bit i + 511 */
    int mismatch = 0, ones = 0;
    for (int i = 0; i < 511; i++) {
        int x = (a[i / 8] >> (7 - i % 8)) & 1;
        int j = i + 511;
        int y = (a[j / 8] >> (7 - j % 8)) & 1;
        mismatch += x != y;
        ones += x;
    }
    CHECK(mismatch == 0, "period is not 511 bits (%d mismatches)", mismatch);
    CHECK(ones == 256, "%d ones in one period, want 256", ones);

    /* every hop index 0..49 has a non-zero seed (no all-zero pattern) */
    for (uint16_t h = 0; h < 50; h++) {
        range_pattern(h, a, 8);
        CHECK(a[0] | a[1] | a[2] | a[3], "hop %u: all-zero pattern", h);
    }
}

static void test_round_trip_and_errors(void)
{
    printf("payload: counters round trip, bit errors counted only where known\n");
    range_info_t in = {.seq = 0xBEEF, .up_ok = 321, .up_rssi_dbm = -97, .up_snr_q4 = -30}, out;
    uint8_t p[RANGE_PAYLOAD_LEN + 4];
    CHECK(range_build(&in, 17, p, RANGE_PAYLOAD_LEN - 1) == 0, "built into a short buffer");
    CHECK(range_build(&in, 17, p, sizeof(p)) == RANGE_PAYLOAD_LEN, "length");
    CHECK(range_parse(p, RANGE_PAYLOAD_LEN, &out) && out.seq == 0xBEEF && out.up_ok == 321 &&
              out.up_rssi_dbm == -97 && out.up_snr_q4 == -30, "counters changed");
    CHECK(!range_parse(p, RANGE_PAYLOAD_LEN - 1, &out), "short payload parsed");

    uint32_t bits = 0;
    CHECK(range_bit_errors(p, RANGE_PAYLOAD_LEN, 17, &bits) == 0 && bits == 568,
          "clean payload: %u bits", bits);
    CHECK(range_bit_errors(p, RANGE_PAYLOAD_LEN, 18, &bits) > 200,
          "wrong hop should look like ~50%% errors");

    p[3] ^= 0xFF; /* counters aren't checked */
    CHECK(range_bit_errors(p, RANGE_PAYLOAD_LEN, 17, &bits) == 0, "counter bytes counted");
    p[0] ^= 0x01;  /* magic */
    p[10] ^= 0x81; /* pattern: 2 bits */
    p[76] ^= 0x10; /* last byte */
    CHECK(range_bit_errors(p, RANGE_PAYLOAD_LEN, 17, &bits) == 4, "want 4 bit errors");
    CHECK(range_bit_errors(p, 70, 17, &bits) == 4 + 7 * 8 - 1, "truncated payload: missing bytes "
          "count as errors (bit flipped in byte 76 is among them)");
}

int main(void)
{
    test_pattern();
    test_round_trip_and_errors();
    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
