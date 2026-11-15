/* Host tests for the fixed-layout telemetry frame. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pod_cmd.h"
#include "telem_frame.h"

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

static const telem_schema_t *S = &telem_schema_placeholder;

/* hub message with the placeholder's sensors for that hub, readings base + id */
static can_hub_msg_t hub_msg(uint8_t hub, uint8_t n, uint16_t base)
{
    can_hub_msg_t m = {.hub = hub, .count = n};
    for (uint8_t i = 0; i < n; i++) {
        m.sensor[i] = (can_sensor_t){.type = 1, .id = i, .reading = (uint16_t)(base + i)};
    }
    return m;
}

static void test_layout(void)
{
    printf("placeholder layout: 38 sensors -> 78-byte frame\n");
    CHECK(telem_schema_valid(S), "placeholder schema invalid");
    CHECK(S->count == 38, "%u sensors", S->count);
    CHECK(telem_frame_length(S) == 78, "frame %zu bytes", telem_frame_length(S));

    telem_slot_t dup[] = {{1, 1, 0}, {2, 1, 0}, {1, 1, 0}};
    telem_schema_t bad = {.version = 1, .count = 3, .slot = dup};
    CHECK(!telem_schema_valid(&bad), "duplicate slot accepted");
    bad.count = 2;
    CHECK(telem_schema_valid(&bad), "two distinct slots rejected");
    bad.version = 0;
    CHECK(!telem_schema_valid(&bad), "version 0 accepted");
    bad.version = 16;
    CHECK(!telem_schema_valid(&bad), "version 16 does not fit 4 bits");
    telem_slot_t range[] = {{16, 0, 0}};
    telem_schema_t big = {.version = 1, .count = 1, .slot = range};
    CHECK(!telem_schema_valid(&big), "hub 16 accepted");
}

static void test_round_trip(void)
{
    printf("hub messages -> snapshot -> frame -> readings, byte layout\n");
    telem_snapshot_t snap;
    telem_snapshot_init(&snap);
    CHECK(!snap.updated, "fresh snapshot marked updated");
    for (uint8_t h = 1; h <= 4; h++) {
        can_hub_msg_t m = hub_msg(h, h == 4 ? 8 : 10, (uint16_t)(1000 * h));
        CHECK(telem_snapshot_update(&snap, S, &m) == m.count, "hub %u not fully mapped", h);
    }
    CHECK(snap.updated && snap.unmapped == 0, "updated %d, unmapped %u", snap.updated, snap.unmapped);

    uint8_t f[80];
    CHECK(telem_frame_build(&snap, S, f, 77) == 0, "built into a 77-byte buffer");
    CHECK(telem_frame_build(&snap, S, f, sizeof(f)) == 78, "frame length");
    CHECK(!snap.updated, "updated not cleared by build");
    CHECK(f[0] == 0x20, "byte 0 = 0x%02X, want version 2, seq 0", f[0]);
    CHECK(f[1] == 0x00, "byte 1 = 0x%02X, want state 0", f[1]);
    /* slot 0 = hub 1 sensor 0 = 1000 = 0x03E8, big-endian; slot 37 = hub 4 sensor 7 */
    CHECK(f[2] == 0x03 && f[3] == 0xE8, "first reading bytes %02X %02X", f[2], f[3]);
    CHECK(f[76] == (4007 >> 8) && f[77] == (4007 & 0xFF), "last reading bytes");

    uint16_t r[38];
    uint8_t seq = 99;
    CHECK(telem_frame_decode(S, f, 78, r, &seq, NULL) && seq == 0, "decode failed");
    int wrong = 0;
    for (int i = 0; i < 38; i++) {
        uint16_t want = (uint16_t)(1000 * (1 + i / 10) + i % 10);
        wrong += r[i] != want;
    }
    CHECK(wrong == 0, "%d readings wrong after decode", wrong);
}

static void test_faults_and_unmapped(void)
{
    printf("no data yet and sensor errors -> 0xFFFF; unknown sensors counted\n");
    telem_snapshot_t snap;
    telem_snapshot_init(&snap);
    can_hub_msg_t m = hub_msg(2, 10, 500);
    m.sensor[3].error = true;
    m.sensor[4].reading = 0xFFFF; /* a real reading equal to the fault code */
    telem_snapshot_update(&snap, S, &m);
    can_hub_msg_t odd = hub_msg(9, 2, 0); /* hub not in the layout */
    odd.sensor[1].type = 2;
    CHECK(telem_snapshot_update(&snap, S, &odd) == 0 && snap.unmapped == 2, "unmapped %u",
          snap.unmapped);
    can_hub_msg_t extra = hub_msg(4, 10, 0); /* hub 4 has only ids 0-7 */
    CHECK(telem_snapshot_update(&snap, S, &extra) == 8 && snap.unmapped == 4, "unmapped %u",
          snap.unmapped);

    uint8_t f[80];
    uint16_t r[38];
    uint8_t seq;
    telem_frame_build(&snap, S, f, sizeof(f));
    telem_frame_decode(S, f, 78, r, &seq, NULL);
    CHECK(r[0] == TELEM_FAULT && r[29] == TELEM_FAULT, "hubs 1 and 3 should read no data");
    CHECK(r[13] == TELEM_FAULT, "error bit not sent as fault");
    CHECK(r[14] == 0xFFFE, "real 0xFFFF reading sent as %u", r[14]);
    CHECK(r[10] == 500 && r[19] == 509, "hub 2 readings");

    /* ground rebuilds hub 2's message: error bit back on, tags restored */
    can_hub_msg_t back;
    CHECK(telem_frame_hub_msg(S, r, 2, &back) == 10, "hub 2 rebuilt with %u sensors", back.count);
    CHECK(back.hub == 2 && back.sensor[3].error && !back.sensor[2].error && back.sensor[2].reading == 502 &&
          back.sensor[9].id == 9 && back.sensor[9].type == 1, "rebuilt hub 2 message wrong");
    CHECK(telem_frame_hub_msg(S, r, 4, &back) == 8, "hub 4 rebuilt with %u sensors", back.count);
    CHECK(telem_frame_hub_msg(S, r, 7, &back) == 0, "hub 7 is not in the layout");
    uint8_t buf[CAN_MSG_MAX_BYTES];
    telem_frame_hub_msg(S, r, 2, &back);
    CHECK(can_msg_encode(&back, buf, sizeof(buf)) == 31, "rebuilt message is not 31 bytes");
}

static void test_rejects_and_sequence(void)
{
    printf("wrong length or layout version rejected; sequence wraps at 16\n");
    telem_snapshot_t snap;
    telem_snapshot_init(&snap);
    uint8_t f[80];
    uint16_t r[38];
    uint8_t seq = 0;
    for (int i = 0; i < 20; i++) {
        telem_frame_build(&snap, S, f, sizeof(f));
        CHECK(telem_frame_decode(S, f, 78, r, &seq, NULL) && seq == i % 16, "frame %d seq %u", i, seq);
    }
    CHECK(!telem_frame_decode(S, f, 77, r, &seq, NULL), "short frame accepted");
    CHECK(!telem_frame_decode(S, f, 79, r, &seq, NULL), "long frame accepted");
    f[0] = (uint8_t)(0x10 | (f[0] & 0x0F));
    CHECK(!telem_frame_decode(S, f, 78, r, &seq, NULL), "version 1 frame accepted by a version 2 layout");
}

static void test_state_and_commands(void)
{
    printf("4-bit pod state in the frame; ground -> pod state command\n");
    telem_snapshot_t snap;
    telem_snapshot_init(&snap);
    uint8_t f[80], st = 99, seq;
    uint16_t r[38];
    telem_frame_build(&snap, S, f, sizeof(f));
    CHECK(!snap.updated, "built frame left snapshot updated");
    telem_snapshot_set_state(&snap, 0x1B); /* only the low nibble counts */
    CHECK(snap.updated && snap.state == 0x0B, "state 0x%X, updated %d", snap.state, snap.updated);
    telem_frame_build(&snap, S, f, sizeof(f));
    CHECK(f[1] == 0xB0, "state byte 0x%02X, want 0xB0", f[1]);
    CHECK(telem_frame_decode(S, f, 78, r, &seq, &st) && st == 0x0B, "decoded state %u", st);
    telem_snapshot_set_state(&snap, 0x0B);
    CHECK(!snap.updated, "same state marked the snapshot updated");

    uint8_t c[8];
    uint16_t cseq = 0;
    CHECK(pod_cmd_build(5, 0x1234, c, 3) == 0, "command built into a 3-byte buffer");
    CHECK(pod_cmd_build(0x15, 0x1234, c, sizeof(c)) == POD_CMD_LEN, "command length");
    CHECK(c[0] == POD_CMD_MAGIC && c[1] == 0x50 && c[2] == 0x12 && c[3] == 0x34, "command bytes");
    CHECK(pod_cmd_parse(c, POD_CMD_LEN, &st, &cseq) && st == 5 && cseq == 0x1234, "parse %u %u", st, cseq);
    CHECK(!pod_cmd_parse(c, 3, &st, &cseq), "short command parsed");
    c[0] ^= 1;
    CHECK(!pod_cmd_parse(c, POD_CMD_LEN, &st, &cseq), "wrong magic parsed");
}

int main(void)
{
    test_layout();
    test_round_trip();
    test_faults_and_unmapped();
    test_rejects_and_sequence();
    test_state_and_commands();
    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
