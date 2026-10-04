/* Host tests for the embedded team's hub message format. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_msg.h"

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

static can_hub_msg_t sample(uint8_t hub, uint8_t n)
{
    can_hub_msg_t m = {.hub = hub, .count = n};
    for (uint8_t i = 0; i < n; i++) {
        m.sensor[i] = (can_sensor_t){
            .error = (i % 5) == 4, .type = (uint8_t)(i % 4), .id = (uint8_t)(i % 16),
            .reading = (uint16_t)(0x1234 + 257u * i),
        };
    }
    return m;
}

static bool same(const can_hub_msg_t *a, const can_hub_msg_t *b)
{
    if (a->hub != b->hub || a->count != b->count) {
        return false;
    }
    for (uint8_t i = 0; i < a->count; i++) {
        const can_sensor_t *x = &a->sensor[i], *y = &b->sensor[i];
        if (x->error != y->error || x->type != y->type || x->id != y->id || x->reading != y->reading) {
            return false;
        }
    }
    return true;
}

static void test_doc_example(void)
{
    printf("document example: 10 sensors -> 31 bytes -> 4 CAN frames\n");
    can_hub_msg_t m = sample(3, 10), back;
    uint8_t buf[CAN_MSG_MAX_BYTES];
    size_t len = can_msg_encode(&m, buf, sizeof(buf));
    CHECK(len == 31, "length %zu", len);
    CHECK(can_msg_frames(len) == 4, "frames %u", can_msg_frames(len));
    CHECK(buf[0] == 0x34, "header byte %02X (hub 3, 4 frames)", buf[0]);
    /* sensor 4: error set, type 0, id 4, reading 0x1234 + 4*257 = 0x1638 */
    CHECK(buf[1 + 12] == 0x84 && buf[2 + 12] == 0x16 && buf[3 + 12] == 0x38,
          "record layout %02X %02X %02X", buf[13], buf[14], buf[15]);
    CHECK(can_msg_decode(buf, len, &back) && same(&m, &back), "round trip");
}

static void test_bit_layout(void)
{
    printf("record bit layout: err | type:2 | unused | id:4\n");
    can_hub_msg_t m = {.hub = 15, .count = 1};
    m.sensor[0] = (can_sensor_t){.error = false, .type = 3, .id = 9, .reading = 0xBEEF};
    uint8_t b[8];
    CHECK(can_msg_encode(&m, b, sizeof(b)) == 4 && b[0] == 0xF1 && b[1] == 0x69 &&
          b[2] == 0xBE && b[3] == 0xEF, "%02X %02X %02X %02X", b[0], b[1], b[2], b[3]);
}

static void test_rejects(void)
{
    printf("rejects bad messages\n");
    can_hub_msg_t m = sample(1, 2), out;
    uint8_t b[CAN_MSG_MAX_BYTES];
    size_t len = can_msg_encode(&m, b, sizeof(b));
    CHECK(!can_msg_decode(b, len - 1, &out), "truncated record accepted");
    b[0] = (uint8_t)((b[0] & 0xF0) | 3);
    CHECK(!can_msg_decode(b, len, &out), "wrong frame count accepted");
    CHECK(can_msg_encode(&m, b, len - 1) == 0, "encode overflow accepted");
    m.sensor[0].type = 4;
    CHECK(can_msg_encode(&m, b, sizeof(b)) == 0, "3-bit sensor type accepted");
    can_hub_msg_t big = sample(1, CAN_MSG_MAX_SENSORS);
    CHECK(can_msg_encode(&big, b, sizeof(b)) == 118, "39 sensors = 118 bytes, 15 frames");
}

static void test_reassembly(void)
{
    printf("CAN frames -> reassembled message, for 0..39 sensors\n");
    for (uint8_t n = 0; n <= CAN_MSG_MAX_SENSORS; n++) {
        can_hub_msg_t m = sample(7, n), back;
        uint8_t buf[CAN_MSG_MAX_BYTES], frames[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
        size_t len = can_msg_encode(&m, buf, sizeof(buf));
        uint8_t nf = can_msg_to_frames(buf, len, frames, dlc);
        can_reasm_t r = {0};
        can_reasm_result_t res = CAN_REASM_ERROR;
        for (uint8_t f = 0; f < nf; f++) {
            res = can_reasm_feed(&r, frames[f], dlc[f], 1000u + f);
            CHECK(f + 1 == nf ? res == CAN_REASM_DONE : res == CAN_REASM_PENDING,
                  "%u sensors frame %u: result %d", n, f, res);
        }
        CHECK(res == CAN_REASM_DONE && r.len == len && can_msg_decode(r.buf, r.len, &back) &&
              same(&m, &back), "%u sensors: reassembly mismatch", n);
    }
}

static void test_reassembly_recovery(void)
{
    printf("reassembly recovers from a lost frame via timeout\n");
    can_hub_msg_t m = sample(2, 10), back;
    uint8_t buf[CAN_MSG_MAX_BYTES], frames[4][8], dlc[4];
    size_t len = can_msg_encode(&m, buf, sizeof(buf));
    can_msg_to_frames(buf, len, frames, dlc);
    can_reasm_t r = {0};
    can_reasm_feed(&r, frames[0], dlc[0], 0);
    can_reasm_feed(&r, frames[1], dlc[1], 1); /* frames 2 and 3 lost */
    can_reasm_result_t res = CAN_REASM_PENDING;
    for (int f = 0; f < 4; f++) {
        res = can_reasm_feed(&r, frames[f], dlc[f], 200u + (uint32_t)f);
    }
    CHECK(res == CAN_REASM_DONE && can_msg_decode(r.buf, r.len, &back) && same(&m, &back),
          "did not recover after timeout");
}

static void test_rf_chunks(void)
{
    printf("radio payloads: whole records, each chunk decodes on its own\n");
    for (uint8_t n = 1; n <= CAN_MSG_MAX_SENSORS; n++) {
        can_hub_msg_t m = sample(5, n), part;
        uint8_t buf[CAN_MSG_MAX_BYTES], out[31];
        size_t len = can_msg_encode(&m, buf, sizeof(buf)), pos = 0, chunk;
        uint8_t seen = 0, chunks = 0;
        bool ok = true;
        while ((chunk = can_msg_rf_chunk(buf, len, &pos, out, sizeof(out))) > 0) {
            chunks++;
            ok &= chunk <= 31 && can_msg_decode(out, chunk, &part) && part.hub == 5;
            for (uint8_t i = 0; ok && i < part.count; i++) {
                const can_sensor_t *a = &m.sensor[seen + i], *b = &part.sensor[i];
                ok &= a->id == b->id && a->reading == b->reading && a->error == b->error;
            }
            seen = (uint8_t)(seen + part.count);
        }
        CHECK(ok && seen == n, "%u sensors: chunks wrong (saw %u)", n, seen);
        CHECK(chunks == (n + 9) / 10, "%u sensors: %u chunks, want %u", n, chunks, (n + 9) / 10);
    }
}

int main(void)
{
    test_doc_example();
    test_bit_layout();
    test_rejects();
    test_reassembly();
    test_reassembly_recovery();
    test_rf_chunks();
    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
