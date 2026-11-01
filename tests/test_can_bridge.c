/* Host tests for the CAN <-> radio bridge logic. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_bridge.h"

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

/* fake radio: records payloads, can be made busy or refuse sends */
typedef struct {
    bool busy;
    int refuse_after; /* refuse sends once this many were accepted; -1 never */
    int n;
    uint8_t len[64];
    uint8_t data[64][CAN_MSG_MAX_BYTES];
} radio_t;

static bool radio_idle(void *user)
{
    return !((radio_t *)user)->busy;
}

static bool radio_send(void *user, const uint8_t *data, uint8_t len)
{
    radio_t *r = user;
    if ((r->refuse_after >= 0 && r->n >= r->refuse_after) || r->n == 64) {
        return false;
    }
    r->len[r->n] = len;
    memcpy(r->data[r->n++], data, len);
    return true;
}

static can_bridge_config_t config(radio_t *r)
{
    memset(r, 0, sizeof(*r));
    r->refuse_after = -1;
    return (can_bridge_config_t){
        .rx_id_mask = 0x7F0, .rx_id_match = 0x100, .tx_id_base = 0x200, .max_payload = 31,
        .radio_idle = radio_idle, .radio_send = radio_send, .user = r,
    };
}

static can_hub_msg_t sample(uint8_t hub, uint8_t n, uint16_t base)
{
    can_hub_msg_t m = {.hub = hub, .count = n};
    for (uint8_t i = 0; i < n; i++) {
        m.sensor[i] = (can_sensor_t){
            .error = (i % 7) == 6, .type = (uint8_t)(i % 4), .id = (uint8_t)(i % 16),
            .reading = (uint16_t)(base + 131u * i),
        };
    }
    return m;
}

/* encode m and feed its CAN frames to the bridge as sender `id` */
static void feed(can_bridge_t *b, uint16_t id, const can_hub_msg_t *m, uint32_t t)
{
    uint8_t buf[CAN_MSG_MAX_BYTES], data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    size_t len = can_msg_encode(m, buf, sizeof(buf));
    uint8_t n = can_msg_to_frames(buf, len, data, dlc);
    for (uint8_t f = 0; f < n; f++) {
        can_bridge_on_frame(b, id, data[f], dlc[f], t);
    }
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

static void test_config(void)
{
    printf("config validation\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    CHECK(can_bridge_init(&b, &c), "good config rejected");
    c.max_payload = 3;
    CHECK(!can_bridge_init(&b, &c), "payload too small for one record accepted");
    c = config(&r);
    c.radio_send = NULL;
    CHECK(!can_bridge_init(&b, &c), "missing radio_send accepted");
}

static void test_one_message(void)
{
    printf("10-sensor message: CAN frames in, one radio payload out, CAN frames back\n");
    static can_bridge_t pod, ground;
    radio_t r, unused;
    can_bridge_config_t c = config(&r);
    can_bridge_init(&pod, &c);
    can_bridge_config_t g = config(&unused);
    can_bridge_init(&ground, &g);

    can_hub_msg_t m = sample(3, 10, 1000), back;
    feed(&pod, 0x103, &m, 0);
    CHECK(pod.stats.frames_in == 4 && pod.stats.messages_in == 1, "frames %u messages %u",
          pod.stats.frames_in, pod.stats.messages_in);
    CHECK(r.n == 0, "sent before poll");

    r.busy = true;
    CHECK(!can_bridge_poll(&pod) && r.n == 0, "sent while the radio was busy");
    r.busy = false;
    CHECK(can_bridge_poll(&pod) && r.n == 1, "poll sent %d payloads", r.n);
    CHECK(r.len[0] == 31, "payload %u bytes, want 31", r.len[0]);
    CHECK(!can_bridge_poll(&pod) && r.n == 1, "sent the same message twice");

    uint8_t data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    uint16_t id = 0;
    uint8_t n = can_bridge_from_radio(&ground, r.data[0], r.len[0], &back, data, dlc, &id);
    CHECK(n == 4 && same(&m, &back), "ground decode: %u frames", n);
    CHECK(id == 0x203, "ground resend ID %03X, want 203", id);
    CHECK(dlc[0] == 8 && dlc[3] == 7, "frame lengths %u..%u", dlc[0], dlc[3]);

    /* the rebuilt frames must reassemble into the original bytes */
    can_reasm_t ra = {0};
    can_reasm_result_t res = CAN_REASM_PENDING;
    for (uint8_t f = 0; f < n; f++) {
        res = can_reasm_feed(&ra, data[f], dlc[f], 0);
    }
    CHECK(res == CAN_REASM_DONE && ra.len == 31 && memcmp(ra.buf, r.data[0], 31) == 0,
          "rebuilt frames do not match");
}

static void test_filter_and_bad_payloads(void)
{
    printf("ID filter and invalid radio payloads\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    can_bridge_init(&b, &c);
    can_hub_msg_t m = sample(1, 4, 7);
    feed(&b, 0x0FF, &m, 0);
    feed(&b, 0x110, &m, 0);
    CHECK(b.stats.frames_ignored == 4 && b.stats.messages_in == 0, "foreign IDs not ignored");

    c.rx_id_mask = 0; /* accept everything */
    can_bridge_init(&b, &c);
    feed(&b, 0x7FF, &m, 0);
    CHECK(b.stats.messages_in == 1, "mask 0 did not accept every ID");

    can_hub_msg_t out;
    uint8_t data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    uint16_t id;
    const uint8_t junk[5] = {0x31, 1, 2, 3, 4}; /* length not 1 + 3n */
    const uint8_t wrong_frames[4] = {0x35, 1, 2, 3}; /* says 5 frames, is 1 */
    CHECK(can_bridge_from_radio(&b, junk, 5, &out, data, dlc, &id) == 0, "junk accepted");
    CHECK(can_bridge_from_radio(&b, wrong_frames, 4, &out, data, dlc, &id) == 0, "bad count accepted");
    CHECK(b.stats.radio_bad == 2 && b.stats.radio_in == 0, "bad %u in %u", b.stats.radio_bad,
          b.stats.radio_in);
}

static void test_long_message_chunks(void)
{
    printf("39-sensor message (15 CAN frames) -> 4 radio payloads\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    can_bridge_init(&b, &c);
    can_hub_msg_t m = sample(9, 39, 40000), part;
    feed(&b, 0x109, &m, 0);
    CHECK(b.stats.messages_in == 1, "15-frame message not reassembled");
    can_bridge_poll(&b);
    CHECK(r.n == 4 && b.stats.chunks_sent == 4, "%d payloads", r.n);
    uint8_t seen = 0;
    bool ok = true;
    for (int i = 0; i < r.n; i++) {
        ok &= r.len[i] <= 31 && can_msg_decode(r.data[i], r.len[i], &part) && part.hub == 9;
        for (uint8_t k = 0; ok && k < part.count; k++) {
            ok &= part.sensor[k].reading == m.sensor[seen + k].reading;
        }
        seen = (uint8_t)(seen + part.count);
    }
    CHECK(ok && seen == 39, "payloads do not carry the message in order (saw %u)", seen);

    /* radio refuses partway: stop and count it, then resume from the
     * refused payload once the radio takes it, so nothing is lost */
    can_bridge_init(&b, &c);
    r.n = 0;
    r.refuse_after = 2;
    feed(&b, 0x109, &m, 0);
    CHECK(!can_bridge_poll(&b), "a part-sent message reported as sent");
    CHECK(r.n == 2 && b.stats.radio_refused == 1 && b.stats.chunks_sent == 2,
          "refusal: %d sent, %u refused", r.n, b.stats.radio_refused);
    CHECK(b.stats.messages_sent == 0, "counted as sent before it was");
    r.refuse_after = -1;
    CHECK(can_bridge_poll(&b), "resume did not finish the message");
    CHECK(r.n == 4 && b.stats.chunks_sent == 4 && b.stats.messages_sent == 1,
          "resume: %d payloads, %u sent", r.n, b.stats.messages_sent);
    seen = 0;
    ok = true;
    for (int i = 0; i < r.n; i++) {
        ok &= can_msg_decode(r.data[i], r.len[i], &part);
        for (uint8_t k = 0; ok && k < part.count; k++) {
            ok &= part.sensor[k].reading == m.sensor[seen + k].reading;
        }
        seen = (uint8_t)(seen + part.count);
    }
    CHECK(ok && seen == 39, "resumed payloads skip or repeat sensors (saw %u)", seen);
}

static void test_lossless_queue(void)
{
    printf("default: every message queued and sent in order, overflow counted\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    can_bridge_init(&b, &c);

    /* hub A sends 5 updates and hub B 1 while the radio is busy: all 6 wait */
    r.busy = true;
    for (uint16_t k = 0; k < 5; k++) {
        can_hub_msg_t a = sample(1, 3, (uint16_t)(100 * k));
        feed(&b, 0x101, &a, k * 10u);
    }
    can_hub_msg_t hb = sample(2, 2, 5555);
    feed(&b, 0x102, &hb, 60);
    CHECK(b.stats.messages_in == 6 && b.stats.superseded == 0 && b.stats.queue_peak == 6,
          "in %u superseded %u peak %u", b.stats.messages_in, b.stats.superseded,
          b.stats.queue_peak);
    CHECK(!can_bridge_poll(&b) && r.n == 0, "sent while the radio was busy");

    r.busy = false;
    while (can_bridge_poll(&b)) {
    }
    CHECK(r.n == 6 && b.stats.messages_sent == 6, "%d of 6 sent", r.n);
    bool ok = true;
    can_hub_msg_t got;
    for (uint16_t k = 0; k < 5; k++) {
        can_hub_msg_t a = sample(1, 3, (uint16_t)(100 * k));
        ok &= can_msg_decode(r.data[k], r.len[k], &got) && same(&got, &a);
    }
    ok &= can_msg_decode(r.data[5], r.len[5], &got) && same(&got, &hb);
    CHECK(ok, "messages out of arrival order, or altered");

    /* fill the queue past capacity: the extra is dropped and counted */
    can_bridge_init(&b, &c);
    r.n = 0;
    r.busy = true;
    can_hub_msg_t m = sample(3, 2, 7);
    for (int k = 0; k < CAN_BRIDGE_QUEUE_LEN + 3; k++) {
        feed(&b, 0x103, &m, (uint32_t)k * 10u);
    }
    CHECK(b.stats.messages_in == CAN_BRIDGE_QUEUE_LEN + 3 && b.stats.queue_full == 3,
          "in %u queue_full %u", b.stats.messages_in, b.stats.queue_full);
    r.busy = false;
    while (can_bridge_poll(&b)) {
        if (r.n == 64) {
            r.n = 0;
        }
    }
    CHECK(b.stats.messages_sent + b.stats.queue_full == b.stats.messages_in,
          "books do not balance: sent %u + dropped %u != in %u", b.stats.messages_sent,
          b.stats.queue_full, b.stats.messages_in);
}

static void test_latest_only_keeps_partial_head(void)
{
    printf("latest_only never rewrites a message that is partly on air\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    c.latest_only = true;
    can_bridge_init(&b, &c);
    can_hub_msg_t old = sample(9, 39, 1000), fresh = sample(9, 39, 2000), part;
    feed(&b, 0x109, &old, 0);
    r.refuse_after = 2;
    can_bridge_poll(&b); /* 2 of 4 payloads of `old` go out */
    feed(&b, 0x109, &fresh, 100);
    CHECK(b.stats.superseded == 0, "replaced a message already partly sent");
    r.refuse_after = -1;
    while (can_bridge_poll(&b)) {
    }
    /* old's 4 payloads, then fresh's 4 */
    CHECK(r.n == 8 && b.stats.messages_sent == 2, "%d payloads, %u sent", r.n,
          b.stats.messages_sent);
    bool ok = can_msg_decode(r.data[2], r.len[2], &part) &&
              part.sensor[0].reading == old.sensor[20].reading; /* 10 sensors per payload */
    ok &= can_msg_decode(r.data[4], r.len[4], &part) && part.sensor[0].reading == fresh.sensor[0].reading;
    CHECK(ok, "payloads from the two messages were mixed");
}

static void test_freshest_wins_round_robin(void)
{
    printf("latest_only: newest message per sender, senders take turns\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    c.latest_only = true;
    can_bridge_init(&b, &c);

    /* hub A sends 5 updates while the radio is busy, hub B sends 1 */
    for (uint16_t k = 0; k < 5; k++) {
        can_hub_msg_t a = sample(1, 3, (uint16_t)(100 * k));
        feed(&b, 0x101, &a, k * 10u);
    }
    can_hub_msg_t hb = sample(2, 2, 5555);
    feed(&b, 0x102, &hb, 60);
    CHECK(b.stats.messages_in == 6 && b.stats.superseded == 4, "in %u superseded %u",
          b.stats.messages_in, b.stats.superseded);

    can_hub_msg_t got;
    can_bridge_poll(&b);
    can_bridge_poll(&b);
    CHECK(r.n == 2, "%d payloads", r.n);
    can_msg_decode(r.data[0], r.len[0], &got);
    can_hub_msg_t newest = sample(1, 3, 400);
    CHECK(same(&got, &newest), "hub A: not the newest reading");
    can_msg_decode(r.data[1], r.len[1], &got);
    CHECK(same(&got, &hb), "hub B not sent second");

    /* round-robin: A keeps updating, B must still get turns */
    int a_sent = 0, b_sent = 0;
    for (uint32_t t = 100; t < 2000; t += 10) {
        can_hub_msg_t a = sample(1, 3, (uint16_t)t);
        feed(&b, 0x101, &a, t);
        if (t % 50 == 0) {
            feed(&b, 0x102, &hb, t);
        }
        if (t % 40 == 0) {
            int before = r.n;
            can_bridge_poll(&b);
            if (r.n > before) {
                can_msg_decode(r.data[r.n - 1], r.len[r.n - 1], &got);
                a_sent += got.hub == 1;
                b_sent += got.hub == 2;
            }
            if (r.n > 50) {
                r.n = 2;
            }
        }
    }
    CHECK(b_sent > 10 && a_sent > b_sent, "turns: A %d, B %d", a_sent, b_sent);
}

static void test_interleaved_senders(void)
{
    printf("two hubs interleaving frames on the bus\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    can_bridge_init(&b, &c);
    can_hub_msg_t ma = sample(4, 10, 11), mb = sample(5, 10, 22);
    uint8_t ba[CAN_MSG_MAX_BYTES], bb[CAN_MSG_MAX_BYTES];
    uint8_t da[CAN_MSG_MAX_FRAMES][8], db[CAN_MSG_MAX_FRAMES][8], la[CAN_MSG_MAX_FRAMES],
        lb[CAN_MSG_MAX_FRAMES];
    uint8_t na = can_msg_to_frames(ba, can_msg_encode(&ma, ba, sizeof(ba)), da, la);
    uint8_t nb = can_msg_to_frames(bb, can_msg_encode(&mb, bb, sizeof(bb)), db, lb);
    for (uint8_t f = 0; f < 4; f++) {
        if (f < na) {
            can_bridge_on_frame(&b, 0x104, da[f], la[f], f);
        }
        if (f < nb) {
            can_bridge_on_frame(&b, 0x105, db[f], lb[f], f);
        }
    }
    CHECK(b.stats.messages_in == 2 && b.stats.reasm_errors == 0, "in %u errors %u",
          b.stats.messages_in, b.stats.reasm_errors);
    can_bridge_poll(&b);
    can_bridge_poll(&b);
    can_hub_msg_t x, y;
    CHECK(r.n == 2 && can_msg_decode(r.data[0], r.len[0], &x) && can_msg_decode(r.data[1], r.len[1], &y),
          "payloads");
    CHECK(same(&x, &ma) && same(&y, &mb), "interleaved messages mixed up");
}

static void test_slots(void)
{
    printf("more senders than slots; idle slots are reused\n");
    static can_bridge_t b;
    radio_t r;
    can_bridge_config_t c = config(&r);
    c.rx_id_mask = 0;
    can_bridge_init(&b, &c);
    can_hub_msg_t m = sample(1, 2, 0);
    for (uint16_t id = 0; id < CAN_BRIDGE_MAX_SENDERS + 2; id++) {
        feed(&b, id, &m, 0);
    }
    /* a finished message moves to the queue and frees its slot, so more
     * senders than slots all get through, as long as they are not all
     * mid-message at the same moment */
    CHECK(b.stats.messages_in == CAN_BRIDGE_MAX_SENDERS + 2 && b.stats.no_slot == 0,
          "in %u no_slot %u", b.stats.messages_in, b.stats.no_slot);
    while (can_bridge_poll(&b)) {
    }
    CHECK(r.n == CAN_BRIDGE_MAX_SENDERS + 2, "sent %d", r.n);
    feed(&b, 0x300, &m, 100);
    feed(&b, 0x301, &m, 100);
    CHECK(b.stats.messages_in == CAN_BRIDGE_MAX_SENDERS + 4, "slots not reused");

    /* a sender stuck mid-message holds its slot only until the timeout */
    can_bridge_init(&b, &c);
    can_hub_msg_t big = sample(1, 10, 0);
    uint8_t buf[CAN_MSG_MAX_BYTES], data[CAN_MSG_MAX_FRAMES][8], dlc[CAN_MSG_MAX_FRAMES];
    can_msg_to_frames(buf, can_msg_encode(&big, buf, sizeof(buf)), data, dlc);
    for (uint16_t id = 0; id < CAN_BRIDGE_MAX_SENDERS; id++) {
        can_bridge_on_frame(&b, id, data[0], 8, 0); /* first frame only */
    }
    feed(&b, 0x400, &m, 10);
    CHECK(b.stats.no_slot == 1, "evicted a sender mid-message");
    feed(&b, 0x400, &m, 10 + CAN_MSG_REASM_TIMEOUT_MS + 1);
    CHECK(b.stats.messages_in == 1, "stale partial message kept its slot");
}

int main(void)
{
    test_config();
    test_one_message();
    test_filter_and_bad_payloads();
    test_long_message_chunks();
    test_lossless_queue();
    test_latest_only_keeps_partial_head();
    test_freshest_wins_round_robin();
    test_interleaved_senders();
    test_slots();
    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
