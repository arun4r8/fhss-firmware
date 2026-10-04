#include "can_msg.h"

#include <string.h>

static uint8_t header(uint8_t hub, uint8_t frames)
{
#if CAN_MSG_HUB_IN_HIGH_NIBBLE
    return (uint8_t)(((hub & 0x0F) << 4) | (frames & 0x0F));
#else
    return (uint8_t)(((frames & 0x0F) << 4) | (hub & 0x0F));
#endif
}

static uint8_t header_hub(uint8_t b)
{
#if CAN_MSG_HUB_IN_HIGH_NIBBLE
    return (uint8_t)(b >> 4);
#else
    return (uint8_t)(b & 0x0F);
#endif
}

static uint8_t header_frames(uint8_t b)
{
#if CAN_MSG_HUB_IN_HIGH_NIBBLE
    return (uint8_t)(b & 0x0F);
#else
    return (uint8_t)(b >> 4);
#endif
}

size_t can_msg_length(uint8_t sensors)
{
    return 1u + 3u * sensors;
}

uint8_t can_msg_frames(size_t length)
{
    return (uint8_t)((length + 7u) / 8u);
}

size_t can_msg_encode(const can_hub_msg_t *m, uint8_t *out, size_t max)
{
    size_t len = can_msg_length(m->count);
    if (m->hub > 15 || m->count > CAN_MSG_MAX_SENSORS || len > max) {
        return 0;
    }
    out[0] = header(m->hub, can_msg_frames(len));
    for (uint8_t i = 0; i < m->count; i++) {
        const can_sensor_t *s = &m->sensor[i];
        if (s->type > 3 || s->id > 15) {
            return 0;
        }
        uint8_t *p = &out[1 + 3 * i];
        p[0] = (uint8_t)((s->error ? 0x80 : 0) | (s->type << 5) | s->id);
#if CAN_MSG_READING_BIG_ENDIAN
        p[1] = (uint8_t)(s->reading >> 8);
        p[2] = (uint8_t)s->reading;
#else
        p[1] = (uint8_t)s->reading;
        p[2] = (uint8_t)(s->reading >> 8);
#endif
    }
    return len;
}

bool can_msg_decode(const uint8_t *buf, size_t len, can_hub_msg_t *m)
{
    if (len < 1 || len > CAN_MSG_MAX_BYTES || (len - 1) % 3 != 0 ||
        header_frames(buf[0]) != can_msg_frames(len)) {
        return false;
    }
    m->hub = header_hub(buf[0]);
    m->count = (uint8_t)((len - 1) / 3);
    for (uint8_t i = 0; i < m->count; i++) {
        const uint8_t *p = &buf[1 + 3 * i];
        can_sensor_t *s = &m->sensor[i];
        s->error = (p[0] & 0x80) != 0;
        s->type = (uint8_t)((p[0] >> 5) & 0x03);
        s->id = (uint8_t)(p[0] & 0x0F);
#if CAN_MSG_READING_BIG_ENDIAN
        s->reading = (uint16_t)((p[1] << 8) | p[2]);
#else
        s->reading = (uint16_t)((p[2] << 8) | p[1]);
#endif
    }
    return true;
}

uint8_t can_msg_to_frames(const uint8_t *msg, size_t len, uint8_t data[][8], uint8_t dlc[])
{
    uint8_t n = can_msg_frames(len);
    for (uint8_t f = 0; f < n; f++) {
        size_t off = (size_t)f * 8u;
        dlc[f] = (uint8_t)(len - off < 8 ? len - off : 8);
        memcpy(data[f], &msg[off], dlc[f]);
    }
    return n;
}

can_reasm_result_t can_reasm_feed(can_reasm_t *r, const uint8_t *data, uint8_t dlc, uint32_t now_ms)
{
    if (r->expected && (uint32_t)(now_ms - r->last_ms) > CAN_MSG_REASM_TIMEOUT_MS) {
        r->expected = 0; /* stale partial message: start over */
    }
    r->last_ms = now_ms;

    if (dlc == 0 || dlc > 8) {
        r->expected = 0;
        return CAN_REASM_ERROR;
    }
    if (r->expected == 0) {
        uint8_t frames = header_frames(data[0]);
        if (frames == 0) {
            return CAN_REASM_ERROR;
        }
        r->expected = frames;
        r->got = 0;
        r->len = 0;
    }
    /* every frame but the last must be full */
    if ((r->got + 1u < r->expected && dlc != 8) || r->len + dlc > sizeof(r->buf)) {
        r->expected = 0;
        return CAN_REASM_ERROR;
    }
    memcpy(&r->buf[r->len], data, dlc);
    r->len += dlc;
    if (++r->got < r->expected) {
        return CAN_REASM_PENDING;
    }
    r->expected = 0;
    return (r->len - 1) % 3 == 0 ? CAN_REASM_DONE : CAN_REASM_ERROR;
}

size_t can_msg_rf_chunk(const uint8_t *msg, size_t len, size_t *pos, uint8_t *out, size_t max)
{
    if (*pos == 0) {
        *pos = 1; /* skip the header; each chunk gets its own */
    }
    if (*pos >= len || max < 4) {
        return 0;
    }
    size_t records = (len - *pos) / 3;
    size_t fit = (max - 1) / 3;
    if (records > fit) {
        records = fit;
    }
    if (records == 0) {
        return 0;
    }
    size_t n = 1 + records * 3;
    memcpy(&out[1], &msg[*pos], records * 3);
    out[0] = header(header_hub(msg[0]), can_msg_frames(n));
    *pos += records * 3;
    return n;
}
