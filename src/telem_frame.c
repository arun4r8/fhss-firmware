#include "telem_frame.h"

#include <string.h>

#define S(h, i) {.hub = (h), .type = 1, .id = (i)}
static const telem_slot_t placeholder_slots[] = {
    S(1, 0), S(1, 1), S(1, 2), S(1, 3), S(1, 4), S(1, 5), S(1, 6), S(1, 7), S(1, 8), S(1, 9),
    S(2, 0), S(2, 1), S(2, 2), S(2, 3), S(2, 4), S(2, 5), S(2, 6), S(2, 7), S(2, 8), S(2, 9),
    S(3, 0), S(3, 1), S(3, 2), S(3, 3), S(3, 4), S(3, 5), S(3, 6), S(3, 7), S(3, 8), S(3, 9),
    S(4, 0), S(4, 1), S(4, 2), S(4, 3), S(4, 4), S(4, 5), S(4, 6), S(4, 7),
};
#undef S

const telem_schema_t telem_schema_placeholder = {
    .version = 1,
    .count = (uint8_t)(sizeof(placeholder_slots) / sizeof(placeholder_slots[0])),
    .slot = placeholder_slots,
};

static bool same_slot(const telem_slot_t *a, const telem_slot_t *b)
{
    return a->hub == b->hub && a->type == b->type && a->id == b->id;
}

bool telem_schema_valid(const telem_schema_t *s)
{
    if (!s || !s->slot || s->version == 0 || s->version > 15 || s->count == 0 ||
        s->count > TELEM_MAX_SENSORS) {
        return false;
    }
    for (uint8_t i = 0; i < s->count; i++) {
        const telem_slot_t *a = &s->slot[i];
        if (a->hub > 15 || a->type > 3 || a->id > 15) {
            return false;
        }
        for (uint8_t j = 0; j < i; j++) {
            if (same_slot(a, &s->slot[j])) {
                return false;
            }
        }
    }
    return true;
}

size_t telem_frame_length(const telem_schema_t *s)
{
    return 1u + 2u * s->count;
}

void telem_snapshot_init(telem_snapshot_t *snap)
{
    memset(snap, 0, sizeof(*snap));
    for (int i = 0; i < TELEM_MAX_SENSORS; i++) {
        snap->reading[i] = TELEM_FAULT;
    }
}

static int find_slot(const telem_schema_t *s, uint8_t hub, const can_sensor_t *r)
{
    for (uint8_t i = 0; i < s->count; i++) {
        const telem_slot_t *t = &s->slot[i];
        if (t->hub == hub && t->type == r->type && t->id == r->id) {
            return i;
        }
    }
    return -1;
}

uint8_t telem_snapshot_update(telem_snapshot_t *snap, const telem_schema_t *s,
                              const can_hub_msg_t *m)
{
    uint8_t mapped = 0;
    for (uint8_t i = 0; i < m->count; i++) {
        const can_sensor_t *r = &m->sensor[i];
        int k = find_slot(s, m->hub, r);
        if (k < 0) {
            snap->unmapped++;
            continue;
        }
        uint16_t v = r->error ? TELEM_FAULT : (r->reading == TELEM_FAULT ? TELEM_FAULT - 1 : r->reading);
        snap->reading[k] = v;
        mapped++;
    }
    if (mapped) {
        snap->updated = true;
    }
    return mapped;
}

size_t telem_frame_build(telem_snapshot_t *snap, const telem_schema_t *s, uint8_t *out,
                         size_t max)
{
    size_t len = telem_frame_length(s);
    if (len > max) {
        return 0;
    }
    out[0] = (uint8_t)((s->version << 4) | (snap->seq & 0x0F));
    for (uint8_t i = 0; i < s->count; i++) {
        out[1 + 2 * i] = (uint8_t)(snap->reading[i] >> 8);
        out[2 + 2 * i] = (uint8_t)snap->reading[i];
    }
    snap->seq = (uint8_t)((snap->seq + 1) & 0x0F);
    snap->updated = false;
    return len;
}

bool telem_frame_decode(const telem_schema_t *s, const uint8_t *in, size_t len,
                        uint16_t *readings, uint8_t *seq)
{
    if (len != telem_frame_length(s) || (in[0] >> 4) != s->version) {
        return false;
    }
    for (uint8_t i = 0; i < s->count; i++) {
        readings[i] = (uint16_t)((in[1 + 2 * i] << 8) | in[2 + 2 * i]);
    }
    *seq = in[0] & 0x0F;
    return true;
}

uint8_t telem_frame_hub_msg(const telem_schema_t *s, const uint16_t *readings, uint8_t hub,
                            can_hub_msg_t *m)
{
    memset(m, 0, sizeof(*m));
    m->hub = hub;
    for (uint8_t i = 0; i < s->count && m->count < CAN_MSG_MAX_SENSORS; i++) {
        const telem_slot_t *t = &s->slot[i];
        if (t->hub != hub) {
            continue;
        }
        bool fault = readings[i] == TELEM_FAULT;
        m->sensor[m->count++] = (can_sensor_t){
            .error = fault, .type = t->type, .id = t->id, .reading = fault ? 0 : readings[i],
        };
    }
    return m->count;
}
