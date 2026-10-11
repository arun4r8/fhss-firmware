#include "can_bridge.h"

#include <string.h>

bool can_bridge_init(can_bridge_t *b, const can_bridge_config_t *cfg)
{
    memset(b, 0, sizeof(*b));
    /* a payload must hold the header and at least one 3-byte sensor record */
    if (!cfg->radio_idle || !cfg->radio_send || cfg->max_payload < 4) {
        return false;
    }
    b->cfg = *cfg;
    return true;
}

static bool reasm_active(const can_bridge_sender_t *s, uint32_t now_ms)
{
    return s->reasm.expected && (uint32_t)(now_ms - s->reasm.last_ms) <= CAN_MSG_REASM_TIMEOUT_MS;
}

static can_bridge_sender_t *find_sender(can_bridge_t *b, uint16_t id, uint32_t now_ms)
{
    can_bridge_sender_t *free_slot = NULL;
    for (int i = 0; i < CAN_BRIDGE_MAX_SENDERS; i++) {
        can_bridge_sender_t *s = &b->sender[i];
        if (s->used && s->id == id) {
            return s;
        }
        /* an unused slot, or one with nothing worth keeping */
        if (!free_slot && (!s->used || (!s->pending && !reasm_active(s, now_ms)))) {
            free_slot = s;
        }
    }
    if (free_slot) {
        memset(free_slot, 0, sizeof(*free_slot));
        free_slot->used = true;
        free_slot->id = id;
    }
    return free_slot;
}

void can_bridge_on_frame(can_bridge_t *b, uint16_t id, const uint8_t *data, uint8_t dlc,
                         uint32_t now_ms)
{
    if ((id & b->cfg.rx_id_mask) != (b->cfg.rx_id_match & b->cfg.rx_id_mask)) {
        b->stats.frames_ignored++;
        return;
    }
    b->stats.frames_in++;
    can_bridge_sender_t *s = find_sender(b, id, now_ms);
    if (!s) {
        b->stats.no_slot++;
        return;
    }
    switch (can_reasm_feed(&s->reasm, data, dlc, now_ms)) {
    case CAN_REASM_PENDING:
        break;
    case CAN_REASM_ERROR:
        b->stats.reasm_errors++;
        break;
    case CAN_REASM_DONE:
        b->stats.messages_in++;
        if (s->pending) {
            b->stats.superseded++;
        }
        memcpy(s->msg, s->reasm.buf, s->reasm.len);
        s->msg_len = s->reasm.len;
        s->pending = true;
        break;
    }
}

bool can_bridge_poll(can_bridge_t *b)
{
    if (!b->cfg.radio_idle(b->cfg.user)) {
        return false;
    }
    for (int n = 0; n < CAN_BRIDGE_MAX_SENDERS; n++) {
        int i = (b->next_sender + n) % CAN_BRIDGE_MAX_SENDERS;
        can_bridge_sender_t *s = &b->sender[i];
        if (!s->pending) {
            continue;
        }
        s->pending = false;
        b->next_sender = (uint8_t)((i + 1) % CAN_BRIDGE_MAX_SENDERS);

        uint8_t chunk[CAN_MSG_MAX_BYTES];
        size_t pos = 0, n_bytes;
        while ((n_bytes = can_msg_rf_chunk(s->msg, s->msg_len, &pos, chunk, b->cfg.max_payload)) > 0) {
            if (!b->cfg.radio_send(b->cfg.user, chunk, (uint8_t)n_bytes)) {
                b->stats.radio_refused++;
                break;
            }
            b->stats.chunks_sent++;
        }
        b->stats.messages_sent++;
        return true;
    }
    return false;
}

uint8_t can_bridge_from_radio(can_bridge_t *b, const uint8_t *payload, uint8_t len,
                              can_hub_msg_t *msg, uint8_t data[][8], uint8_t dlc[],
                              uint16_t *id)
{
    if (!can_msg_decode(payload, len, msg)) {
        b->stats.radio_bad++;
        return 0;
    }
    b->stats.radio_in++;
    *id = (uint16_t)((b->cfg.tx_id_base + msg->hub) & 0x7FF);
    return can_msg_to_frames(payload, len, data, dlc);
}
