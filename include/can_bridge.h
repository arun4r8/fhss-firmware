/*
 * can_bridge.h - Forwards hub-unit messages (can_msg.h) between the pod's
 * CAN bus and the frequency-hopping radio link. No hardware dependencies:
 * the board code feeds in received CAN frames and supplies two radio
 * callbacks, so the same logic runs on the board and in the host tests.
 *
 * Pod side (CAN -> radio):
 *   1. can_bridge_on_frame() reassembles each sender's frames (keyed by
 *      CAN ID, so several hubs can interleave on the bus).
 *   2. A complete message joins a FIFO queue of CAN_BRIDGE_QUEUE_LEN
 *      messages. The link carries about five 31-byte payloads a second each
 *      way, far slower than a CAN bus, so the queue absorbs bursts.
 *        - default: every message is sent, in arrival order. Nothing is
 *          dropped unless the queue is full, and that is counted
 *          (queue_full). This is "buffer without losing CAN data".
 *        - latest_only: a newer message from the same sender replaces its
 *          queued, unsent one (counted as superseded). For telemetry
 *          arriving faster than the link can carry, where only the
 *          freshest reading matters. No buffer can deliver everything if
 *          the average rate exceeds the link; this keeps latency bounded.
 *   3. can_bridge_poll() hands the head message to the radio once the
 *      radio's queue is empty, split by can_msg_rf_chunk() into payloads
 *      that are each a valid message on their own. If the radio refuses a
 *      payload partway through, the next poll resumes from that payload
 *      rather than dropping the rest of the message.
 *
 * Ground side (radio -> CAN):
 *   can_bridge_from_radio() checks a received payload and rebuilds the CAN
 *   frames for it, sent on ID (tx_id_base + hub).
 *
 * Placeholders until the embedded team confirms them: which CAN IDs carry
 * hub messages (rx_id_mask / rx_id_match) and the IDs to re-send them on
 * (tx_id_base).
 */
#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_msg.h"

#define CAN_BRIDGE_MAX_SENDERS 8 /* CAN IDs reassembling a message at once */

#ifndef CAN_BRIDGE_QUEUE_LEN
#define CAN_BRIDGE_QUEUE_LEN 32 /* complete messages waiting for the radio */
#endif

typedef struct {
    /* Accept frames whose ID satisfies (id & rx_id_mask) == rx_id_match.
     * Mask 0 accepts every ID. */
    uint16_t rx_id_mask;
    uint16_t rx_id_match;
    uint16_t tx_id_base; /* ground side: rebuilt frames go out on tx_id_base + hub */
    uint8_t max_payload; /* radio payload limit, e.g. FHSS_LINK_MAX_PAYLOAD */

    /* false (default): every message is queued and sent in order.
     * true: a newer message from a sender replaces its queued unsent one. */
    bool latest_only;

    /* true when the radio has nothing queued (the next payload goes out in
     * this node's next slot instead of waiting behind stale data) */
    bool (*radio_idle)(void *user);
    /* queue one payload; false if it was refused */
    bool (*radio_send)(void *user, const uint8_t *data, uint8_t len);
    void *user;
} can_bridge_config_t;

typedef struct {
    uint32_t frames_in;      /* CAN frames accepted by the ID filter */
    uint32_t frames_ignored; /* CAN frames from other IDs */
    uint32_t messages_in;    /* complete hub messages reassembled */
    uint32_t reasm_errors;   /* malformed frame sequences dropped */
    uint32_t superseded;     /* latest_only: unsent messages replaced by a newer one */
    uint32_t queue_full;     /* messages dropped: queue full (never silent) */
    uint32_t queue_peak;     /* most messages ever waiting at once */
    uint32_t no_slot;        /* frames dropped: more senders mid-message than slots */
    uint32_t messages_sent;  /* messages fully handed to the radio */
    uint32_t chunks_sent;    /* radio payloads queued */
    uint32_t radio_refused;  /* radio_send returned false; retried next poll */
    uint32_t radio_in;       /* valid payloads received from the radio */
    uint32_t radio_bad;      /* received payloads that were not valid messages */
} can_bridge_stats_t;

/* A sender slot only holds a message while it is being reassembled; once
 * complete it moves to the queue, freeing the slot for another sender. */
typedef struct {
    bool used;
    uint16_t id;
    can_reasm_t reasm;
} can_bridge_sender_t;

typedef struct {
    uint16_t id;
    uint8_t msg[CAN_MSG_MAX_BYTES];
    size_t len;
} can_bridge_msg_t;

typedef struct {
    can_bridge_config_t cfg;
    can_bridge_sender_t sender[CAN_BRIDGE_MAX_SENDERS];
    can_bridge_msg_t queue[CAN_BRIDGE_QUEUE_LEN];
    uint8_t q_head;    /* oldest waiting message */
    uint8_t q_count;   /* messages waiting */
    size_t q_pos;      /* resume point inside the head message, 0 = not started */
    can_bridge_stats_t stats;
} can_bridge_t;

/* false if the config is unusable (missing callbacks, payload too small). */
bool can_bridge_init(can_bridge_t *b, const can_bridge_config_t *cfg);

/* Pod side: one received CAN frame. */
void can_bridge_on_frame(can_bridge_t *b, uint16_t id, const uint8_t *data, uint8_t dlc,
                         uint32_t now_ms);

/* Pod side: call from the main loop. Returns true when a whole message has
 * been handed to the radio; false if there was nothing to send, the radio
 * was busy, or it refused a payload (resumed on the next call). */
bool can_bridge_poll(can_bridge_t *b);

/* Ground side: check a radio payload and rebuild its CAN frames. On success
 * fills *msg (decoded), data/dlc (CAN_MSG_MAX_FRAMES entries) and *id, and
 * returns the frame count; returns 0 for an invalid payload. */
uint8_t can_bridge_from_radio(can_bridge_t *b, const uint8_t *payload, uint8_t len,
                              can_hub_msg_t *msg, uint8_t data[][8], uint8_t dlc[],
                              uint16_t *id);

#endif /* CAN_BRIDGE_H */
