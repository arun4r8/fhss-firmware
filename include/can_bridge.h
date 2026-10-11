/*
 * can_bridge.h - Forwards hub-unit messages (can_msg.h) between the pod's
 * CAN bus and the frequency-hopping radio link. No hardware dependencies:
 * the board code feeds in received CAN frames and supplies two radio
 * callbacks, so the same logic runs on the board and in the host tests.
 *
 * Pod side (CAN -> radio):
 *   1. can_bridge_on_frame() reassembles each sender's frames (keyed by
 *      CAN ID, so several hubs can interleave on the bus).
 *   2. A complete message replaces any older unsent one from the same
 *      sender: telemetry is only useful fresh, and the link carries about
 *      five 31-byte payloads a second each way, far slower than a CAN bus.
 *   3. can_bridge_poll() hands the next waiting message to the radio,
 *      round-robin across senders, only once the radio's queue is empty,
 *      split by can_msg_rf_chunk() into payloads that are each a valid
 *      message on their own.
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

#define CAN_BRIDGE_MAX_SENDERS 8 /* distinct CAN IDs tracked at once */

typedef struct {
    /* Accept frames whose ID satisfies (id & rx_id_mask) == rx_id_match.
     * Mask 0 accepts every ID. */
    uint16_t rx_id_mask;
    uint16_t rx_id_match;
    uint16_t tx_id_base; /* ground side: rebuilt frames go out on tx_id_base + hub */
    uint8_t max_payload; /* radio payload limit, e.g. FHSS_LINK_MAX_PAYLOAD */

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
    uint32_t superseded;     /* unsent messages replaced by a newer one */
    uint32_t no_slot;        /* frames dropped: more senders than slots */
    uint32_t messages_sent;  /* messages handed to the radio */
    uint32_t chunks_sent;    /* radio payloads queued */
    uint32_t radio_refused;  /* radio_send returned false */
    uint32_t radio_in;       /* valid payloads received from the radio */
    uint32_t radio_bad;      /* received payloads that were not valid messages */
} can_bridge_stats_t;

typedef struct {
    bool used;
    uint16_t id;
    can_reasm_t reasm;
    bool pending;                 /* complete message waiting for the radio */
    uint8_t msg[CAN_MSG_MAX_BYTES];
    size_t msg_len;
} can_bridge_sender_t;

typedef struct {
    can_bridge_config_t cfg;
    can_bridge_sender_t sender[CAN_BRIDGE_MAX_SENDERS];
    uint8_t next_sender; /* round-robin position */
    can_bridge_stats_t stats;
} can_bridge_t;

/* false if the config is unusable (missing callbacks, payload too small). */
bool can_bridge_init(can_bridge_t *b, const can_bridge_config_t *cfg);

/* Pod side: one received CAN frame. */
void can_bridge_on_frame(can_bridge_t *b, uint16_t id, const uint8_t *data, uint8_t dlc,
                         uint32_t now_ms);

/* Pod side: call from the main loop. Returns true if a message was sent. */
bool can_bridge_poll(can_bridge_t *b);

/* Ground side: check a radio payload and rebuild its CAN frames. On success
 * fills *msg (decoded), data/dlc (CAN_MSG_MAX_FRAMES entries) and *id, and
 * returns the frame count; returns 0 for an invalid payload. */
uint8_t can_bridge_from_radio(can_bridge_t *b, const uint8_t *payload, uint8_t len,
                              can_hub_msg_t *msg, uint8_t data[][8], uint8_t dlc[],
                              uint16_t *id);

#endif /* CAN_BRIDGE_H */
