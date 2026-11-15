/*
 * telem_frame.h - Fixed-layout telemetry frame: every sensor's latest
 * reading in one radio payload, in an order both ends agree on (the schema),
 * so no per-sensor tags go over the air.
 *
 * Frame (radio payload, pod -> ground):
 *   byte 0         [layout version:4][sequence:4]
 *   byte 1         [pod state:4][spare:4]   state the pod is actually in
 *                  (embedded's 4-bit state; spare bits sent as 0)
 *   then per schema slot, in schema order:
 *     2 bytes      16-bit reading, big-endian (same as the hub messages)
 * Length = 2 + 2 * sensors: 78 bytes for 38 sensors (84 on air with the
 * link header, still 149 ms at SF7 / 125 kHz: at SF7 LoRa codes the payload
 * in 28-bit blocks, and 83 and 84 bytes fill the same number).
 *
 * TELEM_FAULT (0xFFFF) in a slot means "no valid reading": the hub flagged
 * the sensor's error bit, or no message with that sensor has arrived yet.
 * A real reading of 0xFFFF is sent as 0xFFFE.
 *
 * The version lets the ground refuse a frame laid out with a different
 * schema instead of showing readings under the wrong sensor names. The
 * sequence counts frames (mod 16) so the ground can see dropped ones.
 *
 * The pod fills the snapshot from hub messages as they arrive on CAN
 * (can_msg.h): each record (hub, type, sensor id) goes to its schema slot.
 */
#ifndef TELEM_FRAME_H
#define TELEM_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_msg.h"

#define TELEM_MAX_SENSORS 64
#define TELEM_FAULT 0xFFFFu

typedef struct {
    uint8_t hub;  /* 0..15 */
    uint8_t type; /* 0..3 */
    uint8_t id;   /* 0..15 */
} telem_slot_t;

typedef struct {
    uint8_t version; /* 1..15; change whenever the slot list changes */
    uint8_t count;   /* sensors in the frame */
    const telem_slot_t *slot;
} telem_schema_t;

typedef struct {
    uint16_t reading[TELEM_MAX_SENSORS];
    uint8_t state;     /* pod state, 0..15 (telem_snapshot_set_state) */
    bool updated;      /* a reading or the state changed since the last frame */
    uint8_t seq;       /* next frame's sequence number */
    uint32_t unmapped; /* hub records with no slot in the schema */
} telem_snapshot_t;

/* Placeholder layout until the embedded team sends the real sensor order:
 * hubs 1-3 with 10 sensors each (type 1, ids 0-9), hub 4 with 8 (ids 0-7). */
extern const telem_schema_t telem_schema_placeholder;

/* false if the schema can't be framed (bad version, too many sensors,
 * duplicate or out-of-range slots). */
bool telem_schema_valid(const telem_schema_t *s);

size_t telem_frame_length(const telem_schema_t *s);

/* Every slot starts at TELEM_FAULT (no data yet). */
void telem_snapshot_init(telem_snapshot_t *snap);

/* Copy one hub message's readings into their slots. Returns how many
 * records found a slot. */
uint8_t telem_snapshot_update(telem_snapshot_t *snap, const telem_schema_t *s,
                              const can_hub_msg_t *m);

/* Set the pod state carried in every frame (0..15); a change marks the
 * snapshot updated so it goes out in the next frame. */
void telem_snapshot_set_state(telem_snapshot_t *snap, uint8_t state);

/* Build a frame from the snapshot (advances the sequence, clears updated).
 * Returns its length, 0 if out is too small. */
size_t telem_frame_build(telem_snapshot_t *snap, const telem_schema_t *s, uint8_t *out,
                         size_t max);

/* Check a received frame against the schema and unpack it. readings needs
 * s->count entries; state may be NULL. */
bool telem_frame_decode(const telem_schema_t *s, const uint8_t *in, size_t len,
                        uint16_t *readings, uint8_t *seq, uint8_t *state);

/* Rebuild one hub's message from decoded readings (schema order, fault
 * readings sent with the error bit set). Returns the sensor count, 0 if the
 * schema has no slot for that hub. */
uint8_t telem_frame_hub_msg(const telem_schema_t *s, const uint16_t *readings, uint8_t hub,
                            can_hub_msg_t *m);

#endif /* TELEM_FRAME_H */
