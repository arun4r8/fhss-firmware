/*
 * can_msg.h - Hub-unit telemetry message format from the embedded team's
 * "CAN and Processing Documentation", plus CAN-frame reassembly and
 * splitting into radio payloads.
 *
 * Message (hub unit -> VCU, and forwarded over the radio unchanged):
 *   byte 0         [hub:4][frames:4]     frames = CAN frames in the message
 *   then per sensor, 3 bytes:
 *     byte A       [err:1][type:2][unused:1][sensor id:4]
 *     bytes B, C   16-bit reading (fixed point, scaling defined by sensor)
 * Length = 1 + 3 * sensors, sent as ceil(length / 8) CAN frames.
 *
 * Details the document leaves open (confirm with the embedded team):
 *   CAN_MSG_HUB_IN_HIGH_NIBBLE  hub ID is the upper nibble of byte 0
 *   CAN_MSG_READING_BIG_ENDIAN  byte B is the high byte of the reading
 *   the last CAN frame's DLC is trimmed to the real length (no padding)
 */
#ifndef CAN_MSG_H
#define CAN_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef CAN_MSG_HUB_IN_HIGH_NIBBLE
#define CAN_MSG_HUB_IN_HIGH_NIBBLE 1
#endif
#ifndef CAN_MSG_READING_BIG_ENDIAN
#define CAN_MSG_READING_BIG_ENDIAN 1
#endif

#define CAN_MSG_MAX_FRAMES  15                      /* 4-bit frame count */
#define CAN_MSG_MAX_BYTES   (CAN_MSG_MAX_FRAMES * 8) /* 120 */
#define CAN_MSG_MAX_SENSORS ((CAN_MSG_MAX_BYTES - 1) / 3) /* 39 */
#define CAN_MSG_REASM_TIMEOUT_MS 50u

typedef struct {
    bool error;       /* hub detected a sensor fault */
    uint8_t type;     /* 0..3: temperature, hall effect, ... */
    uint8_t id;       /* 0..15 */
    uint16_t reading;
} can_sensor_t;

typedef struct {
    uint8_t hub;      /* 0..15: RHU, CHU, FHU, ... */
    uint8_t count;
    can_sensor_t sensor[CAN_MSG_MAX_SENSORS];
} can_hub_msg_t;

/* Bytes needed for n sensors, and the CAN frames that takes. */
size_t can_msg_length(uint8_t sensors);
uint8_t can_msg_frames(size_t length);

/* Encode into out (fills in the frame count). Returns length, 0 on error. */
size_t can_msg_encode(const can_hub_msg_t *m, uint8_t *out, size_t max);

/* Decode a complete message. Rejects bad lengths and frame counts. */
bool can_msg_decode(const uint8_t *buf, size_t len, can_hub_msg_t *m);

/* Split an encoded message into CAN frame payloads. Returns frame count. */
uint8_t can_msg_to_frames(const uint8_t *msg, size_t len, uint8_t data[][8], uint8_t dlc[]);

/* Reassembles one sender's frames back into a message. */
typedef struct {
    uint8_t buf[CAN_MSG_MAX_BYTES];
    size_t len;
    uint8_t expected;
    uint8_t got;
    uint32_t last_ms;
} can_reasm_t;

typedef enum { CAN_REASM_PENDING, CAN_REASM_DONE, CAN_REASM_ERROR } can_reasm_result_t;

/* Feed one received frame. On DONE, r->buf / r->len hold the message. A gap
 * longer than CAN_MSG_REASM_TIMEOUT_MS starts a new message. */
can_reasm_result_t can_reasm_feed(can_reasm_t *r, const uint8_t *data, uint8_t dlc, uint32_t now_ms);

/*
 * Radio payloads: take whole sensor records from *pos onward, as many as fit
 * in max bytes, behind a copy of byte 0 with its frame count recomputed, so
 * every payload is itself a valid message for can_msg_decode(). Start with
 * *pos = 0; returns the payload length, 0 when the message is used up.
 */
size_t can_msg_rf_chunk(const uint8_t *msg, size_t len, size_t *pos, uint8_t *out, size_t max);

#endif /* CAN_MSG_H */
