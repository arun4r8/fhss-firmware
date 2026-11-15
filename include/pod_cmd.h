/*
 * pod_cmd.h - Ground -> pod command: the pod state the ground wants.
 *
 * Packet (radio payload, ground -> pod, 4 bytes, fits the ground's 9-byte
 * slot at 125 kHz):
 *   byte 0        POD_CMD_MAGIC
 *   byte 1        [commanded state:4][spare:4]  (embedded's 4-bit state)
 *   bytes 2-3     command counter, big-endian (so the pod can count misses)
 *
 * The ground sends its current target state every hop. The pod reports the
 * state it is actually in, in every telemetry frame (telem_frame.h byte 1),
 * so the ground sees when the two match; a lost command is simply repeated
 * on the next hop.
 */
#ifndef POD_CMD_H
#define POD_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define POD_CMD_MAGIC 0xC5u
#define POD_CMD_LEN   4u

/* Returns POD_CMD_LEN, or 0 if max is too small. */
size_t pod_cmd_build(uint8_t state, uint16_t seq, uint8_t *out, size_t max);

/* false if it isn't a command (wrong length or magic). */
bool pod_cmd_parse(const uint8_t *in, size_t len, uint8_t *state, uint16_t *seq);

#endif /* POD_CMD_H */
