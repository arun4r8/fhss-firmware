/*
 * can_ring.h - Receive buffer between the CAN interrupt and the main loop.
 *
 * The MCP2515 holds only 2 received frames. With the hubs sending about 160
 * frames/s, any main-loop stall longer than ~12 ms (a radio transaction, a
 * burst of console output) used to risk losing frames. In the pod build (default; CAN_IRQ=0 polls) the
 * MCP2515's INT pin fires an interrupt, the handler moves each frame into
 * this ring right away, and the main loop takes them out when it can.
 *
 * Single producer (the interrupt) and single consumer (the main loop), so
 * no locking: the producer only writes head, the consumer only writes tail,
 * and each publishes its index after the data it guards. Correct on a
 * single-core Cortex-M; the host tests use it the same way.
 */
#ifndef CAN_RING_H
#define CAN_RING_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_RING_SIZE 64u /* power of 2; 0.4 s of hub traffic */

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t ms; /* when the interrupt took it off the chip */
} can_ring_frame_t;

typedef struct {
    can_ring_frame_t slot[CAN_RING_SIZE];
    volatile uint32_t head;      /* next slot to write (producer) */
    volatile uint32_t tail;      /* next slot to read (consumer) */
    volatile uint32_t dropped;   /* frames lost because the ring was full */
    volatile uint32_t max_depth; /* most frames ever waiting at once */
} can_ring_t;

#define CAN_RING_BARRIER() __asm__ volatile("" ::: "memory")

static inline void can_ring_init(can_ring_t *r)
{
    r->head = r->tail = r->dropped = r->max_depth = 0;
}

static inline uint32_t can_ring_count(const can_ring_t *r)
{
    return r->head - r->tail;
}

/* Producer side (interrupt). false if full: the frame is dropped and counted. */
static inline bool can_ring_push(can_ring_t *r, const can_ring_frame_t *f)
{
    uint32_t h = r->head;
    uint32_t used = h - r->tail;
    if (used >= CAN_RING_SIZE) {
        r->dropped++;
        return false;
    }
    r->slot[h % CAN_RING_SIZE] = *f;
    CAN_RING_BARRIER(); /* data before the index that publishes it */
    r->head = h + 1u;
    if (used + 1u > r->max_depth) {
        r->max_depth = used + 1u;
    }
    return true;
}

/* Consumer side (main loop). false if empty. */
static inline bool can_ring_pop(can_ring_t *r, can_ring_frame_t *f)
{
    uint32_t t = r->tail;
    if (r->head == t) {
        return false;
    }
    CAN_RING_BARRIER(); /* read head before the data it publishes */
    *f = r->slot[t % CAN_RING_SIZE];
    CAN_RING_BARRIER(); /* finish copying before freeing the slot */
    r->tail = t + 1u;
    return true;
}

#endif /* CAN_RING_H */
