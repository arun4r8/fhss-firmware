/*
 * mcp2515.h - Minimal MCP2515 CAN controller driver for STM32 HAL.
 *
 * Standard 11-bit IDs only (the embedded team's CAN plan). Polled: the INT
 * pin does not need wiring. On the team's bench setup the MCP2515 sits on a
 * HW-184 board with its TJA1050 removed; an Adafruit TJA1051T/3 breakout is
 * the transceiver (3.3 V logic, makes its own 5 V for the bus).
 */
#ifndef MCP2515_H
#define MCP2515_H

#include <stdbool.h>
#include <stdint.h>

#ifdef FHSS_HAL_HEADER
#include FHSS_HAL_HEADER
#else
#include "main.h"
#endif

typedef enum {
    MCP2515_MODE_NORMAL,
    MCP2515_MODE_LOOPBACK, /* frames loop back internally; bus not touched */
    MCP2515_MODE_LISTEN    /* receive only, never ACKs or transmits */
} mcp2515_mode_t;

typedef enum {
    MCP2515_OK = 0,
    MCP2515_ERR_NOT_FOUND = -1,   /* SPI readback failed: wiring / CS / power */
    MCP2515_ERR_BAD_BITRATE = -2, /* not reachable from this crystal */
    MCP2515_ERR_MODE = -3,        /* chip refused the mode change */
    MCP2515_ERR_TX_BUSY = -4,
    MCP2515_ERR_TX_FAILED = -5,   /* no ACK / bus error: is another node on the bus? */
    MCP2515_ERR_BAD_FRAME = -6
} mcp2515_err_t;

typedef struct {
    uint16_t id;     /* 11-bit standard ID */
    uint8_t dlc;     /* 0..8 */
    uint8_t data[8];
} can_frame_t;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t osc_hz;       /* crystal on the module: usually 8 MHz on HW-184 */
    uint32_t bitrate;      /* bus bit rate, e.g. 500000 */
    mcp2515_mode_t mode;
    uint16_t filter_mask;  /* 0 = accept every ID */
    uint16_t filter_id;    /* accept IDs where (id & mask) == (filter_id & mask) */
} mcp2515_t;

mcp2515_err_t mcp2515_init(mcp2515_t *c);

/* Transmit one frame and wait for it to finish (a few hundred us at 500k). */
mcp2515_err_t mcp2515_send(mcp2515_t *c, const can_frame_t *f);

/* true if a frame was waiting; fills *f. */
bool mcp2515_receive(mcp2515_t *c, can_frame_t *f);

/* EFLG register plus transmit/receive error counters. */
uint8_t mcp2515_error_flags(mcp2515_t *c, uint8_t *tec, uint8_t *rec);

/* CNF1..CNF3 for a crystal and bit rate; false if not reachable. Pure. */
bool mcp2515_bit_timing(uint32_t osc_hz, uint32_t bitrate, uint8_t cnf[3]);

#endif /* MCP2515_H */
