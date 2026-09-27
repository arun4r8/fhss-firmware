/*
 * sx1262.h - Minimal SX1262 LoRa driver for STM32 HAL.
 *
 * Written for the Semtech SX1262MB2xAS mbed shield on a Nucleo-64 Arduino
 * header, but works with any SX1262 wired to SPI + NSS/RESET/BUSY.
 * The SX1262 is command based: wait for BUSY low, then opcode + params.
 * IRQ status is polled over SPI, so DIO1 does not need to be wired.
 */
#ifndef SX1262_H
#define SX1262_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct {
    uint8_t spreading_factor; /* 7..12 */
    uint32_t bandwidth_hz;    /* 125000, 250000 or 500000 */
    uint8_t coding_rate;      /* 1..4 meaning 4/5..4/8 */
    uint16_t preamble_len;    /* symbols, typically 8 */
    uint8_t sync_word;        /* 0x12 = private network; avoid 0x34 (LoRaWAN) */
    int8_t tx_power_dbm;      /* -9..22 */
} sx1262_lora_cfg_t;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
    GPIO_TypeDef *busy_port;
    uint16_t busy_pin;
    GPIO_TypeDef *ant_pwr_port; /* shield RF switch supply, NULL if none */
    uint16_t ant_pwr_pin;
    bool use_tcxo;              /* true for TCXO boards (e.g. *DAS shields) */

    sx1262_lora_cfg_t cfg;
    bool rx_active;
    bool busy_timeout;          /* latched if BUSY ever stuck high */
} sx1262_t;

typedef enum {
    SX1262_OK = 0,
    SX1262_ERR_NOT_FOUND = -1, /* SPI readback failed: wiring / power / SPI mode */
    SX1262_ERR_BAD_CONFIG = -2,
    SX1262_ERR_TIMEOUT = -3,
    SX1262_ERR_CRC = -4,
    SX1262_ERR_TOO_LONG = -5,
    SX1262_ERR_OSC = -6,       /* oscillator failed: use_tcxo probably wrong */
    SX1262_ERR_CALIB = -7,
    SX1262_ERR_BUSY = -8       /* BUSY pin never went low */
} sx1262_err_t;

sx1262_err_t sx1262_init(sx1262_t *r, const sx1262_lora_cfg_t *cfg, uint32_t freq_hz);

/* Retune; keeps receiving if the radio was in RX. */
void sx1262_set_frequency(sx1262_t *r, uint32_t freq_hz);

/* Blocking transmit, then back to continuous RX. */
sx1262_err_t sx1262_send(sx1262_t *r, const uint8_t *data, uint8_t len);

void sx1262_start_rx(sx1262_t *r);

/* Returns bytes copied (>0), 0 if nothing received, or a negative error. */
int sx1262_read_packet(sx1262_t *r, uint8_t *buf, uint8_t max, int16_t *rssi_dbm);

/* Time on air per Semtech formula (SF7..12), rounded up to whole ms. */
uint32_t sx1262_airtime_ms(const sx1262_lora_cfg_t *cfg, uint8_t payload_len);

uint32_t sx1262_rf_freq_from_hz(uint32_t freq_hz);

#endif /* SX1262_H */
