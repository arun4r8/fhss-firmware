/*
 * sx1276.h - Minimal SX1276/RFM95W LoRa driver for STM32 HAL.
 *
 * Only what the FHSS link needs: init, retune, blocking send, continuous
 * receive. IRQ flags are polled over SPI, so DIO0 does not need wiring.
 */
#ifndef SX1276_H
#define SX1276_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct {
    uint8_t spreading_factor; /* 7..12 */
    uint32_t bandwidth_hz;    /* 125000, 250000 or 500000 */
    uint8_t coding_rate;      /* 1..4 meaning 4/5..4/8 */
    uint16_t preamble_len;    /* symbols, typically 8 */
    uint8_t sync_word;        /* 0x12 = private network; avoid 0x34 (LoRaWAN) */
    int8_t tx_power_dbm;      /* 2..17 on PA_BOOST (RFM95W) */
} sx1276_lora_cfg_t;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
    sx1276_lora_cfg_t cfg;
    uint8_t rx_active;
} sx1276_t;

typedef enum {
    SX1276_OK = 0,
    SX1276_ERR_NOT_FOUND = -1, /* wrong wiring / no chip: version != 0x12 */
    SX1276_ERR_BAD_CONFIG = -2,
    SX1276_ERR_TIMEOUT = -3,
    SX1276_ERR_CRC = -4,
    SX1276_ERR_TOO_LONG = -5
} sx1276_err_t;

sx1276_err_t sx1276_init(sx1276_t *r, const sx1276_lora_cfg_t *cfg, uint32_t freq_hz);

/* Retune; keeps receiving if the radio was in RX. */
void sx1276_set_frequency(sx1276_t *r, uint32_t freq_hz);

/* Blocking transmit, then back to continuous RX. */
sx1276_err_t sx1276_send(sx1276_t *r, const uint8_t *data, uint8_t len);

void sx1276_start_rx(sx1276_t *r);

/* Returns bytes copied (>0), 0 if nothing received, or a negative error. */
int sx1276_read_packet(sx1276_t *r, uint8_t *buf, uint8_t max, int16_t *rssi_dbm);

/* Time on air per Semtech AN1200.13, rounded up to whole ms. */
uint32_t sx1276_airtime_ms(const sx1276_lora_cfg_t *cfg, uint8_t payload_len);

uint32_t sx1276_frf_from_hz(uint32_t freq_hz);

#endif /* SX1276_H */
