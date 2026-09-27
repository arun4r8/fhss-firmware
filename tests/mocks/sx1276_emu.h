/* Emulated SX1276 (LoRa mode) on a shared virtual "air". Host tests only. */
#ifndef SX1276_EMU_H
#define SX1276_EMU_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define EMU_MAX_LOG 8192

typedef struct {
    uint32_t t_ms;
    uint32_t frf;
} emu_frf_event_t;

typedef struct {
    int radio_id;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t frf;
    uint8_t len;
} emu_tx_event_t;

typedef struct sx1276_emu {
    int id;
    bool present; /* false: chip missing, reads return 0 */
    uint8_t regs[128];
    uint8_t fifo[256];

    bool cs_low;
    bool first_byte;
    bool writing;
    uint8_t addr;

    bool tx_active;
    uint32_t tx_start_ms;
    uint32_t tx_end_ms;
    uint32_t rx_since_ms; /* listening on current FRF since */

    emu_frf_event_t frf_log[EMU_MAX_LOG];
    int frf_log_n;

    SPI_HandleTypeDef spi;
    GPIO_TypeDef port;
} sx1276_emu_t;

extern uint32_t emu_now_ms;
extern bool emu_air_enabled;
extern emu_tx_event_t emu_tx_log[EMU_MAX_LOG];
extern int emu_tx_log_n;

void emu_reset_world(void);
void emu_add_radio(sx1276_emu_t *e, int id);
uint32_t emu_frf(const sx1276_emu_t *e);
uint8_t emu_mode(const sx1276_emu_t *e);

#endif
