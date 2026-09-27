/*
 * Emulated SX1262 (LoRa, command interface) on a shared virtual "air".
 * Host tests only. Commands issued in a state the datasheet forbids are
 * counted in `violations` so tests can assert the driver never does it.
 */
#ifndef SX1262_EMU_H
#define SX1262_EMU_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define EMU_MAX_LOG 8192

typedef enum { EMU_STBY_RC = 2, EMU_FS = 4, EMU_RX = 5, EMU_TX = 6 } emu_mode_t;

typedef struct {
    uint32_t t_ms;
    uint32_t rf_freq;
} emu_freq_event_t;

typedef struct {
    int radio_id;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t rf_freq;
    uint8_t len;
} emu_tx_event_t;

typedef struct sx1262_emu {
    int id;
    bool present;    /* false: nothing on the bus, MISO reads 0 */
    bool has_tcxo;   /* board clock: TCXO (true) or crystal (false) */
    bool busy_stuck; /* BUSY pin never releases */

    emu_mode_t mode;
    bool cs_low;
    bool read_phase;
    uint8_t cmd[300];
    int cmd_n;
    uint8_t resp[300];
    int resp_i;
    uint32_t busy_until_ms;

    uint32_t rf_freq;
    uint8_t packet_type;
    uint8_t mod[4];
    uint8_t pkt[6];
    uint8_t tx_base, rx_base;
    uint8_t buffer[256];
    uint8_t regs[0x1000];
    uint16_t irq;
    uint16_t errors;
    int8_t tx_power;
    bool tcxo_cfg, calibrated, dio2_rf_switch, ant_power;
    uint8_t rx_len, rx_ptr;

    bool tx_active;
    uint32_t tx_start_ms, tx_end_ms;
    uint32_t rx_since_ms;

    int violations;
    char last_violation[96];

    emu_freq_event_t freq_log[EMU_MAX_LOG];
    int freq_log_n;

    SPI_HandleTypeDef spi;
    GPIO_TypeDef port;
} sx1262_emu_t;

extern uint32_t emu_now_ms;
extern bool emu_air_enabled;
extern emu_tx_event_t emu_tx_log[EMU_MAX_LOG];
extern int emu_tx_log_n;

void emu_reset_world(void);
void emu_add_radio(sx1262_emu_t *e, int id);

#endif
