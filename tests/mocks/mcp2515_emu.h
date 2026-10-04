/*
 * Emulated MCP2515 CAN controllers on a shared virtual CAN bus. Host tests
 * only. Models the SPI instruction set, operating modes, loopback, ACK
 * (needs another node in normal mode), RX buffer rollover and filters.
 */
#ifndef MCP2515_EMU_H
#define MCP2515_EMU_H

#include <stdbool.h>
#include <stdint.h>

#include "mock_hal.h"

typedef struct {
    bool present;    /* false: nothing on the bus, MISO reads 0 */
    bool mode_stuck; /* refuses mode changes */

    uint8_t regs[128];
    bool cs_low;
    uint8_t ins;
    int nbytes;      /* bytes clocked in this transaction */
    uint8_t addr;
    uint8_t bm_mask;

    int frames_sent;
    int violations;

    SPI_HandleTypeDef spi;
    GPIO_TypeDef port;
} mcp2515_emu_t;

extern uint32_t emu_now_ms;

void can_emu_reset_world(void);
void can_emu_add(mcp2515_emu_t *e);

#endif
