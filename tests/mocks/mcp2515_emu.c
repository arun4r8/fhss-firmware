#include "mcp2515_emu.h"

#include <string.h>

#define MAX_NODES 4

uint32_t emu_now_ms;
static uint32_t tick_calls;
static mcp2515_emu_t *nodes[MAX_NODES];
static int n_nodes;

/* Every 8 polls of the clock count as 1 ms, so driver timeouts expire. */
uint32_t HAL_GetTick(void)
{
    if (++tick_calls % 8 == 0) {
        emu_now_ms++;
    }
    return emu_now_ms;
}

void HAL_Delay(uint32_t ms) { emu_now_ms += ms; }

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port;
    (void)pin;
    return GPIO_PIN_RESET;
}

/* ---- chip model -------------------------------------------------------- */

#define CANSTAT  0x0E
#define CANCTRL  0x0F
#define TEC      0x1C
#define CANINTF  0x2C
#define EFLG     0x2D
#define TXB0CTRL 0x30
#define RXB0CTRL 0x60
#define RXB1CTRL 0x70

static void chip_reset(mcp2515_emu_t *e)
{
    memset(e->regs, 0, sizeof(e->regs));
    e->regs[CANSTAT] = 0x80; /* configuration mode */
    e->regs[CANCTRL] = 0x87;
}

static uint8_t mode_of(const mcp2515_emu_t *e) { return e->regs[CANSTAT] & 0xE0; }

static uint16_t sid_at(const mcp2515_emu_t *e, uint8_t addr)
{
    return (uint16_t)((e->regs[addr] << 3) | (e->regs[addr + 1] >> 5));
}

static bool accepts(const mcp2515_emu_t *e, uint8_t ctrl, uint16_t id, uint8_t mask_addr,
                    const uint8_t *filters, int nf)
{
    if ((e->regs[ctrl] & 0x60) == 0x60) {
        return true;
    }
    uint16_t mask = sid_at(e, mask_addr);
    for (int i = 0; i < nf; i++) {
        if (((sid_at(e, filters[i]) ^ id) & mask) == 0) {
            return true;
        }
    }
    return false;
}

static void deliver(mcp2515_emu_t *rx, const uint8_t *frame /* SIDH..D7 */)
{
    static const uint8_t f0[] = {0x00, 0x04}, f1[] = {0x08, 0x10, 0x14, 0x18};
    uint16_t id = (uint16_t)((frame[0] << 3) | (frame[1] >> 5));
    bool ok0 = accepts(rx, RXB0CTRL, id, 0x20, f0, 2);
    bool ok1 = accepts(rx, RXB1CTRL, id, 0x24, f1, 4);

    if (ok0 && !(rx->regs[CANINTF] & 0x01)) {
        memcpy(&rx->regs[0x61], frame, 13);
        rx->regs[CANINTF] |= 0x01;
    } else if ((ok1 || (ok0 && (rx->regs[RXB0CTRL] & 0x04))) && !(rx->regs[CANINTF] & 0x02)) {
        memcpy(&rx->regs[0x71], frame, 13);
        rx->regs[CANINTF] |= 0x02;
    } else if (ok0 || ok1) {
        rx->regs[EFLG] |= 0x40; /* RX overflow */
    }
}

static void request_tx(mcp2515_emu_t *e)
{
    e->regs[TXB0CTRL] |= 0x08; /* TXREQ */
    const uint8_t *frame = &e->regs[0x31];
    if (mode_of(e) == 0x40) { /* loopback */
        deliver(e, frame);
    } else if (mode_of(e) == 0x00) {
        bool acked = false;
        for (int i = 0; i < n_nodes; i++) {
            if (nodes[i] != e && mode_of(nodes[i]) == 0x00) {
                acked = true;
            }
        }
        if (!acked) { /* nobody ACKs: real chips retry forever */
            e->regs[TXB0CTRL] |= 0x10; /* TXERR */
            e->regs[TEC] = (uint8_t)(e->regs[TEC] + 8);
            return;
        }
        for (int i = 0; i < n_nodes; i++) {
            if (nodes[i] != e && (mode_of(nodes[i]) == 0x00 || mode_of(nodes[i]) == 0x60)) {
                deliver(nodes[i], frame);
            }
        }
    } else {
        return; /* config / listen-only: nothing goes out */
    }
    e->regs[TXB0CTRL] &= (uint8_t)~0x18;
    e->regs[CANINTF] |= 0x04;
    e->frames_sent++;
}

static void write_reg(mcp2515_emu_t *e, uint8_t addr, uint8_t v)
{
    addr &= 0x7F;
    if (addr == CANCTRL) {
        e->regs[CANCTRL] = v;
        if (!e->mode_stuck) {
            e->regs[CANSTAT] = (uint8_t)((e->regs[CANSTAT] & 0x1F) | (v & 0xE0));
        }
        return;
    }
    if (addr == CANSTAT) {
        return; /* read-only */
    }
    if ((addr & 0x0F) == 0x0F || (addr & 0x0F) == 0x0E) {
        /* CANSTAT/CANCTRL mirrors at xEh/xFh */
        return;
    }
    if (addr >= 0x28 && addr <= 0x2A && mode_of(e) != 0x80) {
        e->violations++; /* CNFx only writable in config mode */
        return;
    }
    e->regs[addr] = v;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)pin;
    mcp2515_emu_t *e = port->emu;
    if (state == GPIO_PIN_RESET) {
        e->cs_low = true;
        e->nbytes = 0;
        return;
    }
    if (e->cs_low && e->present) {
        if (e->ins == 0x90) e->regs[CANINTF] &= (uint8_t)~0x01;
        if (e->ins == 0x94) e->regs[CANINTF] &= (uint8_t)~0x02;
    }
    e->cs_low = false;
    e->ins = 0;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    mcp2515_emu_t *e = h->emu;
    if (!e->cs_low) {
        e->violations++;
    }
    for (uint16_t i = 0; i < n; i++) {
        uint8_t b = data[i];
        int k = e->nbytes++;
        if (!e->present) {
            continue;
        }
        if (k == 0) {
            e->ins = b;
            if (b == 0xC0) chip_reset(e);
            if (b == 0x40) e->addr = 0x31;
            if (b == 0x81) request_tx(e);
            continue;
        }
        switch (e->ins) {
        case 0x02: /* WRITE */
            if (k == 1) e->addr = b;
            else write_reg(e, e->addr++, b);
            break;
        case 0x03: /* READ: address byte */
            if (k == 1) e->addr = b;
            break;
        case 0x05: /* BIT MODIFY */
            if (k == 1) e->addr = b;
            else if (k == 2) e->bm_mask = b;
            else if (k == 3) {
                uint8_t cur = e->regs[e->addr & 0x7F];
                uint8_t v = (uint8_t)((cur & ~e->bm_mask) | (b & e->bm_mask));
                if ((e->addr & 0x7F) == TXB0CTRL) {
                    e->regs[TXB0CTRL] = v; /* clearing TXREQ aborts */
                } else {
                    write_reg(e, e->addr, v);
                }
            }
            break;
        case 0x40: /* LOAD TX BUFFER 0 */
            e->regs[e->addr++ & 0x7F] = b;
            break;
        default:
            e->violations++;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    mcp2515_emu_t *e = h->emu;
    for (uint16_t i = 0; i < n; i++, e->nbytes++) {
        if (!e->present) {
            data[i] = 0;
            continue;
        }
        switch (e->ins) {
        case 0x03:
            data[i] = e->regs[e->addr++ & 0x7F];
            break;
        case 0x90:
            data[i] = e->regs[0x61 + (e->nbytes - 1)];
            break;
        case 0x94:
            data[i] = e->regs[0x71 + (e->nbytes - 1)];
            break;
        case 0xB0:
            data[i] = (uint8_t)(((e->regs[CANINTF] & 0x01) ? 0x40 : 0) |
                                ((e->regs[CANINTF] & 0x02) ? 0x80 : 0));
            break;
        default:
            e->violations++;
            data[i] = 0;
        }
    }
    return HAL_OK;
}

/* ---- world ------------------------------------------------------------- */

void can_emu_reset_world(void)
{
    emu_now_ms = 0;
    tick_calls = 0;
    n_nodes = 0;
}

void can_emu_add(mcp2515_emu_t *e)
{
    memset(e, 0, sizeof(*e));
    e->present = true;
    e->spi.emu = e;
    e->port.emu = e;
    chip_reset(e);
    if (n_nodes < MAX_NODES) {
        nodes[n_nodes++] = e;
    }
}
