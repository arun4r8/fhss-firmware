#include "sx1276_emu.h"

#include <string.h>

#include "sx1276.h"

#define MAX_RADIOS 4

uint32_t emu_now_ms;
bool emu_air_enabled = true;
emu_tx_event_t emu_tx_log[EMU_MAX_LOG];
int emu_tx_log_n;

static sx1276_emu_t *radios[MAX_RADIOS];
static int n_radios;

/* ---- HAL ---------------------------------------------------------------- */

uint32_t HAL_GetTick(void) { return emu_now_ms; }
void HAL_Delay(uint32_t ms) { emu_now_ms += ms; }

static void power_on_reset(sx1276_emu_t *e)
{
    memset(e->regs, 0, sizeof(e->regs));
    e->regs[0x01] = 0x09; /* FSK standby, low-frequency mode */
    e->regs[0x42] = 0x12; /* version */
    e->tx_active = false;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    sx1276_emu_t *e = port->emu;
    if (pin == GPIO_PIN_NSS) {
        e->cs_low = state == GPIO_PIN_RESET;
        e->first_byte = e->cs_low;
    } else if (pin == GPIO_PIN_RST && state == GPIO_PIN_RESET) {
        power_on_reset(e);
    }
}

/* ---- radio model ------------------------------------------------------- */

uint32_t emu_frf(const sx1276_emu_t *e)
{
    return ((uint32_t)e->regs[0x06] << 16) | ((uint32_t)e->regs[0x07] << 8) | e->regs[0x08];
}

uint8_t emu_mode(const sx1276_emu_t *e) { return e->regs[0x01] & 0x07; }

static sx1276_lora_cfg_t emu_lora_cfg(const sx1276_emu_t *e)
{
    static const uint32_t bw[] = {7800, 10400, 15600, 20800, 31250, 41700, 62500,
                                  125000, 250000, 500000};
    sx1276_lora_cfg_t c = {
        .spreading_factor = (uint8_t)(e->regs[0x1E] >> 4),
        .bandwidth_hz = bw[(e->regs[0x1D] >> 4) % 10],
        .coding_rate = (uint8_t)((e->regs[0x1D] >> 1) & 0x07),
        .preamble_len = (uint16_t)((e->regs[0x20] << 8) | e->regs[0x21]),
    };
    return c;
}

static bool same_modem(const sx1276_emu_t *a, const sx1276_emu_t *b)
{
    return a->regs[0x1D] == b->regs[0x1D] && (a->regs[0x1E] & 0xF4) == (b->regs[0x1E] & 0xF4) &&
           a->regs[0x39] == b->regs[0x39];
}

static void finish_tx(sx1276_emu_t *tx)
{
    tx->tx_active = false;
    tx->regs[0x12] |= 0x08; /* TxDone */
    tx->regs[0x01] = (uint8_t)((tx->regs[0x01] & 0xF8) | 0x01); /* back to standby */

    if (!emu_air_enabled) {
        return;
    }
    uint8_t len = tx->regs[0x22];
    for (int i = 0; i < n_radios; i++) {
        sx1276_emu_t *rx = radios[i];
        bool listening = rx != tx && (rx->regs[0x01] & 0x80) && emu_mode(rx) == 0x05 &&
                         emu_frf(rx) == emu_frf(tx) && same_modem(rx, tx) &&
                         rx->rx_since_ms <= tx->tx_start_ms;
        if (!listening) {
            continue;
        }
        uint8_t base = rx->regs[0x0F];
        for (uint8_t k = 0; k < len; k++) {
            rx->fifo[(uint8_t)(base + k)] = tx->fifo[(uint8_t)(tx->regs[0x0E] + k)];
        }
        rx->regs[0x10] = base;
        rx->regs[0x13] = len;
        rx->regs[0x1A] = 100; /* -57 dBm */
        rx->regs[0x1C] = (tx->regs[0x1E] & 0x04) ? 0x40 : 0x00;
        rx->regs[0x12] |= 0x50; /* RxDone | ValidHeader */
    }
}

static void write_reg(sx1276_emu_t *e, uint8_t addr, uint8_t v)
{
    switch (addr) {
    case 0x00:
        e->fifo[e->regs[0x0D]++] = v;
        return;
    case 0x12:
        e->regs[0x12] &= (uint8_t)~v;
        return;
    case 0x01: {
        uint8_t mode = v & 0x07;
        e->regs[0x01] = v;
        if (mode == 0x03 && (v & 0x80)) {
            sx1276_lora_cfg_t c = emu_lora_cfg(e);
            e->tx_active = true;
            e->tx_start_ms = emu_now_ms;
            e->tx_end_ms = emu_now_ms + sx1276_airtime_ms(&c, e->regs[0x22]);
            if (emu_tx_log_n < EMU_MAX_LOG) {
                emu_tx_log[emu_tx_log_n++] = (emu_tx_event_t){
                    e->id, e->tx_start_ms, e->tx_end_ms, emu_frf(e), e->regs[0x22]};
            }
        } else if (mode == 0x05) {
            e->rx_since_ms = emu_now_ms;
        }
        return;
    }
    case 0x08:
        e->regs[0x08] = v;
        e->rx_since_ms = emu_now_ms;
        if (e->frf_log_n < EMU_MAX_LOG &&
            (e->frf_log_n == 0 || e->frf_log[e->frf_log_n - 1].frf != emu_frf(e))) {
            e->frf_log[e->frf_log_n++] = (emu_frf_event_t){emu_now_ms, emu_frf(e)};
        }
        return;
    default:
        e->regs[addr & 0x7F] = v;
    }
}

static uint8_t read_reg(sx1276_emu_t *e, uint8_t addr)
{
    if (!e->present) {
        return 0;
    }
    if (addr == 0x00) {
        return e->fifo[e->regs[0x0D]++];
    }
    if (addr == 0x12 && e->tx_active) {
        emu_now_ms++; /* polling TxDone takes time */
        if (emu_now_ms >= e->tx_end_ms) {
            finish_tx(e);
        }
    }
    return e->regs[addr & 0x7F];
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    sx1276_emu_t *e = h->emu;
    for (uint16_t i = 0; i < n; i++) {
        if (e->first_byte) {
            e->addr = data[i] & 0x7F;
            e->writing = (data[i] & 0x80) != 0;
            e->first_byte = false;
        } else if (e->writing) {
            write_reg(e, e->addr, data[i]);
            if (e->addr != 0x00) {
                e->addr++;
            }
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    sx1276_emu_t *e = h->emu;
    for (uint16_t i = 0; i < n; i++) {
        data[i] = read_reg(e, e->addr);
        if (e->addr != 0x00) {
            e->addr++;
        }
    }
    return HAL_OK;
}

/* ---- world ------------------------------------------------------------- */

void emu_reset_world(void)
{
    emu_now_ms = 0;
    emu_air_enabled = true;
    emu_tx_log_n = 0;
    n_radios = 0;
}

void emu_add_radio(sx1276_emu_t *e, int id)
{
    memset(e, 0, sizeof(*e));
    e->id = id;
    e->present = true;
    e->spi.emu = e;
    e->port.emu = e;
    power_on_reset(e);
    if (n_radios < MAX_RADIOS) {
        radios[n_radios++] = e;
    }
}
