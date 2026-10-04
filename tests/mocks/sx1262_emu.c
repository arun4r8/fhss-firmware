#include "sx1262_emu.h"

#include <stdio.h>
#include <string.h>

#include "sx1262.h"

#define MAX_RADIOS 4

uint32_t emu_now_ms;
bool emu_air_enabled = true;
emu_tx_event_t emu_tx_log[EMU_MAX_LOG];
int emu_tx_log_n;

static sx1262_emu_t *radios[MAX_RADIOS];
static int n_radios;

static void violation(sx1262_emu_t *e, const char *what)
{
    e->violations++;
    snprintf(e->last_violation, sizeof(e->last_violation), "t=%u op=0x%02X: %s", emu_now_ms,
             e->cmd_n ? e->cmd[0] : 0, what);
}

static void power_on_reset(sx1262_emu_t *e)
{
    e->mode = EMU_STBY_RC;
    e->rf_freq = 0;
    e->packet_type = 0;
    memset(e->mod, 0, sizeof(e->mod));
    memset(e->pkt, 0, sizeof(e->pkt));
    memset(e->regs, 0, sizeof(e->regs));
    e->regs[0x0740] = 0x14; /* reset sync word 0x1424 */
    e->regs[0x0741] = 0x24;
    e->irq = 0;
    /* datasheet: with a TCXO, XOSC_START_ERR is raised at power-on */
    e->errors = e->has_tcxo ? 0x0020 : 0;
    e->tcxo_cfg = e->calibrated = e->dio2_rf_switch = false;
    e->noise_rssi = 220; /* -110 dBm */
    e->tx_active = false;
    e->busy_until_ms = emu_now_ms + 4;
}

/* ---- HAL ---------------------------------------------------------------- */

uint32_t HAL_GetTick(void) { return emu_now_ms; }
void HAL_Delay(uint32_t ms) { emu_now_ms += ms; }

static void execute(sx1262_emu_t *e);

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    sx1262_emu_t *e = port->emu;
    if (pin == GPIO_PIN_NSS) {
        if (state == GPIO_PIN_RESET) {
            if (emu_now_ms < e->busy_until_ms || e->busy_stuck) {
                violation(e, "NSS asserted while BUSY high");
            }
            e->cs_low = true;
            e->cmd_n = 0;
            e->read_phase = false;
        } else {
            if (e->cs_low && !e->read_phase && e->cmd_n > 0 && e->present) {
                execute(e);
            }
            e->cs_low = false;
        }
    } else if (pin == GPIO_PIN_RST && state == GPIO_PIN_RESET) {
        power_on_reset(e);
    } else if (pin == GPIO_PIN_ANT) {
        e->ant_power = state == GPIO_PIN_SET;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    sx1262_emu_t *e = port->emu;
    if (pin == GPIO_PIN_BUSY && (e->busy_stuck || emu_now_ms < e->busy_until_ms)) {
        emu_now_ms++; /* time passes while the MCU spins on BUSY */
        return GPIO_PIN_SET;
    }
    return GPIO_PIN_RESET;
}

/* ---- radio model ------------------------------------------------------- */

static sx1262_lora_cfg_t lora_cfg(const sx1262_emu_t *e)
{
    static const uint32_t bw[] = {7810, 15630, 31250, 62500, 125000, 250000, 500000};
    sx1262_lora_cfg_t c = {
        .spreading_factor = e->mod[0],
        .bandwidth_hz = e->mod[1] <= 6 ? bw[e->mod[1]] : 125000,
        .coding_rate = e->mod[2],
        .preamble_len = (uint16_t)((e->pkt[0] << 8) | e->pkt[1]),
    };
    return c;
}

static bool same_modem(const sx1262_emu_t *a, const sx1262_emu_t *b)
{
    return a->packet_type == b->packet_type && memcmp(a->mod, b->mod, 3) == 0 &&
           a->regs[0x0740] == b->regs[0x0740] && a->regs[0x0741] == b->regs[0x0741];
}

static void finish_tx(sx1262_emu_t *tx)
{
    tx->tx_active = false;
    tx->irq |= 0x0001; /* TxDone */
    tx->mode = EMU_STBY_RC;
    if (!emu_air_enabled) {
        return;
    }
    uint8_t len = tx->pkt[3];
    for (int i = 0; i < n_radios; i++) {
        sx1262_emu_t *rx = radios[i];
        bool listening = rx != tx && rx->mode == EMU_RX && rx->rf_freq == tx->rf_freq &&
                         same_modem(rx, tx) && rx->rx_since_ms <= tx->tx_start_ms;
        if (!listening) {
            continue;
        }
        for (uint8_t k = 0; k < len; k++) {
            rx->buffer[(uint8_t)(rx->rx_base + k)] = tx->buffer[(uint8_t)(tx->tx_base + k)];
        }
        rx->rx_len = len;
        rx->rx_ptr = rx->rx_base;
        rx->irq |= 0x0002 | 0x0010; /* RxDone | HeaderValid */
    }
}

static uint8_t status_byte(const sx1262_emu_t *e) { return (uint8_t)(e->mode << 4); }

static void build_response(sx1262_emu_t *e)
{
    uint8_t op = e->cmd[0];
    memset(e->resp, 0, sizeof(e->resp));
    e->resp[0] = status_byte(e);
    e->resp_i = 0;
    if (!e->present) {
        e->resp[0] = 0;
        return;
    }
    switch (op) {
    case 0xC0: /* GetStatus */
        break;
    case 0x12: /* GetIrqStatus: polling during TX lets time pass */
        if (e->tx_active) {
            emu_now_ms++;
            if (emu_now_ms >= e->tx_end_ms) {
                finish_tx(e);
            }
        }
        e->resp[1] = (uint8_t)(e->irq >> 8);
        e->resp[2] = (uint8_t)e->irq;
        break;
    case 0x17: /* GetDeviceErrors */
        e->resp[1] = (uint8_t)(e->errors >> 8);
        e->resp[2] = (uint8_t)e->errors;
        break;
    case 0x13: /* GetRxBufferStatus */
        e->resp[1] = e->rx_len;
        e->resp[2] = e->rx_ptr;
        break;
    case 0x14: /* GetPacketStatus: RssiPkt = -dBm * 2 */
        e->resp[1] = 114;
        break;
    case 0x15: /* GetRssiInst */
        if (e->mode != EMU_RX) {
            violation(e, "GetRssiInst outside RX");
        }
        e->resp[1] = e->noise_rssi;
        break;
    case 0x1D: { /* ReadRegister */
        uint16_t a = (uint16_t)((e->cmd[1] << 8) | e->cmd[2]);
        for (int i = 0; i < 16; i++) {
            e->resp[1 + i] = e->regs[(a + i) & 0xFFF];
        }
        break;
    }
    case 0x1E: /* ReadBuffer */
        for (int i = 0; i < 256; i++) {
            e->resp[1 + i] = e->buffer[(uint8_t)(e->cmd[1] + i)];
        }
        break;
    default:
        violation(e, "read on a write-only opcode");
    }
}

static void execute(sx1262_emu_t *e)
{
    uint8_t op = e->cmd[0];
    const uint8_t *p = &e->cmd[1];
    int np = e->cmd_n - 1;

    if (e->tx_active && op != 0x80) {
        violation(e, "command during TX");
    }

    switch (op) {
    case 0x80: /* SetStandby */
        e->mode = EMU_STBY_RC;
        e->tx_active = false;
        break;
    case 0x82: /* SetRx */
        if (!e->calibrated || e->packet_type != 1 || !e->dio2_rf_switch) {
            violation(e, "SetRx before calibration / packet type / RF switch");
        }
        e->mode = EMU_RX;
        e->rx_since_ms = emu_now_ms;
        break;
    case 0x83: { /* SetTx */
        if (!e->calibrated || e->packet_type != 1 || !e->dio2_rf_switch || !e->ant_power) {
            violation(e, "SetTx before calibration / packet type / RF switch / ant power");
        }
        sx1262_lora_cfg_t c = lora_cfg(e);
        e->mode = EMU_TX;
        e->tx_active = true;
        e->tx_start_ms = emu_now_ms;
        e->tx_end_ms = emu_now_ms + sx1262_airtime_ms(&c, e->pkt[3]);
        if (emu_tx_log_n < EMU_MAX_LOG) {
            emu_tx_log[emu_tx_log_n++] =
                (emu_tx_event_t){e->id, e->tx_start_ms, e->tx_end_ms, e->rf_freq, e->pkt[3]};
        }
        break;
    }
    case 0x86: /* SetRfFrequency */
        if (e->mode == EMU_RX || e->mode == EMU_TX) {
            violation(e, "SetRfFrequency while in RX/TX");
        }
        e->rf_freq = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
        e->rx_since_ms = emu_now_ms;
        if (e->freq_log_n < EMU_MAX_LOG &&
            (e->freq_log_n == 0 || e->freq_log[e->freq_log_n - 1].rf_freq != e->rf_freq)) {
            e->freq_log[e->freq_log_n++] = (emu_freq_event_t){emu_now_ms, e->rf_freq};
        }
        break;
    case 0x89: /* Calibrate */
        if (e->has_tcxo != e->tcxo_cfg) {
            e->errors |= 0x0020; /* XOSC_START_ERR */
        }
        e->calibrated = true;
        e->busy_until_ms = emu_now_ms + 4;
        break;
    case 0xC1: /* SetFs */
        if (!e->calibrated) violation(e, "SetFs before calibration");
        e->mode = EMU_FS;
        if (e->pll_fail || e->rf_freq < sx1262_rf_freq_from_hz(150000000u) ||
            e->rf_freq > sx1262_rf_freq_from_hz(960000000u)) {
            e->errors |= 0x0040; /* PLL_LOCK_ERR */
        }
        break;
    case 0x8A: e->packet_type = p[0]; break;
    case 0x8B:
        if (e->packet_type != 1) violation(e, "modulation params before packet type");
        memcpy(e->mod, p, 4);
        break;
    case 0x8C:
        if (e->packet_type != 1) violation(e, "packet params before packet type");
        memcpy(e->pkt, p, 6);
        break;
    case 0x8E: e->tx_power = (int8_t)p[0]; break;
    case 0x8F: e->tx_base = p[0]; e->rx_base = p[1]; break;
    case 0x95: case 0x96: case 0x98: case 0x08: break;
    case 0x97: e->tcxo_cfg = true; break;
    case 0x9D: e->dio2_rf_switch = p[0] != 0; break;
    case 0x02: e->irq &= (uint16_t)~((p[0] << 8) | p[1]); break;
    case 0x07: e->errors = 0; break;
    case 0x0D: {
        uint16_t a = (uint16_t)((p[0] << 8) | p[1]);
        for (int i = 2; i < np; i++) e->regs[(a + i - 2) & 0xFFF] = p[i];
        break;
    }
    case 0x0E:
        for (int i = 1; i < np; i++) e->buffer[(uint8_t)(p[0] + i - 1)] = p[i];
        break;
    default:
        violation(e, "unknown opcode");
    }
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    sx1262_emu_t *e = h->emu;
    if (!e->cs_low) violation(e, "SPI without NSS");
    for (uint16_t i = 0; i < n && e->cmd_n < (int)sizeof(e->cmd); i++) {
        e->cmd[e->cmd_n++] = data[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    sx1262_emu_t *e = h->emu;
    if (!e->read_phase) {
        build_response(e);
        e->read_phase = true;
    }
    for (uint16_t i = 0; i < n; i++) {
        data[i] = e->resp_i < (int)sizeof(e->resp) ? e->resp[e->resp_i++] : 0;
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

void emu_add_radio(sx1262_emu_t *e, int id)
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
