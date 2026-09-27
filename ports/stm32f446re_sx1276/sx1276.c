#include "sx1276.h"

/* LoRa-mode register map (SX1276 datasheet, section 6.4) */
#define REG_FIFO              0x00
#define REG_OP_MODE           0x01
#define REG_FRF_MSB           0x06
#define REG_PA_CONFIG         0x09
#define REG_LNA               0x0C
#define REG_FIFO_ADDR_PTR     0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT   0x10
#define REG_IRQ_FLAGS         0x12
#define REG_RX_NB_BYTES       0x13
#define REG_PKT_RSSI_VALUE    0x1A
#define REG_HOP_CHANNEL       0x1C
#define REG_MODEM_CONFIG_1    0x1D
#define REG_MODEM_CONFIG_2    0x1E
#define REG_PREAMBLE_MSB      0x20
#define REG_PREAMBLE_LSB      0x21
#define REG_PAYLOAD_LENGTH    0x22
#define REG_MODEM_CONFIG_3    0x26
#define REG_SYNC_WORD         0x39
#define REG_VERSION           0x42

#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP      0x00
#define MODE_STDBY      0x01
#define MODE_TX         0x03
#define MODE_RX_CONT    0x05

#define IRQ_RX_DONE    0x40
#define IRQ_CRC_ERROR  0x20
#define IRQ_TX_DONE    0x08

#define HOP_CHANNEL_CRC_ON 0x40
#define SX1276_VERSION     0x12
#define FXOSC_HZ           32000000u
#define SPI_TIMEOUT_MS     10u
#define TX_TIMEOUT_MARGIN_MS 50u

/* ---- SPI access -------------------------------------------------------- */

static void cs_low(sx1276_t *r) { HAL_GPIO_WritePin(r->nss_port, r->nss_pin, GPIO_PIN_RESET); }
static void cs_high(sx1276_t *r) { HAL_GPIO_WritePin(r->nss_port, r->nss_pin, GPIO_PIN_SET); }

static void write_burst(sx1276_t *r, uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t addr = reg | 0x80;
    cs_low(r);
    HAL_SPI_Transmit(r->spi, &addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Transmit(r->spi, (uint8_t *)data, len, SPI_TIMEOUT_MS);
    cs_high(r);
}

static void read_burst(sx1276_t *r, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t addr = reg & 0x7F;
    cs_low(r);
    HAL_SPI_Transmit(r->spi, &addr, 1, SPI_TIMEOUT_MS);
    HAL_SPI_Receive(r->spi, data, len, SPI_TIMEOUT_MS);
    cs_high(r);
}

static void write_reg(sx1276_t *r, uint8_t reg, uint8_t value) { write_burst(r, reg, &value, 1); }

static uint8_t read_reg(sx1276_t *r, uint8_t reg)
{
    uint8_t v = 0;
    read_burst(r, reg, &v, 1);
    return v;
}

static void set_mode(sx1276_t *r, uint8_t mode) { write_reg(r, REG_OP_MODE, MODE_LONG_RANGE | mode); }

/* ---- helpers ----------------------------------------------------------- */

static int bw_code(uint32_t bw_hz)
{
    switch (bw_hz) {
    case 125000: return 0x7;
    case 250000: return 0x8;
    case 500000: return 0x9;
    default: return -1;
    }
}

static uint32_t symbol_time_us(const sx1276_lora_cfg_t *c)
{
    return (uint32_t)(((uint64_t)1000000u << c->spreading_factor) / c->bandwidth_hz);
}

/* Required by the datasheet when a symbol is longer than 16 ms. */
static int low_data_rate_optimize(const sx1276_lora_cfg_t *c) { return symbol_time_us(c) > 16000u; }

uint32_t sx1276_frf_from_hz(uint32_t freq_hz)
{
    return (uint32_t)(((uint64_t)freq_hz << 19) / FXOSC_HZ);
}

uint32_t sx1276_airtime_ms(const sx1276_lora_cfg_t *c, uint8_t payload_len)
{
    int32_t sf = c->spreading_factor;
    int32_t de = low_data_rate_optimize(c);
    int32_t num = 8 * (int32_t)payload_len - 4 * sf + 28 + 16 /* CRC on */ - 0 /* explicit hdr */;
    int32_t den = 4 * (sf - 2 * de);
    int32_t blocks = num > 0 ? (num + den - 1) / den : 0;
    uint32_t payload_syms = 8u + (uint32_t)(blocks * (c->coding_rate + 4));

    /* preamble adds 4.25 symbols: work in quarter-symbols to stay integer */
    uint32_t quarter_syms = 4u * (c->preamble_len + payload_syms) + 17u;
    uint64_t us = (uint64_t)quarter_syms * symbol_time_us(c) / 4u;
    return (uint32_t)((us + 999u) / 1000u);
}

/* ---- public API -------------------------------------------------------- */

sx1276_err_t sx1276_init(sx1276_t *r, const sx1276_lora_cfg_t *cfg, uint32_t freq_hz)
{
    int bw = bw_code(cfg->bandwidth_hz);
    if (bw < 0 || cfg->spreading_factor < 7 || cfg->spreading_factor > 12 ||
        cfg->coding_rate < 1 || cfg->coding_rate > 4 || cfg->tx_power_dbm < 2 ||
        cfg->tx_power_dbm > 17 || cfg->preamble_len < 6) {
        return SX1276_ERR_BAD_CONFIG;
    }
    r->cfg = *cfg;
    r->rx_active = 0;

    cs_high(r);
    HAL_GPIO_WritePin(r->rst_port, r->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(r->rst_port, r->rst_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    if (read_reg(r, REG_VERSION) != SX1276_VERSION) {
        return SX1276_ERR_NOT_FOUND;
    }

    /* LoRa mode can only be selected from sleep */
    write_reg(r, REG_OP_MODE, MODE_SLEEP);
    set_mode(r, MODE_SLEEP);

    sx1276_set_frequency(r, freq_hz);
    write_reg(r, REG_FIFO_TX_BASE_ADDR, 0);
    write_reg(r, REG_FIFO_RX_BASE_ADDR, 0);
    write_reg(r, REG_LNA, 0x23); /* max gain, HF LNA boost */
    write_reg(r, REG_MODEM_CONFIG_1, (uint8_t)((bw << 4) | (cfg->coding_rate << 1)));
    write_reg(r, REG_MODEM_CONFIG_2, (uint8_t)((cfg->spreading_factor << 4) | 0x04)); /* CRC on */
    write_reg(r, REG_MODEM_CONFIG_3,
              (uint8_t)(0x04 /* AGC */ | (low_data_rate_optimize(cfg) ? 0x08 : 0)));
    write_reg(r, REG_PREAMBLE_MSB, (uint8_t)(cfg->preamble_len >> 8));
    write_reg(r, REG_PREAMBLE_LSB, (uint8_t)(cfg->preamble_len & 0xFF));
    write_reg(r, REG_SYNC_WORD, cfg->sync_word);
    /* RFM95W only has the PA_BOOST pin connected: Pout = 17 - (15 - OutputPower) */
    write_reg(r, REG_PA_CONFIG, (uint8_t)(0x80 | 0x70 | (cfg->tx_power_dbm - 2)));

    set_mode(r, MODE_STDBY);
    return SX1276_OK;
}

void sx1276_set_frequency(sx1276_t *r, uint32_t freq_hz)
{
    uint32_t frf = sx1276_frf_from_hz(freq_hz);
    uint8_t bytes[3] = {(uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf};

    if (r->rx_active) {
        set_mode(r, MODE_STDBY); /* FRF changes only apply outside RX/TX */
    }
    write_burst(r, REG_FRF_MSB, bytes, 3);
    if (r->rx_active) {
        set_mode(r, MODE_RX_CONT);
    }
}

void sx1276_start_rx(sx1276_t *r)
{
    set_mode(r, MODE_STDBY);
    write_reg(r, REG_FIFO_ADDR_PTR, 0);
    write_reg(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_RX_CONT);
    r->rx_active = 1;
}

sx1276_err_t sx1276_send(sx1276_t *r, const uint8_t *data, uint8_t len)
{
    if (len == 0) {
        return SX1276_ERR_BAD_CONFIG;
    }
    set_mode(r, MODE_STDBY);
    r->rx_active = 0;

    write_reg(r, REG_FIFO_ADDR_PTR, 0);
    write_burst(r, REG_FIFO, data, len);
    write_reg(r, REG_PAYLOAD_LENGTH, len);
    write_reg(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_TX);

    uint32_t start = HAL_GetTick();
    uint32_t timeout = sx1276_airtime_ms(&r->cfg, len) + TX_TIMEOUT_MARGIN_MS;
    sx1276_err_t err = SX1276_OK;
    while (!(read_reg(r, REG_IRQ_FLAGS) & IRQ_TX_DONE)) {
        if (HAL_GetTick() - start > timeout) {
            err = SX1276_ERR_TIMEOUT;
            break;
        }
    }
    write_reg(r, REG_IRQ_FLAGS, 0xFF);
    sx1276_start_rx(r);
    return err;
}

int sx1276_read_packet(sx1276_t *r, uint8_t *buf, uint8_t max, int16_t *rssi_dbm)
{
    uint8_t flags = read_reg(r, REG_IRQ_FLAGS);
    if (!(flags & IRQ_RX_DONE)) {
        return 0;
    }
    write_reg(r, REG_IRQ_FLAGS, 0xFF);

    /* A packet sent without CRC never raises CRC_ERROR, so require it on. */
    if ((flags & IRQ_CRC_ERROR) || !(read_reg(r, REG_HOP_CHANNEL) & HOP_CHANNEL_CRC_ON)) {
        return SX1276_ERR_CRC;
    }

    uint8_t len = read_reg(r, REG_RX_NB_BYTES);
    if (len > max) {
        return SX1276_ERR_TOO_LONG;
    }
    write_reg(r, REG_FIFO_ADDR_PTR, read_reg(r, REG_FIFO_RX_CURRENT));
    read_burst(r, REG_FIFO, buf, len);

    if (rssi_dbm) {
        *rssi_dbm = (int16_t)(-157 + read_reg(r, REG_PKT_RSSI_VALUE)); /* HF port */
    }
    return len;
}
