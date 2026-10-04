#include "hal_shim.h"

static volatile uint32_t ticks_ms;

void SysTick_Handler(void)
{
    ticks_ms++;
}

uint32_t HAL_GetTick(void)
{
    return ticks_ms;
}

void HAL_Delay(uint32_t ms)
{
    uint32_t start = ticks_ms;
    /* +1 so a 1 ms delay always waits at least one full tick */
    while ((uint32_t)(ticks_ms - start) < ms + 1u) {
    }
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    port->BSRR = state == GPIO_PIN_SET ? pin : (uint32_t)pin << 16;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* SPI1 runs with an 8-bit frame and FRXTH set, so each byte written to DR
 * clocks exactly one byte back in. Byte-wide DR access is required. */
static uint8_t spi_xfer(SPI_TypeDef *spi, uint8_t out)
{
    while (!(spi->SR & SPI_SR_TXE)) {
    }
    *(volatile uint8_t *)&spi->DR = out;
    while (!(spi->SR & SPI_SR_RXNE)) {
    }
    return *(volatile uint8_t *)&spi->DR;
}

static void spi_drain(SPI_TypeDef *spi)
{
    while (spi->SR & SPI_SR_BSY) {
    }
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    for (uint16_t i = 0; i < n; i++) {
        (void)spi_xfer(h->Instance, data[i]);
    }
    spi_drain(h->Instance);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    for (uint16_t i = 0; i < n; i++) {
        data[i] = spi_xfer(h->Instance, 0x00);
    }
    spi_drain(h->Instance);
    return HAL_OK;
}
