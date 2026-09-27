/*
 * Host-side stand-in for the STM32 HAL. SPI traffic is routed to an
 * emulated SX1276 (tests/mocks/sx1276_emu.c); HAL_GetTick() is virtual time.
 */
#ifndef MOCK_STM32F4XX_HAL_H
#define MOCK_STM32F4XX_HAL_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

struct sx1276_emu;

typedef struct {
    struct sx1276_emu *emu;
} SPI_HandleTypeDef;

typedef struct {
    struct sx1276_emu *emu;
} GPIO_TypeDef;

#define GPIO_PIN_NSS 0x0001u
#define GPIO_PIN_RST 0x0002u

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *data, uint16_t n, uint32_t timeout);

#endif
