/*
 * board.h - NUCLEO-WL55JC (MB1389) with a Semtech SX1262MB2xAS shield on the
 * Arduino header. The WL55's own sub-GHz radio is not used: on the JC2
 * (low-band) board it is matched for 433-510 MHz, not 902-928 MHz.
 *
 * Pin map: shield signals from Semtech LoRaMac-node (sx1262mbx*-board.c),
 * Arduino-to-STM32 mapping from the Zephyr nucleo_wl55jc board files.
 *
 *   Shield signal   Arduino   WL55JC
 *   SCK/MISO/MOSI   D13/12/11 PA5/PA6/PA7  (SPI1, AF5)
 *   NSS             D7        PC1
 *   RESET           A0        PB1
 *   BUSY            D3        PB3
 *   ANT_SW power    D8        PC2
 *   DIO1            D5        PB8  (not used: IRQs are polled)
 *
 *   CAN: MCP2515 (HW-184 board) shares SPI1 with the radio; its chip-select
 *   is Arduino D4 (PB5), which the SX1262 shield leaves free. Everything the
 *   CAN side needs is on labeled Arduino sockets. INT goes to Arduino D6
 *   (PB10, CN10 pin 25) for interrupt-driven receive (CAN_IRQ=1 builds);
 *   left unwired, the firmware polls as before.
 *   The MCP2515's TXCAN/RXCAN go to an Adafruit TJA1051T/3 transceiver.
 *
 *   Console: LPUART1 PA2/PA3 -> ST-LINK virtual COM port, 115200 8N1
 *   LEDs: blue PB15, green PB9, red PB11
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include "hal_shim.h"

#define RADIO_NSS_PORT  GPIOC
#define RADIO_NSS_PIN   GPIO_PIN_1
#define RADIO_RST_PORT  GPIOB
#define RADIO_RST_PIN   GPIO_PIN_1
#define RADIO_BUSY_PORT GPIOB
#define RADIO_BUSY_PIN  GPIO_PIN_3
#define RADIO_ANT_PORT  GPIOC
#define RADIO_ANT_PIN   GPIO_PIN_2

#define CAN_CS_PORT     GPIOB
#define CAN_CS_PIN      GPIO_PIN_5   /* Arduino D4 */
#define CAN_INT_PORT    GPIOB
#define CAN_INT_PIN     GPIO_PIN_10  /* Arduino D6, CN10 pin 25; MCP2515 INT, active low */

#define LED_BLUE_PORT   GPIOB
#define LED_BLUE_PIN    GPIO_PIN_15
#define LED_GREEN_PORT  GPIOB
#define LED_GREEN_PIN   GPIO_PIN_9
#define LED_RED_PORT    GPIOB
#define LED_RED_PIN     GPIO_PIN_11

extern SPI_HandleTypeDef board_spi1;

void board_init(void);

/* MCP2515 INT on PB10 (EXTI line 10, falling edge). handler runs in
 * interrupt context each time INT goes low. While either chip-select on the
 * shared SPI bus is low, the interrupt is held off (HAL_GPIO_WritePin in
 * hal_shim.c), so the handler never splits a radio or CAN transaction. */
void board_can_irq_init(void (*handler)(void));
int board_can_int_low(void);  /* 1 while the MCP2515 has a frame waiting */
void board_can_irq_kick(void); /* rerun the handler if INT is low (missed edge) */
void board_spi_cs_changed(GPIO_TypeDef *port, uint16_t pin, int low); /* from hal_shim */

void uart_puts(const char *s);
int uart_getc(void); /* next console character, -1 if none */
void uart_service(void); /* called from SysTick: moves buffered output to the UART */
void uart_put_int(int32_t v);
void uart_put_hex(uint32_t v, int digits);

void led_set(GPIO_TypeDef *port, uint16_t pin, int on);
void led_toggle(GPIO_TypeDef *port, uint16_t pin);

#endif /* BOARD_H */
