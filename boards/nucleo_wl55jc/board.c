#include "board.h"

/* The WL55 comes out of reset on the 4 MHz MSI. That is plenty for polled
 * SPI and a 115200 baud console, and avoids touching flash wait states. */
uint32_t SystemCoreClock = 4000000u;

SPI_HandleTypeDef board_spi1 = {SPI1};

#define MODE_INPUT  0u
#define MODE_OUTPUT 1u
#define MODE_AF     2u

static void pin_mode(GPIO_TypeDef *port, uint32_t pin_num, uint32_t mode, uint32_t af)
{
    port->MODER = (port->MODER & ~(3u << (pin_num * 2))) | (mode << (pin_num * 2));
    port->OSPEEDR |= 3u << (pin_num * 2); /* high speed */
    port->PUPDR &= ~(3u << (pin_num * 2));
    if (mode == MODE_AF) {
        volatile uint32_t *afr = &port->AFR[pin_num / 8];
        uint32_t shift = (pin_num % 8) * 4;
        *afr = (*afr & ~(0xFu << shift)) | (af << shift);
    }
}

static void output(GPIO_TypeDef *port, uint32_t pin_num, int high)
{
    HAL_GPIO_WritePin(port, (uint16_t)(1u << pin_num), high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    pin_mode(port, pin_num, MODE_OUTPUT, 0);
}

void board_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    (void)RCC->APB1ENR2; /* let the enables settle before touching peripherals */

    SysTick_Config(SystemCoreClock / 1000u);

    /* LEDs off */
    output(GPIOB, 15, 0);
    output(GPIOB, 9, 0);
    output(GPIOB, 11, 0);

    /* Radio control: NSS idle high, RESET released, antenna switch off until
     * the driver powers it, BUSY as a plain input (PB3 defaults to JTDO). */
    output(GPIOC, 1, 1);
    output(GPIOB, 1, 1);
    output(GPIOC, 2, 0);
    pin_mode(GPIOB, 3, MODE_INPUT, 0);

    /* SPI1 on PA5/PA6/PA7, AF5: master, mode 0, software NSS, PCLK/2 = 2 MHz */
    pin_mode(GPIOA, 5, MODE_AF, 5);
    pin_mode(GPIOA, 6, MODE_AF, 5);
    pin_mode(GPIOA, 7, MODE_AF, 5);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 = (7u << 8) /* DS = 8-bit */ | SPI_CR2_FRXTH;
    SPI1->CR1 |= SPI_CR1_SPE;

    /* LPUART1 on PA2 (TX) / PA3 (RX), AF8, 115200 from the 4 MHz PCLK1 */
    pin_mode(GPIOA, 2, MODE_AF, 8);
    pin_mode(GPIOA, 3, MODE_AF, 8);
    LPUART1->BRR = (256u * 4000000u + 115200u / 2u) / 115200u;
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_putc(char c)
{
    while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF)) {
    }
    LPUART1->TDR = (uint8_t)c;
}

void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') {
            uart_putc('\r');
        }
        uart_putc(*s++);
    }
}

void uart_put_int(int32_t v)
{
    char buf[12];
    int i = 0;
    uint32_t u = v < 0 ? (uint32_t)(-(v + 1)) + 1u : (uint32_t)v;
    do {
        buf[i++] = (char)('0' + u % 10u);
        u /= 10u;
    } while (u);
    if (v < 0) {
        uart_putc('-');
    }
    while (i) {
        uart_putc(buf[--i]);
    }
}

void uart_put_hex(uint32_t v, int digits)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int i = digits - 1; i >= 0; i--) {
        uart_putc(hex[(v >> (i * 4)) & 0xFu]);
    }
}

void led_set(GPIO_TypeDef *port, uint16_t pin, int on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void led_toggle(GPIO_TypeDef *port, uint16_t pin)
{
    led_set(port, pin, !(port->ODR & pin));
}

/* Any fault: red LED on solid, stop. */
void HardFault_Handler(void)
{
    GPIOB->BSRR = GPIO_PIN_11;
    for (;;) {
    }
}
