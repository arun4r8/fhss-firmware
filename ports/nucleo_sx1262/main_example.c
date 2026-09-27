/*
 * main_example.c - what to add to a CubeMX-generated main.c on a Nucleo-64
 * (e.g. NUCLEO-F446RE) with the Semtech SX1262MB2xAS shield on top. Copy each block into the matching USER CODE section;
 * this file is not built on its own.
 *
 * Demo: each node sends "L<n>" (leader) or "F<n>" (follower) once a
 * second and prints everything it receives, plus link status, on the
 * ST-LINK virtual COM port (USART2, 115200 8N1).
 *
 * Flash one board with NODE_IS_LEADER 1 and the other with 0.
 */

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "fhss_link.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define NODE_IS_LEADER 1          /* 1 on exactly one board, 0 on the other */
#define FHSS_SEED      0x6A5D10u  /* same on both boards */
#define FHSS_NET_ID    0x47       /* same on both boards */
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static fhss_link_t link;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void on_rx(void *user, const uint8_t *data, uint8_t len, int16_t rssi_dbm)
{
    (void)user;
    char line[80];
    snprintf(line, sizeof(line), "rx \"%.*s\" rssi %d dBm\r\n", len, (const char *)data, rssi_dbm);
    uart_print(line);
}
/* USER CODE END 0 */

int main(void)
{
    /* ... CubeMX init: HAL_Init, SystemClock_Config, MX_GPIO_Init,
     *     MX_SPI1_Init, MX_USART2_UART_Init ... */

    /* USER CODE BEGIN 2 */
    fhss_link_config_t cfg = {
        .spi = &hspi1,
        .nss_port = RADIO_NSS_GPIO_Port, .nss_pin = RADIO_NSS_Pin,     /* D7 / PA8 */
        .rst_port = RADIO_RST_GPIO_Port, .rst_pin = RADIO_RST_Pin,     /* A0 / PA0 */
        .busy_port = RADIO_BUSY_GPIO_Port, .busy_pin = RADIO_BUSY_Pin, /* D3 / PB3 */
        .ant_pwr_port = RADIO_ANT_GPIO_Port, .ant_pwr_pin = RADIO_ANT_Pin, /* D8 / PA9 */
        .use_tcxo = false, /* true only if the shield sticker says ...DAS (TCXO) */
        .leader = NODE_IS_LEADER,
        .seed = FHSS_SEED,
        .net_id = FHSS_NET_ID,
        .tx_power_dbm = 5, /* keep low on the bench; range -9..22 */
        .on_rx = on_rx,
        .user = NULL,
    };
    if (fhss_link_init(&link, &cfg) != FHSS_LINK_OK) {
        char err[96];
        snprintf(err, sizeof(err), "radio init failed: %d (see ports/nucleo_sx1262/README.md)\r\n",
                 (int)link.radio_err);
        uart_print(err);
        Error_Handler();
    }
    uart_print(NODE_IS_LEADER ? "fhss leader up\r\n" : "fhss follower up, searching\r\n");

    uint32_t next_msg = HAL_GetTick() + 1000;
    uint32_t count = 0;
    int was_locked = -1;
    /* USER CODE END 2 */

    while (1) {
        /* USER CODE BEGIN 3 */
        fhss_link_poll(&link);

        int locked = fhss_link_locked(&link);
        if (locked != was_locked) {
            uart_print(locked ? "locked to hop sequence\r\n" : "searching for leader\r\n");
            was_locked = locked;
        }

        if ((int32_t)(HAL_GetTick() - next_msg) >= 0) {
            next_msg += 1000;
            char msg[FHSS_LINK_MAX_PAYLOAD + 1];
            int n = snprintf(msg, sizeof(msg), "%c%lu", NODE_IS_LEADER ? 'L' : 'F',
                             (unsigned long)count++);
            if (locked) {
                fhss_link_send(&link, (const uint8_t *)msg, (uint8_t)n);
            }
        }
        /* USER CODE END 3 */
    }
}
