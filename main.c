/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 32                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                         /**< UART RX buffer size. */

#define GPRS_POWER_ON 6
#define GPRS_TXD 12
#define GPRS_RXD 20
#define GPRS_RESET 14
//#define GPRS_PWRKEY 15

uint8_t recvBytes[20] = {0};
uint8_t recvCnt = 0;

void uart_error_handle(app_uart_evt_t * p_event)
{
    uint8_t rxChar = 0;
    
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    } else if(p_event->evt_type == APP_UART_DATA_READY) 
    {
        app_uart_get(&rxChar);
        recvBytes[recvCnt] = rxChar;
        recvCnt++;
    }
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code;

    //bsp_board_init(BSP_INIT_LEDS);

    nrf_gpio_cfg_output(GPRS_POWER_ON);
    nrf_gpio_cfg_output(GPRS_RESET);
    nrf_delay_ms(500);
    nrf_gpio_pin_clear(GPRS_RESET);
    nrf_delay_ms(500);
    nrf_gpio_pin_set(GPRS_RESET);
    nrf_delay_ms(500);

    const app_uart_comm_params_t comm_params =
      {
          GPRS_RXD,
          GPRS_TXD,
          0,
          0,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          NRF_UART_BAUDRATE_9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(GPRS_POWER_ON);
    nrf_delay_ms(8000);


    app_uart_put('A');
    app_uart_put('T');
    app_uart_put('\r');
    app_uart_put('\n');
    recvCnt = 0;
    app_uart_put('A');
    app_uart_put('T');
    app_uart_put('I');
    app_uart_put('\r');
    app_uart_put('\n');
    recvCnt = 0;
}


/** @} */

