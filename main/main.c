#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

void app_main(void)
{
    const uart_port_t uart_num = UART_NUM_0;

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 0, NULL, 0));

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    while (1)
    {
        // Read data from UART.
        uint8_t data[128];
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t *)&length));
        length = uart_read_bytes(uart_num, data, length, pdMS_TO_TICKS(1000));
        if (length > 0)
        {
            // Echo back the received data
            uart_write_bytes(uart_num, (const char *)data, length);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}