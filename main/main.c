#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#define CR 0x0D
#define LF 0x0A
#define wait_time_out 300 // milliseconds
#define loop_delay 5000   // milliseconds

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
        const uint8_t IDRequestMsg[] = {'/', '?', '!', CR, LF};

        uart_write_bytes(uart_num, (const char *)IDRequestMsg, sizeof(IDRequestMsg));

        // Read data from UART.
        uint8_t data[128];

        int length = uart_read_bytes(uart_num, data, sizeof(data), pdMS_TO_TICKS(wait_time_out));
        uint8_t IDResponse_expicted[10] = {'/', 'M', '0', '4', '1', '5', '2', '2', CR, LF}; // hardcoded response

        if (length > 0)
        {
            if (strncmp((const char *)data, (const char *)IDResponse_expicted, length) == 0)
            {
                // blink LED or indicate success
            }
        }

        vTaskDelay(pdMS_TO_TICKS(loop_delay));
    }
}