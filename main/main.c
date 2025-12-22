#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "led_strip.h"

#define CR 0x0D
#define LF 0x0A
#define wait_time_out 300 // milliseconds
#define loop_delay 5000   // milliseconds

#define RGB_LED_GPIO 48 // change if your board is different
#define LED_STRIP_LED_NUM 1
led_strip_handle_t led_strip;
void led_setup(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = LED_STRIP_LED_NUM,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

void led_blink(void)
{
    // WHITE
    led_strip_set_pixel(led_strip, 0, 255, 255, 255);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(100));

    // OFF
    led_strip_clear(led_strip);
    // vTaskDelay(pdMS_TO_TICKS(500));
}
void app_main(void)
{
    const uart_port_t uart_num = UART_NUM_0;
    led_setup();
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 0, NULL, 0));

    uart_config_t uart_config = {
        .baud_rate = 2400,
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
                led_blink();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(loop_delay));
    }
}