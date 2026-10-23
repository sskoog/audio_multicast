/*
 * Status LED Driver for ESP32-C6 DevKit (WS2812 RGB LED on GPIO 8)
 *
 * Operational Modes:
 * - IDLE: 0.5 Hz Green Blink, Duty Cycle 32/255
 * - TRANSMITTING: 3.0 Hz Blue Blink, Duty Cycle 32/255
 * - ADVERTISING: 1.0 Hz Cyan Blink, Duty Cycle 32/255
 * - RESET: 2.0 Hz Purple Blink, Duty Cycle 32/255
 * - ERROR: 5.0 Hz Red Strobe, Duty Cycle 64/255
 */

#include "status_led.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define DEFAULT_LED_GPIO        8
#define LED_BRIGHTNESS          32  // Scaled brightness (0 - 255)

static led_strip_handle_t s_strip_handle = NULL;
static TaskHandle_t s_led_task_handle = NULL;

static volatile status_led_state_t s_current_state = STATUS_LED_IDLE;
static volatile int64_t s_last_activity_us = 0;
static volatile int64_t s_last_error_us = 0;
static bool s_running = false;

static void update_hardware_led(bool is_on, uint8_t target_r, uint8_t target_g, uint8_t target_b, uint8_t brightness)
{
    if (!s_strip_handle) return;

    if (is_on) {
        uint32_t scaled_r = ((uint32_t)target_r * brightness + 127) / 255;
        uint32_t scaled_g = ((uint32_t)target_g * brightness + 127) / 255;
        uint32_t scaled_b = ((uint32_t)target_b * brightness + 127) / 255;

        // Hardware die order compensation (ESP32-C6-DevKit WS2812 maps first byte to Red, second to Green)
        led_strip_set_pixel(s_strip_handle, 0, scaled_g, scaled_r, scaled_b);
    } else {
        led_strip_set_pixel(s_strip_handle, 0, 0, 0, 0);
    }
    led_strip_refresh(s_strip_handle);
}

static void status_led_task(void *pvParameters)
{
    while (s_running) {
        int64_t now_us = esp_timer_get_time();
        bool is_error = (s_last_error_us > 0) && ((now_us - s_last_error_us) < 1000000);        // 1.0s error window
        bool is_active = (s_last_activity_us > 0) && ((now_us - s_last_activity_us) < 600000); // 600ms activity window

        status_led_state_t state;
        if (is_error) {
            state = STATUS_LED_ERROR;
        } else if (is_active) {
            state = STATUS_LED_TRANSMITTING;
        } else {
            state = s_current_state;
        }

        uint8_t r = 0, g = 0, b = 0;
        uint8_t brightness = LED_BRIGHTNESS;
        uint32_t on_ms = 0;
        uint32_t off_ms = 0;

        switch (state) {
            case STATUS_LED_ERROR:
                // 5.0 Hz Red, Period = 200 ms, Duty = 64/255 (~50 ms ON, 150 ms OFF)
                r = 255; g = 0; b = 0;
                brightness = 64;
                on_ms = 50;
                off_ms = 150;
                break;

            case STATUS_LED_TRANSMITTING:
                // 3.0 Hz Blue, Period = 333 ms, Duty = 32/255 (~42 ms ON, 291 ms OFF)
                r = 0; g = 0; b = 255;
                on_ms = 42;
                off_ms = 291;
                break;

            case STATUS_LED_ADVERTISING:
                // 1.0 Hz Cyan/Teal, Period = 1000 ms, Duty = 16/255 (~63 ms ON, 937 ms OFF)
                r = 0; g = 200; b = 255;
                on_ms = 63;
                off_ms = 937;
                break;

            case STATUS_LED_RESET:
                // 2.0 Hz Purple, Period = 500 ms, Duty = 32/255 (~62 ms ON, 438 ms OFF)
                r = 220; g = 0; b = 255;
                on_ms = 62;
                off_ms = 438;
                break;

            case STATUS_LED_IDLE:
            default:
                // 0.5 Hz Green, Period = 2000 ms, Duty = 16/255 (~125 ms ON, 1875 ms OFF)
                r = 0; g = 255; b = 0;
                on_ms = 125;
                off_ms = 1875;
                break;

            case STATUS_LED_OFF:
                update_hardware_led(false, 0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
        }

        // ON phase
        update_hardware_led(true, r, g, b, brightness);
        vTaskDelay(pdMS_TO_TICKS(on_ms));

        // OFF phase (check periodically for quick responsive state transitions)
        update_hardware_led(false, 0, 0, 0, 0);
        uint32_t remaining_off = off_ms;
        while (remaining_off > 0 && s_running) {
            uint32_t step = (remaining_off > 50) ? 50 : remaining_off;
            vTaskDelay(pdMS_TO_TICKS(step));
            remaining_off -= step;

            int64_t current_us = esp_timer_get_time();
            if ((s_last_error_us > 0) && ((current_us - s_last_error_us) < 1000000)) {
                break;
            }
            if ((s_last_activity_us > 0) && ((current_us - s_last_activity_us) < 600000) && state != STATUS_LED_TRANSMITTING) {
                break;
            }
        }
    }

    if (s_strip_handle) {
        led_strip_clear(s_strip_handle);
    }
    vTaskDelete(NULL);
}

esp_err_t status_led_init(int gpio_num)
{
    if (s_strip_handle != NULL) {
        return ESP_OK;
    }

    if (gpio_num < 0) {
        gpio_num = DEFAULT_LED_GPIO;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio_num,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz RMT resolution
        .flags = {
            .with_dma = false,
        }
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    led_strip_clear(s_strip_handle);

    s_running = true;
    s_current_state = STATUS_LED_IDLE;

    xTaskCreate(
        status_led_task,
        "status_led_task",
        2048,
        NULL,
        1,
        &s_led_task_handle
    );

    return ESP_OK;
}

void status_led_set_state(status_led_state_t state)
{
    s_current_state = state;
}

void status_led_report_activity(void)
{
    s_last_activity_us = esp_timer_get_time();
}

void status_led_report_error(void)
{
    s_last_error_us = esp_timer_get_time();
}
