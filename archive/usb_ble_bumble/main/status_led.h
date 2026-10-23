/*
 * Status LED Controller for ESP32-C6 DevKit (WS2812 RGB LED on GPIO 8)
 *
 * Feedback States:
 * - IDLE / Waiting for H4 commands: Blink Slow (0.5 Hz) Green, Duty Cycle 16/255
 * - ADVERTISING / PA/EA Active: Blink Medium (1.0 Hz) Cyan/Teal, Duty Cycle 16/255
 * - TRANSMITTING / Active Audio Streaming: Blink Fast (3.0 Hz) Blue, Duty Cycle 32/255
 * - RESET / Host Sync: Double-Blink (2.0 Hz) Magenta/Purple, Duty Cycle 32/255
 * - ERROR / Buffer Overflow: Fast Strobe (5.0 Hz) Red, Duty Cycle 64/255
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATUS_LED_IDLE,          // 0.5 Hz Green,  duty 16/255 (Waiting for host)
    STATUS_LED_ADVERTISING,   // 1.0 Hz Cyan,   duty 16/255 (Auracast PA/EA active)
    STATUS_LED_TRANSMITTING,  // 3.0 Hz Blue,   duty 32/255 (Active ISO audio broadcast)
    STATUS_LED_RESET,         // 2.0 Hz Purple, duty 32/255 (HCI Reset / Init)
    STATUS_LED_ERROR,         // 5.0 Hz Red,    duty 64/255 (Error / Buffer drop)
    STATUS_LED_OFF
} status_led_state_t;

/**
 * @brief Initialize WS2812B status LED on specified GPIO pin
 * @param gpio_num GPIO pin connected to WS2812 DIN (default 8 on ESP32-C6 DevKit)
 * @return ESP_OK on success
 */
esp_err_t status_led_init(int gpio_num);

/**
 * @brief Manually set the base LED state
 * @param state Target state
 */
void status_led_set_state(status_led_state_t state);

/**
 * @brief Report HCI communication activity (automatically triggers TRANSMITTING state)
 */
void status_led_report_activity(void);

/**
 * @brief Report HCI error or buffer exhaustion
 */
void status_led_report_error(void);

#ifdef __cplusplus
}
#endif
