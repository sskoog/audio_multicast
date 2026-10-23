#include "status_led.hpp"
#include "esp_log.h"
#include "led_strip.h"
#include <cmath>

static const char* TAG = "LED";

namespace Hardware {

StatusLed::StatusLed() {
    m_mutex = xSemaphoreCreateMutex();
}

StatusLed::~StatusLed() {
    m_running = false;
    if (m_task_handle) {
        vTaskDelete(m_task_handle);
        m_task_handle = nullptr;
    }
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
    if (m_strip_handle) {
        led_strip_clear(m_strip_handle);
    }
}

esp_err_t StatusLed::init(int gpio_num) {
    if (m_strip_handle != nullptr) {
        return ESP_OK; // Already initialized
    }

    m_gpio_num = gpio_num;

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = m_gpio_num;
    strip_config.max_leds = 1;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz RMT resolution

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &m_strip_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WS2812 RMT on GPIO %d: %s", m_gpio_num, esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(m_strip_handle);

    // Initial default: IDLE state (BLINK_SLOW, GREEN @ 32)
    setSystemState(SystemState::IDLE);

    // Start background non-blocking blinking task
    m_running = true;
    BaseType_t task_ret = xTaskCreate(ledTaskRoutine, "status_led_task", 2048, this, 1, &m_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status_led_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WS2812B Status LED Controller Initialized on GPIO %d.", m_gpio_num);
    return ESP_OK;
}

void StatusLed::setColor(RgbColor color, uint8_t brightness) {
    setColor(color.r, color.g, color.b, brightness);
}

void StatusLed::setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_r = r;
        m_g = g;
        m_b = b;
        m_brightness = brightness;
        xSemaphoreGive(m_mutex);
    }
}

void StatusLed::setBlink(BlinkConfig blink) {
    setBlink(blink.duty_cycle, blink.blink_freq);
}

void StatusLed::setBlink(uint8_t duty_cycle, float blink_freq_hz) {
    if (blink_freq_hz < 0.1f) blink_freq_hz = 0.1f;
    if (blink_freq_hz > 10.0f) blink_freq_hz = 10.0f;

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_duty_cycle = duty_cycle;
        m_blink_freq = blink_freq_hz;
        xSemaphoreGive(m_mutex);
    }
}

void StatusLed::setPattern(RgbColor color, uint8_t brightness, BlinkConfig blink) {
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_r = color.r;
        m_g = color.g;
        m_b = color.b;
        m_brightness = brightness;
        m_duty_cycle = blink.duty_cycle;
        m_blink_freq = (blink.blink_freq < 0.1f) ? 0.1f : (blink.blink_freq > 10.0f ? 10.0f : blink.blink_freq);
        xSemaphoreGive(m_mutex);
    }
}

void StatusLed::setSystemState(SystemState state) {
    m_current_state = state;
    switch (state) {
        case SystemState::IDLE:
            // IDLE: BLINK_SLOW, color(COLOR_GREEN, 32)
            setPattern(LED_COLOR_GREEN, 32, BLINK_SLOW);
            break;
        case SystemState::BROADCASTING:
            // BROADCASTING: BLINK_FAST, color(COLOR_BLUE, 32)
            setPattern(LED_COLOR_BLUE, 32, BLINK_FAST);
            break;
        case SystemState::STREAMING:
            // STREAMING: BLINK_FAST, color(COLOR_TEAL, 32)
            setPattern(LED_COLOR_TEAL, 32, BLINK_FAST);
            break;
        case SystemState::BT_SYNC:
            // BT_SYNC: BLINK_SLOW, color(COLOR_TEAL, 32)
            setPattern(LED_COLOR_TEAL, 32, BLINK_SLOW);
            break;
    }
}

void StatusLed::off() {
    setBlink(BLINK_OFF);
}

void StatusLed::triggerUnderrunFlash(uint32_t duration_ms) {
    TickType_t now = xTaskGetTickCount();
    m_flash_red_until_tick.store(now + pdMS_TO_TICKS(duration_ms), std::memory_order_release);
    if (m_task_handle) {
        xTaskNotifyGive(m_task_handle);
    }
}

void StatusLed::triggerUnderrunFlashFromISR(uint32_t duration_ms) {
    TickType_t now = xTaskGetTickCountFromISR();
    m_flash_red_until_tick.store(now + pdMS_TO_TICKS(duration_ms), std::memory_order_release);
    if (m_task_handle) {
        vTaskNotifyGiveFromISR(m_task_handle, nullptr);
    }
}

void StatusLed::updateHardwareLed(bool is_on) {
    if (!m_strip_handle) return;

    if (is_on && m_brightness > 0) {
        uint32_t scaled_r = (static_cast<uint32_t>(m_r) * m_brightness + 127) / 255;
        uint32_t scaled_g = (static_cast<uint32_t>(m_g) * m_brightness + 127) / 255;
        uint32_t scaled_b = (static_cast<uint32_t>(m_b) * m_brightness + 127) / 255;
        // Hardware die order compensation (ESP32-C6 WS2812 maps first byte to Green, second to Red)
        led_strip_set_pixel(m_strip_handle, 0, scaled_g, scaled_r, scaled_b);
    } else {
        led_strip_set_pixel(m_strip_handle, 0, 0, 0, 0);
    }
    led_strip_refresh(m_strip_handle);
}

void StatusLed::ledTaskRoutine(void* pvParameters) {
    auto* instance = static_cast<StatusLed*>(pvParameters);

    while (instance->m_running) {
        TickType_t flash_until = instance->m_flash_red_until_tick.load(std::memory_order_acquire);
        TickType_t now = xTaskGetTickCount();
        if (flash_until > now) {
            // Flash bright RED for DMA underrun
            if (instance->m_strip_handle) {
                // WS2812 GRB: G=0, R=48, B=0
                led_strip_set_pixel(instance->m_strip_handle, 0, 0, 48, 0);
                led_strip_refresh(instance->m_strip_handle);
            }
            TickType_t remaining = flash_until - now;
            if (remaining > pdMS_TO_TICKS(200)) remaining = pdMS_TO_TICKS(200);
            vTaskDelay(remaining);
            continue;
        }

        uint8_t duty = instance->m_duty_cycle;
        float freq = instance->m_blink_freq;

        if (duty == 0) {
            instance->updateHardwareLed(false);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
            continue;
        }

        if (duty >= 255) {
            instance->updateHardwareLed(true);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
            continue;
        }

        // Calculate ON and OFF durations based on frequency and duty cycle (0 - 255)
        float period_ms = 1000.0f / freq;
        if (period_ms < 50.0f) period_ms = 50.0f;
        if (period_ms > 10000.0f) period_ms = 10000.0f;

        float on_fraction = static_cast<float>(duty) / 255.0f;
        uint32_t on_ms = static_cast<uint32_t>(period_ms * on_fraction + 0.5f);
        if (on_ms < 10) on_ms = 10;
        if (on_ms > static_cast<uint32_t>(period_ms - 10)) on_ms = static_cast<uint32_t>(period_ms - 10);
        uint32_t off_ms = static_cast<uint32_t>(period_ms) - on_ms;

        // Turn ON
        instance->updateHardwareLed(true);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(on_ms)) != 0) {
            continue; // Wake up immediately if underrun flash triggered
        }

        // Turn OFF
        instance->updateHardwareLed(false);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(off_ms)) != 0) {
            continue; // Wake up immediately if underrun flash triggered
        }
    }

    vTaskDelete(NULL);
}

StatusLed& getStatusLed() {
    static StatusLed s_instance;
    return s_instance;
}

} // namespace Hardware
