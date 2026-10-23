#include "status_led.hpp"
#include "esp_log.h"
#include "led_strip.h"
#include "driver/ledc.h"
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
        led_strip_set_pixel(m_strip_handle, 0, 0, 0, 0);
        led_strip_refresh(m_strip_handle);
    }
    if (m_is_pwm_led && m_gpio_num >= 0) {
        uint32_t off_duty = m_active_low ? 255 : 0;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, off_duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

esp_err_t StatusLed::init(int gpio_num, int num_leds, bool active_low) {
    if (gpio_num < 0) {
        m_gpio_num = -1;
        ESP_LOGI(TAG, "Status LED disabled (GPIO < 0)");
        return ESP_OK;
    }

    m_gpio_num = gpio_num;
    m_num_leds = num_leds;
    m_active_low = active_low;

    if (num_leds >= 1) {
        // WS2812B RMT Controller
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = m_gpio_num;
        strip_config.max_leds = static_cast<uint32_t>(num_leds);
        strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;
        strip_config.flags.invert_out = false;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz RMT resolution

        esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &m_strip_handle);
        if (ret == ESP_OK) {
            led_strip_clear(m_strip_handle);
            m_is_pwm_led = false;
            ESP_LOGI(TAG, "WS2812B RGB LED initialized on GPIO %d", m_gpio_num);
        } else {
            ESP_LOGW(TAG, "Failed to init WS2812 on GPIO %d (%s), falling back to LEDC PWM", m_gpio_num, esp_err_to_name(ret));
            m_strip_handle = nullptr;
            num_leds = 0;
        }
    }

    if (num_leds == 0) {
        // Discrete LED via LEDC Hardware PWM (5 kHz, 8-bit resolution)
        ledc_timer_config_t timer_cfg = {};
        timer_cfg.speed_mode       = LEDC_LOW_SPEED_MODE;
        timer_cfg.duty_resolution  = LEDC_TIMER_8_BIT;
        timer_cfg.timer_num        = LEDC_TIMER_0;
        timer_cfg.freq_hz          = 5000;
        timer_cfg.clk_cfg          = LEDC_AUTO_CLK;
        ledc_timer_config(&timer_cfg);

        ledc_channel_config_t chan_cfg = {};
        chan_cfg.gpio_num       = m_gpio_num;
        chan_cfg.speed_mode     = LEDC_LOW_SPEED_MODE;
        chan_cfg.channel        = LEDC_CHANNEL_0;
        chan_cfg.intr_type      = LEDC_INTR_DISABLE;
        chan_cfg.timer_sel      = LEDC_TIMER_0;
        chan_cfg.duty           = m_active_low ? 255 : 0; // initially off
        chan_cfg.hpoint         = 0;
        ledc_channel_config(&chan_cfg);

        m_is_pwm_led = true;
        ESP_LOGI(TAG, "Discrete LED (LEDC PWM) initialized on GPIO %d (Active %s, 5 kHz, 30%% max pulse)",
                 m_gpio_num, m_active_low ? "LOW" : "HIGH");
    }

    // Default initial state: IDLE (PULSE_SLOW @ 0.2 Hz)
    setSystemState(SystemState::IDLE);

    if (!m_running) {
        m_running = true;
#if SOC_CPU_CORES_NUM > 1
        BaseType_t task_ret = xTaskCreatePinnedToCore(ledTaskRoutine, "status_led_task", 4096, this, 4, &m_task_handle, 1);
#else
        BaseType_t task_ret = xTaskCreate(ledTaskRoutine, "status_led_task", 4096, this, 4, &m_task_handle);
#endif
        if (task_ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create status_led_task");
            return ESP_FAIL;
        }
    }

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
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_duty_cycle = blink.duty_cycle;
        m_blink_freq = (blink.blink_freq < 0.05f) ? 0.05f : (blink.blink_freq > 10.0f ? 10.0f : blink.blink_freq);
        m_mode = blink.mode;
        xSemaphoreGive(m_mutex);
    }
}

void StatusLed::setBlink(uint8_t duty_cycle, float blink_freq_hz) {
    if (blink_freq_hz < 0.05f) blink_freq_hz = 0.05f;
    if (blink_freq_hz > 10.0f) blink_freq_hz = 10.0f;

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_duty_cycle = duty_cycle;
        m_blink_freq = blink_freq_hz;
        m_mode = LedPatternMode::BLINK;
        xSemaphoreGive(m_mutex);
    }
}

void StatusLed::setPulse(float pulse_freq_hz, uint8_t max_brightness) {
    if (pulse_freq_hz < 0.05f) pulse_freq_hz = 0.05f;
    if (pulse_freq_hz > 10.0f) pulse_freq_hz = 10.0f;

    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_blink_freq = pulse_freq_hz;
        m_brightness = max_brightness;
        m_mode = LedPatternMode::PULSE;
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
        m_blink_freq = (blink.blink_freq < 0.05f) ? 0.05f : (blink.blink_freq > 10.0f ? 10.0f : blink.blink_freq);
        m_mode = blink.mode;
        xSemaphoreGive(m_mutex);
    }
}

void StatusLed::setSystemState(SystemState state) {
    m_current_state = state;
    switch (state) {
        case SystemState::IDLE:
            // PULSE_SLOW (0.2 Hz), 30% max duty
            setPattern(LED_COLOR_GREEN, DEFAULT_PULSE_MAX_BRIGHTNESS, PULSE_SLOW);
            break;
        case SystemState::SCANNING:
            setPattern(LED_COLOR_BLUE, DEFAULT_PULSE_MAX_BRIGHTNESS, PULSE_SLOW);
            break;
        case SystemState::BROADCASTING:
        case SystemState::BROADCASTING_TONE:
            // PULSE_FAST (1.0 Hz) for internal test tone, 30% max duty
            setPattern(LED_COLOR_BLUE, DEFAULT_PULSE_MAX_BRIGHTNESS, PULSE_FAST);
            break;
        case SystemState::BROADCASTING_STREAM:
            // PULSE_MEDIUM (0.5 Hz) for PC stream / USB, 30% max duty
            setPattern(LED_COLOR_TEAL, DEFAULT_PULSE_MAX_BRIGHTNESS, PULSE_MEDIUM);
            break;
        case SystemState::STREAMING:
            setPattern(LED_COLOR_TEAL, DEFAULT_LED_BRIGHTNESS, BLINK_FAST);
            break;
        case SystemState::BT_SYNC:
            setPattern(LED_COLOR_TEAL, DEFAULT_LED_BRIGHTNESS, BLINK_SLOW);
            break;
    }
}

void StatusLed::off() {
    setBlink(BLINK_OFF);
}

void StatusLed::triggerUnderrunFlash(uint32_t duration_ms) {
    TickType_t now = xTaskGetTickCount();
    m_flash_until_tick.store(now + pdMS_TO_TICKS(duration_ms), std::memory_order_release);
}

void StatusLed::triggerUnderrunFlashFromISR(uint32_t duration_ms) {
    TickType_t now = xTaskGetTickCountFromISR();
    m_flash_until_tick.store(now + pdMS_TO_TICKS(duration_ms), std::memory_order_release);
}

void StatusLed::updateHardwareLed(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    if (m_strip_handle) {
        if (brightness > 0) {
            uint32_t scaled_r = (static_cast<uint32_t>(r) * brightness + 127) / 255;
            uint32_t scaled_g = (static_cast<uint32_t>(g) * brightness + 127) / 255;
            uint32_t scaled_b = (static_cast<uint32_t>(b) * brightness + 127) / 255;
            led_strip_set_pixel(m_strip_handle, 0, scaled_g, scaled_r, scaled_b);
            led_strip_refresh(m_strip_handle);
        } else {
            led_strip_clear(m_strip_handle);
        }
    } else if (m_is_pwm_led && m_gpio_num >= 0) {
        // Discrete LED via LEDC Hardware PWM
        // Active-LOW: duty 0 -> output HIGH (OFF); duty 255 -> output LOW (100% full bright)
        uint32_t duty_hw = m_active_low ? (255 - brightness) : brightness;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_hw);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void StatusLed::ledTaskRoutine(void* pvParameters) {
    auto* instance = static_cast<StatusLed*>(pvParameters);
    constexpr uint32_t UPDATE_INTERVAL_MS = 25; // 40 Hz smooth update cadence

    while (instance->m_running) {
        TickType_t flash_until = instance->m_flash_until_tick.load(std::memory_order_acquire);
        TickType_t now = xTaskGetTickCount();
        if (flash_until > now) {
            // Flash 100% full-bright for Error/Underrun event (Red on WS2812, 100% ON on discrete LED)
            instance->updateHardwareLed(255, 0, 0, 255);
            TickType_t remaining = flash_until - now;
            if (remaining > pdMS_TO_TICKS(200)) remaining = pdMS_TO_TICKS(200);
            vTaskDelay(remaining);
            continue;
        }

        uint8_t r = 0, g = 0, b = 0, max_brightness = 0, duty = 0;
        float freq = 0.2f;
        LedPatternMode mode = LedPatternMode::PULSE;

        if (xSemaphoreTake(instance->m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            r = instance->m_r;
            g = instance->m_g;
            b = instance->m_b;
            max_brightness = instance->m_brightness;
            duty = instance->m_duty_cycle;
            freq = instance->m_blink_freq;
            mode = instance->m_mode;
            xSemaphoreGive(instance->m_mutex);
        }

        if (mode == LedPatternMode::OFF || max_brightness == 0) {
            instance->updateHardwareLed(0, 0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (mode == LedPatternMode::SOLID) {
            instance->updateHardwareLed(r, g, b, max_brightness);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (mode == LedPatternMode::FLASH) {
            instance->updateHardwareLed(255, 0, 0, 255);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (mode == LedPatternMode::PULSE) {
            if (freq < 0.05f) freq = 0.05f;
            float period_ms = 1000.0f / freq;
            uint32_t now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            float t = fmodf(static_cast<float>(now_ms), period_ms) / period_ms; // 0.0 .. 1.0
            
            // Symmetric triangle wave from 0 to 1 and back to 0
            float l1 = (t < 0.5f) ? (2.0f * t) : (2.0f - 2.0f * t);
            // Quadratic easing: q1 = l1 * l1
            float q1 = l1 * l1;
            
            uint8_t current_brightness = static_cast<uint8_t>(max_brightness * q1 + 0.5f);
            instance->updateHardwareLed(r, g, b, current_brightness);
            vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
            continue;
        }

        // BLINK mode
        if (mode == LedPatternMode::BLINK) {
            if (duty == 0) {
                instance->updateHardwareLed(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (duty >= 255) {
                instance->updateHardwareLed(r, g, b, max_brightness);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            float period_ms = 1000.0f / freq;
            if (period_ms < 50.0f) period_ms = 50.0f;
            uint32_t now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            float t = fmodf(static_cast<float>(now_ms), period_ms) / period_ms;
            float on_fraction = static_cast<float>(duty) / 255.0f;

            if (t < on_fraction) {
                instance->updateHardwareLed(r, g, b, max_brightness);
            } else {
                instance->updateHardwareLed(0, 0, 0, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
            continue;
        }
    }

    vTaskDelete(NULL);
}

StatusLed& getStatusLed() {
    static StatusLed s_instance;
    return s_instance;
}

} // namespace Hardware
