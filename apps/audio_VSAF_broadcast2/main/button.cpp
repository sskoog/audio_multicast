#include "button.hpp"
#include "esp_log.h"

static const char* TAG = "BUTTON";

namespace Hardware {

Button::Button(int gpio_num, bool active_low, uint32_t debounce_ms)
    : m_gpio_num(gpio_num), m_active_low(active_low), m_debounce_ms(debounce_ms) {}

Button::~Button() {
    m_running = false;
    if (m_task_handle) {
        vTaskDelete(m_task_handle);
        m_task_handle = nullptr;
    }
}

esp_err_t Button::init(button_callback_t callback, void* user_data) {
    if (m_gpio_num < 0) {
        return ESP_OK; // Disabled
    }

    m_callback = callback;
    m_user_data = user_data;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << m_gpio_num);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = m_active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d for user button: %s", m_gpio_num, esp_err_to_name(ret));
        return ret;
    }

    m_running = true;
    BaseType_t task_ret = xTaskCreate(buttonTaskRoutine, "btn_task", 3072, this, 3, &m_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task for GPIO %d", m_gpio_num);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "User Button Initialized on GPIO %d (Active %s, Debounce %lu ms)",
             m_gpio_num, m_active_low ? "LOW" : "HIGH", (unsigned long)m_debounce_ms);
    return ESP_OK;
}

void Button::setCallback(button_callback_t callback, void* user_data) {
    m_callback = callback;
    m_user_data = user_data;
}

bool Button::isPressed() const {
    if (m_gpio_num < 0) return false;
    int level = gpio_get_level(static_cast<gpio_num_t>(m_gpio_num));
    return m_active_low ? (level == 0) : (level == 1);
}

void Button::buttonTaskRoutine(void* pvParameters) {
    auto* self = static_cast<Button*>(pvParameters);

    bool debounced_pressed = false;
    TickType_t last_transition_time = xTaskGetTickCount();

    while (self->m_running) {
        vTaskDelay(pdMS_TO_TICKS(15)); // 15 ms sampling interval

        int current_raw_level = gpio_get_level(static_cast<gpio_num_t>(self->m_gpio_num));
        bool is_raw_pressed = self->m_active_low ? (current_raw_level == 0) : (current_raw_level == 1);

        TickType_t now = xTaskGetTickCount();

        if (is_raw_pressed != debounced_pressed) {
            // Check if debounce time has elapsed since the level change
            if (pdTICKS_TO_MS(now - last_transition_time) >= self->m_debounce_ms) {
                debounced_pressed = is_raw_pressed;
                last_transition_time = now;

                if (debounced_pressed) {
                    ESP_LOGI(TAG, ">>> Button on GPIO %d PRESSED (Debounced %lu ms) <<<",
                             self->m_gpio_num, (unsigned long)self->m_debounce_ms);
                    if (self->m_callback) {
                        self->m_callback(self->m_user_data);
                    }
                } else {
                    ESP_LOGD(TAG, "Button on GPIO %d RELEASED", self->m_gpio_num);
                }
            }
        } else {
            last_transition_time = now;
        }
    }

    vTaskDelete(nullptr);
}

} // namespace Hardware
