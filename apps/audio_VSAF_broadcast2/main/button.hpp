#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace Hardware {

typedef void (*button_callback_t)(void* user_data);

class Button {
public:
    Button(int gpio_num = 9, bool active_low = true, uint32_t debounce_ms = 150);
    ~Button();

    esp_err_t init(button_callback_t callback = nullptr, void* user_data = nullptr);
    void setCallback(button_callback_t callback, void* user_data = nullptr);
    bool isPressed() const;

private:
    static void buttonTaskRoutine(void* pvParameters);

    int m_gpio_num;
    bool m_active_low;
    uint32_t m_debounce_ms;
    button_callback_t m_callback = nullptr;
    void* m_user_data = nullptr;
    TaskHandle_t m_task_handle = nullptr;
    bool m_running = false;
};

} // namespace Hardware
