#include <atomic>
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "led_strip.h"

namespace Hardware {

// =====================================================================
//                       COLOR DEFINITIONS
// =====================================================================
struct RgbColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    constexpr RgbColor(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0)
        : r(red), g(green), b(blue) {}
};

inline constexpr RgbColor LED_COLOR_BLUE{0, 0, 255};
inline constexpr RgbColor LED_COLOR_GREEN{0, 255, 0};
inline constexpr RgbColor LED_COLOR_RED{255, 0, 0};
inline constexpr RgbColor LED_COLOR_TEAL{0, 170, 170};
inline constexpr RgbColor LED_COLOR_PURPLE{200, 0, 255};
inline constexpr RgbColor LED_COLOR_ORANGE{255, 100, 0};
inline constexpr RgbColor LED_COLOR_YELLOW{200, 200, 0};

// Helper function to create scaled color
inline RgbColor color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 255) {
    uint8_t scaled_r = static_cast<uint8_t>((static_cast<uint32_t>(r) * brightness + 127) / 255);
    uint8_t scaled_g = static_cast<uint8_t>((static_cast<uint32_t>(g) * brightness + 127) / 255);
    uint8_t scaled_b = static_cast<uint8_t>((static_cast<uint32_t>(b) * brightness + 127) / 255);
    return RgbColor(scaled_r, scaled_g, scaled_b);
}

inline RgbColor color(RgbColor c, uint8_t brightness) {
    return color(c.r, c.g, c.b, brightness);
}

// =====================================================================
//                   BLINK PATTERN CONSTANTS
// =====================================================================
struct BlinkConfig {
    uint8_t duty_cycle; // 0 to 255 (0 = OFF, 255 = SOLID ON, 32 = ~12.5% pulse)
    float   blink_freq; // 0.1 Hz to 10.0 Hz

    constexpr BlinkConfig(uint8_t duty = 32, float freq = 1.0f)
        : duty_cycle(duty), blink_freq(freq) {}
};

inline constexpr BlinkConfig BLINK_SLOW{32, 1.0f}; // Duty 32/255, 1.0 Hz
inline constexpr BlinkConfig BLINK_FAST{32, 3.0f}; // Duty 32/255, 3.0 Hz
inline constexpr BlinkConfig BLINK_SOLID{255, 1.0f};
inline constexpr BlinkConfig BLINK_OFF{0, 1.0f};

// =====================================================================
//                       SYSTEM STATES
// =====================================================================
enum class SystemState {
    IDLE,           // BLINK_SLOW, LED_COLOR_GREEN @ 32 brightness
    BROADCASTING,   // BLINK_FAST, LED_COLOR_BLUE  @ 32 brightness
    STREAMING,      // BLINK_FAST, LED_COLOR_TEAL  @ 32 brightness
    BT_SYNC         // BLINK_SLOW, LED_COLOR_TEAL  @ 32 brightness
};

// =====================================================================
//                       STATUS LED CONTROLLER
// =====================================================================
class StatusLed {
public:
    StatusLed();
    ~StatusLed();

    // Initializes WS2812B RMT peripheral on the specified GPIO (default GPIO 8)
    esp_err_t init(int gpio_num = 8);

    // Sets color with brightness scaling (0 - 255)
    void setColor(RgbColor color, uint8_t brightness = 32);
    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 32);

    // Sets blinking parameters
    void setBlink(BlinkConfig blink);
    void setBlink(uint8_t duty_cycle, float blink_freq_hz);

    // Configures combined pattern
    void setPattern(RgbColor color, uint8_t brightness, BlinkConfig blink);

    // High-level system state setter
    void setSystemState(SystemState state);

    // Turn LED completely off
    void off();

    // Instantaneous, non-blocking trigger for 0.2s Red LED flash on I2S DMA underrun
    void triggerUnderrunFlash(uint32_t duration_ms = 200);
    void triggerUnderrunFlashFromISR(uint32_t duration_ms = 200);

private:
    static void ledTaskRoutine(void* pvParameters);
    void updateHardwareLed(bool is_on);

    int m_gpio_num = 8;
    led_strip_handle_t m_strip_handle = nullptr;
    TaskHandle_t m_task_handle = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;

    bool m_running = false;
    uint8_t m_r = 0;
    uint8_t m_g = 255;
    uint8_t m_b = 0;
    uint8_t m_brightness = 32;
    uint8_t m_duty_cycle = 32;
    float   m_blink_freq = 1.0f;
    SystemState m_current_state = SystemState::IDLE;
    std::atomic<TickType_t> m_flash_red_until_tick{0};
};

// Global singleton instance for convenient cross-module access
StatusLed& getStatusLed();

} // namespace Hardware
