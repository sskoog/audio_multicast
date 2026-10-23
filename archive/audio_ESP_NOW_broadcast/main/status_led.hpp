#pragma once

#include <cstdint>
#include <atomic>
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

// Default brightness: 25% (64/255) for RGB
// Default max duty cycle for pulsing discrete LED (S3): 70% (179/255)
inline constexpr uint8_t DEFAULT_LED_BRIGHTNESS = 64; 
inline constexpr uint8_t DEFAULT_PULSE_MAX_BRIGHTNESS = 179; // 70% max duty cycle
inline constexpr uint8_t DEFAULT_LED_DUTY_CYCLE = 48; 

// =====================================================================
//                   PATTERN MODES & CONSTANTS
// =====================================================================
enum class LedPatternMode {
    OFF,
    SOLID,
    BLINK,
    PULSE,
    FLASH
};

struct BlinkConfig {
    uint8_t duty_cycle; // 0 to 255 (0 = OFF, 255 = SOLID ON, 48 = ~19% pulse, 179 = ~70% max pulse)
    float   blink_freq; // Frequency in Hz (0.05 Hz to 10.0 Hz)
    LedPatternMode mode = LedPatternMode::BLINK;

    constexpr BlinkConfig(uint8_t duty = DEFAULT_LED_DUTY_CYCLE, float freq = 1.0f, LedPatternMode m = LedPatternMode::BLINK)
        : duty_cycle(duty), blink_freq(freq), mode(m) {}
};

inline constexpr BlinkConfig BLINK_SLOW{DEFAULT_LED_DUTY_CYCLE, 1.0f, LedPatternMode::BLINK}; // 1.0 Hz
inline constexpr BlinkConfig BLINK_FAST{DEFAULT_LED_DUTY_CYCLE, 2.5f, LedPatternMode::BLINK}; // 2.5 Hz fast pulse
inline constexpr BlinkConfig BLINK_SOLID{255, 1.0f, LedPatternMode::SOLID};
inline constexpr BlinkConfig BLINK_OFF{0, 1.0f, LedPatternMode::OFF};

// Quadratic Pulse Modes (fades 0 -> max duty (70%) -> 0 smoothly)
inline constexpr BlinkConfig PULSE_SLOW{DEFAULT_PULSE_MAX_BRIGHTNESS, 0.2f, LedPatternMode::PULSE};   // 0.2 Hz (5.0s period) - IDLE
inline constexpr BlinkConfig PULSE_MEDIUM{DEFAULT_PULSE_MAX_BRIGHTNESS, 0.5f, LedPatternMode::PULSE}; // 0.5 Hz (2.0s period) - PC Stream
inline constexpr BlinkConfig PULSE_FAST{DEFAULT_PULSE_MAX_BRIGHTNESS, 1.0f,  LedPatternMode::PULSE};  // 1.0 Hz (1.0s period) - Test Tone
inline constexpr BlinkConfig FLASH_ERROR{255, 5.0f, LedPatternMode::FLASH};                           // 100% full bright for 200ms

// =====================================================================
//                       SYSTEM STATES
// =====================================================================
enum class SystemState {
    IDLE,                 // PULSE_SLOW (0.2 Hz), Green @ 70% max
    SCANNING,             // PULSE_SLOW (0.25 Hz), Blue @ 70% max
    BROADCASTING_TONE,    // PULSE_FAST (1.0 Hz), Blue @ 70% max
    BROADCASTING_STREAM,  // PULSE_MEDIUM (0.5 Hz), Teal @ 70% max
    BROADCASTING,         // Alias for BROADCASTING_TONE
    STREAMING,            // BLINK_FAST (2.5 Hz), Teal
    BT_SYNC               // BLINK_SLOW (1.0 Hz), Teal
};

// =====================================================================
//                       STATUS LED CONTROLLER
// =====================================================================
class StatusLed {
public:
    StatusLed();
    ~StatusLed();

    // Initializes WS2812B RMT (if num_leds >= 1) or LEDC Hardware PWM on discrete LED (if num_leds == 0)
    esp_err_t init(int gpio_num = 8, int num_leds = 1, bool active_low = true);

    // Sets color with brightness scaling (0 - 255)
    void setColor(RgbColor color, uint8_t brightness = DEFAULT_LED_BRIGHTNESS);
    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = DEFAULT_LED_BRIGHTNESS);

    // Sets blinking parameters
    void setBlink(BlinkConfig blink);
    void setBlink(uint8_t duty_cycle, float blink_freq_hz);
    void setPulse(float pulse_freq_hz = 0.2f, uint8_t max_brightness = DEFAULT_PULSE_MAX_BRIGHTNESS);

    // Configures combined pattern
    void setPattern(RgbColor color, uint8_t brightness, BlinkConfig blink);

    // High-level system state setter
    void setSystemState(SystemState state);

    // Turn LED completely off
    void off();

    // Instantaneous, non-blocking trigger for 100% full-bright Flash on Error/Underrun (200ms)
    void triggerUnderrunFlash(uint32_t duration_ms = 200);
    void triggerUnderrunFlashFromISR(uint32_t duration_ms = 200);
    void triggerErrorFlash(uint32_t duration_ms = 200) { triggerUnderrunFlash(duration_ms); }

private:
    static void ledTaskRoutine(void* pvParameters);
    void updateHardwareLed(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

    int m_gpio_num = 8;
    int m_num_leds = 1;
    bool m_is_pwm_led = false;
    bool m_active_low = true;
    led_strip_handle_t m_strip_handle = nullptr;
    TaskHandle_t m_task_handle = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;

    bool m_running = false;
    uint8_t m_r = 0;
    uint8_t m_g = 0;
    uint8_t m_b = 255;
    uint8_t m_brightness = DEFAULT_PULSE_MAX_BRIGHTNESS;
    uint8_t m_duty_cycle = DEFAULT_LED_DUTY_CYCLE;
    float   m_blink_freq = 0.2f;
    LedPatternMode m_mode = LedPatternMode::PULSE;
    SystemState m_current_state = SystemState::IDLE;
    std::atomic<TickType_t> m_flash_until_tick{0};
};

StatusLed& getStatusLed();

} // namespace Hardware
