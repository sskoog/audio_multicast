#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include "esp_err.h"
#include "driver/gpio.h"

namespace Hardware {

// Display Dimensions (Landscape Mode)
constexpr int LCD_WIDTH = 320;
constexpr int LCD_HEIGHT = 172;

// Pin Configuration for Waveshare ESP32-C6-LCD-1.47
constexpr gpio_num_t LCD_PIN_MOSI = GPIO_NUM_6;
constexpr gpio_num_t LCD_PIN_SCLK = GPIO_NUM_7;
constexpr gpio_num_t LCD_PIN_CS   = GPIO_NUM_14;
constexpr gpio_num_t LCD_PIN_DC   = GPIO_NUM_15;
constexpr gpio_num_t LCD_PIN_RST  = GPIO_NUM_21;
constexpr gpio_num_t LCD_PIN_BL   = GPIO_NUM_22;
constexpr gpio_num_t RGB_LED_PIN  = GPIO_NUM_8;

// RGB565 Color Definitions (Corrected for Waveshare ST7789 SPI)
constexpr uint16_t COLOR_BLACK       = 0x0000;
constexpr uint16_t COLOR_WHITE       = 0xFFFF;
constexpr uint16_t COLOR_RED         = 0x1F00;
constexpr uint16_t COLOR_GREEN       = 0xE007;
constexpr uint16_t COLOR_BLUE        = 0x00F8;
constexpr uint16_t COLOR_CYAN        = 0xE0FF;
constexpr uint16_t COLOR_MAGENTA     = 0x7FF0;
constexpr uint16_t COLOR_YELLOW      = 0xFF07;
constexpr uint16_t COLOR_ORANGE      = 0xBF03;
constexpr uint16_t COLOR_DARK_GRAY   = 0x252A;
constexpr uint16_t COLOR_MEDIUM_GRAY = 0x4840;
constexpr uint16_t COLOR_NEON_GREEN  = 0xE007;

class LcdDisplay {
public:
    LcdDisplay();
    ~LcdDisplay();

    esp_err_t init();
    void setBacklight(bool enable);
    void setBacklightBrightness(uint8_t percent);

    void clear(uint16_t bg_color = COLOR_BLACK);
    void printLine(int row, const char* text, uint16_t color = COLOR_WHITE, uint16_t bg_color = COLOR_BLACK);
    void drawHeader(const char* title);

    // RGB LED Status Control (GPIO 8)
    void setRgbColor(uint8_t red, uint8_t green, uint8_t blue);

    // Refresh entire console screen to ST7789 LCD
    void flush();

    bool isInitialized() const { return m_initialized; }

private:
    void drawChar(int x, int y, char c, uint16_t color, uint16_t bg_color);
    void drawString(int x, int y, const char* str, uint16_t color, uint16_t bg_color);

    void* m_panel_handle = nullptr;
    void* m_io_handle = nullptr;
    void* m_led_strip_handle = nullptr;
    bool m_initialized = false;
    uint16_t* m_framebuffer = nullptr;
};

} // namespace Hardware
