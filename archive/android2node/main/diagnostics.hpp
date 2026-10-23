#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_audio_receiver.hpp"
#include "dsp_filter.hpp"

namespace Hardware { class LcdConsole; }

namespace Diagnostics {

/**
 * @brief 2 Hz System Diagnostic Monitor Task.
 * 
 * Periodically every 500 ms (2 Hz frequency), prints CPU utilization, onboard chip temperature,
 * Bluetooth connection status, and audio streaming parameters to USB Serial Console and ST7789 LCD.
 */
class DiagnosticMonitor {
public:
    DiagnosticMonitor(Bluetooth::BleAudioReceiver& ble_receiver, Audio::DspHighPassFilter& filter, Hardware::LcdConsole* lcd_console = nullptr);
    ~DiagnosticMonitor();

    /**
     * @brief Start 2 Hz telemetry diagnostic task.
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t start();

    /**
     * @brief Stop diagnostic task.
     */
    void stop();

private:
    static void taskRoutine(void* pvParameters);
    void printDiagnostics();

    Bluetooth::BleAudioReceiver& m_ble_receiver;
    Audio::DspHighPassFilter& m_filter;
    Hardware::LcdConsole* m_lcd_console;
    TaskHandle_t m_task_handle{nullptr};
    bool m_is_running{false};
    uint32_t m_loop_count{0};
};

} // namespace Diagnostics
