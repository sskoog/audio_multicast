#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_audio_broadcast.hpp"
#include "tone_generator.hpp"
#include "lcd_display.hpp"

namespace Diagnostics {

// CPU Frequency Management Helpers
float getCPUfreq_MHz();
esp_err_t setCPUfreq_MHz(int mhz);

class DiagnosticMonitor {
public:
    DiagnosticMonitor(Bluetooth::BleAudioBroadcast& ble_broadcast, 
                      Audio::ToneGenerator* tone_gen = nullptr, 
                      Hardware::LcdDisplay* lcd_display = nullptr);
    ~DiagnosticMonitor();

    esp_err_t start();
    void stop();

private:
    static void taskRoutine(void* pvParameters);
    void printDiagnostics();
    void updateCpuLoadHistory();

    static constexpr size_t CPU_HISTORY_SIZE = 4; // 2.0s rolling window (4 samples @ 500 ms)
    int m_cpu_history[CPU_HISTORY_SIZE] = {0};
    size_t m_cpu_hist_idx = 0;
    size_t m_cpu_hist_count = 0;
    int m_cpu_mean_pct = 0;
    int m_cpu_peak_pct = 0;
    TickType_t m_last_cpu_calc_tick = 0;

    Bluetooth::BleAudioBroadcast& m_ble_broadcast;
    Audio::ToneGenerator* m_tone_gen = nullptr;
    Hardware::LcdDisplay* m_lcd_display = nullptr;

    TaskHandle_t m_task_handle = nullptr;
    bool m_is_running = false;
    uint32_t m_loop_count = 0;
};

} // namespace Diagnostics
