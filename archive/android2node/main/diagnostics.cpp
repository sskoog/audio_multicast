#include "diagnostics.hpp"
#include "config.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "driver/temperature_sensor.h"
#include "lcd_console.hpp"
#include <cstring>

static const char* TAG = "DIAG_2HZ";

namespace Diagnostics {

static temperature_sensor_handle_t temp_handle = nullptr;

static int calculateCpuUsagePct() {
    static uint32_t last_idle_time = 0;
    static uint32_t last_total_time = 0;

    UBaseType_t task_count = uxTaskGetNumberOfTasks();
    TaskStatus_t* task_status_array = static_cast<TaskStatus_t*>(pvPortMalloc(task_count * sizeof(TaskStatus_t)));
    if (!task_status_array) return 0;

    uint32_t total_time = 0;
    uint32_t idle_time = 0;

    UBaseType_t num_tasks = uxTaskGetSystemState(task_status_array, task_count, &total_time);

    for (UBaseType_t i = 0; i < num_tasks; i++) {
        if (strcmp(task_status_array[i].pcTaskName, "IDLE") == 0 ||
            strcmp(task_status_array[i].pcTaskName, "IDLE0") == 0) {
            idle_time += task_status_array[i].ulRunTimeCounter;
        }
    }
    vPortFree(task_status_array);

    uint32_t delta_total = total_time - last_total_time;
    uint32_t delta_idle = idle_time - last_idle_time;

    last_total_time = total_time;
    last_idle_time = idle_time;

    if (delta_total == 0) return 0;

    float cpu_busy_pct = (1.0f - (static_cast<float>(delta_idle) / static_cast<float>(delta_total))) * 100.0f;
    if (cpu_busy_pct < 0.0f) cpu_busy_pct = 0.0f;
    if (cpu_busy_pct > 100.0f) cpu_busy_pct = 100.0f;

    return static_cast<int>(cpu_busy_pct + 0.5f);
}

DiagnosticMonitor::DiagnosticMonitor(Bluetooth::BleAudioReceiver& ble_receiver, Audio::DspHighPassFilter& filter, Hardware::LcdConsole* lcd_console)
    : m_ble_receiver(ble_receiver), m_filter(filter), m_lcd_console(lcd_console) {}

DiagnosticMonitor::~DiagnosticMonitor() {
    stop();
}

esp_err_t DiagnosticMonitor::start() {
    // Initialize onboard ESP32-C6 Temperature Sensor
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    esp_err_t ret = temperature_sensor_install(&temp_cfg, &temp_handle);
    if (ret == ESP_OK) {
        temperature_sensor_enable(temp_handle);
    } else {
        ESP_LOGW(TAG, "Temperature sensor install failed: %s", esp_err_to_name(ret));
    }

    // Create 2 Hz (500 ms interval) FreeRTOS telemetry task
    m_is_running = true;
    BaseType_t task_ret = xTaskCreate(taskRoutine, "diag_2hz_task", DIAGNOSTICS_TASK_STACK_SIZE, this, DIAGNOSTICS_TASK_PRIORITY, &m_task_handle);
    if (task_ret == pdPASS) {
        ESP_LOGI(TAG, "2 Hz Diagnostic Telemetry Task Started (USB Serial & ST7789 LCD Console).");
        return ESP_OK;
    }
    m_is_running = false;
    return ESP_FAIL;
}

void DiagnosticMonitor::stop() {
    if (m_is_running && m_task_handle) {
        vTaskDelete(m_task_handle);
        m_task_handle = nullptr;
        m_is_running = false;
    }
    if (temp_handle) {
        temperature_sensor_disable(temp_handle);
        temperature_sensor_uninstall(temp_handle);
        temp_handle = nullptr;
    }
}

void DiagnosticMonitor::taskRoutine(void* pvParameters) {
    auto* instance = static_cast<DiagnosticMonitor*>(pvParameters);
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval_ticks = pdMS_TO_TICKS(DIAGNOSTICS_TASK_INTERVAL_MS); // 2 Hz frequency

    while (instance->m_is_running) {
        instance->printDiagnostics();
        vTaskDelayUntil(&last_wake_time, interval_ticks);
    }

    vTaskDelete(NULL);
}

void DiagnosticMonitor::printDiagnostics() {
    m_loop_count++;

    // 1. Temperature Sensor Reading
    float cpu_temp_celsius = 0.0f;
    if (temp_handle) {
        temperature_sensor_get_celsius(temp_handle, &cpu_temp_celsius);
    }

    // 2. Memory & Heap Utilization
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free_heap = esp_get_minimum_free_heap_size();
    int cpu_usage_pct = calculateCpuUsagePct();

    // 3. Bluetooth Connection & Streaming Information
    const auto& stream = m_ble_receiver.getStreamInfo();
    const char* conn_state = m_ble_receiver.getConnectionStateStr();

    // Print formatted 2 Hz telemetry header to USB Virtual Serial Console
    ESP_LOGI(TAG, "==================== ESP32-C6 BLE AUDIO DIAGNOSTICS #%lu ====================", 
            m_loop_count);
    ESP_LOGI(TAG, " [SYSTEM]  CPU Core: RISC-V 160 MHz | Util: %d%% | Temp: %d C | Free Heap: %lu B (Min: %lu B)",
            cpu_usage_pct, static_cast<int>(cpu_temp_celsius + 0.5f), free_heap, min_free_heap);
    ESP_LOGI(TAG, " [BT LINK] Target: Google Pixel 10 | Status: %s | ASCS: %s | BASS ID: 0x%06lX | RSSI: %d dBm",
        conn_state, m_ble_receiver.getAseStateStr(), m_ble_receiver.getBroadcastId(), stream.rssi_dbm);
    ESP_LOGI(TAG, " [AUDIO]   Codec: %s | Format: %.1f kHz / %u-ch | Bitrate: %lu kbps | VCS Vol: %d%%",
             stream.codec_name.c_str(), static_cast<float>(stream.sample_rate) / 1000.0f, stream.channels, stream.bitrate_kbps, stream.volume_percent);
    ESP_LOGI(TAG, " [DSP HPF] 2nd-Order Butterworth HPF Active: Cutoff = %.1f Hz (esp-dsp Q31)", 
             m_filter.getCutoffFreq());

    // 4. Render 2 Hz Telemetry to ST7789 1.47" LCD Console in Landscape Mode
    if (m_lcd_console) {
        char buf[64];

        snprintf(buf, sizeof(buf), "TICK: #%lu | CPU: 160 MHz (%d%%)", m_loop_count, cpu_usage_pct);
        m_lcd_console->printLine(0, buf, Hardware::COLOR_WHITE);

        snprintf(buf, sizeof(buf), "TEMP: %d C | HEAP: %lu KB", static_cast<int>(cpu_temp_celsius + 0.5f), free_heap / 1024);
        m_lcd_console->printLine(1, buf, Hardware::COLOR_GREEN);

        snprintf(buf, sizeof(buf), "BT STATUS: %s (%d dBm)", conn_state, stream.rssi_dbm);
        m_lcd_console->printLine(2, buf, Hardware::COLOR_YELLOW);

        snprintf(buf, sizeof(buf), "AUDIO: %s %.1f kHz %u-ch", stream.codec_name.c_str(), static_cast<float>(stream.sample_rate) / 1000.0f, stream.channels);
        m_lcd_console->printLine(3, buf, Hardware::COLOR_CYAN);

        snprintf(buf, sizeof(buf), "BITRATE: %lu kbps | VOL: %d%%", stream.bitrate_kbps, stream.volume_percent);
        m_lcd_console->printLine(4, buf, Hardware::COLOR_CYAN);

        snprintf(buf, sizeof(buf), "DSP HPF: %.0f Hz Q31 Butterworth", m_filter.getCutoffFreq());
        m_lcd_console->printLine(5, buf, Hardware::COLOR_MAGENTA);

        snprintf(buf, sizeof(buf), "DAC: I2S MAX98357A (No Int DAC)");
        m_lcd_console->printLine(6, buf, Hardware::COLOR_WHITE);

        m_lcd_console->flush();

        // Update RGB LED Status (GPIO 8)
        if (strcmp(conn_state, "STREAMING") == 0) {
            m_lcd_console->setRgbColor(0, 48, 0); // Green when playing
        } else if (strcmp(conn_state, "SCANNING") == 0) {
            m_lcd_console->setRgbColor(0, 0, 48); // Blue when scanning
        } else {
            m_lcd_console->setRgbColor(32, 16, 0); // Orange when idle
        }
    }
}

} // namespace Diagnostics
