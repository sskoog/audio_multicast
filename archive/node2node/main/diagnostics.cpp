#include <cmath>
#include "wifi_manager.hpp"
#include "web_dashboard.hpp"
#include "diagnostics.hpp"
#include "status_led.hpp"
#include "config.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_clk_tree.h"
#include "esp_pm.h"
#include "driver/temperature_sensor.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "HBEAT";

namespace Diagnostics {

float getCPUfreq_MHz() {
    uint32_t freq_hz = 0;
    if (esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX, &freq_hz) == ESP_OK) {
        return static_cast<float>(freq_hz) / 1000000.0f;
    }
    return 160.0f;
}

esp_err_t setCPUfreq_MHz(int mhz) {
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = mhz,
        .min_freq_mhz = mhz,
        .light_sleep_enable = false
    };
    return esp_pm_configure(&pm_config);
#else
    ESP_LOGW(TAG, "PM not enabled in sdkconfig (CONFIG_PM_ENABLE=y required for dynamic frequency switching)");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

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

void DiagnosticMonitor::updateCpuLoadHistory() {
    int current_load = calculateCpuUsagePct();
    m_cpu_history[m_cpu_hist_idx] = current_load;
    m_cpu_hist_idx = (m_cpu_hist_idx + 1) % CPU_HISTORY_SIZE;
    if (m_cpu_hist_count < CPU_HISTORY_SIZE) {
        m_cpu_hist_count++;
    }

    int sum = 0;
    int peak = 0;
    for (size_t i = 0; i < m_cpu_hist_count; ++i) {
        sum += m_cpu_history[i];
        if (m_cpu_history[i] > peak) {
            peak = m_cpu_history[i];
        }
    }
    m_cpu_mean_pct = (m_cpu_hist_count > 0) ? (sum / static_cast<int>(m_cpu_hist_count)) : current_load;
    m_cpu_peak_pct = peak;
}

DiagnosticMonitor::DiagnosticMonitor(Bluetooth::BleAudioBroadcast& ble_broadcast, 
                                     Audio::ToneGenerator* tone_gen, 
                                     Hardware::LcdDisplay* lcd_display)
    : m_ble_broadcast(ble_broadcast), m_tone_gen(tone_gen), m_lcd_display(lcd_display) {}

DiagnosticMonitor::~DiagnosticMonitor() {
    stop();
}

esp_err_t DiagnosticMonitor::start() {
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    esp_err_t ret = temperature_sensor_install(&temp_cfg, &temp_handle);
    if (ret == ESP_OK) {
        temperature_sensor_enable(temp_handle);
    } else {
        ESP_LOGW(TAG, "Temperature sensor install failed: %s", esp_err_to_name(ret));
    }

    m_is_running = true;
    BaseType_t task_ret = xTaskCreate(taskRoutine, "diag_1hz_task", DIAGNOSTICS_TASK_STACK_SIZE, this, DIAGNOSTICS_TASK_PRIORITY, &m_task_handle);
    if (task_ret == pdPASS) {
        ESP_LOGI(TAG, "1 Hz Diagnostic Telemetry Task Started.");
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
    const TickType_t interval_ticks = pdMS_TO_TICKS(DIAGNOSTICS_TASK_INTERVAL_MS);

    while (instance->m_is_running) {
        instance->printDiagnostics();
        vTaskDelayUntil(&last_wake_time, interval_ticks);
    }

    vTaskDelete(NULL);
}

void DiagnosticMonitor::printDiagnostics() {
    m_loop_count++;

    int cpu_temp_c = 0;
    if (temp_handle) {
        float raw_temp = 0.0f;
        if (temperature_sensor_get_celsius(temp_handle, &raw_temp) == ESP_OK) {
            cpu_temp_c = static_cast<int>(raw_temp + 0.5f);
        }
    }

    uint32_t uptime_sec = (m_loop_count * DIAGNOSTICS_TASK_INTERVAL_MS) / 1000;
    uint32_t free_heap = esp_get_free_heap_size();
            /* Calculate CPU usage every 500 ms and update 10-element (5-sec) history ringbuffer */
    TickType_t now_ticks = xTaskGetTickCount();
    if (m_last_cpu_calc_tick == 0 || (now_ticks - m_last_cpu_calc_tick) >= pdMS_TO_TICKS(500)) {
        m_last_cpu_calc_tick = now_ticks;
        updateCpuLoadHistory();
    }

    const auto& stream = m_ble_broadcast.getStreamTelemetry();
    const char* bt_state = m_ble_broadcast.getStateString();
    const system_config_t* cfg = get_system_config();

    if ((m_loop_count % DIAGNOSTICS_REFRESH_RATE_HZ) == 0) {
        uint32_t hb_count = m_loop_count / DIAGNOSTICS_REFRESH_RATE_HZ;
        //ESP_LOGI("", "========== [%s] ==== beat #%lu ==== up %lu s ==========", cfg->device_name, hb_count, uptime_sec);
        ESP_LOGI("", "========== [%s] ==========", cfg->device_name);
        if (cfg->node_role == NODE_ROLE_SOURCE) {
            auto& wifi = Network::WifiManager::getInstance();
            ESP_LOGI("[NET]", "Wi-Fi %s (%s) | SSID '%s' | URL http://%s",
                     wifi.isConnected() ? "ONLINE" : "OFFLINE",
                     wifi.isSoftAp() ? "SoftAP" : "Station",
                     wifi.getSsid().c_str(),
                     wifi.getIpAddress().c_str());
        }
        ESP_LOGI("[SYS]", "CPU %d-%d%% @ %.0f MHz | Temp %d C | Heap %lu KB",
                 m_cpu_mean_pct, m_cpu_peak_pct, getCPUfreq_MHz(), cpu_temp_c, free_heap / 1024);
        ESP_LOGI("[BT]", "%s | %s BIS %u | %lu packets | RSSI %d dBm | ",
                 (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK",
                 bt_state, stream.bis_index, stream.packets_count, stream.rssi_dbm);

        bool is_streaming = (m_ble_broadcast.getState() == Bluetooth::BluetoothState::STREAMING);
        float rms_db = is_streaming ? m_ble_broadcast.getAudioFrameRMS_dBFS() : -INFINITY;
        float peak_db = is_streaming ? m_ble_broadcast.getAudioFramePeak_dBFS() : -INFINITY;

        char rms_str[32];
        if (std::isinf(rms_db) || rms_db <= -95.0f) {
            snprintf(rms_str, sizeof(rms_str), "-∞");
        } else {
            snprintf(rms_str, sizeof(rms_str), "%.1f", rms_db);
        }

        char peak_str[32];
        if (std::isinf(peak_db) || peak_db <= -95.0f) {
            snprintf(peak_str, sizeof(peak_str), "-∞");
        } else {
            snprintf(peak_str, sizeof(peak_str), "%.1f", peak_db);
        }

        uint32_t dma_udr = m_ble_broadcast.getAndResetDmaUnderrunCount();
        uint32_t fifo_udr = m_ble_broadcast.getAndResetFifoUnderrunCount();
        uint32_t fifo_ovr = m_ble_broadcast.getAndResetFifoOverflowCount();
        uint32_t plc_count = m_ble_broadcast.getAndResetPlcCount();
        uint32_t mbuf_udr = m_ble_broadcast.getAndResetMbufUnderrunCount();
        uint32_t mbuf_ovr = m_ble_broadcast.getAndResetMbufOverflowCount();

        if (cfg->node_role == NODE_ROLE_SINK) {
            ESP_LOGI("[AUDIO]", "RMS|Pk %s|%s dBFS | VCS %u%% | DMA_UDR %lu | FIFO_OV/UD %lu/%lu | PLC %lu | MBUF_OV/UD %lu/%lu | %s | %s",
                     rms_str, peak_str, 
                     m_ble_broadcast.getVolumePercent(),
                     (unsigned long)dma_udr,
                     (unsigned long)fifo_ovr, (unsigned long)fifo_udr,
                     (unsigned long)plc_count,
                     (unsigned long)mbuf_ovr, (unsigned long)mbuf_udr,
                     stream.getStatusString().c_str(),
                     stream.getCodecString().c_str()
                    );
        } else {
            ESP_LOGI("[AUDIO]", "RMS|Pk %s|%s dBFS | %s | %s",
                     rms_str, peak_str,
                     stream.getStatusString().c_str(),
                     stream.getCodecString().c_str()
                    );
        }

        if (cfg->node_role == NODE_ROLE_SOURCE) {
            if (m_tone_gen) {
                ESP_LOGI("[SOURCE]", "Tone %.1f Hz | VFO %.2f Hz | Gain %.0f%% (%.1f dB)",
                    m_tone_gen->getCurrentFrequency(), m_tone_gen->getModulationRate(),
                    m_tone_gen->get_gain_pct(), m_tone_gen->get_gain_dB()
                        );
            }

            /* Print Tracked SINK Nodes Table on SOURCE */
            const auto& sinks = m_ble_broadcast.getTrackedSinks();
            uint32_t now = xTaskGetTickCount();
            ESP_LOGI("[SINKS]", "=== Tracked SINK Nodes (%u / %d max) ===", (unsigned int)sinks.size(), MAX_GATT_SINK_NODES);
            for (size_t i = 0; i < sinks.size(); ++i) {
                const auto& s = sinks[i];
                uint32_t age_ms = (now - s.last_seen_tick) * portTICK_PERIOD_MS;
                ESP_LOGI("[SINK_NODE]", "  [%u] '%s' | ConnHandle %u | Vol %.1f%% | BASS %s | Age %lu ms",
                         (unsigned int)(i + 1), s.device_name.c_str(), s.conn_handle, s.volume_percent,
                         (s.pa_sync_state == 2) ? "PA_SYNCED" : (s.connected ? "CONNECTED" : "DISCONNECTED"), age_ms);
            }
        }
    }

    // Render to 1.47" ST7789 LCD Console if available (Node20)
    if (m_lcd_display && m_lcd_display->isInitialized()) {
        char buf[128];

        snprintf(buf, sizeof(buf), "NODE %s | UP %lu s", 
            (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK", uptime_sec);
        m_lcd_display->printLine(0, buf, Hardware::COLOR_WHITE);

        snprintf(buf, sizeof(buf), "CPU %2d-%2d%% | %2d C | %3lu KB", 
            m_cpu_mean_pct, m_cpu_peak_pct, cpu_temp_c, free_heap / 1024);
        m_lcd_display->printLine(1, buf, Hardware::COLOR_GREEN);
        
        if (cfg->node_role == NODE_ROLE_SOURCE) {
            snprintf(buf, sizeof(buf), "BT %s | BIS #%u", bt_state, stream.bis_index);
        } else {
            if (stream.is_synced) {
                snprintf(buf, sizeof(buf), "BT %s | %+02d dBm | %.1f kpkts", bt_state, stream.rssi_dbm, static_cast<float>(stream.packets_count) / 1000.0f);
            } else {
                snprintf(buf, sizeof(buf), "BT Scanning...");
            }
        }
        m_lcd_display->printLine(2, buf, stream.is_synced ? Hardware::COLOR_CYAN : Hardware::COLOR_BLUE);

        snprintf(buf, sizeof(buf), "BIS #%u @ %s", stream.bis_index, stream.source_name.c_str());
        m_lcd_display->printLine(3, buf, Hardware::COLOR_CYAN);

        snprintf(buf, sizeof(buf), "AUDIO %s", stream.getStatusString().c_str());
        m_lcd_display->printLine(4, buf, Hardware::COLOR_ORANGE);

        snprintf(buf, sizeof(buf), "CODEC %s", stream.getCodecString().c_str());
        m_lcd_display->printLine(5, buf, Hardware::COLOR_ORANGE);

        if (cfg->node_role == NODE_ROLE_SOURCE && m_tone_gen) {
            snprintf(buf, sizeof(buf), "VCO %.1f Hz (VFO %.2fHz)", 
            m_tone_gen->getCurrentFrequency(), m_tone_gen->getModulationRate());
            m_lcd_display->printLine(6, buf, Hardware::COLOR_ORANGE);
        } else {
            snprintf(buf, sizeof(buf), "VOL %3u%% | DAC OK", m_ble_broadcast.getVolumePercent());
            m_lcd_display->printLine(6, buf, Hardware::COLOR_YELLOW);
        }

        m_lcd_display->flush();
    }

    // Broadcast live telemetry to connected Web Dashboard clients (3 Hz)
    if (cfg->node_role == NODE_ROLE_SOURCE && Web::WebDashboard::getInstance().getActiveClientCount() > 0) {
        Web::DashboardTelemetry dash_data;
        dash_data.node_name = cfg->device_name;
        dash_data.bt_state = bt_state;
        dash_data.cpu_mean_pct = m_cpu_mean_pct;
        dash_data.cpu_peak_pct = m_cpu_peak_pct;
        dash_data.cpu_temp_c = cpu_temp_c;
        dash_data.free_heap_kb = free_heap / 1024;
        dash_data.uptime_sec = uptime_sec;
        if (m_tone_gen) {
            dash_data.vco_freq_hz = m_tone_gen->getCurrentFrequency();
            dash_data.vfo_mod_rate_hz = m_tone_gen->getModulationRate();
            dash_data.tone_gain_pct = m_tone_gen->get_gain_pct();
        }
        dash_data.packets_count = stream.packets_count;

        for (const auto& s : m_ble_broadcast.getTrackedSinks()) {
            if (!s.connected) continue;
            Web::TrackedSinkInfo sink_info;
            sink_info.name = s.device_name;
            sink_info.conn_handle = s.conn_handle;
            sink_info.volume_percent = s.volume_percent;
            sink_info.is_synced = (s.pa_sync_state == 2);
            sink_info.rssi_dbm = s.rssi;
            sink_info.age_ms = (xTaskGetTickCount() - s.last_seen_tick) * portTICK_PERIOD_MS;
            dash_data.sinks.push_back(sink_info);
        }

        Web::WebDashboard::getInstance().broadcastTelemetry(dash_data);
    }

    // Status LED
    if (strcmp(bt_state, "BROADCASTING") == 0) {
        Hardware::getStatusLed().setSystemState(Hardware::SystemState::BROADCASTING);
    } else if (strcmp(bt_state, "STREAMING") == 0) {
        Hardware::getStatusLed().setSystemState(Hardware::SystemState::STREAMING);
    } else if (strcmp(bt_state, "SCANNING") == 0) {
        Hardware::getStatusLed().setSystemState(Hardware::SystemState::IDLE);
    } else {
        Hardware::getStatusLed().off();
    }
}

} // namespace Diagnostics
