#include "diagnostics.hpp"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_clk_tree.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstring>

namespace Diagnostics {

SystemDiagnostics::SystemDiagnostics(AudioNet::EspNowAudioBroadcast& espnow_broadcast, Hardware::StatusLed& status_led)
    : m_espnow_broadcast(espnow_broadcast), m_status_led(status_led) {}

SystemDiagnostics::~SystemDiagnostics() {
    if (m_temp_sensor) {
        temperature_sensor_disable(m_temp_sensor);
        temperature_sensor_uninstall(m_temp_sensor);
        m_temp_sensor = nullptr;
    }
}

void SystemDiagnostics::init() {
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    esp_err_t ret = temperature_sensor_install(&temp_sensor_config, &m_temp_sensor);
    if (ret == ESP_OK) {
        temperature_sensor_enable(m_temp_sensor);
    }
}

static const char* getState5Char(AudioNet::NetworkState state) {
    switch (state) {
        case AudioNet::NetworkState::OFF:          return "OFF  ";
        case AudioNet::NetworkState::IDLE:         return "IDLE ";
        case AudioNet::NetworkState::SCANNING:     return "SCAN ";
        case AudioNet::NetworkState::PREFILL:      return "FILL ";
        case AudioNet::NetworkState::STREAMING:    return "STRM ";
        case AudioNet::NetworkState::BROADCASTING: return "BROAD";
        default:                                   return "UNKWN";
    }
}

void SystemDiagnostics::tick() {
    m_loop_count++;
    m_espnow_broadcast.update10HzTimeOffsetStats();

    const system_config_t* cfg = get_system_config();

    if ((m_loop_count % 10) == 0) { // 1 Hz periodic telemetry printout
        int64_t now_diag_us = esp_timer_get_time();
        int64_t elapsed_us = (m_last_print_time_us > 0) ? (now_diag_us - m_last_print_time_us) : 1000000;
        if (elapsed_us <= 0) elapsed_us = 1000000;
        m_last_print_time_us = now_diag_us;

        const auto& stream = m_espnow_broadcast.getStreamTelemetry();

        float temp_c = 0.0f;
        if (m_temp_sensor) {
            temperature_sensor_get_celsius(m_temp_sensor, &temp_c);
        }

        uint32_t cpu_freq_mhz = 160;
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
        cpu_freq_mhz = 240;
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
        cpu_freq_mhz = 160;
#endif

        uint32_t dma_udr = m_espnow_broadcast.getDmaUnderrunCount();
        uint32_t plc_count = m_espnow_broadcast.getPlcCount();
        uint32_t fifo_ud = m_espnow_broadcast.getFifoUnderrunCount();
        uint32_t prev_rec = m_espnow_broadcast.getPrevFrameRecoveryCount();

        uint32_t delta_dma = (dma_udr >= m_last_dma_udr) ? (dma_udr - m_last_dma_udr) : 0;
        uint32_t delta_fifo = (fifo_ud >= m_last_fifo_udr) ? (fifo_ud - m_last_fifo_udr) : 0;
        m_last_dma_udr = dma_udr;
        m_last_fifo_udr = fifo_ud;

        // FreeRTOS CPU load measurement sampled over the 1-second interval
        int cpu_load_pct = m_cpu_pct;
#if (configGENERATE_RUN_TIME_STATS == 1 && configUSE_TRACE_FACILITY == 1)
        UBaseType_t task_count = uxTaskGetNumberOfTasks();
        if (task_count > 0) {
            TaskStatus_t* task_status_array = static_cast<TaskStatus_t*>(pvPortMalloc(task_count * sizeof(TaskStatus_t)));
            if (task_status_array) {
                uint32_t total_runtime_dummy = 0;
                UBaseType_t num_tasks = uxTaskGetSystemState(task_status_array, task_count, &total_runtime_dummy);
                uint32_t total_tasks_runtime = 0;
                uint32_t idle_runtime = 0;
                for (UBaseType_t i = 0; i < num_tasks; ++i) {
                    total_tasks_runtime += task_status_array[i].ulRunTimeCounter;
                    if (strncmp(task_status_array[i].pcTaskName, "IDLE", 4) == 0) {
                        idle_runtime += task_status_array[i].ulRunTimeCounter;
                    }
                }
                vPortFree(task_status_array);

                if (m_has_prev_runtime) {
                    uint32_t delta_total = total_tasks_runtime - m_last_total_runtime;
                    uint32_t delta_idle = idle_runtime - m_last_idle_runtime;
                    if (delta_total > 0 && delta_idle <= delta_total) {
                        uint32_t active_time = delta_total - delta_idle;
                        cpu_load_pct = static_cast<int>((static_cast<uint64_t>(active_time) * 100ULL + (delta_total / 2)) / delta_total);
                        if (cpu_load_pct > 100) cpu_load_pct = 100;
                        if (cpu_load_pct < 0) cpu_load_pct = 0;
                        m_cpu_pct = cpu_load_pct;
                    }
                } else {
                    m_has_prev_runtime = true;
                }
                m_last_total_runtime = total_tasks_runtime;
                m_last_idle_runtime = idle_runtime;
            }
        }
#endif

        if (cfg->node_role == NODE_ROLE_SINK &&
            m_espnow_broadcast.getState() == AudioNet::NetworkState::STREAMING &&
            (delta_dma > 0 || plc_count > 0 || delta_fifo > 0)) {
            m_status_led.triggerUnderrunFlash(200);
        }

        bool is_audio_active = (m_espnow_broadcast.getState() == AudioNet::NetworkState::STREAMING ||
                                m_espnow_broadcast.getState() == AudioNet::NetworkState::BROADCASTING ||
                                m_espnow_broadcast.getState() == AudioNet::NetworkState::PREFILL);

        float rms_db = is_audio_active ? m_espnow_broadcast.getAudioFrameRMS_dBFS() : -INFINITY;
        float peak_db = is_audio_active ? m_espnow_broadcast.getAudioFramePeak_dBFS() : -INFINITY;

        char rms_str[8], peak_str[8];
        if (!is_audio_active || std::isinf(rms_db) || rms_db <= -95.0f) {
            snprintf(rms_str, sizeof(rms_str), "  off");
        } else {
            snprintf(rms_str, sizeof(rms_str), "%5.1f", rms_db);
        }

        if (!is_audio_active || std::isinf(peak_db) || peak_db <= -95.0f) {
            snprintf(peak_str, sizeof(peak_str), "  off");
        } else {
            snprintf(peak_str, sizeof(peak_str), "%5.1f", peak_db);
        }

        // 1. Read WiFi Channel dynamically from WiFi hardware
        uint8_t wifi_ch = cfg->default_channel;
        wifi_second_chan_t second_ch = WIFI_SECOND_CHAN_NONE;
        uint8_t current_hw_ch = 0;
        if (esp_wifi_get_channel(&current_hw_ch, &second_ch) == ESP_OK && current_hw_ch > 0) {
            wifi_ch = current_hw_ch;
        }

        // 2. Read WiFi RSSI / TX Gain dynamically
        char rssi_str[8];
        if (cfg->node_role == NODE_ROLE_SOURCE) {
            int8_t actual_tx_power = 0;
            if (esp_wifi_get_max_tx_power(&actual_tx_power) == ESP_OK) {
                snprintf(rssi_str, sizeof(rssi_str), "%+4.1f", actual_tx_power * 0.25f);
            } else {
                snprintf(rssi_str, sizeof(rssi_str), "+3.0");
            }
        } else if (m_espnow_broadcast.getState() == AudioNet::NetworkState::OFF ||
                   m_espnow_broadcast.getState() == AudioNet::NetworkState::IDLE) {
            snprintf(rssi_str, sizeof(rssi_str), "  - ");
        } else {
            int8_t rssi_val = m_espnow_broadcast.getLastRssi();
            if (rssi_val <= -120) {
                snprintf(rssi_str, sizeof(rssi_str), "  - ");
            } else {
                snprintf(rssi_str, sizeof(rssi_str), "%4d", rssi_val);
            }
        }

        // 3. Read WiFi PHY rate dynamically
        const char* phy_str = m_espnow_broadcast.getWifiPhyRateString();

        // 4. Read Audio Codec dynamically from audio pipeline
        const char* enc_str = m_espnow_broadcast.getActiveCodecName();

        // 5. Sample Rate (SR kHz)
        char sr_str[8];
        if (is_audio_active) {
            snprintf(sr_str, sizeof(sr_str), "%3.0f", stream.sample_rate / 1000.0f);
        } else {
            snprintf(sr_str, sizeof(sr_str), " -  ");
        }

        // 6. Packet Duration (PD ms)
        char pd_str[8];
        if (is_audio_active) {
            snprintf(pd_str, sizeof(pd_str), "%s", (stream.frame_duration_us == 10000) ? " 10" : "7.5");
        } else {
            snprintf(pd_str, sizeof(pd_str), " - ");
        }

        // 7. Codec Execution Duration (Avg & Pk in ms from 10-element SPSC ring buffer)
        float codec_avg_ms = 0.0f;
        float codec_peak_ms = 0.0f;
        bool has_codec_data = false;
        m_espnow_broadcast.getCodecDurationStats(codec_avg_ms, codec_peak_ms, has_codec_data);

        char codec_avg_str[8], codec_pk_str[8];
        if (has_codec_data && is_audio_active) {
            snprintf(codec_avg_str, sizeof(codec_avg_str), "%5.2f", codec_avg_ms);
            snprintf(codec_pk_str, sizeof(codec_pk_str), "%5.2f", codec_peak_ms);
        } else {
            snprintf(codec_avg_str, sizeof(codec_avg_str), "  -  ");
            snprintf(codec_pk_str, sizeof(codec_pk_str), "  -  ");
        }

        // 8. Software Gain (SW) in dBFS
        char gain_sw_str[8];
        if (!is_audio_active || stream.is_muted) {
            snprintf(gain_sw_str, sizeof(gain_sw_str), " - ");
        } else {
            int gain_sw_db = (stream.volume_percent > 0)
                             ? static_cast<int>(std::round(20.0f * std::log10(static_cast<float>(stream.volume_percent) / 100.0f)))
                             : -99;
            snprintf(gain_sw_str, sizeof(gain_sw_str), "%3d", gain_sw_db);
        }

        // 9. Hardware Gain (HW)
        char gain_hw_str[8];
        if (cfg->node_role == NODE_ROLE_SOURCE || !m_espnow_broadcast.hasLocalAudioOutput()) {
            snprintf(gain_hw_str, sizeof(gain_hw_str), " -");
        } else {
            snprintf(gain_hw_str, sizeof(gain_hw_str), "%+3d", cfg->max98357a_gain_db);
        }

        // 10. Audio Packets 1/s
        char pkts_str[8];
        uint32_t raw_pkts = (cfg->node_role == NODE_ROLE_SOURCE)
                            ? m_espnow_broadcast.getAndResetTxPacketsSec()
                            : m_espnow_broadcast.getAndResetRxPacketsSec();
        uint32_t pkts_sec = static_cast<uint32_t>((static_cast<uint64_t>(raw_pkts) * 1000000ULL) / elapsed_us);
        if (!is_audio_active && raw_pkts == 0) {
            snprintf(pkts_str, sizeof(pkts_str), "   -");
        } else {
            snprintf(pkts_str, sizeof(pkts_str), "%4lu", (unsigned long)pkts_sec);
        }

        // 11. PLC 1/s
        char plc_str[8];
        if (cfg->node_role == NODE_ROLE_SOURCE) {
            snprintf(plc_str, sizeof(plc_str), " - ");
        } else {
            snprintf(plc_str, sizeof(plc_str), "%3lu", (unsigned long)plc_count);
        }

        // 12. DMA UDR
        char dma_udr_str[8];
        if (cfg->node_role == NODE_ROLE_SOURCE || !m_espnow_broadcast.hasLocalAudioOutput()) {
            snprintf(dma_udr_str, sizeof(dma_udr_str), " - ");
        } else {
            snprintf(dma_udr_str, sizeof(dma_udr_str), "%3lu", (unsigned long)dma_udr);
        }

        // 13. FIFO UDR
        char fifo_udr_str[8];
        if (cfg->node_role == NODE_ROLE_SOURCE) {
            snprintf(fifo_udr_str, sizeof(fifo_udr_str), " -  ");
        } else {
            snprintf(fifo_udr_str, sizeof(fifo_udr_str), "%4lu", (unsigned long)fifo_ud);
        }

        // 14. PREV REC
        char prev_rec_str[8];
        if (cfg->node_role == NODE_ROLE_SOURCE) {
            snprintf(prev_rec_str, sizeof(prev_rec_str), " - ");
        } else {
            snprintf(prev_rec_str, sizeof(prev_rec_str), "%4lu", (unsigned long)prev_rec);
        }

        uint32_t t_local = static_cast<uint32_t>((esp_timer_get_time() / 1000ULL) % 1000000ULL);
        char master_time_str[12];
        if (m_espnow_broadcast.isMasterTimeValid()) {
            uint32_t t_master = static_cast<uint32_t>(m_espnow_broadcast.getMasterTimeMs() % 1000000ULL);
            snprintf(master_time_str, sizeof(master_time_str), "%6lu", (unsigned long)t_master);
        } else {
            snprintf(master_time_str, sizeof(master_time_str), "   -  ");
        }

        // Time Offset Diagnostics (EMA_offs, RB_med, RB_rng)
        float ema_ms = 0.0f, rb_med_ms = 0.0f, rb_rng_ms = 0.0f;
        bool has_offset_stats = false;
        m_espnow_broadcast.getTimeOffsetStats(ema_ms, rb_med_ms, rb_rng_ms, has_offset_stats);

        char ema_str[8], med_str[8], rng_str[8];
        if (has_offset_stats && cfg->node_role == NODE_ROLE_SINK) {
            snprintf(ema_str, sizeof(ema_str), "%5.2f", ema_ms);
            snprintf(med_str, sizeof(med_str), "%5.2f", rb_med_ms);
            snprintf(rng_str, sizeof(rng_str), "%5.2f", rb_rng_ms);
        } else {
            snprintf(ema_str, sizeof(ema_str), "  -  ");
            snprintf(med_str, sizeof(med_str), "  -  ");
            snprintf(rng_str, sizeof(rng_str), "  -  ");
        }

        char title_str[64];
        snprintf(title_str, sizeof(title_str), " %s [%s] ", cfg->device_name, (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK");
        size_t title_len = strlen(title_str);

        char audio_block[96];
        snprintf(audio_block, sizeof(audio_block),
                 "  %-3.3s  %5.5s %5.5s  %4.4s  %3.3s %5.5s %5.5s  %3.3s %3.3s  %4.4s  %3.3s  %3.3s  %4.4s %4.4s ",
                 enc_str, rms_str, peak_str, sr_str, pd_str, codec_avg_str, codec_pk_str, gain_sw_str, gain_hw_str,
                 pkts_str, plc_str, dma_udr_str, fifo_udr_str, prev_rec_str);

        char time_sync_block[48];
        snprintf(time_sync_block, sizeof(time_sync_block),
                 " %6lu  %6.6s  %5.5s %5.5s %5.5s ",
                 (unsigned long)t_local, master_time_str, ema_str, med_str, rng_str);

        size_t total_inner = 159;
        size_t left_pad = (total_inner > title_len) ? (total_inner - title_len) / 2 : 0;
        size_t right_pad = (total_inner > title_len) ? (total_inner - title_len - left_pad) : 0;

        char border_line[192];
        int pos = 0;
        border_line[pos++] = '+';
        for (size_t i = 0; i < left_pad; ++i) border_line[pos++] = '=';
        memcpy(border_line + pos, title_str, title_len);
        pos += title_len;
        for (size_t i = 0; i < right_pad; ++i) border_line[pos++] = '=';
        border_line[pos++] = '+';
        border_line[pos++] = '\0';

        char row4_buf[192];
        snprintf(row4_buf, sizeof(row4_buf),
                 "| %2d  %2d  %3u | %-5.5s | %4.4s %02u %-3.3s |%s|%s|",
                 cpu_load_pct, (int)(temp_c + 0.5f), (unsigned)cpu_freq_mhz,
                 getState5Char(m_espnow_broadcast.getState()),
                 rssi_str, (unsigned)wifi_ch, phy_str,
                 audio_block,
                 time_sync_block);

        if ((m_header_counter % 5) == 0) {
            printf("%s\n", border_line);
            printf("|    CPU      | STATE |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms   AMP dB  PKTS  PLC  DMA  FIFO  PREV |         TIME & SYNCHRONIZATION (ms)         |\n");
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                printf("|  %%   C  MHz |       | GAIN Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk    SW  HW   1/s  tot  UDR   UDR   REC |  Local  Master  EMA_offs RB_med RB_rng |\n");
            } else {
                printf("|  %%   C  MHz |       | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk    SW  HW   1/s  tot  UDR   UDR   REC |  Local  Master  EMA_offs RB_med RB_rng |\n");
            }
        }
        printf("%s\n", row4_buf);
        fflush(stdout);
        m_header_counter++;
    }
}

} // namespace Diagnostics
