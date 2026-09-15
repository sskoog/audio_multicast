#include "diagnostics.hpp"
#include "console.hpp"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>
#include <cmath>

namespace Diagnostics {

SystemDiagnostics::SystemDiagnostics(AudioNet::EspNowUnicastEngine& unicast_engine,
                                     Hardware::StatusLed& status_led)
    : m_unicast_engine(unicast_engine),
      m_status_led(status_led) {
}

SystemDiagnostics::~SystemDiagnostics() {
    if (m_temp_sensor) {
        temperature_sensor_disable(m_temp_sensor);
        temperature_sensor_uninstall(m_temp_sensor);
        m_temp_sensor = nullptr;
    }
}

void SystemDiagnostics::init() {
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    esp_err_t err = temperature_sensor_install(&temp_sensor_config, &m_temp_sensor);
    if (err == ESP_OK) {
        temperature_sensor_enable(m_temp_sensor);
    } else {
        m_temp_sensor = nullptr;
    }
}

static const char* getState5Char(AudioNet::NetworkState state) {
    switch (state) {
        case AudioNet::NetworkState::OFF:          return "OFF  ";
        case AudioNet::NetworkState::IDLE:         return "IDLE ";
        case AudioNet::NetworkState::SCANNING:     return "SCAN ";
        case AudioNet::NetworkState::PREFILL:      return "FILL ";
        case AudioNet::NetworkState::STREAM:       return "STRM ";
        case AudioNet::NetworkState::CAST:         return "CAST ";
        default:                                   return "UNKWN";
    }
}

static const char* getChannelStr(uint8_t ch) {
    switch (ch) {
        case 0:  return "LEFT";
        case 1:  return "RGHT";
        case 2:  return "CNTR";
        case 3:  return "LSUR";
        case 4:  return "RSUR";
        case 5:  return "SUB";
        default: return "CH?";
    }
}

void SystemDiagnostics::tick() {
    m_loop_count++;
    m_unicast_engine.update10HzTimeOffsetStats();

    const system_config_t* cfg = get_system_config();

    if ((m_loop_count % 10) == 0) { // 1 Hz periodic telemetry printout
        int64_t now_diag_us = esp_timer_get_time();
        int64_t elapsed_us = (m_last_print_time_us > 0) ? (now_diag_us - m_last_print_time_us) : 1000000;
        if (elapsed_us <= 0) elapsed_us = 1000000;
        m_last_print_time_us = now_diag_us;

        const auto& stream = m_unicast_engine.getStreamTelemetry();

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

        uint32_t dma_udr = m_unicast_engine.getDmaUnderrunCount();
        uint32_t plc_count = m_unicast_engine.getPlcCount();
        uint32_t fifo_ud = m_unicast_engine.getFifoUnderrunCount();

        uint32_t delta_dma = (dma_udr >= m_last_dma_udr) ? (dma_udr - m_last_dma_udr) : 0;
        uint32_t delta_plc = (plc_count >= m_last_plc_count) ? (plc_count - m_last_plc_count) : 0;
        uint32_t delta_fifo = (fifo_ud >= m_last_fifo_udr) ? (fifo_ud - m_last_fifo_udr) : 0;
        m_last_dma_udr = dma_udr;
        m_last_plc_count = plc_count;
        m_last_fifo_udr = fifo_ud;

        // FreeRTOS CPU load measurement sampled over the 1-second interval (zero dynamic allocation)
        int cpu_load_pct = m_cpu_pct;
#if (configGENERATE_RUN_TIME_STATS == 1 && configUSE_TRACE_FACILITY == 1)
        static TaskStatus_t s_task_status_array[32];
        UBaseType_t task_count = uxTaskGetNumberOfTasks();
        if (task_count > 0) {
            UBaseType_t query_count = (task_count <= 32) ? task_count : 32;
            uint32_t total_runtime_dummy = 0;
            UBaseType_t num_tasks = uxTaskGetSystemState(s_task_status_array, query_count, &total_runtime_dummy);
            uint32_t total_tasks_runtime = 0;
            uint32_t idle_runtime = 0;
            for (UBaseType_t i = 0; i < num_tasks; ++i) {
                total_tasks_runtime += s_task_status_array[i].ulRunTimeCounter;
                if (strncmp(s_task_status_array[i].pcTaskName, "IDLE", 4) == 0) {
                    idle_runtime += s_task_status_array[i].ulRunTimeCounter;
                }
            }

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
#endif

        if (cfg->node_role == NODE_ROLE_SINK &&
            m_unicast_engine.getState() == AudioNet::NetworkState::STREAM &&
            (delta_dma > 0 || delta_plc > 0 || delta_fifo > 0)) {
            m_status_led.triggerUnderrunFlash(200);
        }

        bool is_audio_active = (m_unicast_engine.getState() == AudioNet::NetworkState::STREAM ||
                                m_unicast_engine.getState() == AudioNet::NetworkState::CAST ||
                                m_unicast_engine.getState() == AudioNet::NetworkState::PREFILL);

        float rms_db = is_audio_active ? m_unicast_engine.getAudioFrameRMS_dBFS() : -INFINITY;
        float peak_db = is_audio_active ? m_unicast_engine.getAudioPeak_dBFS() : -INFINITY;

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
                snprintf(rssi_str, sizeof(rssi_str), "+9.0");
            }
        } else if (m_unicast_engine.getState() == AudioNet::NetworkState::OFF ||
                   m_unicast_engine.getState() == AudioNet::NetworkState::IDLE) {
            snprintf(rssi_str, sizeof(rssi_str), "  - ");
        } else {
            int8_t rssi_val = m_unicast_engine.getLastRssi();
            if (rssi_val <= -120) {
                snprintf(rssi_str, sizeof(rssi_str), "  - ");
            } else {
                snprintf(rssi_str, sizeof(rssi_str), "%4d", rssi_val);
            }
        }

        // 3. Read WiFi PHY rate dynamically
        const char* phy_str = m_unicast_engine.getWifiPhyRateString();

        // 4. Read Audio Codec dynamically from audio pipeline
        const char* enc_str = m_unicast_engine.getActiveCodecName();

        // 5. Sample Rate (SR kHz)
        char sr_str[8];
        if (is_audio_active) {
            snprintf(sr_str, sizeof(sr_str), "%4.0f", stream.sample_rate / 1000.0f);
        } else {
            snprintf(sr_str, sizeof(sr_str), "  - ");
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
        m_unicast_engine.getCodecDurationStats(codec_avg_ms, codec_peak_ms, has_codec_data);

        char codec_avg_str[8], codec_pk_str[8];
        if (has_codec_data && is_audio_active) {
            snprintf(codec_avg_str, sizeof(codec_avg_str), "%5.2f", codec_avg_ms);
            snprintf(codec_pk_str, sizeof(codec_pk_str), "%5.2f", codec_peak_ms);
        } else {
            snprintf(codec_avg_str, sizeof(codec_avg_str), "  -  ");
            snprintf(codec_pk_str, sizeof(codec_pk_str), "  -  ");
        }

        char role_col_str[8];
        char mid_block[64];

        if (cfg->node_role == NODE_ROLE_SOURCE) {
            // SOURCE specifics: Node status string (e.g. '1OOOO1' for 6 slots: Ch 0..5)
            m_unicast_engine.getNodeStatusString(role_col_str, sizeof(role_col_str));

            const char* input_str = m_unicast_engine.isToneTestMode() ? "TONE  " : "UAC2  ";

            uint32_t raw_tx_pkts = m_unicast_engine.getAndResetTxPacketsSec();
            uint32_t tx_pkts_sec = static_cast<uint32_t>((static_cast<uint64_t>(raw_tx_pkts) * 1000000ULL) / elapsed_us);
            char tx_pkts_str[8];
            if (!is_audio_active && raw_tx_pkts == 0) {
                snprintf(tx_pkts_str, sizeof(tx_pkts_str), "   -");
            } else {
                snprintf(tx_pkts_str, sizeof(tx_pkts_str), "%4lu", (unsigned long)tx_pkts_sec);
            }

            uint32_t acks_sec = m_unicast_engine.getAndResetTxAcksSec();
            uint32_t ack_fails_sec = m_unicast_engine.getAndResetTxAckFailsSec();
            uint32_t attempts_sec = acks_sec + ack_fails_sec;
            char ack_pct_str[8];
            if (attempts_sec > 0) {
                float ack_pct = (static_cast<float>(acks_sec) * 100.0f) / static_cast<float>(attempts_sec);
                snprintf(ack_pct_str, sizeof(ack_pct_str), "%3.0f%%", ack_pct);
            } else if (!is_audio_active) {
                snprintf(ack_pct_str, sizeof(ack_pct_str), "   -");
            } else {
                snprintf(ack_pct_str, sizeof(ack_pct_str), "100%%");
            }

            char ack_fails_str[8];
            snprintf(ack_fails_str, sizeof(ack_fails_str), "%4lu", (unsigned long)ack_fails_sec);

            uint32_t tx_pkts_total = m_unicast_engine.getTxPacketsTotal();
            char tx_tot_str[8];
            if (tx_pkts_total >= 1000000) {
                snprintf(tx_tot_str, sizeof(tx_tot_str), "%4.1fM", tx_pkts_total / 1000000.0f);
            } else if (tx_pkts_total >= 1000) {
                snprintf(tx_tot_str, sizeof(tx_tot_str), "%4luK", (unsigned long)(tx_pkts_total / 1000));
            } else {
                snprintf(tx_tot_str, sizeof(tx_tot_str), "%4lu ", (unsigned long)tx_pkts_total);
            }

            snprintf(mid_block, sizeof(mid_block),
                     "  %-6.6s    %4.4s  %4.4s  %4.4s  %5.5s ",
                     input_str, tx_pkts_str, ack_pct_str, ack_fails_str, tx_tot_str);
        } else {
            // SINK specifics: Target Channel string
            snprintf(role_col_str, sizeof(role_col_str), "%-6.6s", getChannelStr(m_unicast_engine.getTargetChannel()));

            char gain_sw_str[8];
            uint8_t vol_u8 = m_unicast_engine.getVolume();
            if (!is_audio_active || vol_u8 == 0) {
                snprintf(gain_sw_str, sizeof(gain_sw_str), " - ");
            } else {
                int gain_sw_db = static_cast<int>(std::round(m_unicast_engine.getTargetVolumeDb()));
                snprintf(gain_sw_str, sizeof(gain_sw_str), "%3d", gain_sw_db);
            }

            char gain_hw_str[8];
            if (!m_unicast_engine.hasLocalAudioOutput()) {
                snprintf(gain_hw_str, sizeof(gain_hw_str), " -");
            } else {
                snprintf(gain_hw_str, sizeof(gain_hw_str), "%+3d", m_unicast_engine.getHardwareGainDb());
            }

            char pkts_str[8];
            uint32_t raw_rx_pkts = m_unicast_engine.getAndResetRxPacketsSec();
            uint32_t rx_pkts_sec = static_cast<uint32_t>((static_cast<uint64_t>(raw_rx_pkts) * 1000000ULL) / elapsed_us);
            if (!is_audio_active && raw_rx_pkts == 0) {
                snprintf(pkts_str, sizeof(pkts_str), "   -");
            } else {
                snprintf(pkts_str, sizeof(pkts_str), "%4lu", (unsigned long)rx_pkts_sec);
            }

            char plc_str[8];
            snprintf(plc_str, sizeof(plc_str), "%3lu", (unsigned long)plc_count);

            char dma_udr_str[8];
            if (!m_unicast_engine.hasLocalAudioOutput()) {
                snprintf(dma_udr_str, sizeof(dma_udr_str), " - ");
            } else {
                snprintf(dma_udr_str, sizeof(dma_udr_str), "%3lu", (unsigned long)dma_udr);
            }

            char fifo_udr_str[8];
            snprintf(fifo_udr_str, sizeof(fifo_udr_str), "%3lu", (unsigned long)fifo_ud);

            uint32_t fifo_ovr = m_unicast_engine.getFifoOverflowCount();
            char fifo_ovr_str[8];
            snprintf(fifo_ovr_str, sizeof(fifo_ovr_str), "%3lu", (unsigned long)fifo_ovr);

            snprintf(mid_block, sizeof(mid_block),
                     " %3.3s %3.3s  %4.4s  %3.3s  %3.3s  %3.3s  %3.3s ",
                     gain_sw_str, gain_hw_str, pkts_str, plc_str, dma_udr_str, fifo_udr_str, fifo_ovr_str);
        }

        uint32_t t_local = static_cast<uint32_t>((esp_timer_get_time() / 1000ULL) % 1000000ULL);
        char master_time_str[12];
        if (m_unicast_engine.isMasterTimeValid()) {
            uint32_t t_master = static_cast<uint32_t>(m_unicast_engine.getMasterTimeMs() % 1000000ULL);
            snprintf(master_time_str, sizeof(master_time_str), "%6lu", (unsigned long)t_master);
        } else {
            snprintf(master_time_str, sizeof(master_time_str), "   -  ");
        }

        // Time Offset Diagnostics (EMA_offs, RB_med, RB_rng)
        float ema_ms = 0.0f, rb_med_ms = 0.0f, rb_rng_ms = 0.0f;
        bool has_offset_stats = false;
        m_unicast_engine.getTimeOffsetStats(ema_ms, rb_med_ms, rb_rng_ms, has_offset_stats);

        char ema_str[8], med_str[8], rng_str[8];
        if (has_offset_stats && cfg->node_role == NODE_ROLE_SINK) {
            snprintf(ema_str, sizeof(ema_str), "%+5.2f", ema_ms);
            snprintf(med_str, sizeof(med_str), "%+5.2f", rb_med_ms);
            snprintf(rng_str, sizeof(rng_str), "%5.2f", rb_rng_ms);
        } else {
            snprintf(ema_str, sizeof(ema_str), "  -  ");
            snprintf(med_str, sizeof(med_str), "  -  ");
            snprintf(rng_str, sizeof(rng_str), "  -  ");
        }

        char audio_block[64];
        snprintf(audio_block, sizeof(audio_block),
                 "  %-3.3s  %5.5s %5.5s  %4.4s  %3.3s %5.5s %5.5s ",
                 enc_str, rms_str, peak_str, sr_str, pd_str, codec_avg_str, codec_pk_str);

        char time_sync_block[64];
        if (cfg->node_role == NODE_ROLE_SOURCE) {
            const auto* p0 = m_unicast_engine.getPeer(0); // Left (Node 23)
            const auto* p1 = m_unicast_engine.getPeer(1); // Right (Node 24)

            char l_tot_str[16], l_dwl_str[16], l_net_str[16];
            char r_tot_str[16], r_dwl_str[16], r_net_str[16];

            if (p0 && p0->status == AudioNet::PeerStatus::ONLINE && p0->last_total_rtt_us > 0) {
                snprintf(l_tot_str, sizeof(l_tot_str), "%5lu", (unsigned long)p0->last_total_rtt_us);
                snprintf(l_dwl_str, sizeof(l_dwl_str), "%5lu", (unsigned long)p0->last_dwell_us);
                snprintf(l_net_str, sizeof(l_net_str), "%5lu", (unsigned long)p0->last_net_rtt_us);
            } else {
                snprintf(l_tot_str, sizeof(l_tot_str), "    -");
                snprintf(l_dwl_str, sizeof(l_dwl_str), "    -");
                snprintf(l_net_str, sizeof(l_net_str), "    -");
            }

            if (p1 && p1->status == AudioNet::PeerStatus::ONLINE && p1->last_total_rtt_us > 0) {
                snprintf(r_tot_str, sizeof(r_tot_str), "%5lu", (unsigned long)p1->last_total_rtt_us);
                snprintf(r_dwl_str, sizeof(r_dwl_str), "%5lu", (unsigned long)p1->last_dwell_us);
                snprintf(r_net_str, sizeof(r_net_str), "%5lu", (unsigned long)p1->last_net_rtt_us);
            } else {
                snprintf(r_tot_str, sizeof(r_tot_str), "    -");
                snprintf(r_dwl_str, sizeof(r_dwl_str), "    -");
                snprintf(r_net_str, sizeof(r_net_str), "    -");
            }

            snprintf(time_sync_block, sizeof(time_sync_block),
                     "  %5.5s  %5.5s   %5.5s  %5.5s   %5.5s  %5.5s ",
                     l_tot_str, l_dwl_str, r_tot_str, r_dwl_str, l_net_str, r_net_str);
        } else {
            snprintf(time_sync_block, sizeof(time_sync_block),
                     " %6lu  %6.6s   %5.5s   %5.5s  %5.5s  ",
                     (unsigned long)t_local, master_time_str, ema_str, med_str, rng_str);
        }

        char title_str[64];
        snprintf(title_str, sizeof(title_str), " %s [%s] ", cfg->device_name, (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK");
        size_t title_len = strlen(title_str);

        size_t total_inner = 160;
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

        char row_buf[256];
        snprintf(row_buf, sizeof(row_buf),
                 "| %2d  %2d  %3u | %-5.5s | %-6.6s | %4.4s %02u %-3.3s |%s|%s|%s|",
                 cpu_load_pct, (int)(temp_c + 0.5f), (unsigned)cpu_freq_mhz,
                 getState5Char(m_unicast_engine.getState()),
                 role_col_str,
                 rssi_str, (unsigned)wifi_ch, phy_str,
                 audio_block,
                 mid_block,
                 time_sync_block);

        if ((m_header_counter % 10) == 0) {
            print_console("%s\n", border_line);
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                print_console("|    CPU      | STATE | NODES  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  |  SOURCE      PKTS  ACK%%  FAIL   TOT  |        ROUND-TRIP TIME & DWELL (us)    |\n");
                print_console("|  %%   C  MHz |       | 012345 | GAIN Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  INPUT        1/s   1/s   1/s  pkts   |  L_Tot  L_Dwl   R_Tot  R_Dwl   L_Net  R_Net |\n");
            } else {
                print_console("|    CPU      | STATE |  CHAN  |    WIFI     | AUDIO     dBFS      SR   PD    CODEC ms  | AMP dB   PKTS  PLC  DMA   FIFO    |         TIME & SYNCHRONIZATION (ms)    |\n");
                print_console("|  %%   C  MHz |       |        | RSSI Ch PHY |  Enc    RMS   Pk   kHz   ms   Avg   Pk   |  SW  HW   1/s  tot  UDR  UDR  OVR |  Local  Master  EMA_offs RB_med RB_rng |\n");
            }
        }
        print_console("%s\n", row_buf);
        m_header_counter++;
    }
}

} // namespace Diagnostics
