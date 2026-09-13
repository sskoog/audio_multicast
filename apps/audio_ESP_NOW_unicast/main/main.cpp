#include "config.h"
#include "console.hpp"
#include "lc3_codec.hpp"
#include "tone_generator.hpp"
#include "i2s_audio.hpp"
#include "espnow_unicast_engine.hpp"
#include "status_led.hpp"
#include "button.hpp"
#include "diagnostics.hpp"
#include "lc3_benchmark.hpp"
#include "usb_audio.hpp"
#include "stats_test.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "tusb.h"
#include "soc/rtc_cntl_reg.h"
#endif
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "soc/usb_serial_jtag_struct.h"
#include "hal/usb_serial_jtag_ll.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <fcntl.h>
#include <unistd.h>

static const char* TAG = "MAIN";

static Hardware::StatusLed*            s_status_led = nullptr;
static Hardware::Button*               s_user_button = nullptr;
static Hardware::I2sAudioDriver*       s_i2s_dac = nullptr;
static Codec::Lc3CodecEngine           s_lc3_codec;
static Audio::ToneGenerator            s_tone_gen;
static AudioNet::EspNowUnicastEngine*  s_unicast_engine = nullptr;
static Diagnostics::SystemDiagnostics* s_diagnostics = nullptr;
static Benchmark::Lc3BenchmarkSuite*   s_bench_suite = nullptr;

static void on_user_button_pressed(void* user_data) {
    if (!s_unicast_engine) return;

    const system_config_t* cfg = get_system_config();
    AudioNet::NetworkState current_state = s_unicast_engine->getState();

    ESP_LOGI(TAG, ">>> USER BUTTON TRIGGERED! Current State: %s (Role: %s) <<<",
             s_unicast_engine->getStateString(),
             (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK");

    if (cfg->node_role == NODE_ROLE_SOURCE) {
        if (current_state == AudioNet::NetworkState::IDLE) {
            ESP_LOGI(TAG, "SOURCE: Transitioning from IDLE -> CAST (Resuming audio unicast)");
            s_unicast_engine->transitionTo(AudioNet::NetworkState::CAST);
        } else {
            ESP_LOGI(TAG, "SOURCE: Transitioning from %s -> IDLE (Stopping unicast)",
                     s_unicast_engine->getStateString());
            s_unicast_engine->transitionTo(AudioNet::NetworkState::IDLE);
        }
    } else {
        if (current_state == AudioNet::NetworkState::IDLE) {
            ESP_LOGI(TAG, "SINK: Transitioning from IDLE -> SCANNING (Resuming audio receiver)");
            s_unicast_engine->transitionTo(AudioNet::NetworkState::SCANNING);
        } else {
            ESP_LOGI(TAG, "SINK: Transitioning from %s -> IDLE (Muting receiver)",
                     s_unicast_engine->getStateString());
            s_unicast_engine->transitionTo(AudioNet::NetworkState::IDLE);
        }
    }
}


static bool parse_mac_address(const char* str, uint8_t* out_mac) {
    if (!str || !out_mac) return false;
    unsigned int m[6];
    if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6 ||
        sscanf(str, "%02X:%02X:%02X:%02X:%02X:%02X", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6 ||
        sscanf(str, "%02x-%02x-%02x-%02x-%02x-%02x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
        for (int i = 0; i < 6; i++) out_mac[i] = static_cast<uint8_t>(m[i]);
        return true;
    }
    return false;
}

static void handle_ascii_command(const char* raw_line) {
    if (!raw_line) return;
    while (*raw_line == ' ' || *raw_line == '\t' || *raw_line == '\r' || *raw_line == '\n') raw_line++;
    if (strlen(raw_line) == 0) return;

    char line[128];
    strncpy(line, raw_line, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    int len = strlen(line);
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t' || line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[--len] = '\0';
    }

    if (strcasecmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        print_console("\n================ MULTI-UNICAST CONSOLE COMMANDS ================\n"
                      "  peer list                  - Display registered SINK peers, uptime, and ACK statistics\n"
                      "  peer add <mac> <ch> [name] - Register a new SINK peer (ch 0..5)\n"
                      "  peer del <mac>             - Remove a SINK peer\n"
                      "  peer enable <mac>          - Enable unicast transmission to peer\n"
                      "  peer disable <mac>         - Disable unicast transmission to peer\n"
                      "  start / play / cast     - Transition SOURCE to CAST / SINK to SCANNING\n"
                      "  stop / pause               - Stop transmission / receiver (transition to IDLE)\n"
                      "  tone [on|off]              - Toggle internal test tone generator (overrides auto-USB)\n"

                      "  phy <rate>                 - Switch PHY rate (mc0..mc7, 6m, 9m, 12m, 18m, 24m)\n"
                      "  mode mono|stereo|surround  - Switch audio channel generation mode\n"
                      "  ch <0..5>                  - Set SINK target channel (0: Left, 1: Right, 5: Sub)\n"
                      "  sublp <20..500>            - Set Subwoofer 4th-order LR low-pass cutoff (Hz)\n""  octets <60..120>           - Set LC3 frame length in octets (default: 120)\n"
                      "  sr <16k|24k|32k|48k|96k>   - Set audio sample rate\n"
                      "  vol <0..100>               - Set volume percentage (0=Mute, 100=0dB)\n"
                      "  voldb <-96..0>             - Set volume in dB (-96.0 dB to 0.0 dB)\n"
                      "  volu8 <0..255>             - Set raw uint8 volume\n"
                      "  volch <ch> <0..255>        - Set volume for specific channel (SOURCE)\n"
                      "  mute / unmute              - Mute / Unmute audio (slew-limited)\n"
                      "  gain <0|3|6|9|12|15>       - Set I2S DAC hardware gain (dB)\n"
                      "  bench                      - Run in-depth LC3 codec benchmark suite\n"
                      "  clear / cls                - Reset / clear error counters\n"
                      "  diag                       - Print system telemetry report\n"
                      "  reset / reboot             - Reboot microcontroller\n"
                      "================================================================\n\n");
    } else if (strncasecmp(line, "peer add ", 9) == 0) {
        char mac_str[32] = {0};
        int ch = 0;
        char name[16] = {0};
        int parsed = sscanf(line + 9, "%31s %d %15s", mac_str, &ch, name);
        if (parsed >= 2) {
            uint8_t mac[6];
            if (parse_mac_address(mac_str, mac)) {
                if (s_unicast_engine && s_unicast_engine->addPeer(mac, static_cast<uint8_t>(ch), (parsed >= 3) ? name : nullptr)) {
                    print_console("[OK] Added peer %02X:%02X:%02X:%02X:%02X:%02X on Channel %d (%s)\n",
                                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], ch, (parsed >= 3) ? name : "unnamed");
                } else {
                    print_console("[ERROR] Failed to add peer (table full or engine uninitialized)\n");
                }
            } else {
                print_console("[ERROR] Invalid MAC address format: %s\n", mac_str);
            }
        } else {
            print_console("[USAGE] peer add <MAC> <CH (0..5)> [NAME]\n");
        }
    } else if (strncasecmp(line, "peer del ", 9) == 0) {
        char mac_str[32] = {0};
        if (sscanf(line + 9, "%31s", mac_str) == 1) {
            uint8_t mac[6];
            if (parse_mac_address(mac_str, mac)) {
                if (s_unicast_engine && s_unicast_engine->removePeer(mac)) {
                    print_console("[OK] Removed peer %02X:%02X:%02X:%02X:%02X:%02X\n",
                                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                } else {
                    print_console("[ERROR] Peer not found in table\n");
                }
            } else {
                print_console("[ERROR] Invalid MAC format: %s\n", mac_str);
            }
        }
    } else if (strncasecmp(line, "peer enable ", 12) == 0) {
        char mac_str[32] = {0};
        if (sscanf(line + 12, "%31s", mac_str) == 1) {
            uint8_t mac[6];
            if (parse_mac_address(mac_str, mac)) {
                if (s_unicast_engine && s_unicast_engine->setPeerEnabled(mac, true)) {
                    print_console("[OK] Enabled peer %02X:%02X:%02X:%02X:%02X:%02X\n",
                                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                }
            }
        }
    } else if (strncasecmp(line, "peer disable ", 13) == 0) {
        char mac_str[32] = {0};
        if (sscanf(line + 13, "%31s", mac_str) == 1) {
            uint8_t mac[6];
            if (parse_mac_address(mac_str, mac)) {
                if (s_unicast_engine && s_unicast_engine->setPeerEnabled(mac, false)) {
                    print_console("[OK] Disabled peer %02X:%02X:%02X:%02X:%02X:%02X\n",
                                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                }
            }
        }
    } else if (strcasecmp(line, "peer list") == 0 || strcasecmp(line, "peers") == 0) {
        if (s_unicast_engine) {
            int cnt = s_unicast_engine->getPeerCount();
            print_console("\n--- REGISTERED UNICAST SINK PEERS (%d/%d) ---\n", cnt, MAX_UNICAST_SINKS);
            for (int i = 0; i < cnt; i++) {
                const auto* p = s_unicast_engine->getPeer(i);
                if (!p) continue;
                print_console(" [%d] %-8s | MAC: %02X:%02X:%02X:%02X:%02X:%02X | CH: %u | %s | Sent: %lu | ACKs: %lu | Fails: %lu\n",
                              i, p->name,
                              p->mac[0], p->mac[1], p->mac[2], p->mac[3], p->mac[4], p->mac[5],
                              p->channel_id, p->is_enabled ? "ENABLED " : "DISABLED",
                              (unsigned long)p->packets_sent,
                              (unsigned long)p->acks_received,
                              (unsigned long)p->ack_failures);
            }
            print_console("----------------------------------------------\n\n");
        }
    } else if (strcasecmp(line, "start") == 0 || strcasecmp(line, "play") == 0 || strcasecmp(line, "unicast") == 0 || strcasecmp(line, "cast") == 0) {
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                s_unicast_engine->transitionTo(AudioNet::NetworkState::CAST);
                print_console("[OK] SOURCE transitioned to CAST (will stream when USB audio active)\n");
            } else {
                s_unicast_engine->transitionTo(AudioNet::NetworkState::SCANNING);
                print_console("[OK] SINK transitioned to SCANNING\n");
            }
        }
    } else if (strcasecmp(line, "stop") == 0 || strcasecmp(line, "pause") == 0) {
        if (s_unicast_engine) {
            s_unicast_engine->setToneTestMode(false);
            s_unicast_engine->transitionTo(AudioNet::NetworkState::IDLE);
            print_console("[OK] Transitioned to IDLE (streaming paused)\n");
        }
    } else if (strncasecmp(line, "tone", 4) == 0) {
        if (s_unicast_engine) {
            if (strcasestr(line, "on")) {
                s_unicast_engine->setToneTestMode(true);
                s_unicast_engine->transitionTo(AudioNet::NetworkState::CAST);
                print_console("[OK] Internal test tone ENABLED (CAST mode forced)\n");
            } else if (strcasestr(line, "off")) {
                s_unicast_engine->setToneTestMode(false);
                s_unicast_engine->transitionTo(AudioNet::NetworkState::IDLE);
                print_console("[OK] Internal test tone DISABLED (Auto-USB streaming mode)\n");
            } else {
                bool cur = s_unicast_engine->isToneTestMode();
                s_unicast_engine->setToneTestMode(!cur);
                if (!cur) {
                    s_unicast_engine->transitionTo(AudioNet::NetworkState::CAST);
                }
                print_console("[OK] Internal test tone %s\n", (!cur) ? "ENABLED (CAST mode forced)" : "DISABLED (Auto-USB streaming mode)");
            }
        }
    }
 else if (strncasecmp(line, "phy ", 4) == 0) {
        const char* rate_str = line + 4;
        wifi_phy_mode_t mode = WIFI_PHY_MODE_HT20;
        wifi_phy_rate_t rate = WIFI_PHY_RATE_MCS1_LGI;

        if (strcasecmp(rate_str, "mc0") == 0 || strcasecmp(rate_str, "mcs0") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS0_LGI;
        } else if (strcasecmp(rate_str, "mc1") == 0 || strcasecmp(rate_str, "mcs1") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS1_LGI;
        } else if (strcasecmp(rate_str, "mc2") == 0 || strcasecmp(rate_str, "mcs2") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS2_LGI;
        } else if (strcasecmp(rate_str, "mc3") == 0 || strcasecmp(rate_str, "mcs3") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS3_LGI;
        } else if (strcasecmp(rate_str, "6m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_6M;
        } else if (strcasecmp(rate_str, "9m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_9M;
        } else if (strcasecmp(rate_str, "12m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_12M;
        } else if (strcasecmp(rate_str, "18m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_18M;
        } else if (strcasecmp(rate_str, "24m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_24M;
        } else if (strcasecmp(rate_str, "36m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_36M;
        } else if (strcasecmp(rate_str, "48m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_48M;
        } else if (strcasecmp(rate_str, "54m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_54M;
        }

        if (s_unicast_engine) {
            s_unicast_engine->setWifiPhyRate(mode, rate);
            print_console("[OK] Switched Wi-Fi PHY Rate to %s\n", s_unicast_engine->getWifiPhyRateString());
        }
    } else if (strncasecmp(line, "ch ", 3) == 0) {
        int ch = atoi(line + 3);
        if (ch >= 0 && ch <= 5 && s_unicast_engine) {
            s_unicast_engine->setTargetChannel(static_cast<uint8_t>(ch));
            print_console("[OK] SINK target channel set to %d (%s)\n",
                          ch, (ch == 0) ? "Left" : (ch == 1) ? "Right" : (ch == 5) ? "Subwoofer" : "Surround");
        }
    } else if (strncasecmp(line, "octets ", 7) == 0) {
        int oct = atoi(line + 7);
        if (oct >= 20 && oct <= MAX_LC3_FRAME_OCTETS && s_unicast_engine) {
            s_unicast_engine->setFrameLen(static_cast<uint16_t>(oct));
            print_console("[OK] LC3 Frame Length set to %d octets\n", oct);
        }
    } else if (strncasecmp(line, "sr ", 3) == 0) {
        char sr_str[16] = {0};
        if (sscanf(line + 3, "%15s", sr_str) == 1 && s_unicast_engine) {
            uint32_t sr = 48000;
            if (strcasecmp(sr_str, "8k") == 0 || strcasecmp(sr_str, "8000") == 0) sr = 8000;
            else if (strcasecmp(sr_str, "16k") == 0 || strcasecmp(sr_str, "16000") == 0) sr = 16000;
            else if (strcasecmp(sr_str, "24k") == 0 || strcasecmp(sr_str, "24000") == 0) sr = 24000;
            else if (strcasecmp(sr_str, "32k") == 0 || strcasecmp(sr_str, "32000") == 0) sr = 32000;
            else if (strcasecmp(sr_str, "48k") == 0 || strcasecmp(sr_str, "48000") == 0) sr = 48000;
            else if (strcasecmp(sr_str, "96k") == 0 || strcasecmp(sr_str, "96000") == 0) sr = 96000;
            s_unicast_engine->setSampleRate(sr);
            print_console("[OK] Sample rate switched to %lu Hz\n", (unsigned long)sr);
        }
    } else if (strcasecmp(line, "clear") == 0 || strcasecmp(line, "cls") == 0) {
        if (s_unicast_engine) {
            s_unicast_engine->resetStreamingCounters();
            s_unicast_engine->resetErrorCounters();
            print_console("[OK] Counters and error metrics cleared\n");
        }
    } else if (strcasecmp(line, "bench") == 0) {
        if (s_bench_suite) {
            print_console("[RUN] Running LC3 Codec Benchmark Suite...\n");
            s_bench_suite->runAllBenchmarks();
        }
    } else if (strncasecmp(line, "vol ", 4) == 0) {
        int pct = atoi(line + 4);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        uint8_t vol_u8 = (pct == 0) ? 0 : static_cast<uint8_t>(1 + (pct * 254) / 100);
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                s_unicast_engine->sendVolumeCommand(0xFF, vol_u8);
                print_console("[OK] SOURCE broadcast VOLUME_SET: %d%% (%u/255) to all SINKs\n", pct, vol_u8);
            } else {
                s_unicast_engine->setVolume(vol_u8);
                print_console("[OK] SINK Volume set: %d%% (%u/255)\n", pct, vol_u8);
            }
        }
    } else if (strncasecmp(line, "voldb ", 6) == 0) {
        float db = atof(line + 6);
        if (db < CONFIG_VOLUME_MIN_DB) db = CONFIG_VOLUME_MIN_DB;
        if (db > CONFIG_VOLUME_MAX_DB) db = CONFIG_VOLUME_MAX_DB;
        uint8_t vol_u8 = static_cast<uint8_t>(1.0f + ((db - CONFIG_VOLUME_MIN_DB) / (CONFIG_VOLUME_MAX_DB - CONFIG_VOLUME_MIN_DB)) * 254.0f);
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                s_unicast_engine->sendVolumeCommand(0xFF, vol_u8);
                print_console("[OK] SOURCE broadcast VOLUME_SET: %+5.1fdB (%u/255) to all SINKs\n", db, vol_u8);
            } else {
                s_unicast_engine->setVolume(vol_u8);
                print_console("[OK] SINK Volume set: %+5.1fdB (%u/255)\n", db, vol_u8);
            }
        }
    } else if (strncasecmp(line, "volu8 ", 6) == 0) {
        int u8_val = atoi(line + 6);
        if (u8_val < 0) u8_val = 0;
        if (u8_val > 255) u8_val = 255;
        uint8_t vol_u8 = static_cast<uint8_t>(u8_val);
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                s_unicast_engine->sendVolumeCommand(0xFF, vol_u8);
                print_console("[OK] SOURCE broadcast raw VOLUME_SET: %u/255\n", vol_u8);
            } else {
                s_unicast_engine->setVolume(vol_u8);
                print_console("[OK] SINK raw Volume set: %u/255\n", vol_u8);
            }
        }
    } else if (strncasecmp(line, "volch ", 6) == 0) {
        int ch = 0, u8_val = 255;
        if (sscanf(line + 6, "%d %d", &ch, &u8_val) == 2 && s_unicast_engine) {
            if (u8_val < 0) u8_val = 0;
            if (u8_val > 255) u8_val = 255;
            s_unicast_engine->sendVolumeCommand(static_cast<uint8_t>(ch), static_cast<uint8_t>(u8_val));
            print_console("[OK] SOURCE sent VOLUME_SET to Channel %d: %u/255\n", ch, u8_val);
        }
    } else if (strcasecmp(line, "mute") == 0) {
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                s_unicast_engine->sendVolumeCommand(0xFF, 0);
                print_console("[OK] SOURCE broadcast MUTE (0/255) to all SINKs\n");
            } else {
                s_unicast_engine->setVolume(0);
                print_console("[OK] SINK Muted\n");
            }
        }
    } else if (strcasecmp(line, "unmute") == 0) {
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg->node_role == NODE_ROLE_SOURCE) {
                s_unicast_engine->sendVolumeCommand(0xFF, 255);
                print_console("[OK] SOURCE broadcast UNMUTE (255/255 = 0.0dB) to all SINKs\n");
            } else {
                s_unicast_engine->setVolume(255);
                print_console("[OK] SINK Unmuted (255/255 = 0.0dB)\n");
            }
        }
    } else if (strcasecmp(line, "reset") == 0 || strcasecmp(line, "reboot") == 0) {
        print_console("[SYS] Rebooting system...\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    } else if (strcasecmp(line, "bootloader") == 0 || strcasecmp(line, "download") == 0) {
        print_console("[SYS] Rebooting into ROM download bootloader (COM16)...\n");
        vTaskDelay(pdMS_TO_TICKS(100));
#if defined(RTC_CNTL_OPTION1_REG) && defined(RTC_CNTL_FORCE_DOWNLOAD_BOOT)
        REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
#endif
        esp_restart();
    } else {
        print_console("[UNKNOWN CMD] '%s'. Type 'help' for command list.\n", line);
    }
}

// Background High-Speed USB / UART Binary Stream & CLI Reader Task on Core 0
static void usb_serial_cli_task(void* pvParameters) {
    char line_buf[128];
    size_t line_idx = 0;
    static uint8_t ring_buf[8192];
    size_t ring_len = 0;

    const system_config_t* cfg = get_system_config();
    bool is_s3_source = false;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    is_s3_source = (cfg && cfg->node_role == NODE_ROLE_SOURCE);
#endif

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // On ESP32-S3 SOURCE, TinyUSB owns the USB OTG PHY. Do NOT install USB-Serial-JTAG driver!
    if (!is_s3_source) {
        usb_serial_jtag_driver_config_t jtag_cfg = {
            .tx_buffer_size = 512,
            .rx_buffer_size = 4096,
        };
        usb_serial_jtag_driver_install(&jtag_cfg);
    }
#endif

    // 2. Install UART0 driver (2MBaud)
    int uart_baud = 2000000;
    uart_config_t uart_cfg = {
        .baud_rate = uart_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_driver_install(UART_NUM_0, 4096, 512, 0, NULL, 0);

    print_console("\n[CONSOLE READY] CLI and Binary PCM Stream input active on USB-Serial and UART0 (%d baud).\n", uart_baud);

    uint8_t rx_buf[1024];
    while (true) {
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        if (!is_s3_source) {
            // Read available bytes from USB-Serial-JTAG
            int n_usb = usb_serial_jtag_read_bytes(rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(1));
            if (n_usb > 0) {
                if (ring_len + n_usb <= sizeof(ring_buf)) {
                    memcpy(ring_buf + ring_len, rx_buf, n_usb);
                    ring_len += n_usb;
                }
            }
        }
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (is_s3_source) {
            // Read from TinyUSB CDC ACM
            if (tud_cdc_available()) {
                uint32_t n_cdc = tud_cdc_read(rx_buf, sizeof(rx_buf));
                if (n_cdc > 0 && ring_len + n_cdc <= sizeof(ring_buf)) {
                    memcpy(ring_buf + ring_len, rx_buf, n_cdc);
                    ring_len += n_cdc;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
#else
        vTaskDelay(pdMS_TO_TICKS(1));
#endif

        // Read available bytes from UART0
        int n_uart = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf), 0);
        if (n_uart > 0) {
            if (ring_len + n_uart <= sizeof(ring_buf)) {
                memcpy(ring_buf + ring_len, rx_buf, n_uart);
                ring_len += n_uart;
            }
        }

        // Fast parse dynamic VSAF LC3 packets or ASCII CLI commands
        while (ring_len > 0) {
            // Check for VSAF LC3 Magic (0x1337 -> 0x37, 0x13 in little-endian)
            if (ring_len >= 2 && ring_buf[0] == 0x37 && ring_buf[1] == 0x13) {
                if (ring_len >= sizeof(AudioNet::vsaf_usb_header_t)) {
                    const auto* usb_hdr = reinterpret_cast<const AudioNet::vsaf_usb_header_t*>(ring_buf);
                    size_t pkt_len = sizeof(AudioNet::vsaf_usb_header_t) + usb_hdr->octets;
                    if (ring_len >= pkt_len) {
                        if (s_unicast_engine) {
                            s_unicast_engine->processUsbVsafPacket(ring_buf, pkt_len);
                        }
                        if (ring_len > pkt_len) {
                            memmove(ring_buf, ring_buf + pkt_len, ring_len - pkt_len);
                        }
                        ring_len -= pkt_len;
                        line_idx = 0;
                        continue;
                    } else {
                        // Waiting for remaining bytes of full LC3 packet
                        break;
                    }
                } else {
                    // Waiting for header
                    break;
                }
            } else {
                // Process ASCII CLI character
                char c = static_cast<char>(ring_buf[0]);
                if (c == '\r' || c == '\n') {
                    if (line_idx > 0) {
                        line_buf[line_idx] = '\0';
                        handle_ascii_command(line_buf);
                        line_idx = 0;
                    }
                } else if (c == '\b' || c == 0x7F) {
                    if (line_idx > 0) line_idx--;
                } else if (c >= 32 && c <= 126) {
                    if (line_idx < sizeof(line_buf) - 1) {
                        line_buf[line_idx++] = c;
                    }
                }
                if (ring_len > 1) {
                    memmove(ring_buf, ring_buf + 1, ring_len - 1);
                }
                ring_len -= 1;
            }
        }
    }
}

extern "C" void app_main(void) {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 1;
#endif

    setvbuf(stdout, NULL, _IONBF, 0);

    // 0. Configure Task Watchdog Timer (TWDT) to 1.0 second (1000 ms)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    if (esp_task_wdt_reconfigure(&twdt_config) != ESP_OK) {
        esp_task_wdt_init(&twdt_config);
    }
    ESP_LOGI(TAG, "Task Watchdog Timer (TWDT) configured: 1.0s timeout (panic on failure).");

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "   ESP-NOW MULTI-UNICAST AUDIO STREAMING ENGINE   ");
    ESP_LOGI(TAG, "   High-Fidelity LC3 Multi-Speaker Network        ");
    ESP_LOGI(TAG, "==================================================");

    const system_config_t* cfg = get_system_config();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Status LED
    s_status_led = &Hardware::getStatusLed();
    s_status_led->init(cfg->status_led_gpio, cfg->status_led_num, (cfg->status_led_gpio == 21));
    s_status_led->setSystemState(Hardware::SystemState::IDLE);

    // 2. User Button
    if (cfg->user_button_gpio >= 0) {
        s_user_button = new Hardware::Button(cfg->user_button_gpio, true, 150);
        s_user_button->init(on_user_button_pressed, nullptr);
    }

    // 3. I2S DAC (for SINK node)
    if (cfg->node_role == NODE_ROLE_SINK) {
        s_i2s_dac = new Hardware::I2sAudioDriver(
            cfg->i2s_bclk_gpio,
            cfg->i2s_ws_gpio,
            cfg->i2s_dout_gpio,
            -1,
            0
        );
        s_i2s_dac->init(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 10000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        s_i2s_dac->setHardwareGain(static_cast<Hardware::Max98357Gain>(cfg->max98357a_gain_db));
    }

    // 4. Unicast Engine
    s_unicast_engine = new AudioNet::EspNowUnicastEngine(s_lc3_codec, &s_tone_gen, s_i2s_dac);

    // Auto-detect SINK channel based on MAC address
    if (cfg->node_role == NODE_ROLE_SINK) {
        uint8_t base_mac[6] = {0};
        esp_read_mac(base_mac, ESP_MAC_WIFI_STA);
        if (base_mac[5] == 0x44 || base_mac[4] == 0x38) {
            s_unicast_engine->setTargetChannel(0); // Left speaker (Node 23)
        } else if (base_mac[5] == 0xE4 || base_mac[4] == 0x18) {
            s_unicast_engine->setTargetChannel(1); // Right speaker (Node 24)
        } else {
            s_unicast_engine->setTargetChannel(0);
        }
    } else if (cfg->node_role == NODE_ROLE_SOURCE) {
        // Initialize USB Audio + CDC for SOURCE
        usb_audio_init();
    }

    s_unicast_engine->init(cfg->node_role, cfg->node_id, cfg->default_channel);
    s_unicast_engine->start();

    // 5. Diagnostics
    s_diagnostics = new Diagnostics::SystemDiagnostics(*s_unicast_engine, *s_status_led);
    s_diagnostics->init();

    // 6. Benchmark Suite
    s_bench_suite = new Benchmark::Lc3BenchmarkSuite(s_lc3_codec);

    // Run stats component self-test suite
    run_stats_self_test();

    // 7. Start Background CLI Task
    xTaskCreatePinnedToCore(usb_serial_cli_task, "cli_task", 4096, nullptr, 2, nullptr, 0);

    ESP_LOGI(TAG, "Device Node ID: %d | Role: %s | Name: %s",
             cfg->node_id,
             (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE (Transmitter)" : "SINK (Receiver)",
             cfg->device_name);
    ESP_LOGI(TAG, "System initialization complete! Streaming started.");

    // Main 10 Hz telemetry loop
    while (true) {
        s_diagnostics->tick();
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms = 10 Hz
    }
}
