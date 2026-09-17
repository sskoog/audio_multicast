#include "console.hpp"
#include "config.h"
#include "vsaf_broadcast_engine.hpp"
// #include "lc3_benchmark.hpp" (Disabled)
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <strings.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "tusb.h"
#include "soc/rtc_cntl_reg.h"
#endif

static AudioNet::EspNowUnicastEngine* s_unicast_engine = nullptr;
// static Benchmark::Lc3BenchmarkSuite*  s_bench_suite = nullptr; (Disabled)

static SemaphoreHandle_t s_console_mutex = nullptr;
static StaticSemaphore_t s_console_mutex_buf;

static char s_raw_buf[512];
static char s_out_buf[1024];

/**
 * @brief Thread-safe formatted console printing routing to stdout, UART0, and TinyUSB CDC ACM.
 *
 * Expands standalone newline characters ('\n') to CRLF ("\r\n") for standard serial terminal compatibility,
 * and writes simultaneously to:
 * 1. Standard stdout (VFS console)
 * 2. Hardware UART0 (if driver is installed)
 * 3. TinyUSB CDC ACM virtual serial port (COM116 on ESP32-S3 Node 16) when endpoints are ready.
 *
 * @param format Printf-style format string (const char*, input)
 * @param ... Variable arguments matching format
 */
void print_console(const char* format, ...) {
    if (!s_console_mutex) {
        s_console_mutex = xSemaphoreCreateMutexStatic(&s_console_mutex_buf);
    }

    if (s_console_mutex) {
        xSemaphoreTake(s_console_mutex, portMAX_DELAY);
    }

    va_list args;
    va_start(args, format);
    int raw_len = vsnprintf(s_raw_buf, sizeof(s_raw_buf), format, args);
    va_end(args);

    if (raw_len > 0) {
        // Expand standalone '\n' to "\r\n" for clean serial terminal display without staircasing
        int out_len = 0;
        for (int i = 0; i < raw_len && out_len < static_cast<int>(sizeof(s_out_buf) - 2); ++i) {
            if (s_raw_buf[i] == '\n' && (i == 0 || s_raw_buf[i - 1] != '\r')) {
                s_out_buf[out_len++] = '\r';
            }
            s_out_buf[out_len++] = s_raw_buf[i];
        }
        s_out_buf[out_len] = '\0';

        // 1. Stdout / VFS console
        fwrite(s_out_buf, 1, out_len, stdout);
        fflush(stdout);

        // 2. Hardware UART0 if driver is active
        if (uart_is_driver_installed(UART_NUM_0)) {
            uart_write_bytes(UART_NUM_0, s_out_buf, out_len);
        }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        // 3. TinyUSB CDC ACM (COM116 on ESP32-S3 Node 16)
        if (tud_cdc_ready()) {
            tud_cdc_write(s_out_buf, static_cast<uint32_t>(out_len));
            tud_cdc_write_flush();
        }
#endif
    }

    if (s_console_mutex) {
        xSemaphoreGive(s_console_mutex);
    }
}

/**
 * @brief Register active engine for console command dispatch.
 *
 * @param unicast_engine Pointer to active EspNowUnicastEngine instance (input)
 */
void console_init(AudioNet::EspNowUnicastEngine* unicast_engine) {
    s_unicast_engine = unicast_engine;
}

/**
 * @brief Parse standard 6-byte colon or hyphen separated MAC address string.
 *
 * @param str Input string containing hex MAC (const char*, input)
 * @param out_mac Destination buffer for 6-byte raw MAC address (uint8_t*, output)
 * @return true if 6 octets were parsed successfully, false otherwise
 */
bool parse_mac_address(const char* str, uint8_t* out_mac) {
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

/**
 * @brief Parse and execute an ASCII CLI command line received via UART or USB CDC ACM.
 *
 * Dispatches management commands including peer manipulation, volume adjustment,
 * sample rate / PHY rate changes, codec benchmarks, diagnostics, and system reboot.
 *
 * @param raw_line Null-terminated ASCII command string (const char*, input)
 */
void handle_ascii_command(const char* raw_line) {
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
                      "  scan [auto] / survey       - Passive 802.11 RF sniffer survey across channels 1..13\n"
                      "  wifich <1..13>             - Set Wi-Fi channel manually\n"
                      "  start / play / cast        - Transition SOURCE to CAST / SINK to SCANNING\n"
                      "  stop / pause               - Stop transmission / receiver (transition to IDLE)\n"
                      "  tone [on|off]              - Toggle internal test tone generator (overrides auto-USB)\n"
                      "  phy <rate>                 - Switch PHY rate (primary/ht3, secondary/12m, tertiary/ht0)\n"
                      "  mode mono|stereo|surround  - Switch audio channel generation mode\n"
                      "  ch <0..5>                  - Set SINK target channel (0: Left, 1: Right, 5: Sub)\n"
                      "  sublp <20..500>            - Set Subwoofer 4th-order LR low-pass cutoff (Hz)\n"
                      "  octets <60..120>           - Set LC3 frame length in octets (default: 120)\n"
                      "  sr <16k|24k|32k|48k|96k>   - Set audio sample rate\n"
                      "  vol <0..100>               - Set volume percentage (0=Mute, 100=0dB)\n"
                      "  voldb <-96..0>             - Set volume in dB (-96.0 dB to 0.0 dB)\n"
                      "  volu8 <0..255>             - Set raw uint8 volume\n"
                      "  volch <ch> <0..255>        - Set volume for specific channel (SOURCE)\n"
                      "  mute / unmute              - Mute / Unmute audio (slew-limited)\n"
                      "  gain <0|3|6|9|12|15>       - Set I2S DAC hardware gain (dB)\n"
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
    } else if (strncasecmp(line, "peer enable ", 12) == 0) {
        int ch = atoi(line + 12);
        if (ch >= 0 && ch < static_cast<int>(AudioNet::MAX_SINK_NODES)) {
            if (s_unicast_engine && s_unicast_engine->setPeerEnabled(ch, true)) {
                print_console("[OK] Enabled channel %d\n", ch);
            }
        } else {
            print_console("[USAGE] peer enable <0..5>\n");
        }
    } else if (strncasecmp(line, "peer disable ", 13) == 0) {
        int ch = atoi(line + 13);
        if (ch >= 0 && ch < static_cast<int>(AudioNet::MAX_SINK_NODES)) {
            if (s_unicast_engine && s_unicast_engine->setPeerEnabled(ch, false)) {
                print_console("[OK] Disabled channel %d\n", ch);
            }
        } else {
            print_console("[USAGE] peer disable <0..5>\n");
        }
    } else if (strcasecmp(line, "peer list") == 0 || strcasecmp(line, "peers") == 0) {
        if (s_unicast_engine) {
            int cnt = s_unicast_engine->getPeerCount();
            print_console("\n--- REGISTERED BROADCAST SINK CHANNELS (%d/%d) ---\n", cnt, static_cast<int>(AudioNet::MAX_SINK_NODES));
            for (int i = 0; i < cnt; i++) {
                const auto* p = s_unicast_engine->getPeer(i);
                const char* st_str = !p->is_enabled ? "DISABLED" : (p->status == AudioNet::PeerStatus::ONLINE ? "ONLINE  " : "OFFLINE ");
                print_console(" [%d] %-10s | CH: %u | %s | RTT: %3lu us (avg %3lu us) | PTP: %+5ld us | RSSI(U/D): %3d/%3d dBm | Sent: %lu | ACKs: %lu | ARQ: %lu/%lu | Fails: %lu\n",
                              i, p->name,
                              p->channel_id, st_str,
                              (unsigned long)p->last_rtt_us,
                              (unsigned long)p->avg_rtt_us,
                              (long)p->clock_offset_us,
                              p->last_rssi, p->downlink_rssi,
                              (unsigned long)p->packets_sent,
                              (unsigned long)p->acks_received,
                              (unsigned long)p->arq_retries,
                              (unsigned long)p->arq_successes,
                              (unsigned long)p->ack_failures);
            }
            print_console("--------------------------------------------------------------------------------------------------------------------\n\n");
        }
    } else if (strcasecmp(line, "start") == 0 || strcasecmp(line, "play") == 0 || strcasecmp(line, "unicast") == 0 || strcasecmp(line, "cast") == 0) {
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg && cfg->node_role == NODE_ROLE_SOURCE) {
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
    } else if (strncasecmp(line, "tone", 4) == 0 || strncasecmp(line, "synth", 5) == 0) {
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
    } else if (strncasecmp(line, "phy ", 4) == 0) {
        const char* rate_str = line + 4;
        wifi_phy_mode_t mode = WIFI_PHY_MODE_HT20;
        wifi_phy_rate_t rate = WIFI_PHY_RATE_MCS1_LGI;

        if (strcasecmp(rate_str, "primary") == 0 || strcasecmp(rate_str, "1") == 0 ||
            strcasecmp(rate_str, "ht3") == 0 || strcasecmp(rate_str, "mcs3") == 0 || strcasecmp(rate_str, "mc3") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS3_LGI;
        } else if (strcasecmp(rate_str, "secondary") == 0 || strcasecmp(rate_str, "2") == 0 ||
                   strcasecmp(rate_str, "ofd") == 0 || strcasecmp(rate_str, "12m") == 0 || strcasecmp(rate_str, "ofdm") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_12M;
        } else if (strcasecmp(rate_str, "tertiary") == 0 || strcasecmp(rate_str, "3") == 0 ||
                   strcasecmp(rate_str, "ht0") == 0 || strcasecmp(rate_str, "mcs0") == 0 || strcasecmp(rate_str, "mc0") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS0_LGI;
        } else if (strcasecmp(rate_str, "mc1") == 0 || strcasecmp(rate_str, "mcs1") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS1_LGI;
        } else if (strcasecmp(rate_str, "mc2") == 0 || strcasecmp(rate_str, "mcs2") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS2_LGI;
        } else if (strcasecmp(rate_str, "6m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_6M;
        } else if (strcasecmp(rate_str, "9m") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_9M;
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
            print_console("[OK] Switched Wi-Fi PHY Rate to %s (%s)\n",
                          s_unicast_engine->getWifiPhyRateString(),
                          (mode == WIFI_PHY_MODE_HT20) ? "802.11n HT20" : "802.11g OFDM");
        }
    } else if (strncasecmp(line, "scan", 4) == 0 || strncasecmp(line, "survey", 6) == 0) {
        if (s_unicast_engine) {
            bool auto_apply = (strcasestr(line, "auto") != nullptr || strcasestr(line, "apply") != nullptr || strcasestr(line, "set") != nullptr);
            uint32_t dwell_ms = 300;
            int val = 0;
            if (sscanf(line, "%*s %d", &val) == 1 && val >= 50 && val <= 2000) {
                dwell_ms = static_cast<uint32_t>(val);
            }
            s_unicast_engine->scanAndSelectBestChannel(dwell_ms, auto_apply, true);
        }
    } else if (strncasecmp(line, "wifich ", 7) == 0 || strncasecmp(line, "channel ", 8) == 0) {
        int ch = 0;
        if ((sscanf(line, "%*s %d", &ch) == 1) && ch >= 1 && ch <= 13 && s_unicast_engine) {
            s_unicast_engine->setWifiChannel(static_cast<uint8_t>(ch));
            print_console("[OK] Wi-Fi channel set to %d\n", ch);
        } else {
            print_console("[USAGE] wifich <1..13>\n");
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
        print_console("[INFO] LC3 benchmark suite is disabled in this firmware build.\n");
    } else if (strncasecmp(line, "vol ", 4) == 0) {
        int pct = atoi(line + 4);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        uint8_t vol_u8 = (pct == 0) ? 0 : static_cast<uint8_t>(1 + (pct * 254) / 100);
        if (s_unicast_engine) {
            const system_config_t* cfg = get_system_config();
            if (cfg && cfg->node_role == NODE_ROLE_SOURCE) {
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
            if (cfg && cfg->node_role == NODE_ROLE_SOURCE) {
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
            if (cfg && cfg->node_role == NODE_ROLE_SOURCE) {
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
            if (cfg && cfg->node_role == NODE_ROLE_SOURCE) {
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
            if (cfg && cfg->node_role == NODE_ROLE_SOURCE) {
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
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        // Set bit 0 of RTC_CNTL_OPTION1_REG (0x6000812C) to force ROM bootloader (COM16)
        *((volatile uint32_t*)0x6000812C) = 1;
#endif
        esp_restart();
    } else {
        print_console("[UNKNOWN CMD] '%s'. Type 'help' for command list.\n", line);
    }
}

