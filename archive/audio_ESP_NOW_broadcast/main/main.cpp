#include "config.h"
#include "lc3_codec.hpp"
#include "tone_generator.hpp"
#include "i2s_audio.hpp"
#include "espnow_audio_broadcast.hpp"
#include "status_led.hpp"
#include "button.hpp"
#include "diagnostics.hpp"
#include "lc3_benchmark.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"




#include "soc/usb_serial_jtag_struct.h"
#include "hal/usb_serial_jtag_ll.h"
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

static Hardware::StatusLed*        s_status_led = nullptr;
static Hardware::Button*           s_user_button = nullptr;
static Hardware::I2sAudioDriver*    s_i2s_dac = nullptr;
static Codec::Lc3CodecEngine       s_lc3_codec;
static Audio::ToneGenerator        s_tone_gen;
static AudioNet::EspNowAudioBroadcast* s_espnow_broadcast = nullptr;
static Diagnostics::SystemDiagnostics* s_diagnostics = nullptr;
static Benchmark::Lc3BenchmarkSuite*   s_bench_suite = nullptr;

static void on_user_button_pressed(void* user_data) {
    if (!s_espnow_broadcast) return;

    const system_config_t* cfg = get_system_config();
    AudioNet::NetworkState current_state = s_espnow_broadcast->getState();

    ESP_LOGI(TAG, ">>> USER BUTTON TRIGGERED! Current State: %s (Role: %s) <<<",
             s_espnow_broadcast->getStateString(),
             (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK");

    if (cfg->node_role == NODE_ROLE_SOURCE) {
        if (current_state == AudioNet::NetworkState::IDLE) {
            ESP_LOGI(TAG, "SOURCE: Transitioning from IDLE -> BROADCASTING (Resuming audio broadcast)");
            s_espnow_broadcast->transitionTo(AudioNet::NetworkState::BROADCASTING);
        } else {
            ESP_LOGI(TAG, "SOURCE: Transitioning from %s -> IDLE (Stopping audio broadcast)",
                     s_espnow_broadcast->getStateString());
            s_espnow_broadcast->transitionTo(AudioNet::NetworkState::IDLE);
        }
    } else { // SINK
        if (current_state == AudioNet::NetworkState::IDLE) {
            ESP_LOGI(TAG, "SINK: Transitioning from IDLE -> SCANNING (Activating audio receiver)");
            s_espnow_broadcast->transitionTo(AudioNet::NetworkState::SCANNING);
        } else {
            ESP_LOGI(TAG, "SINK: Transitioning from %s -> IDLE (Muting audio and entering IDLE)",
                     s_espnow_broadcast->getStateString());
            s_espnow_broadcast->transitionTo(AudioNet::NetworkState::IDLE);
        }
    }
}

static uint32_t parse_sample_rate_arg(const char* str) {
    if (!str) return 0;
    while (*str == ' ' || *str == '=' || *str == ':' || *str == '\t') str++;
    if (strncasecmp(str, "48", 2) == 0) return 48000;
    if (strncasecmp(str, "32", 2) == 0) return 32000;
    if (strncasecmp(str, "24", 2) == 0) return 24000;
    if (strncasecmp(str, "16", 2) == 0) return 16000;
    if (strncasecmp(str, "8", 1) == 0) return 8000;
    uint32_t val = atoi(str);
    if (val == 48 || val == 48000) return 48000;
    if (val == 32 || val == 32000) return 32000;
    if (val == 24 || val == 24000) return 24000;
    if (val == 16 || val == 16000) return 16000;
    if (val == 8 || val == 8000) return 8000;
    return val;
}

static uint32_t get_next_sample_rate(uint32_t current_sr) {
    switch (current_sr) {
        case 48000: return 32000;
        case 32000: return 24000;
        case 24000: return 16000;
        case 16000: return 8000;
        case 8000:  return 48000;
        default:    return 48000;
    }
}

static SemaphoreHandle_t s_console_mutex = nullptr;

static void print_console(const char* fmt, ...) {
    if (!s_console_mutex) {
        s_console_mutex = xSemaphoreCreateMutex();
    }
    if (s_console_mutex) {
        xSemaphoreTake(s_console_mutex, portMAX_DELAY);
    }
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
    if (s_console_mutex) {
        xSemaphoreGive(s_console_mutex);
    }
}

static void handle_ascii_command(const char* raw_line) {
    if (!raw_line) return;
    while (*raw_line == ' ' || *raw_line == '\t' || *raw_line == '\r' || *raw_line == '\n') raw_line++;
    if (strlen(raw_line) == 0) return;

    char line[64];
    strncpy(line, raw_line, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    int len = strlen(line);
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t' || line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[--len] = '\0';
    }

    if (strncasecmp(line, "sr ", 3) == 0 || strncasecmp(line, "rate ", 5) == 0) {
        const char* p = strchr(line, ' ');
        uint32_t rate = parse_sample_rate_arg(p);
        if (rate > 0 && s_espnow_broadcast) {
            s_espnow_broadcast->setSampleRate(rate);
            if (s_espnow_broadcast->getState() == AudioNet::NetworkState::IDLE) {
                s_espnow_broadcast->transitionTo(AudioNet::NetworkState::BROADCASTING);
            }
            print_console("[OK] Audio Sample Rate changed to %lu Hz (BROADCASTING)\n", (unsigned long)rate);
            ESP_LOGW(TAG, "Audio Sample Rate changed to: %lu Hz", (unsigned long)rate);
        } else {
            print_console("[ERROR] Invalid sample rate '%s'. Supported: 48k, 32k, 24k, 16k, 8k\n", p ? p : "");
            ESP_LOGE(TAG, "Invalid sample rate");
        }
    } else if (strcasecmp(line, "sr") == 0 || strcasecmp(line, "rate") == 0) {
        if (s_espnow_broadcast) {
            uint32_t next_sr = get_next_sample_rate(s_espnow_broadcast->getSampleRate());
            s_espnow_broadcast->setSampleRate(next_sr);
            if (s_espnow_broadcast->getState() == AudioNet::NetworkState::IDLE) {
                s_espnow_broadcast->transitionTo(AudioNet::NetworkState::BROADCASTING);
            }
            print_console("[OK] Cycled Audio Sample Rate to %lu Hz (BROADCASTING)\n", (unsigned long)next_sr);
            ESP_LOGW(TAG, "Cycled Audio Sample Rate to: %lu Hz", (unsigned long)next_sr);
        }
    } else if (strncasecmp(line, "bits ", 5) == 0 || strncasecmp(line, "bd ", 3) == 0 || strncasecmp(line, "bitdepth ", 9) == 0) {
        const char* p = strchr(line, ' ');
        if (p && s_espnow_broadcast) {
            uint8_t bd = static_cast<uint8_t>(atoi(p + 1));
            if (bd == 16 || bd == 24 || bd == 32) {
                s_espnow_broadcast->setBitDepth(bd);
                print_console("[OK] Audio Bit Depth changed to %u-bit\n", bd);
                ESP_LOGW(TAG, "Audio Bit Depth changed to: %u-bit", bd);
            } else {
                print_console("[ERROR] Invalid bit depth '%s'. Supported: 16, 24, 32\n", p + 1);
                ESP_LOGE(TAG, "Invalid bit depth");
            }
        }
    } else if (strcasecmp(line, "bits") == 0 || strcasecmp(line, "bd") == 0) {
        if (s_espnow_broadcast) {
            uint8_t curr = s_espnow_broadcast->getBitDepth();
            uint8_t next = (curr == 16) ? 24 : ((curr == 24) ? 32 : 16);
            s_espnow_broadcast->setBitDepth(next);
            print_console("[OK] Cycled Audio Bit Depth to %u-bit\n", next);
            ESP_LOGW(TAG, "Cycled Audio Bit Depth to: %u-bit", next);
        }
    } else if (strcasecmp(line, "start") == 0 || strcasecmp(line, "play") == 0 || strcasecmp(line, "broadcast") == 0) {
        if (s_espnow_broadcast) {
            s_espnow_broadcast->transitionTo(AudioNet::NetworkState::BROADCASTING);
            print_console("[OK] SOURCE transitioned to BROADCASTING (Audio streaming active)\n");
            ESP_LOGW(TAG, "SOURCE: Transitioned to BROADCASTING (Audio stream active)");
        }
    } else if (strcasecmp(line, "stop") == 0 || strcasecmp(line, "idle") == 0 || strcasecmp(line, "pause") == 0) {
        if (s_espnow_broadcast) {
            s_espnow_broadcast->transitionTo(AudioNet::NetworkState::IDLE);
            print_console("[OK] SOURCE transitioned to IDLE (Audio streaming stopped)\n");
            ESP_LOGW(TAG, "SOURCE: Transitioned to IDLE (Audio stream stopped)");
        }
    } else if (strncasecmp(line, "octets ", 7) == 0 || strncasecmp(line, "len ", 4) == 0) {
        const char* p = strchr(line, ' ');
        if (p && s_espnow_broadcast) {
            uint16_t octets = static_cast<uint16_t>(atoi(p + 1));
            if (octets >= 20 && octets <= 120) {
                s_espnow_broadcast->setFrameLen(octets);
                print_console("[OK] Frame Octets set to %u bytes\n", octets);
                ESP_LOGW(TAG, "Frame Octets set to: %u bytes", octets);
            } else {
                print_console("[ERROR] Invalid octets: %u (range: 20..120)\n", octets);
                ESP_LOGE(TAG, "Invalid octets: %u (range: 20..120)", octets);
            }
        }
    } else if (strncasecmp(line, "ch ", 3) == 0 || strncasecmp(line, "channel ", 8) == 0) {
        const char* p = (strncasecmp(line, "ch ", 3) == 0) ? (line + 3) : (line + 8);
        uint8_t ch = atoi(p);
        if (s_espnow_broadcast) {
            s_espnow_broadcast->setTargetChannel(ch);
            print_console("[OK] Target Listening Audio Channel set to Ch %u\n", ch);
            ESP_LOGW(TAG, "Target Listening Audio Channel set to: Ch %u", ch);
        }
    } else if (strncasecmp(line, "magic ", 6) == 0) {
        uint16_t magic = strtoul(line + 6, nullptr, 0);
        if (s_espnow_broadcast) {
            s_espnow_broadcast->setTestMagicWord(magic);
            print_console("[OK] Active Magic Word set to 0x%04X\n", magic);
            ESP_LOGW(TAG, "Active Magic Word set to: 0x%04X", magic);
        }
    } else if (strncasecmp(line, "dur ", 4) == 0 || strncasecmp(line, "duration ", 9) == 0) {
        const char* p = (strncasecmp(line, "dur ", 4) == 0) ? (line + 4) : (line + 9);
        uint32_t dur_us = (strstr(p, "7.5") != nullptr || strstr(p, "7500") != nullptr) ? 7500 : 10000;
        if (s_espnow_broadcast) {
            s_espnow_broadcast->setFrameDuration(dur_us);
            print_console("[OK] Frame Duration set to %.1f ms\n", dur_us / 1000.0f);
            ESP_LOGW(TAG, "Frame Duration set to: %.1f ms", dur_us / 1000.0f);
        }
    } else if (strcasecmp(line, "dur") == 0) {
        if (s_espnow_broadcast) {
            uint32_t next_dur = (s_espnow_broadcast->getFrameDurationUs() == 10000) ? 7500 : 10000;
            s_espnow_broadcast->setFrameDuration(next_dur);
            print_console("[OK] Toggled Frame Duration to %.1f ms\n", next_dur / 1000.0f);
            ESP_LOGW(TAG, "Toggled Frame Duration to: %.1f ms", next_dur / 1000.0f);
        }
    } else if (strcasecmp(line, "drop") == 0) {
        if (s_espnow_broadcast) {
            s_espnow_broadcast->triggerSimulatedPacketDrop();
            print_console("[OK] Simulated Packet Drop triggered for next frame\n");
        }
    } else if (strncasecmp(line, "phy ", 4) == 0) {
        const char* p = line + 4;
        while (*p == ' ') p++;
        wifi_phy_mode_t mode = WIFI_PHY_MODE_11G;
        wifi_phy_rate_t rate = WIFI_PHY_RATE_12M;
        const char* rate_name = "12 Mbps (OFDM)";
        bool valid = true;

        if (strcasecmp(p, "5.5m") == 0 || strcasecmp(p, "5.5") == 0 || strcasecmp(p, "5m") == 0) {
            mode = WIFI_PHY_MODE_11B; rate = WIFI_PHY_RATE_5M_S; rate_name = "5.5 Mbps (802.11b CCK Short Preamble)";
        } else if (strcasecmp(p, "6m") == 0 || strcasecmp(p, "6") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_6M; rate_name = "6.0 Mbps (802.11g OFDM)";
        } else if (strcasecmp(p, "6.5m") == 0 || strcasecmp(p, "6.5") == 0 || strcasecmp(p, "mc0") == 0 || strcasecmp(p, "mcs0") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS0_LGI; rate_name = "6.5 Mbps (802.11n HT20 MCS0 LGI)";
        } else if (strcasecmp(p, "7.2m") == 0 || strcasecmp(p, "7.2") == 0 || strcasecmp(p, "mc0s") == 0 || strcasecmp(p, "mcs0_sgi") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS0_SGI; rate_name = "7.2 Mbps (802.11n HT20 MCS0 SGI)";
        } else if (strcasecmp(p, "9m") == 0 || strcasecmp(p, "9") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_9M; rate_name = "9.0 Mbps (802.11g OFDM)";
        } else if (strcasecmp(p, "11m") == 0 || strcasecmp(p, "11") == 0) {
            mode = WIFI_PHY_MODE_11B; rate = WIFI_PHY_RATE_11M_S; rate_name = "11.0 Mbps (802.11b CCK Short Preamble)";
        } else if (strcasecmp(p, "12m") == 0 || strcasecmp(p, "12") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_12M; rate_name = "12.0 Mbps (802.11g OFDM)";
        } else if (strcasecmp(p, "13m") == 0 || strcasecmp(p, "13") == 0 || strcasecmp(p, "mc1") == 0 || strcasecmp(p, "mcs1") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS1_LGI; rate_name = "13.0 Mbps (802.11n HT20 MCS1 LGI)";
        } else if (strcasecmp(p, "14.4m") == 0 || strcasecmp(p, "14.4") == 0 || strcasecmp(p, "mc1s") == 0 || strcasecmp(p, "mcs1_sgi") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS1_SGI; rate_name = "14.4 Mbps (802.11n HT20 MCS1 SGI)";
        } else if (strcasecmp(p, "18m") == 0 || strcasecmp(p, "18") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_18M; rate_name = "18.0 Mbps (802.11g OFDM)";
        } else if (strcasecmp(p, "19.5m") == 0 || strcasecmp(p, "19.5") == 0 || strcasecmp(p, "mc2") == 0 || strcasecmp(p, "mcs2") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS2_LGI; rate_name = "19.5 Mbps (802.11n HT20 MCS2 LGI)";
        } else if (strcasecmp(p, "21.7m") == 0 || strcasecmp(p, "21.7") == 0 || strcasecmp(p, "mc2s") == 0 || strcasecmp(p, "mcs2_sgi") == 0) {
            mode = WIFI_PHY_MODE_HT20; rate = WIFI_PHY_RATE_MCS2_SGI; rate_name = "21.7 Mbps (802.11n HT20 MCS2 SGI)";
        } else if (strcasecmp(p, "24m") == 0 || strcasecmp(p, "24") == 0) {
            mode = WIFI_PHY_MODE_11G; rate = WIFI_PHY_RATE_24M; rate_name = "24.0 Mbps (802.11g OFDM)";
        } else {
            valid = false;
            print_console("[ERROR] Invalid PHY rate '%s'. Supported (5-25 Mbps): 5.5m, 6m, mc0 (6.5m), 7.2m, 9m, 11m, 12m, mc1 (13m), 14.4m, 18m, mc2 (19.5m), 21.7m, 24m\n", p);
            ESP_LOGE(TAG, "Invalid PHY rate: %s", p);
        }

        if (valid && s_espnow_broadcast) {
            esp_err_t err = s_espnow_broadcast->setWifiPhyRate(mode, rate);
            if (err == ESP_OK) {
                print_console("[OK] Wi-Fi & ESP-NOW PHY Rate set to %s\n", rate_name);
                ESP_LOGW(TAG, "Wi-Fi & ESP-NOW PHY Rate set to: %s", rate_name);
            } else {
                print_console("[ERROR] Failed to set PHY rate: %s\n", esp_err_to_name(err));
            }
        }
    } else if (strcasecmp(line, "reset") == 0 || strcasecmp(line, "clear") == 0) {
        if (s_espnow_broadcast) {
            s_espnow_broadcast->resetErrorCounters();
            print_console("[OK] Streaming error counters cleared\n");
            ESP_LOGW(TAG, "Streaming error counters (DMA UDR, FIFO UDR, PREV REC, PLC) cleared.");
        }
    } else if (strcasecmp(line, "mono") == 0 || strncasecmp(line, "mode mono", 9) == 0) {
        if (s_espnow_broadcast) {
            s_espnow_broadcast->setStereo(false);
            print_console("[OK] SOURCE Audio Mode set to MONO (1 LC3 encode -> Ch0 & Ch1 duplicated)\n");
            ESP_LOGW(TAG, "SOURCE Audio Mode set to MONO");
        }
    } else if (strcasecmp(line, "stereo") == 0 || strncasecmp(line, "mode stereo", 11) == 0) {
        if (s_espnow_broadcast) {
            s_espnow_broadcast->setStereo(true);
            print_console("[OK] SOURCE Audio Mode set to STEREO (2 distinct LC3 encodes for Ch0 & Ch1)\n");
            ESP_LOGW(TAG, "SOURCE Audio Mode set to STEREO");
        }
    } else if (strcasecmp(line, "mode") == 0 || strcasecmp(line, "ch_mode") == 0) {
        if (s_espnow_broadcast) {
            bool new_st = !s_espnow_broadcast->isStereo();
            s_espnow_broadcast->setStereo(new_st);
            print_console("[OK] Toggled SOURCE Audio Mode to: %s\n", new_st ? "STEREO (2 encodes)" : "MONO (1 encode duplicated)");
            ESP_LOGW(TAG, "Toggled SOURCE Audio Mode to: %s", new_st ? "STEREO" : "MONO");
        }
    } else if (strcasecmp(line, "bench") == 0 || strcasecmp(line, "test") == 0 || strcasecmp(line, "benchmark") == 0) {
        if (s_bench_suite) {
            print_console("[OK] Starting LC3 Hardware Benchmark Suite...\n");
            ESP_LOGW(TAG, "Starting LC3 Hardware Benchmark Suite...");
            s_bench_suite->runAllBenchmarks();
        }
    } else if (strcasecmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        print_console("\n================ AVAILABLE CONSOLE COMMANDS ================\n"
                      "  start / play        : Start audio broadcast (BROADCASTING)\n"
                      "  stop / idle         : Stop audio broadcast (IDLE)\n"
                      "  mono                : Set Mono mode (1 encode -> duplicated to Ch0 & Ch1)\n"
                      "  stereo              : Set Stereo mode (2 separate encodes for Ch0 & Ch1)\n"
                      "  mode / ch_mode      : Toggle Mono <-> Stereo mode\n"
                      "  sr <hz> / rate <hz> : Set Sample Rate (48k, 32k, 24k, 16k, 8k)\n"
                      "  sr / rate           : Cycle to next supported sample rate\n"
                      "  bits <16|24|32> / bd: Set Audio Bit Depth (16, 24, 32 bits)\n"
                      "  bits / bd           : Cycle to next supported bit depth (16->24->32->16)\n"
                      "  dur <10|7.5>        : Set Frame Duration (10.0 ms or 7.5 ms)\n"
                      "  dur                 : Toggle Frame Duration (10ms <-> 7.5ms)\n"
                      "  octets <N>          : Set LC3 Frame Octets (20..120 bytes)\n"
                      "  ch <0..7>           : Set SINK target listening channel\n"
                      "  drop                : Simulate dropping a packet for PLC test\n"
                      "  reset / clear       : Reset diagnostic error counters\n"
                      "  bench               : Run automated LC3 hardware benchmark suite\n"
                      "============================================================\n\n");
    } else {
        print_console("[UNKNOWN CMD] '%s'. Type 'help' for command list.\n", line);
    }
}

static void diagnostics_task_routine(void* pvParameters) {
    while (true) {
        if (s_diagnostics) {
            s_diagnostics->tick();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz tick
    }
}

static void console_task_routine(void* pvParameters) {
    char line_buf[128];
    int line_idx = 0;
    int64_t last_char_time_us = 0;
    static uint8_t ring_buf[4096];
    size_t ring_len = 0;

    // 1. Install USB-SERIAL-JTAG driver with interrupt-driven ring buffer (4KB)
    usb_serial_jtag_driver_config_t jtag_cfg = {
        .tx_buffer_size = 512,
        .rx_buffer_size = 4096,
    };
    usb_serial_jtag_driver_install(&jtag_cfg);
    
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

    print_console("\n[CONSOLE READY] CLI & Binary VSAF input active on USB-Serial and UART0 (%d baud).\n", uart_baud);

    uint8_t rx_buf[1024];
    while (true) {
        // Read available bytes from USB-Serial-JTAG
        int n_usb = usb_serial_jtag_read_bytes(rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(1));
        if (n_usb > 0) {
            if (ring_len + n_usb <= sizeof(ring_buf)) {
                memcpy(ring_buf + ring_len, rx_buf, n_usb);
                ring_len += n_usb;
            }
        }

        // Read available bytes from UART0
        int n_uart = uart_read_bytes(UART_NUM_0, rx_buf, sizeof(rx_buf), 0);
        if (n_uart > 0) {
            if (ring_len + n_uart <= sizeof(ring_buf)) {
                memcpy(ring_buf + ring_len, rx_buf, n_uart);
                ring_len += n_uart;
            }
        }

        // Fast parse dynamic VSAF packets or ASCII CLI commands
        while (ring_len > 0) {
            // Check for VSAF Magic (0x1337 -> 0x37, 0x13 in little-endian)
            if (ring_len >= 2 && ring_buf[0] == 0x37 && ring_buf[1] == 0x13) {
                size_t octets = (s_espnow_broadcast ? s_espnow_broadcast->getFrameLen() : 120);
                size_t pkt_len = AudioNet::VSAF_HEADER_LEN + 2 * octets;

                if (ring_len >= pkt_len) {
                    if (s_espnow_broadcast) {
                        s_espnow_broadcast->processUsbVsafPacket(ring_buf, pkt_len);
                    }
                    if (ring_len > pkt_len) {
                        memmove(ring_buf, ring_buf + pkt_len, ring_len - pkt_len);
                    }
                    ring_len -= pkt_len;
                    line_idx = 0;
                    last_char_time_us = 0;
                    continue;
                } else {
                    // Waiting for the rest of the packet
                    break;
                }
            } else {
                // Not magic byte: process ASCII CLI character
                char c = static_cast<char>(ring_buf[0]);
                if (c == '\r' || c == '\n') {
                    if (line_idx > 0) {
                        line_buf[line_idx] = '\0';
                        handle_ascii_command(line_buf);
                        line_idx = 0;
                        last_char_time_us = 0;
                    }
                } else if (c == '\b' || c == 0x7F) {
                    if (line_idx > 0) {
                        line_idx--;
                        last_char_time_us = esp_timer_get_time();
                    }
                } else if (line_idx < static_cast<int>(sizeof(line_buf) - 1) && c >= 32 && c <= 126) {
                    line_buf[line_idx++] = c;
                    last_char_time_us = esp_timer_get_time();
                }

                if (ring_len > 1) {
                    memmove(ring_buf, ring_buf + 1, ring_len - 1);
                }
                ring_len--;
            }
        }

        // Fallback idle timeout: If user sent text without Enter
        if (line_idx > 0 && last_char_time_us > 0) {
            int64_t elapsed_us = esp_timer_get_time() - last_char_time_us;
            if (elapsed_us > 250000) { // 250 ms idle
                line_buf[line_idx] = '\0';
                handle_ascii_command(line_buf);
                line_idx = 0;
                last_char_time_us = 0;
            }
        }
    }
}

extern "C" void app_main(void) {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    // Disable automatic hardware chip reset triggered by USB CDC line state (DTR/RTS) changes on connect/disconnect
    USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 1;
#endif

    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "=========================================================");
    ESP_LOGI(TAG, "   audioESP-NOW: High-Fidelity LC3 Audio Streamer        ");
    ESP_LOGI(TAG, "   Target: Multi-Target | Wi-Fi ESP-NOW | BLE DISABLED   ");
    ESP_LOGI(TAG, "=========================================================");

    const system_config_t* cfg = get_system_config();
    ESP_LOGI(TAG, "Booting Node: %s (Role: %s, ID: %u)",
             cfg->device_name, (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK", cfg->node_id);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_status_led = &Hardware::getStatusLed();
    s_status_led->init(cfg->status_led_gpio, cfg->status_led_num, (cfg->status_led_gpio == 21));
    s_status_led->setSystemState(Hardware::SystemState::IDLE);

    s_user_button = new Hardware::Button(cfg->user_button_gpio, true, 150);
    s_user_button->init(on_user_button_pressed, nullptr);

    if (cfg->node_role == NODE_ROLE_SOURCE) {
        s_tone_gen.init(CONFIG_ESPNOW_SAMPLE_RATE_HZ);
    } else { // SINK
        s_i2s_dac = new Hardware::I2sAudioDriver(cfg->i2s_bclk_gpio, cfg->i2s_ws_gpio, cfg->i2s_dout_gpio, -1, 0);
        s_i2s_dac->init(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 10000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    }

    s_espnow_broadcast = new AudioNet::EspNowAudioBroadcast(s_lc3_codec, &s_tone_gen, s_i2s_dac);
    s_espnow_broadcast->init(cfg->node_role);

    s_bench_suite = new Benchmark::Lc3BenchmarkSuite(s_lc3_codec);
    s_bench_suite->init();

    s_diagnostics = new Diagnostics::SystemDiagnostics(*s_espnow_broadcast, *s_status_led);
    s_diagnostics->init();

    s_espnow_broadcast->transitionTo(AudioNet::NetworkState::IDLE);
    s_status_led->setSystemState(Hardware::SystemState::IDLE);

    if (cfg->node_role == NODE_ROLE_SINK) {
        s_espnow_broadcast->transitionTo(AudioNet::NetworkState::SCANNING);
        s_status_led->setSystemState(Hardware::SystemState::SCANNING);
    }

    s_espnow_broadcast->startAudioTask();

#if SOC_CPU_CORES_NUM > 1
    xTaskCreatePinnedToCore(diagnostics_task_routine, "diagnostics", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(console_task_routine, "console", 4096, nullptr, 3, nullptr, 0);
#else
    xTaskCreate(diagnostics_task_routine, "diagnostics", 4096, nullptr, 2, nullptr);
    xTaskCreate(console_task_routine, "console", 4096, nullptr, 3, nullptr);
#endif

    ESP_LOGI(TAG, "audioESP-NOW node initialized and running. Type 'bench' to run benchmark.");
}
