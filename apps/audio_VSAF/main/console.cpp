#include "console.hpp"
#include "config.h"
#include "vsaf_broadcast_engine.hpp"
#include "usb_audio.hpp"
// #include "lc3_benchmark.hpp" (Disabled)
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <strings.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "tusb.h"
#include "soc/rtc_cntl_reg.h"
#endif

static AudioNet::EspNowUnicastEngine* s_unicast_engine = nullptr;
// static Benchmark::Lc3BenchmarkSuite*  s_bench_suite = nullptr; (Disabled)

static SemaphoreHandle_t s_console_mutex = nullptr;
static StaticSemaphore_t s_console_mutex_buf;

// ---------------------------------------------------------------------------
// USB-Serial-JTAG console diagnostics. Kept in RTC memory (survives a chip / software reset, not a power-on),
// so after a console stall + reset the previous session's counters can still be read ('usjdiag').
// ---------------------------------------------------------------------------
struct UsjDiag {
    uint32_t magic;
    uint32_t boots;            // resets since the last power-on
    uint32_t write_ok;         // console writes accepted by the USJ VFS
    uint32_t write_fail;       // console writes rejected (EIO: VFS thinks no host is connected)
    uint32_t rx_bytes;         // console bytes received from the host
    uint32_t conn_s;           // seconds with usb_serial_jtag_is_connected() == true
    uint32_t disconn_s;        // seconds with it == false
    uint32_t conn_changes;     // connected <-> disconnected transitions
    uint32_t uptime_s;         // session length when last updated
    uint8_t  last_conn;
};
static constexpr uint32_t USJ_DIAG_MAGIC = 0x55534A44;  // "USJD"
RTC_NOINIT_ATTR static UsjDiag s_usj;                  // current session (also readable after a reset)
static UsjDiag s_usj_prev = {};                         // previous session, copied at boot
static bool s_usj_prev_valid = false;

static void usj_diag_boot() {
    if (s_usj.magic == USJ_DIAG_MAGIC) {
        s_usj_prev = s_usj;
        s_usj_prev_valid = true;
        uint32_t boots = s_usj.boots + 1;
        s_usj = {};
        s_usj.boots = boots;
    } else {
        s_usj = {};
    }
    s_usj.magic = USJ_DIAG_MAGIC;
    s_usj.last_conn = 1;
}

void console_usj_note_rx(int n) {
    if (n > 0) s_usj.rx_bytes += static_cast<uint32_t>(n);
}

/**
 * @brief Sample the USJ connection flag once per second (called from the CLI task loop).
 */
void console_usj_tick() {
    static int64_t last_us = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_us < 1000000) return;
    last_us = now;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    uint8_t conn = usb_serial_jtag_is_connected() ? 1 : 0;
    if (conn) s_usj.conn_s++; else s_usj.disconn_s++;
    if (conn != s_usj.last_conn) s_usj.conn_changes++;
    s_usj.last_conn = conn;
#endif
    s_usj.uptime_s = static_cast<uint32_t>(now / 1000000);
}

static void print_usj_diag(const char* label, const UsjDiag& d) {
    print_console("[USJ] %s: boots=%lu up=%lus | writes ok=%lu fail(EIO)=%lu | rx=%lu B | connected=%lus "
                  "disconnected=%lus changes=%lu now=%s\n",
                  label, (unsigned long)d.boots, (unsigned long)d.uptime_s, (unsigned long)d.write_ok,
                  (unsigned long)d.write_fail, (unsigned long)d.rx_bytes, (unsigned long)d.conn_s,
                  (unsigned long)d.disconn_s, (unsigned long)d.conn_changes, d.last_conn ? "yes" : "no");
}

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

        // 1. Hardware UART0 if driver is active
        if (uart_is_driver_installed(UART_NUM_0)) {
            uart_write_bytes(UART_NUM_0, s_out_buf, out_len);
        }

        // 2. Standard output / USB-Serial-JTAG VFS (COM4 on S3 SINK, COM23/COM24 on C6)
        if (write(STDOUT_FILENO, s_out_buf, out_len) < 0) {
            s_usj.write_fail++;
        } else {
            s_usj.write_ok++;
        }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        // 3. TinyUSB CDC ACM (COM116 on ESP32-S3 Node 16 SOURCE)
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
 * @brief Console 'tasks': per-task CPU time over a 1 s window (FreeRTOS run-time stats, esp_timer based).
 *
 * Interrupt time is charged to whichever task it interrupted, so a task's share includes ISRs that hit it.
 * Output: one row per task (name, priority, core affinity, CPU us in the window, % of one core).
 */
static void print_task_cpu_window(void) {
    constexpr UBaseType_t MAX_TASKS = 40;
    TaskStatus_t* a = static_cast<TaskStatus_t*>(heap_caps_malloc(2 * MAX_TASKS * sizeof(TaskStatus_t), MALLOC_CAP_INTERNAL));
    if (!a) {
        print_console("[TASKS] no memory\n");
        return;
    }
    TaskStatus_t* b = a + MAX_TASKS;
    configRUN_TIME_COUNTER_TYPE t0 = 0, t1 = 0;
    UBaseType_t na = uxTaskGetSystemState(a, MAX_TASKS, &t0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    UBaseType_t nb = uxTaskGetSystemState(b, MAX_TASKS, &t1);
    uint32_t window = static_cast<uint32_t>(t1 - t0);
    if (window == 0) window = 1;
    print_console("\n[TASKS] CPU per task over %lu us (100%% = one core; ISR time counts against the interrupted task)\n",
                  (unsigned long)window);
    print_console("  %-16s %4s %4s %9s %6s\n", "task", "prio", "core", "cpu us", "%");
    for (UBaseType_t i = 0; i < nb; ++i) {
        uint32_t prev = 0;
        for (UBaseType_t j = 0; j < na; ++j) {
            if (a[j].xHandle == b[i].xHandle) {
                prev = static_cast<uint32_t>(a[j].ulRunTimeCounter);
                break;
            }
        }
        uint32_t d = static_cast<uint32_t>(b[i].ulRunTimeCounter) - prev;
        if (d < 200) continue; // hide tasks below 0.02 %
        BaseType_t aff = xTaskGetCoreID(b[i].xHandle);
        int core = (aff == tskNO_AFFINITY) ? -1 : static_cast<int>(aff);
        print_console("  %-16s %4u %4d %9lu %5.1f%%\n", b[i].pcTaskName, (unsigned)b[i].uxCurrentPriority, core,
                      (unsigned long)d, 100.0f * d / window);
    }
    free(a);
}

/**
 * @brief Register active engine for console command dispatch.
 *
 * @param unicast_engine Pointer to active EspNowUnicastEngine instance (input)
 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static vprintf_like_t s_orig_log_vprintf = nullptr;

/**
 * @brief ESP_LOG sink that mirrors log lines to the TinyUSB CDC console (COM116).
 *
 * On Node 16 the IDF console is NONE + secondary USB-Serial/JTAG, which is unreachable while TinyUSB owns
 * the USB PHY, so ESP_LOGx output (Wi-Fi driver warnings, "TX hang detected", ...) was silently lost.
 * Never blocks: if the console mutex is busy or the CDC FIFO is not ready, the line is dropped from CDC
 * (it still goes to the original sink). Safe to call from the real-time TX task.
 *
 * @param fmt  printf-style format (input)
 * @param args format arguments (input)
 * @return Result of the original vprintf sink (characters written), as required by vprintf_like_t.
 */
// RAM ring of recent log output, so boot / fault messages survive even when no terminal is attached
// (console 'log' prints it). Written under a spinlock; never blocks.
static char s_log_ring[4096];
static size_t s_log_ring_head = 0;   // next write position
static bool s_log_ring_wrapped = false;
static portMUX_TYPE s_log_ring_mux = portMUX_INITIALIZER_UNLOCKED;

static void log_ring_append(const char* s, size_t len) {
    portENTER_CRITICAL_SAFE(&s_log_ring_mux);
    for (size_t i = 0; i < len; ++i) {
        s_log_ring[s_log_ring_head++] = s[i];
        if (s_log_ring_head >= sizeof(s_log_ring)) {
            s_log_ring_head = 0;
            s_log_ring_wrapped = true;
        }
    }
    portEXIT_CRITICAL_SAFE(&s_log_ring_mux);
}

static int cdc_log_vprintf(const char* fmt, va_list args) {
    char line[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);
    if (n > 0) {
        log_ring_append(line, static_cast<size_t>(n < static_cast<int>(sizeof(line)) ? n : sizeof(line) - 1));
    }
    if (n > 0 && s_console_mutex && tud_cdc_ready() &&
        xSemaphoreTake(s_console_mutex, 0) == pdTRUE) {
        uint32_t len = static_cast<uint32_t>(n < static_cast<int>(sizeof(line)) ? n : sizeof(line) - 1);
        if (tud_cdc_write_available() >= len + 1) {
            bool lf_end = (len > 0 && line[len - 1] == '\n');
            tud_cdc_write(line, lf_end ? len - 1 : len);
            if (lf_end) {
                tud_cdc_write_str("\r\n");  // ESP_LOG ends lines with '\n'; terminals expect CRLF
            }
            tud_cdc_write_flush();
        }
        xSemaphoreGive(s_console_mutex);
    }
    return s_orig_log_vprintf ? s_orig_log_vprintf(fmt, args) : n;
}
#endif

void console_init(AudioNet::EspNowUnicastEngine* unicast_engine) {
    s_unicast_engine = unicast_engine;
    usj_diag_boot();
    if (!s_console_mutex) {
        s_console_mutex = xSemaphoreCreateMutexStatic(&s_console_mutex_buf);
    }
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!s_orig_log_vprintf) {
        s_orig_log_vprintf = esp_log_set_vprintf(cdc_log_vprintf);
    }
#endif
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
        // Partition help printout into <= 400 char batches to fit in s_raw_buf[512] with blocking flush
        print_console("\n================ MULTI-UNICAST CONSOLE COMMANDS ================\n"
                      "--- PEER & NETWORK MANAGEMENT ---\n"
                      "  peer list                  - Display registered SINK peers and stats\n"
                      "  peer add <mac> <ch> [name] - Register a new SINK peer (ch 0..5)\n"
                      "  peer del <mac>             - Remove a SINK peer\n"
                      "  peer enable/disable <mac>  - Enable/disable unicast to peer\n");
        vTaskDelay(pdMS_TO_TICKS(5));

        print_console("--- WI-FI & RF MANAGEMENT ---\n"
                      "  scan [auto] / survey       - Passive 802.11 sniffer survey ch 1..13\n"
                      "  wifich <1..13>             - Set Wi-Fi channel manually\n"
                      "  phy <rate>                 - Switch PHY rate (primary/ht3, etc.)\n"
                      "  txpower <2..20|max>        - Set Wi-Fi max TX power (dBm)\n"
                      "  start / play / cast        - Transition SOURCE to CAST / SINK to SCAN\n"
                      "  stop / pause               - Stop transmission / receiver (IDLE)\n");
        vTaskDelay(pdMS_TO_TICKS(5));

        print_console("--- AUDIO & CODEC CONFIGURATION ---\n"
                      "  tone [on|off]              - Toggle internal test tone generator\n"
                      "  mode mono|stereo|surround  - Switch audio channel generation mode\n"
                      "  sublp <20..500>            - Set Subwoofer LR low-pass cutoff (Hz)\n"
                      "  sr <16k|24k|32k|48k|96k>   - Set audio sample rate\n");
        vTaskDelay(pdMS_TO_TICKS(5));

        print_console("--- VOLUME & SYSTEM CONTROL ---\n"
                      "  vol <0..100> / voldb <-96..0> - Set volume (%% or dB)\n"
                      "  volu8 <0..255> / volch <c> <v> - Set raw uint8 volume\n"
                      "  postgain <-48..12>         - Set SINK software post-gain (dB)\n"
                      "  mute / unmute              - Mute / Unmute audio (slew-limited)\n"
                      "  gain <0|3|6|9|12|15>       - Set I2S DAC hardware gain (dB)\n"
                      "  clear / cls                - Reset / clear error counters\n"
                      "  diag / reset / reboot      - System telemetry / Reboot node\n"
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
                AudioNet::EspNowBroadcastEngine::PtpChannelStatus ps{};
                s_unicast_engine->getPtpStatus(static_cast<uint8_t>(i), ps);
                char err_str[12];
                if (ps.playout_err_us == AudioNet::VSAF_PLAYOUT_ERR_UNKNOWN) {
                    snprintf(err_str, sizeof(err_str), "  -  ");
                } else {
                    snprintf(err_str, sizeof(err_str), "%+5d", ps.playout_err_us);
                }
                print_console(" [%d] %-10s | CH: %u | %s | RTT: %3lu us (avg %3lu us) | sync rtt %4u us, playout err %s us | RSSI(U/D): %3d/%3d dBm | Sent: %lu | ACKs: %lu | ARQ: %lu/%lu | Fails: %lu\n",
                              i, p->name,
                              p->channel_id, st_str,
                              (unsigned long)p->last_rtt_us,
                              (unsigned long)p->avg_rtt_us,
                              ps.valid ? ps.rtt_us : 0, err_str,
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
    } else if (strncasecmp(line, "caltone", 7) == 0) {
        // SOURCE: calibration signal for SPL level matching.
        // caltone off | caltone pink [rms_dBFS] | caltone sine [rms_dBFS] [sat_Hz] [sub_Hz]
        if (s_unicast_engine) {
            char mode_s[8] = "";
            float db = -20.0f, sat_hz = 1000.0f, sub_hz = 63.0f;
            const int n = sscanf(line + 7, "%7s %f %f %f", mode_s, &db, &sat_hz, &sub_hz);
            db = std::clamp(db, -60.0f, -6.0f);
            sat_hz = std::clamp(sat_hz, 100.0f, 10000.0f);
            sub_hz = std::clamp(sub_hz, 20.0f, 100.0f);
            Audio::CalSignal::Mode m = Audio::CalSignal::Mode::OFF;
            if (n >= 1 && strcasecmp(mode_s, "pink") == 0) {
                m = Audio::CalSignal::Mode::PINK;
            } else if (n >= 1 && strcasecmp(mode_s, "sine") == 0) {
                m = Audio::CalSignal::Mode::SINE;
            } else if (n >= 1 && strcasecmp(mode_s, "off") != 0) {
                print_console("[ERR] usage: caltone off | pink [rms_dBFS] | sine [rms_dBFS] [sat_Hz] [sub_Hz]\n");
                return;
            }
            s_unicast_engine->setCalSignal(m, db, sat_hz, sub_hz);
            if (m == Audio::CalSignal::Mode::OFF) {
                s_unicast_engine->setToneTestMode(false);
                print_console("[OK] Calibration signal OFF (program input again)\n");
            } else {
                // Keep the SOURCE casting even without USB audio
                s_unicast_engine->setToneTestMode(true);
                s_unicast_engine->transitionTo(AudioNet::NetworkState::CAST);
                if (m == Audio::CalSignal::Mode::PINK) {
                    print_console("[OK] Calibration: pink noise %+.1f dBFS RMS, satellites 500-2000 Hz, sub 40-100 Hz\n", db);
                } else {
                    print_console("[OK] Calibration: sine %+.1f dBFS RMS, satellites %.0f Hz, sub %.0f Hz\n", db, sat_hz, sub_hz);
                }
            }
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
            s_unicast_engine->requestWifiPhyRate(mode, rate); // applied at the next frame boundary
            print_console("[OK] Switching Wi-Fi PHY Rate (%s) at the next frame boundary\n",
                          (mode == WIFI_PHY_MODE_HT20) ? "802.11n HT20" : "802.11g OFDM");
        }
    } else if (strncasecmp(line, "scan", 4) == 0 || strncasecmp(line, "survey", 6) == 0) {
        const system_config_t* scan_cfg = get_system_config();
        if (s_unicast_engine && scan_cfg && scan_cfg->node_role == NODE_ROLE_SOURCE &&
            s_unicast_engine->getState() == AudioNet::NetworkState::CAST) {
            // The survey hops the radio across ch 1..13 for seconds; the TX task would keep broadcasting
            // on whatever channel the sniffer is visiting.
            print_console("[ERROR] scan would interrupt the broadcast: run 'stop' first, then 'scan', then 'start'\n");
        } else if (s_unicast_engine) {
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
            s_unicast_engine->requestWifiChannel(static_cast<uint8_t>(ch)); // applied at the next frame boundary
            s_unicast_engine->lockChannel(true);
            print_console("[OK] Wi-Fi channel set and locked to %d\n", ch);
        } else {
            print_console("[USAGE] wifich <1..13>\n");
        }
    } else if (strncasecmp(line, "txp ", 4) == 0 || strncasecmp(line, "txpower ", 8) == 0) {
        float dbm = 0.0f;
        const char* val_str = (strncasecmp(line, "txpower ", 8) == 0) ? line + 8 : line + 4;
        while (*val_str == ' ') val_str++;
        if (strcasecmp(val_str, "max") == 0) {
            dbm = 20.0f;
        } else {
            dbm = atof(val_str);
        }
        if (dbm < 2.0f) dbm = 2.0f;
        if (dbm > 20.0f) dbm = 20.0f;
        int8_t pwr_unit = static_cast<int8_t>(std::round(dbm * 4.0f)); // 0.25 dBm units
        if (pwr_unit < 8) pwr_unit = 8;
        if (pwr_unit > 84) pwr_unit = 84;
        if (s_unicast_engine) {
            s_unicast_engine->setWifiTxPower(pwr_unit);
            print_console("[OK] Wi-Fi TX power set: readback %+4.2f dBm (%d / 0.25 dBm; quantized by the radio)\n",
                          s_unicast_engine->getWifiTxPower() / 4.0f, s_unicast_engine->getWifiTxPower());
        }
    } else if (strncasecmp(line, "driftcorr ", 10) == 0) {
        // SINK: clock-drift compensation on (1, default) / off (0, to measure free-running drift)
        if (s_unicast_engine) {
            s_unicast_engine->setDriftCorrection(atoi(line + 10) != 0);
            print_console("[OK] SINK drift correction %s\n", s_unicast_engine->getDriftCorrection() ? "ON" : "OFF");
        }
    } else if (strncasecmp(line, "postgain ", 9) == 0) {
        float db = atof(line + 9);
        if (db < -48.0f) db = -48.0f;
        if (db > 12.0f) db = 12.0f;
        if (s_unicast_engine) {
            s_unicast_engine->setPostGainDb(db);
            print_console("[OK] Software post-gain set to %+5.1f dB\n", db);
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
    } else if (strcasecmp(line, "timing") == 0) {
        // SOURCE pipeline timing over the last completed ~1 s window (see vsaf_source_pipeline.cpp)
        if (s_unicast_engine) {
            AudioNet::SourceTimingSnapshot t;
            s_unicast_engine->getSourceTiming(t);
            const float fr = t.frames ? static_cast<float>(t.frames) : 1.0f;
            print_console("\n[TIMING] last window: %lu frames | dspload %lu us | overruns %lu | cancelled jobs %lu | "
                          "MQ missing %lu | deadline skips %lu | TX timeouts %lu | src_rf wakes/frame %.1f, active %lu us/frame | MQ waits %lu\n",
                          (unsigned long)t.frames, (unsigned long)s_unicast_engine->getDspExtraLoadUs(),
                          (unsigned long)t.overruns, (unsigned long)t.cancelled_jobs, (unsigned long)t.mq_missing,
                          (unsigned long)t.deadline_skips, (unsigned long)t.tx_timeouts, t.rf_wakes / fr,
                          (unsigned long)(t.rf_active_us / fr), (unsigned long)t.mq_waits);
            print_console("  %-34s %8s %8s %7s\n", "stage (us)", "avg", "max", "count");
            auto row = [](const char* name, const AudioNet::TimingAcc::Snap& s) {
                print_console("  %-34s %8lu %8lu %7lu\n", name, (unsigned long)s.avg(), (unsigned long)s.max,
                              (unsigned long)s.n);
            };
            row("DSP (core 0)", t.dsp);
            row("LC3 HQ: analysis + 120B, 48k", t.enc_hq);
            row("LC3 MQ: 60B on shared analysis", t.enc_mq);
            row("LC3 encode SUB 8k 60B", t.enc_sub);
            row("TX submit -> TX-done", t.tx_lat);
            row("TX-done cb -> src_rf awake", t.tx_wake);
            row("esp_now_send() call (CPU)", t.tx_call);
            row("TX gap (radio idle between pkts)", t.tx_gap);
            row("first packet submitted at", t.tx_first_at);
            row("last packet done at (TX end)", t.tx_last_done_at);
            row("all encode jobs done at", t.jobs_done_at);
            print_console("  encode CPU per frame: core0 %lu us (%lu jobs) | core1 %lu us (%lu jobs)\n",
                          (unsigned long)(t.enc_busy_core_us[0] / fr), (unsigned long)t.enc_jobs_core[0],
                          (unsigned long)(t.enc_busy_core_us[1] / fr), (unsigned long)t.enc_jobs_core[1]);
            print_console("  internal heap: free %lu B, min ever %lu B\n",
                          (unsigned long)t.free_internal_heap, (unsigned long)t.min_free_internal_heap);
        }
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    } else if (strcasecmp(line, "log") == 0) {
        // Dump the RAM log ring (oldest first) in console-sized chunks
        static char chunk[257];
        size_t head, total;
        portENTER_CRITICAL(&s_log_ring_mux);
        head = s_log_ring_head;
        total = s_log_ring_wrapped ? sizeof(s_log_ring) : s_log_ring_head;
        portEXIT_CRITICAL(&s_log_ring_mux);
        size_t start = s_log_ring_wrapped ? head : 0;
        print_console("\n----- log ring (%u B) -----\n", (unsigned)total);
        for (size_t off = 0; off < total; off += 256) {
            size_t n = std::min<size_t>(256, total - off);
            for (size_t i = 0; i < n; ++i) {
                chunk[i] = s_log_ring[(start + off + i) % sizeof(s_log_ring)];
            }
            chunk[n] = '\0';
            print_console("%s", chunk);
        }
        print_console("\n----- end of log ring -----\n");
#endif
    } else if (strcasecmp(line, "ptp") == 0) {
        // Time sync status (SOURCE: per-channel estimates; SINK: offset in use and scheduled playout)
        if (s_unicast_engine) {
            int32_t dmin = 0, dmax = 0;
            uint32_t dn = 0;
            s_unicast_engine->getRxTsProbe(dmin, dmax, dn, true);
            const system_config_t* ptp_cfg = get_system_config();
            if (ptp_cfg && ptp_cfg->node_role == NODE_ROLE_SOURCE) {
                print_console("\n[PTP] SOURCE: latency %lu us, control frame every %lu ms, frame-tick jitter vs 10 ms grid "
                              "max %ld us since last 'ptp'\n",
                              (unsigned long)s_unicast_engine->getPlayoutLatencyUs(), 100UL,
                              (long)s_unicast_engine->takeSrcGridDevMax());
                print_console("  ch valid  offset_us(SINK-SRC)  drift_ppm  rtt_us  win  samples  2step_miss  t1_miss  holdover  trim_us  playout_err_us\n");
                for (uint8_t ch = 0; ch < AudioNet::MAX_SINK_NODES; ++ch) {
                    AudioNet::EspNowBroadcastEngine::PtpChannelStatus ps{};
                    s_unicast_engine->getPtpStatus(ch, ps);
                    char err_str[12];
                    if (ps.playout_err_us == AudioNet::VSAF_PLAYOUT_ERR_UNKNOWN) {
                        snprintf(err_str, sizeof(err_str), "-");
                    } else {
                        snprintf(err_str, sizeof(err_str), "%+d", ps.playout_err_us);
                    }
                    print_console("  %u  %-5s %+19ld %+10.3f %7u %4u %8lu %11lu %8lu %9lu %8d %15s\n", ch, ps.valid ? "yes" : "no",
                                  (long)ps.offset_us, ps.drift_ppb / 1000.0f, ps.rtt_us, ps.samples,
                                  (unsigned long)ps.samples_total, (unsigned long)ps.twostep_miss,
                                  (unsigned long)ps.t1_miss, (unsigned long)ps.holdover_uses, ps.trim_us, err_str);
                }
            } else {
                AudioNet::EspNowBroadcastEngine::SinkSyncStatus ss{};
                s_unicast_engine->getSinkSyncStatus(ss, true);
                print_console("\n[PTP] SINK: sync %s, playout %s | offset in use %+ld us, drift %+.3f ppm, sample rtt %u us, "
                              "control age %lu ms (%lu frames)\n",
                              ss.sync_valid ? "VALID" : "none", ss.aligned ? "ALIGNED" : "not aligned", (long)ss.offset_us,
                              ss.drift_ppb / 1000.0f, ss.rtt_us, (unsigned long)ss.ctrl_age_ms, (unsigned long)ss.ctrl_frames);
                print_console("[PTP] SINK: latency %lu us, trim %d us | playout error filtered %+ld us, max |raw| %ld us "
                              "since last 'ptp' | coarse aligns %lu, fine corrections %lu\n",
                              (unsigned long)ss.latency_us, ss.trim_us, (long)ss.err_filt_us, (long)ss.err_max_abs_us,
                              (unsigned long)ss.coarse_aligns, (unsigned long)ss.fine_corrections);
                for (uint32_t k = 0; k < 8; ++k) {
                    const auto& ev = s_unicast_engine->m_coarse_ring[(s_unicast_engine->m_coarse_head + k) % 8];
                    print_console("[PTP]   coarse ev: err %+ld us fifo %u dma_q %d item %u was_aligned %u after %u frames\n",
                                  (long)ev.err_us, ev.fifo, ev.dma_queued, ev.has_item, ev.was_aligned, ev.frames_since);
                }
                print_console("[PTP]   dbg t %lld played %lld written %lld\n", (long long)s_unicast_engine->m_dbg_t.load(),
                              (long long)s_unicast_engine->m_dbg_played.load(), (long long)s_unicast_engine->m_dbg_written.load());
                print_console("[PTP]   dbg dma_q min %ld max %ld\n", (long)s_unicast_engine->m_dbg_q_min.exchange(1 << 30),
                              (long)s_unicast_engine->m_dbg_q_max.exchange(-(1 << 30)));
                print_console("[PTP]   dma rebases %lu\n", (unsigned long)s_unicast_engine->m_dma_rebase.load());
                print_console("[PTP]   unsynced frames %lu\n", (unsigned long)s_unicast_engine->m_sync_unsynced_frames.load());
                print_console("[PTP] SINK: coarse causes: timeline jump %lu, offset jump %lu, late-frame skip %lu, "
                              "head drop %lu, silence pad %lu\n",
                              (unsigned long)ss.why_jump, (unsigned long)ss.why_theta, (unsigned long)ss.why_skip,
                              (unsigned long)ss.why_head, (unsigned long)ss.why_pad);
            }
            if (dn > 0) {
                print_console("[PTP] RX timestamp probe: esp_timer - rx_ctrl.timestamp = %ld .. %ld us over %lu packets\n",
                              (long)dmin, (long)dmax, (unsigned long)dn);
            } else {
                print_console("[PTP] RX timestamp probe: no packets\n");
            }
        }
    } else if (strncasecmp(line, "trim ", 5) == 0) {
        // SOURCE: per-channel playout trim "trim <ch> <us>" (sent to the SINKs in the control frame)
        int ch = -1, us = 0;
        if (sscanf(line + 5, "%d %d", &ch, &us) == 2 && ch >= 0 && ch < static_cast<int>(AudioNet::MAX_SINK_NODES) &&
            s_unicast_engine) {
            us = std::clamp(us, -30000, 30000);
            s_unicast_engine->setChannelTrimUs(static_cast<uint8_t>(ch), static_cast<int16_t>(us));
            print_console("[OK] Channel %d playout trim %+d us\n", ch, us);
        } else {
            print_console("[ERR] usage: trim <ch 0..5> <us>\n");
        }
    } else if (strncasecmp(line, "latency ", 8) == 0) {
        // SOURCE: playout latency (frame time -> presentation time) in ms
        int ms = atoi(line + 8);
        if (ms >= 20 && ms <= 200 && s_unicast_engine) {
            s_unicast_engine->setPlayoutLatencyUs(static_cast<uint32_t>(ms) * 1000U);
            print_console("[OK] Playout latency %d ms\n", ms);
        } else {
            print_console("[ERR] usage: latency <20..200 ms>\n");
        }
    } else if (strncasecmp(line, "syncoff ", 8) == 0) {
        if (s_unicast_engine) {
            s_unicast_engine->m_sync_disable.store(atoi(line + 8) != 0);
            print_console("[OK] syncoff %d\n", s_unicast_engine->m_sync_disable.load() ? 1 : 0);
        }
    } else if (strncasecmp(line, "syncprobe ", 10) == 0) {
        // SINK: print [SYNC] self-test lines once per second (time base + playout of a common frame)
        if (s_unicast_engine) {
            s_unicast_engine->setSyncProbe(atoi(line + 10) != 0);
            print_console("[OK] syncprobe %s\n", s_unicast_engine->getSyncProbe() ? "ON" : "OFF");
        }
    } else if (strcasecmp(line, "usjdiag") == 0) {
        // USB-Serial-JTAG console health: this session and the session before the last reset
        print_usj_diag("this session", s_usj);
        if (s_usj_prev_valid) {
            print_usj_diag("previous session", s_usj_prev);
        } else {
            print_console("[USJ] previous session: none (power-on boot)\n");
        }
    } else if (strcasecmp(line, "tasks") == 0) {
        print_task_cpu_window();
    } else if (strcasecmp(line, "mem") == 0) {
        // Internal RAM budget and SOURCE pipeline task health
        print_console("[MEM] internal heap: free %lu B | min ever %lu B | largest block %lu B\n",
                      (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                      (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                      (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        if (s_unicast_engine) {
            print_console("[MEM] at start(): heap before tasks %lu B, after %lu B | created: src_rf=%d src_enc1=%d src_dsp0=%d\n",
                          (unsigned long)s_unicast_engine->m_boot_heap_before_tasks,
                          (unsigned long)s_unicast_engine->m_boot_heap_after_tasks,
                          s_unicast_engine->m_boot_task_ok[0], s_unicast_engine->m_boot_task_ok[1],
                          s_unicast_engine->m_boot_task_ok[2]);
            static const char* names[3] = {"src_rf", "src_enc1", "src_dsp0"};
            for (int i = 0; i < 3; ++i) {
                TaskHandle_t h = s_unicast_engine->getSrcTaskHandle(i);
                if (h) {
                    print_console("[MEM] %-8s stack high-water mark: %lu B free\n", names[i],
                                  (unsigned long)(uxTaskGetStackHighWaterMark(h) * sizeof(StackType_t)));
                }
            }
        }
    } else if (strncasecmp(line, "dspload ", 8) == 0) {
        // Synthetic DSP load on core 0 (busy wait) to test headroom for future spatial DSP
        int us = atoi(line + 8);
        if (us < 0) us = 0;
        if (us > 9000) us = 9000;
        if (s_unicast_engine) {
            s_unicast_engine->setDspExtraLoadUs(static_cast<uint32_t>(us));
            print_console("[OK] Synthetic DSP load set to %d us per frame\n", us);
        }
    } else if (strncasecmp(line, "encbench", 8) == 0 && (line[8] == '\0' || line[8] == ' ')) {
        // Debug: LC3 encode cost per test signal. "encbench [0|1] [irq]": core (default 1); irq = interrupts enabled
        int core = 1;
        const char* arg = line + 8;
        while (*arg == ' ') ++arg;
        if (*arg == '0' || *arg == '1') {
            core = *arg - '0';
            ++arg;
        }
        const bool irq = strstr(arg, "irq") != nullptr;
        if (s_unicast_engine) {
            s_unicast_engine->runEncoderBench(print_console, core, !irq);
        }
    } else if (strncasecmp(line, "enccores ", 9) == 0) {
        // Debug: run all LC3 jobs on core 0 only (1) or on both cores (2)
        int n = atoi(line + 9);
        if (s_unicast_engine) {
            s_unicast_engine->setEncoderCores(static_cast<uint32_t>(n));
            print_console("[OK] LC3 encoder cores: %lu\n", (unsigned long)s_unicast_engine->getEncoderCores());
        }
    } else if (strcasecmp(line, "txfault wifi") == 0 || strcasecmp(line, "txfault reboot") == 0) {
        // Test hook: every send fails with NO_MEM until a Wi-Fi restart ('wifi') or never ('reboot'),
        // to exercise the recovery escalation ESP-NOW re-init -> Wi-Fi restart -> reboot
        if (s_unicast_engine) {
            const bool wifi = strcasecmp(line + 8, "wifi") == 0;
            s_unicast_engine->simulateTxWedge(wifi ? 1 : 2);
            print_console("[OK] Simulated TX wedge (%s): expect 2 ESP-NOW re-inits, then %s\n", line + 8,
                          wifi ? "a Wi-Fi restart that heals it" : "3 Wi-Fi restarts and a reboot");
        }
    } else if (strcasecmp(line, "txfault") == 0) {
        // Test hook: deinit ESP-NOW at the next frame boundary; the TX health check must recover within ~1 s
        if (s_unicast_engine) {
            s_unicast_engine->requestTxFaultInjection();
            print_console("[OK] TX fault injected at next frame boundary; expect automatic ESP-NOW recovery\n");
        }
    } else if (strcasecmp(line, "txstat") == 0) {
        // TX path health: distinguishes "sent and completed" from "esp_now_send() failing instantly"
        if (s_unicast_engine) {
            uint8_t primary = 0;
            wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
            esp_wifi_get_channel(&primary, &second);
            int32_t last_err = s_unicast_engine->getLastTxError();
            print_console("[TXSTAT] ch set=%u radio=%u | phy=%s | txpwr=%.2f dBm (setting; radio quantizes it)\n",
                          s_unicast_engine->getWifiChannel(), primary, s_unicast_engine->getWifiPhyRateString(),
                          s_unicast_engine->getWifiTxPower() / 4.0f);
            print_console("[TXSTAT] recoveries=%lu (Wi-Fi restarts %lu, wedge reboots since power-on %lu)\n",
                          (unsigned long)s_unicast_engine->getTxRecoveryCount(),
                          (unsigned long)s_unicast_engine->getWifiRestartCount(),
                          (unsigned long)AudioNet::EspNowBroadcastEngine::getTxWedgeReboots());
            print_console("[TXSTAT] send_err=%lu (no_mem=%lu, last=%s) tx_done_timeouts=%lu tx_status_fail=%lu deadline_drops=%lu\n",
                          (unsigned long)s_unicast_engine->getTxMacErrorCount(),
                          (unsigned long)s_unicast_engine->getTxNoMemCount(),
                          last_err ? esp_err_to_name(static_cast<esp_err_t>(last_err)) : "none",
                          (unsigned long)s_unicast_engine->getTxTimeoutCount(),
                          (unsigned long)s_unicast_engine->getTxFailCount(),
                          (unsigned long)s_unicast_engine->getTxDeadlineDropsCount());
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
        usb_reboot_to_rom_download(); // force-download flag + ROM system reset; never esp_restart()
#else
        esp_restart();
#endif
    } else {
        print_console("[UNKNOWN CMD] '%s'. Type 'help' for command list.\n", line);
    }
}

