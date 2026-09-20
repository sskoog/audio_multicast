#include "config.h"
#include "console.hpp"
#include "lc3_codec.hpp"
#include "tone_generator.hpp"
#include "i2s_audio.hpp"
#include "vsaf_broadcast_engine.hpp"
#include "status_led.hpp"
#include "button.hpp"
#include "diagnostics.hpp"
// #include "lc3_benchmark.hpp" (Disabled)
#include "usb_audio.hpp"
#include "stats_test.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "tusb.h"
#include "soc/rtc_cntl_reg.h"
#endif
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED)
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
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

/**
 * @brief Known hardware node descriptor in the cluster registry.
 */
struct KnownNodeDescriptor {
    uint8_t     mac[6];          // 6-byte IEEE 802.3 MAC address
    uint8_t     node_id;         // Hardware Node ID (16, 20, 21, 23, 24, 25, 26)
    uint8_t     target_channel;  // Assigned audio channel index (0..5)
    const char* channel_name;    // Human-readable channel designation
    const char* board_name;      // Physical board description
};

/**
 * @brief Pre-defined library of known cluster nodes and their factory MAC addresses.
 */
static constexpr KnownNodeDescriptor KNOWN_NODE_LIBRARY[] = {
    // SINK Nodes (Audio Channels 0 to 5)
    { {0xB0, 0xA6, 0x04, 0x99, 0x38, 0x44}, 23, 0, "Left",            "Waveshare ESP32-C6-Zero" },
    { {0xB0, 0xA6, 0x04, 0x99, 0x18, 0xE4}, 24, 1, "Right",           "Waveshare ESP32-C6-Zero" },
    { {0x98, 0xA3, 0x16, 0x9D, 0x57, 0xEC}, 21, 2, "Center",          "ESP32-C6-WROOM-1 DevKit" },
    { {0xE8, 0x3D, 0xC1, 0xFB, 0xDC, 0xC4}, 25, 3, "Surround Left",   "Heemol ESP32-C6 Mini"    },
    { {0x98, 0xA3, 0x16, 0xAC, 0x13, 0x38}, 26, 4, "Surround Right",  "Heemol ESP32-C6 Mini"    },
    { {0xE8, 0x3D, 0xC1, 0xFB, 0xE8, 0x3C},  4, 4, "Surround Right",  "XIAO ESP32-S3 PCM5102A"  },
    { {0xAC, 0xEB, 0xE6, 0x23, 0xDC, 0x24}, 20, 5, "Subwoofer",       "Waveshare ESP32-C6-LCD"  },

    // SOURCE Node (Master Broadcaster)
    { {0xE0, 0x72, 0xA1, 0xD8, 0x4C, 0xD0}, 16, 0xFF, "Source Master", "Seeed Studio XIAO ESP32-S3" },
};

/**
 * @brief Compare two 6-byte MAC addresses for exact equality.
 *
 * @param mac_a Pointer to first 6-byte MAC address (const uint8_t*, input)
 * @param mac_b Pointer to second 6-byte MAC address (const uint8_t*, input)
 * @return true if all 6 bytes match identically, false otherwise.
 */
static inline bool is_matching_mac(const uint8_t* mac_a, const uint8_t* mac_b) {
    return (mac_a != nullptr && mac_b != nullptr) && (std::memcmp(mac_a, mac_b, 6) == 0);
}

/**
 * @brief Search the known node library for an entry matching the provided MAC address.
 *
 * @param mac 6-byte hardware MAC address to look up (const uint8_t*, input)
 * @return const KnownNodeDescriptor* Pointer to matched entry in library, or nullptr if unrecognized.
 */
static const KnownNodeDescriptor* find_known_node_by_mac(const uint8_t* mac) {
    if (!mac) return nullptr;
    for (const auto& node : KNOWN_NODE_LIBRARY) {
        if (is_matching_mac(mac, node.mac)) {
            return &node;
        }
    }
    return nullptr;
}

static Hardware::StatusLed*            s_status_led = nullptr;
static Hardware::Button*               s_user_button = nullptr;
static Hardware::I2sAudioDriver*       s_i2s_dac = nullptr;
static Codec::Lc3CodecEngine           s_lc3_codec;
static Audio::ToneGenerator            s_tone_gen;
static AudioNet::EspNowUnicastEngine*  s_unicast_engine = nullptr;
static Diagnostics::SystemDiagnostics* s_diagnostics = nullptr;
// static Benchmark::Lc3BenchmarkSuite*   s_bench_suite = nullptr; (Disabled)

static void on_user_button_pressed(void* user_data) {
    if (!s_unicast_engine) return;

    const system_config_t* cfg = get_system_config();
    AudioNet::NetworkState current_state = s_unicast_engine->getState();

    if (cfg->node_role == NODE_ROLE_SOURCE) {
        bool is_synth = s_unicast_engine->isToneTestMode();
        if (current_state == AudioNet::NetworkState::IDLE) {
            // Mode 1 (IDLE) -> Mode 2 (CAST from live USB audio stream)
            s_unicast_engine->setToneTestMode(false);
            s_unicast_engine->transitionTo(AudioNet::NetworkState::CAST);
            ESP_LOGI(TAG, ">>> BUTTON: SOURCE Mode 1/3 -> CAST (Live USB Audio Streaming) <<<");
        } else if (!is_synth) {
            // Mode 2 (CAST USB) -> Mode 3 (CAST using synth tone -> Dual LC3 Encoders)
            s_unicast_engine->setToneTestMode(true);
            ESP_LOGI(TAG, ">>> BUTTON: SOURCE Mode 2/3 -> CAST (Internal Synth Tone -> Dual LC3 Encoders) <<<");
        } else {
            // Mode 3 (CAST synth) -> Mode 1 (IDLE)
            s_unicast_engine->setToneTestMode(false);
            s_unicast_engine->transitionTo(AudioNet::NetworkState::IDLE);
            ESP_LOGI(TAG, ">>> BUTTON: SOURCE Mode 3/3 -> IDLE (Streaming Paused) <<<");
        }
    } else {
        if (current_state == AudioNet::NetworkState::IDLE) {
            ESP_LOGI(TAG, ">>> BUTTON: SINK -> SCANNING (Resuming audio receiver) <<<");
            s_unicast_engine->transitionTo(AudioNet::NetworkState::SCANNING);
        } else {
            ESP_LOGI(TAG, ">>> BUTTON: SINK -> IDLE (Muting receiver) <<<");
            s_unicast_engine->transitionTo(AudioNet::NetworkState::IDLE);
        }
    }
}

// ASCII CLI command handling and MAC address parsing have been broken out to console.cpp


// Background High-Speed USB / UART CLI Reader Task on Core 1
static void usb_serial_cli_task(void* pvParameters) {
    char line_buf[128];
    size_t line_idx = 0;

    bool is_s3_source = false;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const system_config_t* cfg = get_system_config();
    is_s3_source = (cfg && cfg->node_role == NODE_ROLE_SOURCE);
#endif

    print_console("\n[CONSOLE READY] CLI command input active on USB-Serial and UART0 (2000000 baud).\n");

    uint8_t rx_buf[128];
    while (true) {
        int n_read = 0;
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED)
        if (!is_s3_source) {
            int r = read(STDIN_FILENO, rx_buf, sizeof(rx_buf));
            if (r > 0) {
                n_read = r;
            }
        }
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (is_s3_source) {
            if (tud_cdc_available()) {
                n_read = tud_cdc_read(rx_buf, sizeof(rx_buf));
            }
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(5));

        // Also check UART0
        int n_uart = uart_read_bytes(UART_NUM_0, rx_buf + n_read, sizeof(rx_buf) - n_read, 0);
        if (n_uart > 0) {
            n_read += n_uart;
        }

        for (int i = 0; i < n_read; ++i) {
            char c = static_cast<char>(rx_buf[i]);
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
        }
    }
}

/**
 * @brief Background 10 Hz system diagnostics & telemetry loop pinned to Core 1
 */
static void sys_diag_task(void* pvParameters) {
    while (true) {
        if (s_diagnostics) {
            s_diagnostics->tick();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms = 10 Hz
    }
}

static void configure_unused_gpios_pulldown(const system_config_t* cfg) {
    uint32_t pulled_count = 0;
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    // ESP32-C6 GPIOs: 0..30
    for (int pin = 0; pin <= 30; ++pin) {
        // Critical system pins to NEVER touch:
        // SPI Flash: GPIO 24..30 (SPICS0, SPIQ, SPIWP, VDD_SPI, SPIHD, SPICLK, SPID)
        // USB Serial/JTAG: GPIO 12, 13
        // UART0 Console: GPIO 16, 17
        if (pin >= 24 && pin <= 30) continue;
        if (pin == 12 || pin == 13) continue;
        if (pin == 16 || pin == 17) continue;

        // Configured active peripherals:
        if (pin == cfg->status_led_gpio) continue;
        if (pin == cfg->user_button_gpio) continue;
        if (pin == cfg->i2s_bclk_gpio) continue;
        if (pin == cfg->i2s_ws_gpio) continue;
        if (pin == cfg->i2s_dout_gpio) continue;
        if (cfg->node_role == NODE_ROLE_SINK && pin == 0) continue; // MAX98357A GAIN pin

        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        pulled_count++;
    }
    ESP_LOGI("GPIO_INIT", "ESP32-C6: Configured %lu unused GPIOs as INPUT with weak PULL-DOWN", (unsigned long)pulled_count);

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3 GPIOs: 0..48 (excluding non-existent 22..25)
    for (int pin = 0; pin <= 48; ++pin) {
        // Non-existent silicon pins on S3
        if (pin >= 22 && pin <= 25) continue;

        // Critical system pins to NEVER touch:
        // Flash & Octal PSRAM: GPIO 26..37, 47, 48
        // USB Native CDC: GPIO 19, 20
        // UART0 Console: GPIO 43, 44
        // JTAG / Wio-SX1262 LoRa B2B: GPIO 38..42, 7..9
        if (pin >= 26 && pin <= 37) continue;
        if (pin == 47 || pin == 48) continue;
        if (pin == 19 || pin == 20) continue;
        if (pin == 43 || pin == 44) continue;
        if (pin >= 38 && pin <= 42) continue;
        if (pin >= 7 && pin <= 9) continue;

        // Configured active peripherals:
        if (pin == cfg->status_led_gpio) continue;
        if (pin == cfg->user_button_gpio) continue;
        if (pin == cfg->i2s_bclk_gpio) continue;
        if (pin == cfg->i2s_ws_gpio) continue;
        if (pin == cfg->i2s_dout_gpio) continue;
        if (pin == cfg->amp_mute_gpio) continue;

        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        pulled_count++;
    }
    ESP_LOGI("GPIO_INIT", "ESP32-S3: Configured %lu unused GPIOs as INPUT with weak PULL-DOWN", (unsigned long)pulled_count);
#endif
}

extern "C" void app_main(void) {
    const system_config_t* cfg = get_system_config();

#if defined(CONFIG_IDF_TARGET_ESP32C6)
    if (cfg->node_role == NODE_ROLE_SINK) {
        USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 1;
    }
#endif

    setvbuf(stdout, NULL, _IONBF, 0);

    // 0. Configure Task Watchdog Timer (TWDT) to 1.0 second (1000 ms)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 1000,
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        .idle_core_mask = (1 << 0), // Core 1 is dedicated real-time audio pump; monitor Core 0
#else
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
#endif
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

    configure_unused_gpios_pulldown(cfg);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Status LED
    s_status_led = &Hardware::getStatusLed();
    s_status_led->init(cfg->status_led_gpio, cfg->status_led_num, (cfg->status_led_gpio == 21));

    // 2. User Button
    if (cfg->user_button_gpio >= 0) {
        s_user_button = new Hardware::Button(cfg->user_button_gpio, true, 100);
        s_user_button->init(on_user_button_pressed, nullptr);
    }

    // 3. I2S DAC (for SINK node)
    if (cfg->node_role == NODE_ROLE_SINK) {
        s_i2s_dac = new Hardware::I2sAudioDriver(
            cfg->i2s_bclk_gpio,
            cfg->i2s_ws_gpio,
            cfg->i2s_dout_gpio,
            -1,
            cfg->is_pcm5102a ? -1 : 0,
            cfg->amp_mute_gpio
        );
        s_i2s_dac->init(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 10000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        if (!cfg->is_pcm5102a && cfg->max98357a_gain_db >= 0) {
            s_i2s_dac->setHardwareGain(static_cast<Hardware::Max98357Gain>(cfg->max98357a_gain_db));
        }
    }

    // 4. Unicast Engine
    s_unicast_engine = new AudioNet::EspNowUnicastEngine(s_lc3_codec, &s_tone_gen, s_i2s_dac);

    // Auto-detect SINK channel based on factory MAC address
    if (cfg->node_role == NODE_ROLE_SINK) {
        uint8_t base_mac[6] = {0};
        esp_read_mac(base_mac, ESP_MAC_WIFI_STA);

        const KnownNodeDescriptor* matched_node = find_known_node_by_mac(base_mac);
        if (matched_node && matched_node->target_channel <= 5) {
            s_unicast_engine->setTargetChannel(matched_node->target_channel);
            ESP_LOGI(TAG, "Recognized node %d (%s) -> Assigned Channel %d (%s) [MAC: %02X:%02X:%02X:%02X:%02X:%02X]",
                     matched_node->node_id, matched_node->board_name,
                     matched_node->target_channel, matched_node->channel_name,
                     base_mac[0], base_mac[1], base_mac[2], base_mac[3], base_mac[4], base_mac[5]);
        } else {
            s_unicast_engine->setTargetChannel(0); // Fallback to Left speaker
            ESP_LOGW(TAG, "Unrecognized SINK MAC %02X:%02X:%02X:%02X:%02X:%02X! Defaulting to Channel 0 (Left)",
                     base_mac[0], base_mac[1], base_mac[2], base_mac[3], base_mac[4], base_mac[5]);
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

    // 6. Benchmark Suite (Disabled)
    // s_bench_suite = new Benchmark::Lc3BenchmarkSuite(s_lc3_codec);

    // Initialize console command handler with active engine
    console_init(s_unicast_engine);

    // Run stats component self-test suite
    run_stats_self_test();

    // 7. Configure non-blocking VFS for USB-Serial-JTAG and install UART0 driver
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED)
    if (cfg->node_role == NODE_ROLE_SINK) {
        usb_serial_jtag_vfs_use_nonblocking();
    }
#endif

    uart_config_t uart_cfg = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_driver_install(UART_NUM_0, 1024, 512, 0, NULL, 0);

    // 8. Start Background CLI Task on Core 1 (Core 0 on single-core targets)
    const BaseType_t app_core = (SOC_CPU_CORES_NUM > 1) ? 1 : 0;
    xTaskCreatePinnedToCore(usb_serial_cli_task, "cli_task", 4096, nullptr, 2, nullptr, app_core);

    // 9. Start Background Diagnostics Task on Core 1 (Core 0 on single-core targets)
    xTaskCreatePinnedToCore(sys_diag_task, "sys_diag", 4096, nullptr, 1, nullptr, app_core);

    ESP_LOGI(TAG, "Device Node ID: %d | Role: %s | Name: %s",
             cfg->node_id,
             (cfg->node_role == NODE_ROLE_SOURCE) ? "SOURCE (Transmitter)" : "SINK (Receiver)",
             cfg->device_name);
    if (cfg->node_role == NODE_ROLE_SOURCE) {
        ESP_LOGI(TAG, "System initialization complete! SOURCE active in CAST state (USB Audio streaming default).");
    } else {
        ESP_LOGI(TAG, "System initialization complete! SINK listening/scanning.");
    }

    // Delete startup main_task to free memory (sys_diag and cli_task now manage runtime on Core 1)
    vTaskDelete(nullptr);
}
