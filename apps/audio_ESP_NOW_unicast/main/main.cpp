#include "config.h"
#include "console.hpp"
#include "lc3_codec.hpp"
#include "tone_generator.hpp"
#include "i2s_audio.hpp"
#include "espnow_unicast_engine.hpp"
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
// static Benchmark::Lc3BenchmarkSuite*   s_bench_suite = nullptr; (Disabled)

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

// ASCII CLI command handling and MAC address parsing have been broken out to console.cpp


// Background High-Speed USB / UART CLI Reader Task on Core 0
static void usb_serial_cli_task(void* pvParameters) {
    char line_buf[128];
    size_t line_idx = 0;

    bool is_s3_source = false;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const system_config_t* cfg = get_system_config();
    is_s3_source = (cfg && cfg->node_role == NODE_ROLE_SOURCE);
#endif

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    // On ESP32-S3 SOURCE, TinyUSB owns the USB OTG PHY. Do NOT install USB-Serial-JTAG driver!
    if (!is_s3_source) {
        usb_serial_jtag_driver_config_t jtag_cfg = {
            .tx_buffer_size = 512,
            .rx_buffer_size = 1024,
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
    uart_driver_install(UART_NUM_0, 1024, 512, 0, NULL, 0);

    print_console("\n[CONSOLE READY] CLI command input active on USB-Serial and UART0 (%d baud).\n", uart_baud);

    uint8_t rx_buf[128];
    while (true) {
        int n_read = 0;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
        if (!is_s3_source) {
            n_read = usb_serial_jtag_read_bytes(rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(1));
        }
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (is_s3_source) {
            if (tud_cdc_available()) {
                n_read = tud_cdc_read(rx_buf, sizeof(rx_buf));
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
#else
        vTaskDelay(pdMS_TO_TICKS(1));
#endif

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

    // 6. Benchmark Suite (Disabled)
    // s_bench_suite = new Benchmark::Lc3BenchmarkSuite(s_lc3_codec);

    // Initialize console command handler with active engine
    console_init(s_unicast_engine);

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
