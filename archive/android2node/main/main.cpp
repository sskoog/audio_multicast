#include <cstdio>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "i2s_dac.hpp"
#include "dsp_filter.hpp"
#include "ble_audio_receiver.hpp"
#include "diagnostics.hpp"
#include "ota_manager.hpp"
#include "lcd_console.hpp"

static const char* TAG = "MAIN_APP";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Waveshare ESP32-C6 LCD BLE Audio Receiver Application...");

    // 1. Initialize Waveshare ESP32-C6-LCD-1.47 (ST7789 320x172 Landscape Display & GPIO 8 RGB LED)
    static Hardware::LcdConsole lcd_console;
    esp_err_t ret = lcd_console.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ST7789 LCD Console: %s", esp_err_to_name(ret));
    }

    // 2. Initialize Wi-Fi 6 Station Mode & OTA Firmware Update Manager
    static System::OtaManager ota_mgr;
    ota_mgr.initWifi();

    // 3. Initialize MAX98357A I²S Audio Output Driver (GPIO 19 BCLK, GPIO 20 WS, GPIO 21 DOUT)
    static Hardware::I2sDacDriver i2s_dac;
    ret = i2s_dac.init(AUDIO_SAMPLE_RATE_DEFAULT_HZ, 
                       static_cast<gpio_num_t>(I2S_DAC_BCLK_PIN), 
                       static_cast<gpio_num_t>(I2S_DAC_WS_PIN), 
                       static_cast<gpio_num_t>(I2S_DAC_DOUT_PIN));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MAX98357A I²S DAC: %s", esp_err_to_name(ret));
    }

    // 4. Initialize 2nd-Order 80 Hz High-Pass DSP Filter for Top Speaker (esp-dsp Q31 fixed-point math)
    static Audio::DspHighPassFilter dsp_hpf;
    dsp_hpf.init(HP_FILTER_CUTOFF_FREQ_HZ, static_cast<float>(AUDIO_SAMPLE_RATE_DEFAULT_HZ));

    // 5. Initialize Bluetooth 5.3 LE Audio Receiver (LC3 fixed-point decoding for Google Pixel 10 transmitter)
    static Bluetooth::BleAudioReceiver ble_receiver(dsp_hpf, i2s_dac);
    ret = ble_receiver.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE Audio Receiver: %s", esp_err_to_name(ret));
    }

    // 6. Initialize 2 Hz System Diagnostic Telemetry Monitor (USB Serial & LCD Console)
    static Diagnostics::DiagnosticMonitor diag_monitor(ble_receiver, dsp_hpf, &lcd_console);
    diag_monitor.start();

    ESP_LOGI(TAG, "Waveshare ESP32-C6 BLE Audio Receiver Initialization Complete.");

    // Background Audio Processing Loop
    uint8_t dummy_lc3_packet[40] = {0};
    while (true) {
        // Process incoming LC3 audio frames and write filtered output to I²S DMA
        ble_receiver.processAudioFrame(dummy_lc3_packet, sizeof(dummy_lc3_packet));
        vTaskDelay(pdMS_TO_TICKS(AUDIO_FRAME_DURATION_MS)); // 10 ms audio frame period
    }
}
