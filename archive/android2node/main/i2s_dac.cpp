#include "i2s_dac.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "I2S_DAC";

namespace Hardware {

I2sDacDriver::I2sDacDriver() = default;

I2sDacDriver::~I2sDacDriver() {
    disable();
    if (m_tx_handle) {
        i2s_del_channel(m_tx_handle);
        m_tx_handle = nullptr;
    }
}

esp_err_t I2sDacDriver::init(uint32_t sample_rate, gpio_num_t bclk_pin, gpio_num_t ws_pin, gpio_num_t dout_pin) {
    m_sample_rate = sample_rate;

    ESP_LOGI(TAG, "Initializing MAX98357A I²S Output (BCLK=GPIO%d, WS=GPIO%d, DOUT=GPIO%d, fs=%lu Hz)",
             bclk_pin, ws_pin, dout_pin, sample_rate);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &m_tx_handle, nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I²S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk_pin,
            .ws = ws_pin,
            .dout = dout_pin,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(m_tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I²S standard mode: %s", esp_err_to_name(ret));
        return ret;
    }

    return enable();
}

esp_err_t I2sDacDriver::enable() {
    if (m_tx_handle && !m_is_enabled) {
        esp_err_t ret = i2s_channel_enable(m_tx_handle);
        if (ret == ESP_OK) {
            m_is_enabled = true;
            ESP_LOGI(TAG, "MAX98357A I²S Driver Enabled.");
        }
        return ret;
    }
    return ESP_OK;
}

esp_err_t I2sDacDriver::disable() {
    if (m_tx_handle && m_is_enabled) {
        esp_err_t ret = i2s_channel_disable(m_tx_handle);
        if (ret == ESP_OK) {
            m_is_enabled = false;
            ESP_LOGI(TAG, "MAX98357A I²S Driver Disabled.");
        }
        return ret;
    }
    return ESP_OK;
}

esp_err_t I2sDacDriver::write(const int16_t* pcm_data, size_t data_size, size_t* bytes_written, uint32_t timeout_ms) {
    if (!m_tx_handle || !m_is_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2s_channel_write(m_tx_handle, pcm_data, data_size, bytes_written, pdMS_TO_TICKS(timeout_ms));
}

} // namespace Hardware
