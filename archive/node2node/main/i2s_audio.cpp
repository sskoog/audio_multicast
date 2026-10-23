#include "status_led.hpp"
#include "i2s_audio.hpp"
#include "esp_log.h"
#include "driver/i2s_std.h"

static const char* TAG = "I2S_AUDIO";

namespace Hardware {

static IRAM_ATTR bool i2s_dma_tx_done_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_arg) {
    auto* driver = static_cast<I2sAudioDriver*>(user_arg);
    BaseType_t high_task_wakeup = pdFALSE;
    if (driver) {
        driver->notifyDmaSlotFreeFromISR(&high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

I2sAudioDriver::I2sAudioDriver() {
    m_dma_free_sem = xSemaphoreCreateBinary();
}

I2sAudioDriver::~I2sAudioDriver() {
    deinit();
    if (m_dma_free_sem) {
        vSemaphoreDelete(m_dma_free_sem);
        m_dma_free_sem = nullptr;
    }
}

esp_err_t I2sAudioDriver::init(uint32_t sample_rate_hz, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, int gain_pin, uint32_t frame_duration_us, uint8_t channels) {
    if (m_initialized) {
        deinit();
    }

    m_sample_rate = sample_rate_hz;
    m_frame_duration_us = frame_duration_us;
    m_channels = channels;
    m_bclk = bclk;
    m_ws = ws;
    m_dout = dout;

    // Configure Hardware Gain pin if assigned (e.g. GP0 on Waveshare Zero)
    if (gain_pin >= 0) {
        m_gain_pin = static_cast<gpio_num_t>(gain_pin);
        setGain(GAIN_12DB); // Default to 12 dB gain for speaker
        ESP_LOGI(TAG, "Configured MAX98357A GAIN pin on GPIO %d (Set to 12 dB)", gain_pin);
    } else {
        m_gain_pin = GPIO_NUM_NC;
    }

    // Dual-Descriptor DMA Configuration (2 x 1 Frame Capacity = 20ms @ 10ms, or 15ms @ 7.5ms)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2; // Exactly 2 descriptors (Dual-descriptor ping-pong)
    chan_cfg.dma_frame_num = calculateDmaFrameNum(sample_rate_hz, frame_duration_us);
    chan_cfg.auto_clear = true;

    i2s_chan_handle_t tx_handle = nullptr;
    esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle, nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }
    m_tx_chan = tx_handle;

    // Register DMA on_sent interrupt callback to trigger the LC3 decoder on free descriptor
    i2s_event_callbacks_t cbs = {
        .on_recv = nullptr,
        .on_recv_q_ovf = nullptr,
        .on_sent = i2s_dma_tx_done_cb,
        .on_send_q_ovf = nullptr,
    };
    i2s_channel_register_event_callback(tx_handle, &cbs, this);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode: %s", esp_err_to_name(ret));
        i2s_del_channel(tx_handle);
        m_tx_chan = nullptr;
        return ret;
    }

    m_initialized = true;
    m_running = false; // Keep clock idle until first audio descriptor is full!

    ESP_LOGI(TAG, "I2S Dual-Descriptor DMA Driver Initialized: Fs=%lu Hz, Duration=%lu us (%u samples/desc, 2x DMA depth), BCLK=%d, WS=%d, DOUT=%d",
             m_sample_rate, m_frame_duration_us, (unsigned)chan_cfg.dma_frame_num, static_cast<int>(bclk), static_cast<int>(ws), static_cast<int>(dout));
    return ESP_OK;
}

esp_err_t I2sAudioDriver::reconfigurePipeline(uint32_t sample_rate_hz, uint32_t frame_duration_us, uint8_t channels) {
    if (!m_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (m_sample_rate == sample_rate_hz && m_frame_duration_us == frame_duration_us && m_channels == channels) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Reconfiguring I2S DMA Pipeline: Fs=%lu Hz, Duration=%lu us, Channels=%u",
             sample_rate_hz, frame_duration_us, channels);

    return init(sample_rate_hz, m_bclk, m_ws, m_dout, static_cast<int>(m_gain_pin), frame_duration_us, channels);
}

esp_err_t I2sAudioDriver::preload(const int16_t* pcm_data, size_t samples_count, size_t* bytes_loaded) {
    if (!m_initialized || !m_tx_chan || !pcm_data) {
        return ESP_ERR_INVALID_STATE;
    }
    auto tx_handle = static_cast<i2s_chan_handle_t>(m_tx_chan);
    size_t bytes_to_load = samples_count * sizeof(int16_t);
    return i2s_channel_preload_data(tx_handle, pcm_data, bytes_to_load, bytes_loaded);
}

esp_err_t I2sAudioDriver::start() {
    if (!m_initialized || !m_tx_chan) return ESP_ERR_INVALID_STATE;
    if (m_running) return ESP_OK;

    auto tx_handle = static_cast<i2s_chan_handle_t>(m_tx_chan);
    // Clear any stale tokens before starting
    if (m_dma_free_sem) {
        xSemaphoreTake(m_dma_free_sem, 0);
    }
    esp_err_t ret = i2s_channel_enable(tx_handle);
    if (ret == ESP_OK) {
        m_running = true;
        ESP_LOGI(TAG, "I2S Hardware Clock Started (Playing from DMA descriptor).");
    }
    return ret;
}

esp_err_t I2sAudioDriver::stop() {
    if (!m_initialized || !m_tx_chan) return ESP_ERR_INVALID_STATE;
    if (!m_running) return ESP_OK;

    auto tx_handle = static_cast<i2s_chan_handle_t>(m_tx_chan);
    esp_err_t ret = i2s_channel_disable(tx_handle);
    if (ret == ESP_OK) {
        m_running = false;
        if (m_dma_free_sem) {
            xSemaphoreTake(m_dma_free_sem, 0); // Clear any pending tokens
        }
        ESP_LOGI(TAG, "I2S Hardware Clock Stopped (Standby / Idle).");
    }
    return ret;
}

esp_err_t I2sAudioDriver::setSampleRate(uint32_t sample_rate_hz) {
    if (!m_initialized || !m_tx_chan) return ESP_ERR_INVALID_STATE;
    if (m_sample_rate == sample_rate_hz) return ESP_OK;

    bool was_running = m_running;
    if (was_running) {
        stop();
    }

    auto tx_handle = static_cast<i2s_chan_handle_t>(m_tx_chan);
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
    esp_err_t ret = i2s_channel_reconfig_std_clock(tx_handle, &clk_cfg);
    if (ret == ESP_OK) {
        m_sample_rate = sample_rate_hz;
        ESP_LOGI(TAG, "Reconfigured I2S Sample Rate to %lu Hz", m_sample_rate);
    }

    if (was_running) {
        start();
    }
    return ret;
}

void I2sAudioDriver::deinit() {
    if (m_initialized && m_tx_chan) {
        auto tx_handle = static_cast<i2s_chan_handle_t>(m_tx_chan);
        if (m_running) {
            i2s_channel_disable(tx_handle);
            m_running = false;
        }
        i2s_del_channel(tx_handle);
        m_tx_chan = nullptr;
        m_initialized = false;
        ESP_LOGI(TAG, "I2S DAC Driver Deinitialized.");
    }
}

void I2sAudioDriver::setGain(Max98357Gain gain) {
    if (m_gain_pin == GPIO_NUM_NC) return;

    if (gain == GAIN_9DB) {
        gpio_set_direction(m_gain_pin, GPIO_MODE_INPUT);
        ESP_LOGI(TAG, "Set MAX98357A GAIN to 9 dB (Floating)");
    } else {
        gpio_set_direction(m_gain_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(m_gain_pin, (gain == GAIN_12DB) ? 1 : 0);
        ESP_LOGI(TAG, "Set MAX98357A GAIN to %s (%d dB)", 
                 (gain == GAIN_12DB) ? "12 dB (HIGH)" : "6 dB (LOW)", 
                 (gain == GAIN_12DB) ? 12 : 6);
    }
}

esp_err_t I2sAudioDriver::write(const int16_t* pcm_data, size_t samples_count, size_t* bytes_written, uint32_t timeout_ms) {
    if (!m_initialized || !m_tx_chan || !pcm_data) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!m_running) {
        start();
    }
    auto tx_handle = static_cast<i2s_chan_handle_t>(m_tx_chan);
    size_t bytes_to_write = samples_count * sizeof(int16_t);
    esp_err_t ret = i2s_channel_write(tx_handle, pcm_data, bytes_to_write, bytes_written, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        incrementUnderrunCount();
    }
    return ret;
}

bool I2sAudioDriver::waitForDmaSlot(uint32_t timeout_ms) {
    if (!m_dma_free_sem) return false;
    return xSemaphoreTake(m_dma_free_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void I2sAudioDriver::notifyDmaSlotFreeFromISR(BaseType_t* high_task_wakeup) {
    if (m_dma_free_sem) {
        xSemaphoreGiveFromISR(m_dma_free_sem, high_task_wakeup);
    }
}

void I2sAudioDriver::incrementUnderrunCount() {
    m_dma_underrun_count.fetch_add(1, std::memory_order_relaxed);
    getStatusLed().triggerUnderrunFlash(200);
}

} // namespace Hardware
