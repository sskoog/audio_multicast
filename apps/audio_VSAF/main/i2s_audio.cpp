#include "i2s_audio.hpp"
#include "esp_log.h"
#include "config.h"
#include "soc/gpio_reg.h"
#include "esp_cpu.h"
#include "esp_clk_tree.h"
#include <algorithm>
#include <cstring>

static const char* TAG = "I2S_AUDIO";

namespace Hardware {

static IRAM_ATTR bool i2s_dma_tx_done_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    auto* driver = static_cast<I2sAudioDriver*>(user_ctx);
    BaseType_t high_task_wakeup = pdFALSE;
    if (driver) {
        driver->onDescriptorSentFromISR();
        driver->notifyDmaSlotFreeFromISR(&high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

static IRAM_ATTR bool i2s_dma_tx_q_ovf_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
    auto* driver = static_cast<I2sAudioDriver*>(user_ctx);
    if (driver) {
        driver->onSendQueueOverflowFromISR();
    }
    return false;
}

I2sAudioDriver::I2sAudioDriver(int bclk_pin, int ws_pin, int dout_pin, int din_pin, int gain_pin, int amp_mute_pin)
    : m_bclk_pin(bclk_pin), m_ws_pin(ws_pin), m_dout_pin(dout_pin), m_din_pin(din_pin), m_gain_pin(gain_pin), m_amp_mute_pin(amp_mute_pin) {
    m_dma_free_sem = xSemaphoreCreateBinary();
    setAmpMute(true); // Default to muted on startup
    configurePinElectricalProperties();
}

I2sAudioDriver::~I2sAudioDriver() {
    setAmpMute(true);
    stop();
    if (m_tx_handle) {
        i2s_del_channel(m_tx_handle);
        m_tx_handle = nullptr;
    }
    if (m_dma_free_sem) {
        vSemaphoreDelete(m_dma_free_sem);
        m_dma_free_sem = nullptr;
    }
}

void I2sAudioDriver::configurePinElectricalProperties() {
    const gpio_drive_cap_t cap = static_cast<gpio_drive_cap_t>(m_drive);
    auto configure_pin = [cap](int pin, bool is_output) {
        if (pin < 0) return;
        gpio_num_t gpio = static_cast<gpio_num_t>(pin);
        // Enable internal pull-down resistor to prevent line floating during idle/init
        gpio_set_pull_mode(gpio, GPIO_PULLDOWN_ONLY);
        if (is_output) {
            // Pad drive strength (default GPIO_DRIVE_CAP_2, ~20 mA; console i2sdrive changes it)
            gpio_set_drive_capability(gpio, cap);
        }
    };

    configure_pin(m_bclk_pin, true);
    configure_pin(m_ws_pin, true);
    configure_pin(m_dout_pin, true);
    if (m_din_pin >= 0) {
        configure_pin(m_din_pin, false);
    }
    ESP_LOGI(TAG, "I2S electrical properties applied: Pull-down enabled, drive strength %d on BCLK=%d, WS=%d, DOUT=%d",
             m_drive, m_bclk_pin, m_ws_pin, m_dout_pin);
}

void I2sAudioDriver::setHardwareGain(Max98357Gain gain) {
    if (m_gain_pin < 0) return;

    gpio_num_t pin = static_cast<gpio_num_t>(m_gain_pin);
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.intr_type = GPIO_INTR_DISABLE;

    switch (gain) {
        case Max98357Gain::GAIN_3DB:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            gpio_config(&io_conf);
            m_current_gain = Max98357Gain::GAIN_3DB;
            ESP_LOGI(TAG, "MAX98357A Hardware GAIN set to 3 dB (Input + Pull-Up on GPIO %d)", m_gain_pin);
            break;

        case Max98357Gain::GAIN_6DB:
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            gpio_config(&io_conf);
            gpio_set_level(pin, 1);
            m_current_gain = Max98357Gain::GAIN_6DB;
            ESP_LOGI(TAG, "MAX98357A Hardware GAIN set to 6 dB (Output HIGH on GPIO %d)", m_gain_pin);
            break;

        case Max98357Gain::GAIN_9DB:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            gpio_config(&io_conf);
            m_current_gain = Max98357Gain::GAIN_9DB;
            ESP_LOGI(TAG, "MAX98357A Hardware GAIN set to 9 dB (Input Floating / High-Z on GPIO %d)", m_gain_pin);
            break;

        case Max98357Gain::GAIN_12DB:
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            gpio_config(&io_conf);
            gpio_set_level(pin, 0);
            m_current_gain = Max98357Gain::GAIN_12DB;
            ESP_LOGI(TAG, "MAX98357A Hardware GAIN set to 12 dB (Output LOW on GPIO %d)", m_gain_pin);
            break;

        case Max98357Gain::GAIN_15DB:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            gpio_config(&io_conf);
            m_current_gain = Max98357Gain::GAIN_15DB;
            ESP_LOGI(TAG, "MAX98357A Hardware GAIN set to 15 dB (Input + Pull-Down on GPIO %d)", m_gain_pin);
            break;
    }
}

void I2sAudioDriver::setAmpMute(bool mute) {
    if (m_amp_mute_pin < 0) return;

    gpio_num_t pin = static_cast<gpio_num_t>(m_amp_mute_pin);
    if (mute) {
        // Active LOW -> MUTE / Shutdown
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);
        ESP_LOGI(TAG, "Power Amplifier MUTED (GPIO %d driven LOW)", m_amp_mute_pin);
    } else {
        // High-Z / Input Floating -> UNMUTE / RUN (Internal pull-up on TPA3118)
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_FLOATING);
        ESP_LOGI(TAG, "Power Amplifier UNMUTED (GPIO %d set to High-Z / Input)", m_amp_mute_pin);
    }
}

esp_err_t I2sAudioDriver::init(uint32_t sample_rate, uint32_t frame_duration_us, i2s_data_bit_width_t bits_per_sample, i2s_slot_mode_t slot_mode) {
    setAmpMute(true);
    if (m_gain_pin >= 0) {
        setHardwareGain(Max98357Gain::GAIN_3DB);
    }
    return reconfigureAudioFormat(sample_rate, bits_per_sample, frame_duration_us, slot_mode);
}

esp_err_t I2sAudioDriver::reconfigureAudioFormat(uint32_t sample_rate, i2s_data_bit_width_t bits_per_sample, uint32_t frame_duration_us, i2s_slot_mode_t slot_mode) {
    if (sample_rate == 0) return ESP_ERR_INVALID_ARG;
    if (frame_duration_us == 0) frame_duration_us = 10000;
    frame_duration_us = (frame_duration_us == 7500) ? 7500 : 10000;

    // Validate bit width
    if (bits_per_sample != I2S_DATA_BIT_WIDTH_16BIT && 
        bits_per_sample != I2S_DATA_BIT_WIDTH_24BIT && 
        bits_per_sample != I2S_DATA_BIT_WIDTH_32BIT) {
        bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT;
    }

    if (m_tx_handle && 
        m_sample_rate == sample_rate && 
        m_frame_duration_us == frame_duration_us && 
        m_bits_per_sample == bits_per_sample && 
        m_slot_mode == slot_mode) {
        return ESP_OK;
    }

    uint32_t frame_samples = (sample_rate * frame_duration_us) / 1000000;
    if (frame_samples < 60) frame_samples = 60;

    stop();

    if (m_tx_handle) {
        i2s_del_channel(m_tx_handle);
        m_tx_handle = nullptr;
    }

    // Dual-descriptor DMA ring, each descriptor 1 / CONFIG_VSAF_DMA_DESC_PER_FRAME of an audio frame (5 ms by
    // default). The SINK writes whole descriptors through its staging buffer, so they need not match LC3 frames.
    const uint32_t desc_samples = std::max<uint32_t>(frame_samples / CONFIG_VSAF_DMA_DESC_PER_FRAME, 40);
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 2; // Strictly 2 descriptors (Dual-descriptor ping-pong)
    chan_cfg.dma_frame_num = desc_samples;
    m_dma_frame_samples = desc_samples;
    chan_cfg.auto_clear = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &m_tx_handle, nullptr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register DMA on_sent interrupt callback
    i2s_event_callbacks_t cbs = {
        .on_recv = nullptr,
        .on_recv_q_ovf = nullptr,
        .on_sent = i2s_dma_tx_done_cb,
        .on_send_q_ovf = i2s_dma_tx_q_ovf_cb,
    };
    i2s_channel_register_event_callback(m_tx_handle, &cbs, this);

    i2s_std_config_t std_cfg = {};
    std_cfg.clk_cfg.sample_rate_hz = sample_rate;
    std_cfg.clk_cfg.clk_src = m_clk_src;
    std_cfg.clk_cfg.mclk_multiple = m_mclk_mult;
    m_bits_per_sample = bits_per_sample;  // makeSlotConfig() reads it
    m_slot_mode = slot_mode;
    m_dma_bytes_per_sample = (bits_per_sample == I2S_DATA_BIT_WIDTH_16BIT) ? 2 : (m_slot24 ? 3 : 4);
    std_cfg.slot_cfg = makeSlotConfig();
    std_cfg.gpio_cfg = makeGpioConfig();

    ret = i2s_channel_init_std_mode(m_tx_handle, &std_cfg);
    if (ret == ESP_OK) {
        configurePinElectricalProperties();
        m_sample_rate = sample_rate;
        m_frame_duration_us = frame_duration_us;
        m_bits_per_sample = bits_per_sample;
        m_slot_mode = slot_mode;
        m_is_running = false;
        ESP_LOGI(TAG, "I2S Philips Master Configured: Fs=%lu Hz, %u-bit (Slot: %u-bit), %s, Frame=%lu samples (%.1f ms), GAIN=%ddB",
                 (unsigned long)m_sample_rate, static_cast<uint8_t>(m_bits_per_sample), 
                 (m_bits_per_sample == I2S_DATA_BIT_WIDTH_16BIT) ? 16 : (m_slot24 ? 24 : 32),
                 (m_slot_mode == I2S_SLOT_MODE_STEREO) ? "Stereo" : "Mono",
                 (unsigned long)frame_samples, m_frame_duration_us / 1000.0f, getHardwareGainDb());
    } else {
        ESP_LOGE(TAG, "Failed to init I2S STD mode: %s", esp_err_to_name(ret));
    }

    return ret;
}

i2s_std_slot_config_t I2sAudioDriver::makeSlotConfig() const {
    // Philips I2S slot configuration:
    // 16-bit: 16-bit data and slot width (2 bytes per sample in DMA, BCLK = 32 x fs).
    // 24 / 32-bit: 32-bit data and slot width (4 bytes per sample in DMA, MSB-aligned, BCLK = 64 x fs).
    // 24-bit data in 24-bit slots (m_slot24): BCLK = 48 x fs, 3 bytes per sample in DMA.
    const bool b16 = (m_bits_per_sample == I2S_DATA_BIT_WIDTH_16BIT);
    const bool s24 = !b16 && m_slot24;
    i2s_std_slot_config_t slot = {};
    slot.data_bit_width = b16 ? I2S_DATA_BIT_WIDTH_16BIT : (s24 ? I2S_DATA_BIT_WIDTH_24BIT : I2S_DATA_BIT_WIDTH_32BIT);
    slot.slot_bit_width = b16 ? I2S_SLOT_BIT_WIDTH_16BIT : (s24 ? I2S_SLOT_BIT_WIDTH_24BIT : I2S_SLOT_BIT_WIDTH_32BIT);
    slot.slot_mode = m_slot_mode;
    slot.slot_mask = (m_slot_mode == I2S_SLOT_MODE_MONO) ? I2S_STD_SLOT_LEFT : I2S_STD_SLOT_BOTH;
    slot.ws_width = b16 ? 16 : (s24 ? 24 : 32);
    slot.ws_pol = false;
    slot.bit_shift = m_bit_shift;  // true: Philips 1-bit delay; false: left-justified
    slot.left_align = true;        // MSB first aligned
    slot.big_endian = false;
    slot.bit_order_lsb = false;    // MSB transmitted first on wire
    return slot;
}

i2s_std_gpio_config_t I2sAudioDriver::makeGpioConfig() const {
    i2s_std_gpio_config_t g = {};
    g.mclk = I2S_GPIO_UNUSED;
    g.bclk = static_cast<gpio_num_t>(m_bclk_pin);
    g.ws = static_cast<gpio_num_t>(m_ws_pin);
    g.dout = static_cast<gpio_num_t>(m_dout_pin);
    g.din = static_cast<gpio_num_t>(m_din_pin);
    g.invert_flags.mclk_inv = false;
    g.invert_flags.bclk_inv = m_bclk_inv;
    g.invert_flags.ws_inv = m_ws_inv;
    return g;
}

esp_err_t I2sAudioDriver::setLinkFormat(bool bit_shift, bool bclk_inv, bool ws_inv) {
    if (!m_tx_handle) return ESP_ERR_INVALID_STATE;
    const bool was_running = m_is_running;
    if (was_running) {
        i2s_channel_disable(m_tx_handle);
    }
    m_bit_shift = bit_shift;
    m_bclk_inv = bclk_inv;
    m_ws_inv = ws_inv;
    const i2s_std_slot_config_t slot = makeSlotConfig();
    const i2s_std_gpio_config_t g = makeGpioConfig();
    esp_err_t err = i2s_channel_reconfig_std_slot(m_tx_handle, &slot);
    if (err == ESP_OK) err = i2s_channel_reconfig_std_gpio(m_tx_handle, &g);
    configurePinElectricalProperties();  // gpio reconfig resets the pads
    if (was_running) {
        i2s_channel_enable(m_tx_handle);
    }
    ESP_LOGI(TAG, "I2S link format: %s, BCLK %s, WS %s: %s", bit_shift ? "Philips I2S" : "left-justified",
             bclk_inv ? "inverted" : "normal", ws_inv ? "inverted" : "normal", esp_err_to_name(err));
    return err;
}

void I2sAudioDriver::captureGpio(uint32_t* buf, size_t n, float* ns_per_sample) {
    for (int pin : {m_bclk_pin, m_ws_pin, m_dout_pin}) {
        if (pin >= 0) gpio_input_enable(static_cast<gpio_num_t>(pin));  // read back the driven level
    }
    static portMUX_TYPE s_cap_mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&s_cap_mux);
    const uint32_t c0 = esp_cpu_get_cycle_count();
    for (size_t i = 0; i < n; ++i) {
        buf[i] = REG_READ(GPIO_IN_REG);
    }
    const uint32_t c1 = esp_cpu_get_cycle_count();
    portEXIT_CRITICAL(&s_cap_mux);
    uint32_t cpu_hz = 0;
    esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &cpu_hz);
    *ns_per_sample = cpu_hz ? static_cast<float>(c1 - c0) * 1e9f / cpu_hz / n : 0.0f;
}

esp_err_t I2sAudioDriver::setDriveStrength(int level) {
    if (level < 0 || level > 3) return ESP_ERR_INVALID_ARG;
    m_drive = level;
    for (int pin : {m_bclk_pin, m_ws_pin, m_dout_pin}) {
        if (pin >= 0) {
            gpio_set_drive_capability(static_cast<gpio_num_t>(pin), static_cast<gpio_drive_cap_t>(level));
        }
    }
    ESP_LOGI(TAG, "I2S pad drive strength %d on BCLK=%d, WS=%d, DOUT=%d", level, m_bclk_pin, m_ws_pin, m_dout_pin);
    return ESP_OK;
}

esp_err_t I2sAudioDriver::setClock(i2s_clock_src_t src, i2s_mclk_multiple_t mclk_multiple) {
    if (!m_tx_handle) return ESP_ERR_INVALID_STATE;
    const bool was_running = m_is_running;
    if (was_running) {
        i2s_channel_disable(m_tx_handle);  // amp stays unmuted: a short gap only
    }
    i2s_std_clk_config_t clk = {};
    clk.sample_rate_hz = m_sample_rate;
    clk.clk_src = src;
    clk.mclk_multiple = mclk_multiple;
    esp_err_t err = i2s_channel_reconfig_std_clock(m_tx_handle, &clk);
    if (err == ESP_OK) {
        m_clk_src = src;
        m_mclk_mult = mclk_multiple;
    }
    if (was_running) {
        i2s_channel_enable(m_tx_handle);
    }
    ESP_LOGI(TAG, "I2S clock source %d, MCLK %d x fs: %s", static_cast<int>(src), static_cast<int>(mclk_multiple),
             esp_err_to_name(err));
    return err;
}

esp_err_t I2sAudioDriver::reconfigureSampleRate(uint32_t sample_rate, uint32_t frame_duration_us) {
    return reconfigureAudioFormat(sample_rate, m_bits_per_sample, frame_duration_us, m_slot_mode);
}

esp_err_t I2sAudioDriver::setBitDepth(i2s_data_bit_width_t bits_per_sample) {
    return reconfigureAudioFormat(m_sample_rate, bits_per_sample, m_frame_duration_us, m_slot_mode);
}

esp_err_t I2sAudioDriver::setBitDepth(uint8_t bit_depth_bits) {
    i2s_data_bit_width_t width = I2S_DATA_BIT_WIDTH_16BIT;
    if (bit_depth_bits == 32) width = I2S_DATA_BIT_WIDTH_32BIT;
    else if (bit_depth_bits == 24) width = I2S_DATA_BIT_WIDTH_24BIT;
    return setBitDepth(width);
}

esp_err_t I2sAudioDriver::start() {
    if (!m_tx_handle || m_is_running) return ESP_OK;
    if (m_dma_free_sem) {
        xSemaphoreTake(m_dma_free_sem, 0); // Clear any stale tokens
    }
    portENTER_CRITICAL(&m_sent_mux);
    m_desc_sent = 0;
    m_last_sent_us = static_cast<uint32_t>(esp_timer_get_time());
    portEXIT_CRITICAL(&m_sent_mux);
    esp_err_t ret = i2s_channel_enable(m_tx_handle);
    if (ret == ESP_OK) {
        m_is_running = true;
        setAmpMute(false); // Unmute amplifier (High-Z)
        ESP_LOGI(TAG, "I2S Hardware Clock Started (Playing preloaded DMA descriptors).");
    }
    return ret;
}

esp_err_t I2sAudioDriver::stop() {
    setAmpMute(true); // Mute amplifier (Drive LOW)
    if (!m_tx_handle || !m_is_running) return ESP_OK;
    esp_err_t ret = i2s_channel_disable(m_tx_handle);
    if (ret == ESP_OK) {
        m_is_running = false;
        configurePinElectricalProperties();
        if (m_dma_free_sem) {
            xSemaphoreTake(m_dma_free_sem, 0);
        }
        ESP_LOGI(TAG, "I2S Hardware Clock Stopped (Standby / Idle).");
    }
    return ret;
}

esp_err_t I2sAudioDriver::preload(const void* src, size_t size, size_t* bytes_written) {
    if (!m_tx_handle || !src) return ESP_ERR_INVALID_STATE;
    return i2s_channel_preload_data(m_tx_handle, src, size, bytes_written);
}

esp_err_t I2sAudioDriver::write(const void* src, size_t size, size_t* bytes_written, uint32_t timeout_ms) {
    if (!m_tx_handle || !src) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = i2s_channel_write(m_tx_handle, src, size, bytes_written, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        m_underrun_count.fetch_add(1, std::memory_order_relaxed);
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

} // namespace Hardware
