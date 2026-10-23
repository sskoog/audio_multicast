#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

namespace Hardware {

/**
 * @brief I²S Output Driver Class for MAX98357A Class-D Audio Amplifier on Waveshare ESP32-C6.
 * 
 * Hardware Pin Mapping:
 * - GPIO 19: BCLK (Bit Clock)
 * - GPIO 20: WS / LRCK (Word Select / Frame Clock)
 * - GPIO 21: DOUT (Serial Data Out)
 */
class I2sDacDriver {
public:
    I2sDacDriver();
    ~I2sDacDriver();

    /**
     * @brief Initialize I²S Master TX channel in standard Philips I²S mode.
     * @param sample_rate Sampling rate in Hz (default 48000 Hz)
     * @param bclk_pin GPIO pin for Bit Clock (default GPIO_NUM_19)
     * @param ws_pin GPIO pin for Word Select (default GPIO_NUM_20)
     * @param dout_pin GPIO pin for Data Out (default GPIO_NUM_21)
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t init(uint32_t sample_rate = 48000, 
                   gpio_num_t bclk_pin = GPIO_NUM_19, 
                   gpio_num_t ws_pin = GPIO_NUM_20, 
                   gpio_num_t dout_pin = GPIO_NUM_21);

    /**
     * @brief Write 16-bit PCM audio samples to the I²S DMA TX queue.
     * @param pcm_data Pointer to 16-bit PCM samples
     * @param data_size Size of pcm_data in bytes
     * @param bytes_written Pointer to receive number of bytes written to DMA
     * @param timeout_ms Timeout in milliseconds
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t write(const int16_t* pcm_data, size_t data_size, size_t* bytes_written, uint32_t timeout_ms = 100);

    /**
     * @brief Enable/Disable I²S audio output.
     */
    esp_err_t enable();
    esp_err_t disable();

    /**
     * @brief Get active sampling rate.
     */
    uint32_t getSampleRate() const { return m_sample_rate; }

private:
    uint32_t m_sample_rate{48000};
    i2s_chan_handle_t m_tx_handle{nullptr};
    bool m_is_enabled{false};
};

} // namespace Hardware
