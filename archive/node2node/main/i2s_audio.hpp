#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace Hardware {

enum Max98357Gain {
    GAIN_6DB,   // GPIO Output LOW
    GAIN_9DB,   // GPIO Input (High-Z Float)
    GAIN_12DB   // GPIO Output HIGH
};

class I2sAudioDriver {
public:
    I2sAudioDriver();
    ~I2sAudioDriver();

    /**
     * @brief Calculates exact DMA buffer frame count per descriptor for any sample rate & duration
     * @param sample_rate_hz e.g. 48000, 44100, 32000, 24000, 22050, 16000, 8000 Hz
     * @param frame_duration_us e.g. 10000 (10ms) or 7500 (7.5ms)
     * @return Number of audio sample frames per DMA descriptor
     */
    static constexpr size_t calculateDmaFrameNum(uint32_t sample_rate_hz, uint32_t frame_duration_us) {
        return (static_cast<uint64_t>(sample_rate_hz) * frame_duration_us + 500000ULL) / 1000000ULL;
    }

    /**
     * @brief Calculates total raw 16-bit PCM bytes per frame
     */
    static constexpr size_t calculatePcmBytesPerFrame(uint32_t sample_rate_hz, uint32_t frame_duration_us, uint8_t channels = 2) {
        return calculateDmaFrameNum(sample_rate_hz, frame_duration_us) * channels * sizeof(int16_t);
    }

    // Initializes dual-descriptor DMA (2 x Frame Duration) for any standard sample rate & frame duration
    esp_err_t init(uint32_t sample_rate_hz, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, int gain_pin = -1, uint32_t frame_duration_us = 10000, uint8_t channels = 2);
    void deinit();

    // Reconfigures hardware sample rate, frame duration, and DMA descriptor size on the fly
    esp_err_t reconfigurePipeline(uint32_t sample_rate_hz, uint32_t frame_duration_us = 10000, uint8_t channels = 2);

    // Preloads 16-bit PCM frames into DMA descriptor while channel is stopped/idle
    esp_err_t preload(const int16_t* pcm_data, size_t samples_count, size_t* bytes_loaded);

    // Starts I2S hardware clock (called when both DMA descriptors are primed)
    esp_err_t start();

    // Stops I2S hardware clock (puts peripheral into idle/standby)
    esp_err_t stop();

    // Reconfigures hardware sample rate on the fly (48k, 44.1k, 32k, 24k, 22.05k, 16k, 8k)
    esp_err_t setSampleRate(uint32_t sample_rate_hz);

    // Sets hardware gain on MAX98357A via tri-state GPIO
    void setGain(Max98357Gain gain);

    // Writes 16-bit PCM frames to I2S DMA (handles mono duplicated and stereo interleaved)
    esp_err_t write(const int16_t* pcm_data, size_t samples_count, size_t* bytes_written, uint32_t timeout_ms = 100);

    // Interrupt-driven synchronization: waits until a DMA descriptor is empty
    bool waitForDmaSlot(uint32_t timeout_ms = 30);

    // Called from DMA on_sent ISR
    void notifyDmaSlotFreeFromISR(BaseType_t* high_task_wakeup);

    bool isInitialized() const { return m_initialized; }
    bool isRunning() const { return m_running; }
    uint32_t getSampleRate() const { return m_sample_rate; }
    uint32_t getFrameDurationUs() const { return m_frame_duration_us; }
    uint8_t  getChannels() const { return m_channels; }
    size_t   getSamplesPerFrame() const { return calculateDmaFrameNum(m_sample_rate, m_frame_duration_us); }

    void incrementUnderrunCount();

    inline uint32_t getAndResetUnderrunCount() {
        return m_dma_underrun_count.exchange(0, std::memory_order_relaxed);
    }

    inline uint32_t getUnderrunCount() const {
        return m_dma_underrun_count.load(std::memory_order_relaxed);
    }

private:
    void* m_tx_chan = nullptr;
    bool m_initialized = false;
    bool m_running = false;
    uint32_t m_sample_rate = 48000;
    uint32_t m_frame_duration_us = 10000;
    uint8_t  m_channels = 2;
    gpio_num_t m_bclk = GPIO_NUM_NC;
    gpio_num_t m_ws = GPIO_NUM_NC;
    gpio_num_t m_dout = GPIO_NUM_NC;
    gpio_num_t m_gain_pin = GPIO_NUM_NC;
    SemaphoreHandle_t m_dma_free_sem = nullptr;
    std::atomic<uint32_t> m_dma_underrun_count{0};
};

} // namespace Hardware
