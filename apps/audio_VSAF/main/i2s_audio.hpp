#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <atomic>
#include "esp_timer.h"

namespace Hardware {

enum class Max98357Gain {
    GAIN_3DB = 3,   // Input with internal pull-up (~45k to VDD -> 3 dB)
    GAIN_6DB = 6,   // Output HIGH (Direct VDD -> 6 dB)
    GAIN_9DB = 9,   // Input High-Z / Floating (No pulls -> 9 dB)
    GAIN_12DB = 12  // Output LOW (Direct GND -> 12 dB)
};

class I2sAudioDriver {
public:
    I2sAudioDriver(int bclk_pin, int ws_pin, int dout_pin, int din_pin = -1, int gain_pin = -1, int amp_mute_pin = -1);
    ~I2sAudioDriver();

    esp_err_t init(uint32_t sample_rate = 32000, 
                   uint32_t frame_duration_us = 10000, 
                   i2s_data_bit_width_t bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT, 
                   i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_STEREO);
    
    // Dynamic sample rate, bit depth & DMA frame size reconfiguration
    esp_err_t reconfigureAudioFormat(uint32_t sample_rate, 
                                     i2s_data_bit_width_t bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT, 
                                     uint32_t frame_duration_us = 10000, 
                                     i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_STEREO);

    esp_err_t reconfigureSampleRate(uint32_t sample_rate, uint32_t frame_duration_us = 10000);
    esp_err_t setBitDepth(i2s_data_bit_width_t bits_per_sample);
    esp_err_t setBitDepth(uint8_t bit_depth_bits);

    uint32_t getSampleRate() const { return m_sample_rate; }
    uint32_t getFrameDurationUs() const { return m_frame_duration_us; }
    i2s_data_bit_width_t getBitWidth() const { return m_bits_per_sample; }
    uint8_t getBitDepth() const { return static_cast<uint8_t>(m_bits_per_sample); }
    i2s_slot_mode_t getSlotMode() const { return m_slot_mode; }
    size_t getBytesPerSample() const { return (m_bits_per_sample == I2S_DATA_BIT_WIDTH_16BIT) ? 2 : 4; }

    // Hardware gain control (3, 6, 9, 12 dB for MAX98357A)
    void setHardwareGain(Max98357Gain gain);
    Max98357Gain getHardwareGain() const { return m_current_gain; }
    uint8_t getHardwareGainDb() const { return static_cast<uint8_t>(m_current_gain); }

    // Power amplifier Mute / SD / Enable control (active LOW mute, High-Z unmute)
    void setAmpMute(bool mute);

    // Starts I2S hardware clocks (BCLK & WS) - called after dual descriptors are pre-filled
    esp_err_t start();
    
    // Stops/gates I2S hardware clocks - enters low-power standby
    esp_err_t stop();
    
    // Preloads PCM frame into DMA descriptor while clocks are stopped
    esp_err_t preload(const void* src, size_t size, size_t* bytes_written);
    
    // Writes PCM frame to I2S DMA
    esp_err_t write(const void* src, size_t size, size_t* bytes_written, uint32_t timeout_ms = 100);
    
    // Interrupt-driven synchronization: waits until a DMA descriptor is empty
    bool waitForDmaSlot(uint32_t timeout_ms = 25);
    
    // Called from DMA on_sent ISR
    void notifyDmaSlotFreeFromISR(BaseType_t* high_task_wakeup);

    // Called from DMA on_sent ISR: one DMA descriptor (one frame of samples) finished playing
    inline void IRAM_ATTR onDescriptorSentFromISR() {
        portENTER_CRITICAL_ISR(&m_sent_mux);
        m_desc_sent++;
        m_last_sent_us = static_cast<uint32_t>(esp_timer_get_time());
        portEXIT_CRITICAL_ISR(&m_sent_mux);
    }

    /**
     * @brief Samples played by the I2S hardware since start() (per channel).
     *
     * Completed DMA descriptors x descriptor size, plus the part of the current descriptor played since the
     * last completion (interpolated with esp_timer, at most one descriptor). Re-anchored on every completion,
     * so no sample-rate assumption accumulates. Includes zero-filled descriptors played during a DMA underrun.
     */
    int64_t getPlayedSamples() {
        portENTER_CRITICAL(&m_sent_mux);
        uint32_t sent = m_desc_sent;
        uint32_t last_us = m_last_sent_us;
        portEXIT_CRITICAL(&m_sent_mux);
        uint32_t elapsed_us = static_cast<uint32_t>(esp_timer_get_time()) - last_us;  // 32-bit modular, wrap safe
        int64_t frac = (static_cast<int64_t>(elapsed_us) * m_sample_rate) / 1000000;
        if (frac > m_dma_frame_samples) frac = m_dma_frame_samples;
        return static_cast<int64_t>(sent) * m_dma_frame_samples + frac;
    }

    // Called from the DMA on_send_q_ovf ISR: the writer missed a descriptor, so one descriptor of zeros
    // (auto_clear) was played that was never written
    inline void IRAM_ATTR onSendQueueOverflowFromISR() { m_q_ovf.fetch_add(1, std::memory_order_relaxed); }

    // DMA underruns (zero descriptors played) since boot; each one is getDmaFrameSamples() samples
    uint32_t getQueueOverflows() const { return m_q_ovf.load(std::memory_order_relaxed); }
    uint32_t getDmaFrameSamples() const { return m_dma_frame_samples; }

    bool isRunning() const { return m_is_running; }
    bool isInitialized() const { return m_tx_handle != nullptr; }

    uint32_t getAndResetUnderrunCount() { return m_underrun_count.exchange(0, std::memory_order_relaxed); }
    uint32_t getUnderrunCount() const { return m_underrun_count.load(std::memory_order_relaxed); }
    void resetUnderrunCount() { m_underrun_count.store(0, std::memory_order_relaxed); }

private:
    void configurePinElectricalProperties();

    int m_bclk_pin;
    int m_ws_pin;
    int m_dout_pin;
    int m_din_pin;
    int m_gain_pin;
    int m_amp_mute_pin;
    Max98357Gain m_current_gain = Max98357Gain::GAIN_3DB;

    i2s_chan_handle_t m_tx_handle = nullptr;
    SemaphoreHandle_t m_dma_free_sem = nullptr;
    bool m_is_running = false;
    uint32_t m_sample_rate = 32000;
    uint32_t m_frame_duration_us = 10000;
    i2s_data_bit_width_t m_bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT;
    i2s_slot_mode_t m_slot_mode = I2S_SLOT_MODE_STEREO;
    std::atomic<uint32_t> m_underrun_count{0};
    // DMA descriptor completions since start() (ISR writes, SINK task reads under m_sent_mux)
    portMUX_TYPE m_sent_mux = portMUX_INITIALIZER_UNLOCKED;
    uint32_t m_desc_sent = 0;
    uint32_t m_last_sent_us = 0;
    uint32_t m_dma_frame_samples = 480;
    std::atomic<uint32_t> m_q_ovf{0};
};

} // namespace Hardware
