#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <atomic>

namespace Hardware {

enum class Max98357Gain {
    GAIN_3DB = 3,   // Input with internal pull-up (~45k to VDD -> 3 dB)
    GAIN_6DB = 6,   // Output HIGH (Direct VDD -> 6 dB)
    GAIN_9DB = 9,   // Input High-Z / Floating (No pulls -> 9 dB)
    GAIN_12DB = 12  // Output LOW (Direct GND -> 12 dB)
};

class I2sAudioDriver {
public:
    I2sAudioDriver(int bclk_pin, int ws_pin, int dout_pin, int din_pin = -1, int gain_pin = 0);
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

    // Hardware gain control (3, 6, 9, 12 dB)
    void setHardwareGain(Max98357Gain gain);
    Max98357Gain getHardwareGain() const { return m_current_gain; }
    uint8_t getHardwareGainDb() const { return static_cast<uint8_t>(m_current_gain); }

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

    bool isRunning() const { return m_is_running; }
    bool isInitialized() const { return m_tx_handle != nullptr; }

    uint32_t getAndResetUnderrunCount() { return m_underrun_count.exchange(0, std::memory_order_relaxed); }
    uint32_t getUnderrunCount() const { return m_underrun_count.load(std::memory_order_relaxed); }
    void resetUnderrunCount() { m_underrun_count.store(0, std::memory_order_relaxed); }

private:
    int m_bclk_pin;
    int m_ws_pin;
    int m_dout_pin;
    int m_din_pin;
    int m_gain_pin;
    Max98357Gain m_current_gain = Max98357Gain::GAIN_3DB;

    i2s_chan_handle_t m_tx_handle = nullptr;
    SemaphoreHandle_t m_dma_free_sem = nullptr;
    bool m_is_running = false;
    uint32_t m_sample_rate = 32000;
    uint32_t m_frame_duration_us = 10000;
    i2s_data_bit_width_t m_bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT;
    i2s_slot_mode_t m_slot_mode = I2S_SLOT_MODE_STEREO;
    std::atomic<uint32_t> m_underrun_count{0};
};

} // namespace Hardware
