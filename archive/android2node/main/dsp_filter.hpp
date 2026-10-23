#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include "esp_err.h"

namespace Audio {

/**
 * @brief 2nd-Order High-Pass Butterworth IIR Filter Class operating in Fixed-Point (Q31 / Q15).
 * 
 * Configured for cutoff frequency fc = 80 Hz at sample rate fs = 48,000 Hz (Q = 0.7071) for top-range speakers.
 * Uses Espressif esp-dsp fixed-point math routines optimized for RISC-V 32-bit integer ALUs.
 */
class DspHighPassFilter {
public:
    DspHighPassFilter();
    ~DspHighPassFilter() = default;

    /**
     * @brief Initialize biquad coefficients for 80 Hz high-pass filtering at 48kHz.
     * @param cutoff_freq Cutoff frequency in Hz (default 80.0 Hz)
     * @param sample_rate Sampling rate in Hz (default 48000.0 Hz)
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t init(float cutoff_freq = 80.0f, float sample_rate = 48000.0f);

    /**
     * @brief Process a buffer of 16-bit PCM audio samples in-place.
     * @param pcm_buffer Pointer to 16-bit PCM samples (interleaved stereo or mono)
     * @param num_samples Number of samples to process
     */
    void process(int16_t* pcm_buffer, size_t num_samples);

    /**
     * @brief Get cutoff frequency in Hz.
     */
    float getCutoffFreq() const { return m_cutoff_freq; }

    /**
     * @brief Get active sampling rate in Hz.
     */
    float getSampleRate() const { return m_sample_rate; }

    /**
     * @brief Get filter order.
     */
    uint8_t getFilterOrder() const { return 2; }

private:
    float m_cutoff_freq{80.0f};
    float m_sample_rate{48000.0f};

    // Biquad Direct Form I coefficients in Q31 format
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    int32_t m_b0_q31{0};
    int32_t m_b1_q31{0};
    int32_t m_b2_q31{0};
    int32_t m_a1_q31{0};
    int32_t m_a2_q31{0};

    // Filter delay lines (per channel)
    int32_t m_w1_l{0}, m_w2_l{0};
    int32_t m_w1_r{0}, m_w2_r{0};
};

} // namespace Audio
