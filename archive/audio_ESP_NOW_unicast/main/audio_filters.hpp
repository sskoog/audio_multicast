#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Dsp {

/**
 * @brief 4th-order Linkwitz-Riley Low-Pass Filter (LR4 LP)
 * 
 * Constructed as two cascaded 2nd-order Butterworth low-pass biquad sections
 * with Q = 1/sqrt(2) = 0.70710678. Provides -6 dB gain at the cutoff frequency
 * and a steep -24 dB/octave (-80 dB/decade) roll-off.
 */
class LinkwitzRiley4thOrderLowPass {
public:
    LinkwitzRiley4thOrderLowPass() = default;

    void init(float sample_rate_hz, float cutoff_hz = 100.0f) {
        m_sample_rate = sample_rate_hz;
        m_cutoff_hz = cutoff_hz;
        calculateCoefficients();
        reset();
    }

    void setCutoff(float cutoff_hz) {
        if (std::abs(m_cutoff_hz - cutoff_hz) > 0.1f) {
            m_cutoff_hz = cutoff_hz;
            calculateCoefficients();
        }
    }

    void reset() {
        m_s1_0 = 0.0f; m_s2_0 = 0.0f;
        m_s1_1 = 0.0f; m_s2_1 = 0.0f;
    }

    /**
     * @brief Process an array of int16 PCM samples in-place or out-of-place
     */
    void process(const int16_t* in_pcm, int16_t* out_pcm, size_t num_samples) {
        if (!in_pcm || !out_pcm || num_samples == 0) return;

        for (size_t i = 0; i < num_samples; i++) {
            float x = static_cast<float>(in_pcm[i]);

            // Section 1 (Direct Form II Transposed)
            float y1 = m_b0 * x + m_s1_0;
            m_s1_0 = m_b1 * x - m_a1 * y1 + m_s2_0;
            m_s2_0 = m_b2 * x - m_a2 * y1;

            // Section 2 (Direct Form II Transposed)
            float y2 = m_b0 * y1 + m_s1_1;
            m_s1_1 = m_b1 * y1 - m_a1 * y2 + m_s2_1;
            m_s2_1 = m_b2 * y1 - m_a2 * y2;

            // Clamping
            if (y2 > 32767.0f) y2 = 32767.0f;
            else if (y2 < -32768.0f) y2 = -32768.0f;
            out_pcm[i] = static_cast<int16_t>(y2);
        }
    }

private:
    void calculateCoefficients() {
        if (m_sample_rate <= 0.0f || m_cutoff_hz <= 0.0f) return;

        // Ensure cutoff is below Nyquist
        float max_cutoff = (m_sample_rate * 0.45f);
        float fc = (m_cutoff_hz > max_cutoff) ? max_cutoff : m_cutoff_hz;

        float w0 = 2.0f * static_cast<float>(M_PI) * fc / m_sample_rate;
        float alpha = std::sin(w0) / std::sqrt(2.0f);
        float cos_w0 = std::cos(w0);

        float b0 = (1.0f - cos_w0) * 0.5f;
        float b1 = 1.0f - cos_w0;
        float b2 = (1.0f - cos_w0) * 0.5f;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cos_w0;
        float a2 = 1.0f - alpha;

        m_b0 = b0 / a0;
        m_b1 = b1 / a0;
        m_b2 = b2 / a0;
        m_a1 = a1 / a0;
        m_a2 = a2 / a0;
    }

    float m_sample_rate = 48000.0f;
    float m_cutoff_hz = 100.0f;

    // Normalized biquad coefficients (identical for both Butterworth sections)
    float m_b0 = 0.0f, m_b1 = 0.0f, m_b2 = 0.0f;
    float m_a1 = 0.0f, m_a2 = 0.0f;

    // Filter state (Section 0)
    float m_s1_0 = 0.0f, m_s2_0 = 0.0f;
    // Filter state (Section 1)
    float m_s1_1 = 0.0f, m_s2_1 = 0.0f;
};

/**
 * @brief Subwoofer Multirate Resampler
 * 
 * Supports factor-of-6 downsampling (48 kHz -> 8 kHz) on SOURCE
 * and factor-of-6 band-limited interpolation (8 kHz -> 48 kHz) on SINK.
 */
class SubwooferResampler {
public:
    /**
     * @brief Decimates 48 kHz mono PCM (480 samples) to 8 kHz mono PCM (80 samples)
     * Factor = 6: 480 -> 80 samples per 10 ms frame
     */
    static void downsample48kTo8k(const int16_t* in_48k, int16_t* out_8k, size_t num_48k_samples = 480) {
        if (!in_48k || !out_8k) return;
        size_t out_samples = num_48k_samples / 6;
        for (size_t i = 0; i < out_samples; i++) {
            out_8k[i] = in_48k[i * 6];
        }
    }

    /**
     * @brief Upsamples 8 kHz mono PCM (80 samples) to 48 kHz mono PCM (480 samples)
     * Factor = 6: 80 -> 480 samples per 10 ms frame with smooth linear interpolation
     */
    static void upsample8kTo48k(const int16_t* in_8k, int16_t* out_48k, size_t num_8k_samples = 80) {
        if (!in_8k || !out_48k || num_8k_samples == 0) return;

        for (size_t i = 0; i < num_8k_samples; i++) {
            int16_t curr = in_8k[i];
            int16_t next = (i + 1 < num_8k_samples) ? in_8k[i + 1] : curr;
            float step = static_cast<float>(next - curr) / 6.0f;

            for (size_t k = 0; k < 6; k++) {
                float val = static_cast<float>(curr) + step * static_cast<float>(k);
                if (val > 32767.0f) val = 32767.0f;
                else if (val < -32768.0f) val = -32768.0f;
                out_48k[i * 6 + k] = static_cast<int16_t>(val);
            }
        }
    }
};

} // namespace Dsp
