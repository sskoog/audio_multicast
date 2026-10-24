#pragma once

#include "dsps_biquad.h"
#include "dsps_biquad_gen.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace DSP {

/**
 * @brief Stereo 4th-Order Linkwitz-Riley High-Pass Filter (Cascaded 2x 2nd-order Butterworth).
 * Optimized with esp-dsp SIMD block processing (dsps_biquad_sf32).
 */
class LinkwitzRiley4Stereo {
public:
    void initHighPass(float cutoff_hz, float sample_rate_hz) {
        float norm_fc = cutoff_hz / sample_rate_hz;
        constexpr float q = 0.70710678f; // Butterworth Q = 1/sqrt(2)
        dsps_biquad_gen_hpf_f32(m_coeffs, norm_fc, q);
        reset();
    }

    void reset() {
        memset(m_w1, 0, sizeof(m_w1));
        memset(m_w2, 0, sizeof(m_w2));
    }

    /**
     * @brief Processes an interleaved stereo float buffer [L0, R0, L1, R1, ...] in-place or out-of-place
     * @param in_interleaved Input stereo float array (size: num_samples * 2)
     * @param out_interleaved Output stereo float array (size: num_samples * 2)
     * @param num_samples Number of stereo sample pairs (e.g., 480)
     */
    void processStereo(const float* in_interleaved, float* out_interleaved, int num_samples) {
        if (!in_interleaved || !out_interleaved || num_samples <= 0) return;
        // Stage 1 (Butterworth 2nd order HPF on stereo interleaved samples)
        dsps_biquad_sf32(in_interleaved, out_interleaved, num_samples, m_coeffs, m_w1);
        // Stage 2 (Cascaded Butterworth 2nd order HPF on stereo interleaved samples in-place)
        dsps_biquad_sf32(out_interleaved, out_interleaved, num_samples, m_coeffs, m_w2);
    }

private:
    float m_coeffs[5]{0.0f};
    float m_w1[4]{0.0f}; // Stage 1 delay line [w0_L, w1_L, w0_R, w1_R]
    float m_w2[4]{0.0f}; // Stage 2 delay line [w0_L, w1_L, w0_R, w1_R]
};

/**
 * @brief Anti-Aliasing Polyphase FIR Decimator (D = 6, 42 taps total, 7 taps/branch).
 * Passband: 0 to 3.5 kHz, Cutoff: 3.75 kHz (-6 dB), Stopband: >= 4.0 kHz (Nyquist of 8 kHz).
 * Latency: ~0.4 ms (41 samples / 2 / 48000 = 0.427 ms).
 */
class AntiAliasingPolyphaseFirDecimator {
public:
    static constexpr size_t TAPS = 42;
    static constexpr size_t DECIMATION = 6;
    static constexpr size_t HISTORY_SIZE = TAPS - 1; // 41 samples

    void init() {
        reset();
    }

    void reset() {
        memset(m_history, 0, sizeof(m_history));
    }

    /**
     * @brief Decimates 48 kHz mono float PCM to 8 kHz mono float PCM (480 -> 80 samples).
     * Polyphase evaluation with continuous frame-to-frame convolution state.
     */
    void process(const float* in_48k, float* out_8k, size_t num_48k_samples = 480) {
        if (!in_48k || !out_8k || num_48k_samples == 0) return;

        size_t num_8k_samples = num_48k_samples / DECIMATION;

        for (size_t m = 0; m < num_8k_samples; ++m) {
            int center_idx = static_cast<int>(m * DECIMATION);
            float sum = 0.0f;

            for (size_t k = 0; k < TAPS; ++k) {
                int sample_idx = center_idx - static_cast<int>(k);
                float val = (sample_idx >= 0) ? in_48k[sample_idx] : m_history[HISTORY_SIZE + sample_idx];
                sum += TAPS_42[k] * val;
            }
            out_8k[m] = sum;
        }

        // Maintain 41-sample history buffer across frames
        if (num_48k_samples >= HISTORY_SIZE) {
            memcpy(m_history, &in_48k[num_48k_samples - HISTORY_SIZE], HISTORY_SIZE * sizeof(float));
        }
    }

private:
    static constexpr float TAPS_42[42] = {
        -3.391196177e-04f, -1.483558342e-04f, +5.404532219e-04f, +1.761418104e-03f, +3.261005524e-03f, +4.455796655e-03f,
        +4.529613201e-03f, +2.684737865e-03f, -1.499538325e-03f, -7.707394291e-03f, -1.466484172e-02f, -2.018097702e-02f,
        -2.151248466e-02f, -1.601102098e-02f, -1.907285887e-03f, +2.098767541e-02f, +5.089325826e-02f, +8.414622183e-02f,
        +1.158011260e-01f, +1.406301183e-01f, +1.542795940e-01f, +1.542795940e-01f, +1.406301183e-01f, +1.158011260e-01f,
        +8.414622183e-02f, +5.089325826e-02f, +2.098767541e-02f, -1.907285887e-03f, -1.601102098e-02f, -2.151248466e-02f,
        -2.018097702e-02f, -1.466484172e-02f, -7.707394291e-03f, -1.499538325e-03f, +2.684737865e-03f, +4.529613201e-03f,
        +4.455796655e-03f, +3.261005524e-03f, +1.761418104e-03f, +5.404532219e-04f, -1.483558342e-04f, -3.391196177e-04f,
    };

    float m_history[HISTORY_SIZE]{0.0f};
};

/**
 * @brief 4th-Order 100 Hz Linkwitz-Riley Low-Pass Filter at 8 kHz (2x Transposed Direct Form II Biquads).
 * Normalized cutoff: fc / fs = 100 / 8000 = 0.0125 (Well-conditioned poles).
 * Computation: 10 MACs / sample.
 * Latency: Group delay confined strictly to 100 Hz transition knee.
 */
class LinkwitzRiley4thOrderLowPass8k {
public:
    void init(float cutoff_hz = 100.0f, float sample_rate_hz = 8000.0f) {
        m_sample_rate = sample_rate_hz;
        m_cutoff_hz = cutoff_hz;
        calculateCoefficients();
        reset();
    }

    void reset() {
        m_s1_0 = 0.0f; m_s2_0 = 0.0f;
        m_s1_1 = 0.0f; m_s2_1 = 0.0f;
    }

    /**
     * @brief Process 8 kHz mono float samples through 2 cascaded Transposed Direct Form II Biquads (10 MACs/sample)
     */
    void process(const float* in_8k, float* out_8k, size_t num_8k_samples = 80) {
        if (!in_8k || !out_8k || num_8k_samples == 0) return;

        for (size_t i = 0; i < num_8k_samples; ++i) {
            float x = in_8k[i];

            // Section 1 (Transposed Direct Form II)
            float y1 = m_b0 * x + m_s1_0;
            m_s1_0   = m_b1 * x - m_a1 * y1 + m_s2_0;
            m_s2_0   = m_b2 * x - m_a2 * y1;

            // Section 2 (Transposed Direct Form II)
            float y2 = m_b0 * y1 + m_s1_1;
            m_s1_1   = m_b1 * y1 - m_a1 * y2 + m_s2_1;
            m_s2_1   = m_b2 * y1 - m_a2 * y2;

            out_8k[i] = y2;
        }
    }

private:
    void calculateCoefficients() {
        float w0 = 2.0f * static_cast<float>(M_PI) * m_cutoff_hz / m_sample_rate;
        float alpha = std::sin(w0) / std::sqrt(2.0f); // Butterworth Q = 1/sqrt(2) = 0.70710678
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

    float m_sample_rate = 8000.0f;
    float m_cutoff_hz   = 100.0f;

    // Normalized biquad coefficients (identical for both Butterworth sections)
    float m_b0 = 0.0f, m_b1 = 0.0f, m_b2 = 0.0f;
    float m_a1 = 0.0f, m_a2 = 0.0f;

    // Transposed Direct Form II state variables (Section 0)
    float m_s1_0 = 0.0f, m_s2_0 = 0.0f;
    // Transposed Direct Form II state variables (Section 1)
    float m_s1_1 = 0.0f, m_s2_1 = 0.0f;
};

/**
 * @brief Subwoofer Multirate Decimator & Crossover Pipeline.
 * 1. Anti-Aliasing Polyphase FIR Decimator (48 kHz -> 8 kHz, D=6, 42 taps total, 7 taps/branch)
 * 2. 4th-Order 100 Hz Linkwitz-Riley Low-Pass Filter at 8 kHz (2x Transposed Direct Form II Biquads)
 */
class SubwooferPolyphaseDecimator {
public:
    void init(float cutoff_hz = 100.0f, float input_sr_hz = 48000.0f) {
        (void)input_sr_hz;
        m_fir_decimator.init();
        m_lpf_8k.init(cutoff_hz, 8000.0f);
    }

    void reset() {
        m_fir_decimator.reset();
        m_lpf_8k.reset();
    }

    /**
     * @brief Decimates and filters 480 mono float samples at 48 kHz into 80 subwoofer float samples at 8 kHz.
     */
    void process(const float* in_mono_48k, float* out_mono_8k) {
        if (!in_mono_48k || !out_mono_8k) return;

        // Stage 1: Anti-Aliasing Polyphase FIR Decimator (48 kHz -> 8 kHz)
        m_fir_decimator.process(in_mono_48k, m_temp_8k, 480);

        // Stage 2: 4th-Order 100 Hz Linkwitz-Riley IIR at 8 kHz
        m_lpf_8k.process(m_temp_8k, out_mono_8k, 80);
    }

private:
    AntiAliasingPolyphaseFirDecimator m_fir_decimator;
    LinkwitzRiley4thOrderLowPass8k    m_lpf_8k;
    float                             m_temp_8k[80]{0.0f};
};

/**
 * @brief Single-channel 4th-Order Linkwitz-Riley Filter using esp-dsp block processing
 */
class LinkwitzRiley4 {
public:
    void initLowPass(float cutoff_hz, float sample_rate_hz) {
        float norm_fc = cutoff_hz / sample_rate_hz;
        dsps_biquad_gen_lpf_f32(m_coeffs, norm_fc, 0.70710678f);
        reset();
    }

    void initHighPass(float cutoff_hz, float sample_rate_hz) {
        float norm_fc = cutoff_hz / sample_rate_hz;
        dsps_biquad_gen_hpf_f32(m_coeffs, norm_fc, 0.70710678f);
        reset();
    }

    void reset() {
        memset(m_w1, 0, sizeof(m_w1));
        memset(m_w2, 0, sizeof(m_w2));
    }

    void process(const float* in, float* out, int len) {
        if (!in || !out || len <= 0) return;
        dsps_biquad_f32(in, out, len, m_coeffs, m_w1);
        dsps_biquad_f32(out, out, len, m_coeffs, m_w2);
    }

private:
    float m_coeffs[5]{0.0f};
    float m_w1[2]{0.0f};
    float m_w2[2]{0.0f};
};

} // namespace DSP
