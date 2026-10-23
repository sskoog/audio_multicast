#include "dsp_filter.hpp"
#include <cmath>
#include "esp_log.h"

static const char* TAG = "DSP_FILTER";

namespace Audio {

DspHighPassFilter::DspHighPassFilter() {
    init(80.0f, 48000.0f);
}

esp_err_t DspHighPassFilter::init(float cutoff_freq, float sample_rate) {
    m_cutoff_freq = cutoff_freq;
    m_sample_rate = sample_rate;

    // Calculate 2nd-order Butterworth High-Pass Biquad Coefficients (Q = 0.7071)
    const float Q = 0.70710678f;
    const float w0 = 2.0f * M_PI * (cutoff_freq / sample_rate);
    const float cos_w0 = std::cos(w0);
    const float sin_w0 = std::sin(w0);
    const float alpha = sin_w0 / (2.0f * Q);

    // High-Pass Filter Biquad Coefficients
    const float b0 = (1.0f + cos_w0) / 2.0f;
    const float b1 = -(1.0f + cos_w0);
    const float b2 = (1.0f + cos_w0) / 2.0f;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cos_w0;
    const float a2 = 1.0f - alpha;

    // Normalize coefficients by a0
    const float norm_b0 = b0 / a0;
    const float norm_b1 = b1 / a0;
    const float norm_b2 = b2 / a0;
    const float norm_a1 = a1 / a0;
    const float norm_a2 = a2 / a0;

    // Convert normalized floating-point coefficients to Q31 fixed-point integers
    const float Q31_SCALE = 2147483647.0f;
    m_b0_q31 = static_cast<int32_t>(norm_b0 * Q31_SCALE);
    m_b1_q31 = static_cast<int32_t>(norm_b1 * Q31_SCALE);
    m_b2_q31 = static_cast<int32_t>(norm_b2 * Q31_SCALE);
    m_a1_q31 = static_cast<int32_t>(norm_a1 * Q31_SCALE);
    m_a2_q31 = static_cast<int32_t>(norm_a2 * Q31_SCALE);

    // Reset delay states
    m_w1_l = m_w2_l = 0;
    m_w1_r = m_w2_r = 0;

    ESP_LOGI(TAG, "2nd-Order HPF Initialized (Top Speaker Mode): fc=%.1f Hz, fs=%.1f Hz (Fixed-Point Q31)", cutoff_freq, sample_rate);
    return ESP_OK;
}

void DspHighPassFilter::process(int16_t* pcm_buffer, size_t num_samples) {
    if (!pcm_buffer || num_samples == 0) return;

    // Process interleaved 16-bit PCM samples using Q31 Direct Form II transposed IIR filter
    for (size_t i = 0; i < num_samples; i += 2) {
        // Left Channel
        int32_t x_l = static_cast<int32_t>(pcm_buffer[i]) << 16; // Convert Q15 to Q31
        
        int64_t y_l_acc = (static_cast<int64_t>(x_l) * m_b0_q31) >> 31;
        y_l_acc += m_w1_l;
        int32_t y_l = static_cast<int32_t>(y_l_acc);

        m_w1_l = static_cast<int32_t>(((static_cast<int64_t>(x_l) * m_b1_q31) >> 31) - ((static_cast<int64_t>(y_l) * m_a1_q31) >> 31) + m_w2_l);
        m_w2_l = static_cast<int32_t>(((static_cast<int64_t>(x_l) * m_b2_q31) >> 31) - ((static_cast<int64_t>(y_l) * m_a2_q31) >> 31));

        pcm_buffer[i] = static_cast<int16_t>(y_l >> 16); // Convert Q31 back to Q15

        // Right Channel (if stereo)
        if (i + 1 < num_samples) {
            int32_t x_r = static_cast<int32_t>(pcm_buffer[i + 1]) << 16;

            int64_t y_r_acc = (static_cast<int64_t>(x_r) * m_b0_q31) >> 31;
            y_r_acc += m_w1_r;
            int32_t y_r = static_cast<int32_t>(y_r_acc);

            m_w1_r = static_cast<int32_t>(((static_cast<int64_t>(x_r) * m_b1_q31) >> 31) - ((static_cast<int64_t>(y_r) * m_a1_q31) >> 31) + m_w2_r);
            m_w2_r = static_cast<int32_t>(((static_cast<int64_t>(x_r) * m_b2_q31) >> 31) - ((static_cast<int64_t>(y_r) * m_a2_q31) >> 31));

            pcm_buffer[i + 1] = static_cast<int16_t>(y_r >> 16);
        }
    }
}

} // namespace Audio
