#include "tone_generator.hpp"
#include "esp_log.h"
#include "esp_random.h"
#include <cmath>

static const char* TAG = "TONE_GEN";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace Audio {

ToneGenerator::ToneGenerator() {}

ToneGenerator::~ToneGenerator() {}

void ToneGenerator::randomizeModRate() {
    // Generate pseudo-random float between VFO_MIN_MOD_RATE_HZ (0.5 Hz) and VFO_MAX_MOD_RATE_HZ (2.0 Hz)
    uint32_t r = esp_random();
    float norm = static_cast<float>(r) / static_cast<float>(UINT32_MAX);
    m_vfo_mod_rate_hz = VFO_MIN_MOD_RATE_HZ + norm * (VFO_MAX_MOD_RATE_HZ - VFO_MIN_MOD_RATE_HZ);
    m_cycle_count++;
    ESP_LOGD(TAG, "VFO Cycle #%lu Completed. New Modulation Rate = %.2f Hz", m_cycle_count, m_vfo_mod_rate_hz);
}

void ToneGenerator::set_gain_dB(float gain_db) {
    if (gain_db > 0.0f) {
        gain_db = 0.0f; // 0 dB max (unity digital scale)
    }

    if (gain_db <= -60.0f) {
        // Less than or equal to -60 dB: map towards mute
        if (gain_db <= -90.0f) {
            m_gain_db = -100.0f;
            m_gain_pct = 0.0f;
            m_peak_amplitude = 0;
            return;
        }
        m_gain_db = gain_db;
        m_gain_pct = 1.0f; // 1% corresponds to -60 dB
    } else {
        m_gain_db = gain_db;
        // Inverse linear mapping from dB [-60 dB, 0 dB] to pct [1.0%, 100.0%]
        m_gain_pct = 1.0f + (gain_db - (-60.0f)) * (99.0f / 60.0f);
    }

    float linear_scale = powf(10.0f, m_gain_db / 20.0f);
    m_peak_amplitude = static_cast<int16_t>(32767.0f * linear_scale + 0.5f);
}

void ToneGenerator::setNominalFrequency(float nominal_freq_hz) {
    if (nominal_freq_hz < 100.0f) nominal_freq_hz = 100.0f;
    if (nominal_freq_hz > 2000.0f) nominal_freq_hz = 2000.0f;
    m_center_freq_hz = nominal_freq_hz;
    m_min_freq_hz = nominal_freq_hz * 0.5f;
    m_max_freq_hz = nominal_freq_hz * 2.0f;
    m_freq_deviation_hz = (m_max_freq_hz - m_min_freq_hz) * 0.5f;
    ESP_LOGI(TAG, "VCO Nominal Frequency updated to %.1f Hz", m_center_freq_hz);
}

void ToneGenerator::set_gain_pct(float pct) {
    if (pct < 0.1f) {
        // 0% or <0.1% means fully muted
        m_gain_pct = 0.0f;
        m_gain_db = -100.0f;
        m_peak_amplitude = 0;
        return;
    }

    if (pct > 100.0f) {
        pct = 100.0f;
    }

    m_gain_pct = pct;
    float clamped_pct = (pct < 1.0f) ? 1.0f : pct;
    // Linear interpolation from pct [1.0%, 100.0%] to dB [-60 dB, 0 dB]
    m_gain_db = -60.0f + (clamped_pct - 1.0f) * (60.0f / 99.0f);

    float linear_scale = powf(10.0f, m_gain_db / 20.0f);
    m_peak_amplitude = static_cast<int16_t>(32767.0f * linear_scale + 0.5f);
}

static int16_t s_sine_table[512];
static bool s_sine_table_initialized = false;

esp_err_t ToneGenerator::init(uint32_t sample_rate_hz, float nominal_freq_hz, float min_freq_hz, float max_freq_hz, float amplitude_pct) {
    m_sample_rate = sample_rate_hz;
    m_min_freq_hz = min_freq_hz;
    m_max_freq_hz = max_freq_hz;
    m_center_freq_hz = (min_freq_hz + max_freq_hz) * 0.5f;
    m_freq_deviation_hz = (max_freq_hz - min_freq_hz) * 0.5f;
    m_carrier_phase = 0.0f;
    m_vfo_phase = 0.0f;
    m_cycle_count = 0;

    // Initialize 512-point Sine LUT once
    if (!s_sine_table_initialized) {
        for (int i = 0; i < 512; i++) {
            s_sine_table[i] = static_cast<int16_t>(sinf((2.0f * static_cast<float>(M_PI) * i) / 512.0f) * 32767.0f);
        }
        s_sine_table_initialized = true;
    }

    set_gain_pct(amplitude_pct);
    randomizeModRate();

    ESP_LOGI(TAG, "VCO Tone Generator Initialized: Fs=%lu Hz, Carrier=%.1f Hz (Sweep %.1f - %.1f Hz), Gain=%.2f dB (%.1f%%, Amp=%d), VFO=%.2f Hz",
             m_sample_rate, nominal_freq_hz, m_min_freq_hz, m_max_freq_hz, m_gain_db, m_gain_pct, m_peak_amplitude, m_vfo_mod_rate_hz);
    return ESP_OK;
}

size_t ToneGenerator::generateFrame(int16_t* out_pcm, size_t num_samples) {
    if (!out_pcm || num_samples == 0) return 0;

    const float two_pi = 2.0f * static_cast<float>(M_PI);
    const float dt = 1.0f / static_cast<float>(m_sample_rate);
    const float gain_factor = static_cast<float>(m_peak_amplitude) / 32767.0f;
    const float rad_to_lut = 512.0f / two_pi;

    for (size_t i = 0; i < num_samples; i++) {
        // Fast table lookup for VFO sine
        uint32_t vfo_lut_idx = static_cast<uint32_t>(m_vfo_phase * rad_to_lut) & 511;
        float vfo_val = static_cast<float>(s_sine_table[vfo_lut_idx]) / 32767.0f;

        m_current_freq_hz = m_center_freq_hz + m_freq_deviation_hz * vfo_val;

        // Fast table lookup for Carrier sine
        uint32_t carrier_lut_idx = static_cast<uint32_t>(m_carrier_phase * rad_to_lut) & 511;
        out_pcm[i] = static_cast<int16_t>(s_sine_table[carrier_lut_idx] * gain_factor);

        m_carrier_phase += two_pi * m_current_freq_hz * dt;
        if (m_carrier_phase >= two_pi) {
            m_carrier_phase -= two_pi;
        }

        m_vfo_phase += two_pi * m_vfo_mod_rate_hz * dt;
        if (m_vfo_phase >= two_pi) {
            m_vfo_phase -= two_pi;
            randomizeModRate();
        }
    }

    return num_samples;
}

} // namespace Audio
