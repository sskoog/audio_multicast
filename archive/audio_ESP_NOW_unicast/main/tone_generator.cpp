#include "tone_generator.hpp"
#include "esp_log.h"
#include "esp_random.h"
#include <cmath>

static const char* TAG = "TONE_GEN";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace Audio {

static int16_t s_sine_table[1024];
static bool s_sine_table_initialized = false;

ToneGenerator::ToneGenerator(uint32_t sample_rate_hz)
    : m_sample_rate(sample_rate_hz) {
    init(sample_rate_hz, 220.0f, 110.0f, 440.0f, 50.0f);
}

ToneGenerator::~ToneGenerator() {}

void ToneGenerator::randomizeModRate() {
    uint32_t r = esp_random();
    float norm = static_cast<float>(r) / static_cast<float>(UINT32_MAX);
    m_vfo_mod_rate_hz = 0.2f + norm * (1.5f - 0.2f); // 0.2 Hz to 1.5 Hz smooth sweep
    m_cycle_count++;
}

void ToneGenerator::set_gain_dB(float gain_db) {
    if (gain_db > 0.0f) gain_db = 0.0f;
    if (gain_db <= -60.0f) {
        if (gain_db <= -90.0f) {
            m_gain_db = -100.0f;
            m_gain_pct = 0.0f;
            m_peak_amplitude = 0;
            return;
        }
        m_gain_db = gain_db;
        m_gain_pct = 1.0f;
    } else {
        m_gain_db = gain_db;
        m_gain_pct = 1.0f + (gain_db - (-60.0f)) * (99.0f / 60.0f);
    }
    float linear_scale = powf(10.0f, m_gain_db / 20.0f);
    m_peak_amplitude = static_cast<int16_t>(32767.0f * linear_scale + 0.5f);
}

void ToneGenerator::setNominalFrequency(float nominal_freq_hz) {
    if (nominal_freq_hz < 50.0f) nominal_freq_hz = 50.0f;
    if (nominal_freq_hz > 4000.0f) nominal_freq_hz = 4000.0f;
    m_center_freq_hz = nominal_freq_hz;
    m_min_freq_hz = nominal_freq_hz * 0.75f;
    m_max_freq_hz = nominal_freq_hz * 1.25f;
    m_freq_deviation_hz = (m_max_freq_hz - m_min_freq_hz) * 0.5f;
    ESP_LOGI(TAG, "Tone Generator Center Frequency set to %.1f Hz", m_center_freq_hz);
}

void ToneGenerator::setSampleRate(uint32_t sample_rate_hz) {
    if (sample_rate_hz == 0 || sample_rate_hz == m_sample_rate) return;
    m_sample_rate = sample_rate_hz;
    ESP_LOGI(TAG, "Tone Generator Sample Rate updated to %lu Hz", (unsigned long)m_sample_rate);
}

void ToneGenerator::set_gain_pct(float pct) {
    if (pct < 0.1f) {
        m_gain_pct = 0.0f;
        m_gain_db = -100.0f;
        m_peak_amplitude = 0;
        return;
    }
    if (pct > 100.0f) pct = 100.0f;
    m_gain_pct = pct;
    float clamped_pct = (pct < 1.0f) ? 1.0f : pct;
    m_gain_db = -60.0f + (clamped_pct - 1.0f) * (60.0f / 99.0f);
    float linear_scale = powf(10.0f, m_gain_db / 20.0f);
    m_peak_amplitude = static_cast<int16_t>(32767.0f * linear_scale + 0.5f);
}

esp_err_t ToneGenerator::init(uint32_t sample_rate_hz, float nominal_freq_hz, float min_freq_hz, float max_freq_hz, float amplitude_pct) {
    m_sample_rate = sample_rate_hz;
    m_min_freq_hz = min_freq_hz;
    m_max_freq_hz = max_freq_hz;
    m_center_freq_hz = (min_freq_hz + max_freq_hz) * 0.5f;
    m_freq_deviation_hz = (max_freq_hz - min_freq_hz) * 0.5f;
    m_carrier_phase = 0.0f;
    m_vfo_phase = 0.0f;
    m_cycle_count = 0;

    if (!s_sine_table_initialized) {
        for (int i = 0; i < 1024; i++) {
            s_sine_table[i] = static_cast<int16_t>(sinf((2.0f * static_cast<float>(M_PI) * i) / 1024.0f) * 32767.0f);
        }
        s_sine_table_initialized = true;
    }

    set_gain_pct(amplitude_pct);
    randomizeModRate();

    ESP_LOGI(TAG, "Continuous Sine+LFO Tone Generator Initialized: Fs=%lu Hz, Center=%.1f Hz, LFO=%.2f Hz, Gain=%.2f dB",
             (unsigned long)m_sample_rate, m_center_freq_hz, m_vfo_mod_rate_hz, m_gain_db);
    return ESP_OK;
}

size_t ToneGenerator::generateFrame(int16_t* out_pcm, size_t num_samples) {
    if (!out_pcm || num_samples == 0) return 0;

    const float two_pi = 2.0f * static_cast<float>(M_PI);
    const float dt = 1.0f / static_cast<float>(m_sample_rate);
    const float gain_factor = static_cast<float>(m_peak_amplitude) / 32767.0f;
    const float rad_to_lut = 1024.0f / two_pi;

    for (size_t i = 0; i < num_samples; i++) {
        // Fast table lookup for LFO sine
        uint32_t vfo_lut_idx = static_cast<uint32_t>(m_vfo_phase * rad_to_lut) & 1023;
        float vfo_val = static_cast<float>(s_sine_table[vfo_lut_idx]) / 32767.0f;

        m_current_freq_hz = m_center_freq_hz + m_freq_deviation_hz * vfo_val;

        // High-precision Carrier lookup
        uint32_t carrier_lut_idx = static_cast<uint32_t>(m_carrier_phase * rad_to_lut) & 1023;
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
