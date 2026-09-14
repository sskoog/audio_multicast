#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"

namespace Audio {

class ToneGenerator {
public:
    ToneGenerator(uint32_t sample_rate_hz = 32000);
    ~ToneGenerator();

    esp_err_t init(uint32_t sample_rate_hz = 32000, 
                   float nominal_freq_hz = 220.0f, 
                   float min_freq_hz = 110.0f, 
                   float max_freq_hz = 440.0f,
                   float amplitude_pct = 50.0f);

    size_t generateFrame(int16_t* out_pcm, size_t num_samples);

    void set_gain_dB(float gain_db);
    void set_gain_pct(float pct);
    void setNominalFrequency(float nominal_freq_hz);
    void setSampleRate(uint32_t sample_rate_hz);
    uint32_t getSampleRate() const { return m_sample_rate; }
    float get_gain_dB() const { return m_gain_db; }
    float get_gain_pct() const { return m_gain_pct; }
    float getCurrentFrequency() const { return m_current_freq_hz; }
    float getModulationRate() const { return m_vfo_mod_rate_hz; }

private:
    void randomizeModRate();

    uint32_t m_sample_rate = 32000;
    float m_min_freq_hz = 110.0f;
    float m_max_freq_hz = 440.0f;
    float m_center_freq_hz = 220.0f;
    float m_freq_deviation_hz = 110.0f;
    int16_t m_peak_amplitude = 16384; // 50%
    float m_gain_db = -6.02f;
    float m_gain_pct = 50.0f;

    float m_carrier_phase = 0.0f;
    float m_vfo_phase = 0.0f;
    float m_vfo_mod_rate_hz = 0.5f; // Current LFO rate (0.2 to 1.5 Hz)
    float m_current_freq_hz = 440.0f;
    uint32_t m_cycle_count = 0;
};

} // namespace Audio
