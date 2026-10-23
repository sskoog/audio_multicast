#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "config.h"

namespace Audio {

class ToneGenerator {
public:
    ToneGenerator();
    ~ToneGenerator();

    esp_err_t init(uint32_t sample_rate_hz = AUDIO_SAMPLE_RATE_HZ, 
                   float nominal_freq_hz = VCO_NOMINAL_FREQ_HZ, 
                   float min_freq_hz = VCO_MIN_FREQ_HZ, 
                   float max_freq_hz = VCO_MAX_FREQ_HZ,
                   float amplitude_pct = VCO_AMPLITUDE_PERCENT);

    // Generates a frame of 16-bit mono PCM samples
    size_t generateFrame(int16_t* out_pcm, size_t num_samples);

    // Volume / Gain Control (Logarithmic mapping: 1% = -60 dB, 100% = 0 dB, <0.1% = Muted)
    void set_gain_dB(float gain_db);
    void set_gain_pct(float pct);
    void setNominalFrequency(float nominal_freq_hz);
    float get_gain_dB() const { return m_gain_db; }
    float get_gain_pct() const { return m_gain_pct; }

    // Telemetry getters
    float getCurrentFrequency() const { return m_current_freq_hz; }
    float getModulationRate() const { return m_vfo_mod_rate_hz; }
    uint32_t getCycleCount() const { return m_cycle_count; }

private:
    void randomizeModRate();

    uint32_t m_sample_rate = AUDIO_SAMPLE_RATE_HZ;
    float m_min_freq_hz = VCO_MIN_FREQ_HZ;
    float m_max_freq_hz = VCO_MAX_FREQ_HZ;
    float m_center_freq_hz = (VCO_MIN_FREQ_HZ + VCO_MAX_FREQ_HZ) * 0.5f;
    float m_freq_deviation_hz = (VCO_MAX_FREQ_HZ - VCO_MIN_FREQ_HZ) * 0.5f;
    int16_t m_peak_amplitude = VCO_PEAK_AMPLITUDE_INT16;
    float m_gain_db = -10.46f; // Nominal ~30% amplitude
    float m_gain_pct = VCO_AMPLITUDE_PERCENT;

    float m_carrier_phase = 0.0f;
    float m_vfo_phase = 0.0f;
    float m_vfo_mod_rate_hz = 1.0f; // Current modulation frequency (0.5 to 2.0 Hz)
    float m_current_freq_hz = VCO_NOMINAL_FREQ_HZ;
    uint32_t m_cycle_count = 0;
};

} // namespace Audio
