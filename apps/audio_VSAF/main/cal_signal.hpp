#pragma once
/**
 * @file cal_signal.hpp
 * @brief SOURCE calibration signal for speaker level matching with an SPL meter (console `caltone`).
 *
 * Replaces the program input on the SOURCE (bypassing the crossover):
 * - satellites (ch 0..4): band-limited pink noise 500 Hz - 2 kHz (mid band, suits A-weighting and keeps
 *   room modes / tweeter differences out of the reading), or a sine (default 1 kHz);
 * - subwoofer (ch 5, 8 kHz stream): band-limited pink noise 40 - 100 Hz, or a sine (default 63 Hz).
 * Both are scaled to the same RMS level in dBFS. Noise is preferred over a sine: a sine in a room produces
 * standing waves, so the reading depends strongly on the meter position.
 *
 * Static state only, no allocation; runs in the SOURCE DSP stage (core 0).
 */
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace Audio {

class CalSignal {
public:
    enum class Mode : uint8_t { OFF = 0, PINK = 1, SINE = 2 };

    /**
     * @brief Configure the signal.
     * @param mode     OFF / PINK / SINE.
     * @param rms_dbfs RMS level of each signal (dBFS, sine and noise alike), e.g. -20.
     * @param sat_hz   Satellite sine frequency (Hz), SINE mode only.
     * @param sub_hz   Sub sine frequency (Hz), SINE mode only.
     */
    void configure(Mode mode, float rms_dbfs, float sat_hz, float sub_hz) {
        m_mode = mode;
        m_rms_dbfs = rms_dbfs;
        m_sat_hz = sat_hz;
        m_sub_hz = sub_hz;
        const float rms = 32767.0f * std::pow(10.0f, rms_dbfs / 20.0f);
        m_sat_phase = m_sub_phase = 0.0f;
        if (mode == Mode::SINE) {
            m_sat_gain = rms * 1.41421356f;  // peak = rms * sqrt(2)
            m_sub_gain = m_sat_gain;
        } else if (mode == Mode::PINK) {
            m_sat.design(48000.0f, 500.0f, 2000.0f);
            m_sub.design(8000.0f, 40.0f, 100.0f);
            // Normalize the band-limited noise to the requested RMS (measured over 2 s of the same generator)
            m_sat_gain = rms / measureRms(m_sat, 96000);
            m_sub_gain = rms / measureRms(m_sub, 16000);
        }
    }

    Mode  mode() const { return m_mode; }
    float rmsDbfs() const { return m_rms_dbfs; }
    float satHz() const { return m_sat_hz; }
    float subHz() const { return m_sub_hz; }

    /** @brief One satellite frame (48 kHz mono, n samples) into out. */
    void satFrame(int16_t* out, size_t n) {
        if (m_mode == Mode::SINE) {
            const float inc = 6.28318531f * m_sat_hz / 48000.0f;
            for (size_t i = 0; i < n; ++i) {
                out[i] = clamp16(m_sat_gain * std::sin(m_sat_phase));
                m_sat_phase += inc;
                if (m_sat_phase > 6.28318531f) m_sat_phase -= 6.28318531f;
            }
        } else if (m_mode == Mode::PINK) {
            for (size_t i = 0; i < n; ++i) {
                out[i] = clamp16(m_sat_gain * m_sat.next());
            }
        } else {
            for (size_t i = 0; i < n; ++i) out[i] = 0;
        }
    }

    /** @brief One sub frame (8 kHz mono, n samples) into out. */
    void subFrame(int16_t* out, size_t n) {
        if (m_mode == Mode::SINE) {
            const float inc = 6.28318531f * m_sub_hz / 8000.0f;
            for (size_t i = 0; i < n; ++i) {
                out[i] = clamp16(m_sub_gain * std::sin(m_sub_phase));
                m_sub_phase += inc;
                if (m_sub_phase > 6.28318531f) m_sub_phase -= 6.28318531f;
            }
        } else if (m_mode == Mode::PINK) {
            for (size_t i = 0; i < n; ++i) {
                out[i] = clamp16(m_sub_gain * m_sub.next());
            }
        } else {
            for (size_t i = 0; i < n; ++i) out[i] = 0;
        }
    }

private:
    // 2nd-order section (RBJ cookbook, Butterworth Q), direct form I
    struct Biquad {
        float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        void design(bool highpass, float fs, float fc) {
            const float w = 6.28318531f * fc / fs, c = std::cos(w), alpha = std::sin(w) / (2.0f * 0.70710678f);
            const float a0 = 1.0f + alpha;
            if (highpass) {
                b0 = (1.0f + c) / 2.0f / a0; b1 = -(1.0f + c) / a0; b2 = b0;
            } else {
                b0 = (1.0f - c) / 2.0f / a0; b1 = (1.0f - c) / a0; b2 = b0;
            }
            a1 = -2.0f * c / a0; a2 = (1.0f - alpha) / a0;
            x1 = x2 = y1 = y2 = 0;
        }
        float run(float x) {
            const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1; x1 = x; y2 = y1; y1 = y;
            return y;
        }
    };

    // Pink noise (Paul Kellet's economy filter on xorshift white noise) through a 4th-order band-pass
    struct BandPink {
        uint32_t rng = 0x12345678u;
        float p0 = 0, p1 = 0, p2 = 0;
        Biquad hp1, hp2, lp1, lp2;
        void design(float fs, float lo, float hi) {
            hp1.design(true, fs, lo); hp2.design(true, fs, lo);
            lp1.design(false, fs, hi); lp2.design(false, fs, hi);
            p0 = p1 = p2 = 0;
        }
        float next() {
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            const float white = static_cast<float>(static_cast<int32_t>(rng)) * (1.0f / 2147483648.0f);
            p0 = 0.99765f * p0 + white * 0.0990460f;
            p1 = 0.96300f * p1 + white * 0.2965164f;
            p2 = 0.57000f * p2 + white * 1.0526913f;
            const float pink = p0 + p1 + p2 + white * 0.1848f;
            return lp2.run(lp1.run(hp2.run(hp1.run(pink))));
        }
    };

    static float measureRms(BandPink& g, int n) {
        for (int i = 0; i < n / 4; ++i) g.next();  // let the filters settle
        double acc = 0;
        for (int i = 0; i < n; ++i) {
            const float v = g.next();
            acc += static_cast<double>(v) * v;
        }
        const float rms = static_cast<float>(std::sqrt(acc / n));
        return rms > 1e-9f ? rms : 1.0f;
    }

    static int16_t clamp16(float v) {
        if (v > 32767.0f) return 32767;
        if (v < -32768.0f) return -32768;
        return static_cast<int16_t>(v);
    }

    Mode     m_mode = Mode::OFF;
    float    m_rms_dbfs = -20.0f, m_sat_hz = 1000.0f, m_sub_hz = 63.0f;
    float    m_sat_gain = 0, m_sub_gain = 0, m_sat_phase = 0, m_sub_phase = 0;
    BandPink m_sat, m_sub;
};

}  // namespace Audio
