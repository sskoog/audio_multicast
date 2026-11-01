#pragma once
/**
 * @file cal_signal.hpp
 * @brief SOURCE test tones pink / sine (parameter cal.mode, console `caltone`), e.g. for speaker level matching.
 *
 * Generated as the 48 kHz mono program input, exactly like the test synth: they go through the master volume,
 * the crossover, the spatial upmix and the per-channel gains like any other input. One frame is the sum of
 * - a satellite band: pink noise 500 Hz - 2 kHz (mid band, suits A-weighting and keeps room modes / tweeter
 *   differences out of the reading), or a sine (default 1 kHz);
 * - a sub band: pink noise 40 - 100 Hz, or a sine (default 63 Hz), which the crossover sends to the sub.
 * Each band has the same RMS level (configure(), the SOURCE uses a fixed reference; the master volume sets
 * the playback level). Noise is preferred over a sine: a sine in a room produces standing waves, so the
 * reading depends strongly on the meter position.
 *
 * Static state only, no allocation; runs in the SOURCE DSP stage (core 0).
 */
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

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
            m_sub.design(48000.0f, 40.0f, 100.0f);
            m_sub.rng = 0x9E3779B9u;  // own noise sequence (uncorrelated with the satellite band)
            // Normalize the band-limited noise to the requested RMS. Computed, not measured: the generator is a
            // linear filter chain on white noise, so its output RMS is exactly sqrt(var_white * sum(h[n]^2)).
            // (Measuring 2 s of noise took ~0.6 s of CPU in the caller's task, which starved the idle task on a
            // busy core 0 past the 1 s task watchdog: the SOURCE rebooted when pink was selected during audio.)
            m_sat_gain = rms / m_sat.outputRms();
            m_sub_gain = rms / m_sub.outputRms();
        }
    }

    Mode  mode() const { return m_mode; }
    float rmsDbfs() const { return m_rms_dbfs; }
    float satHz() const { return m_sat_hz; }
    float subHz() const { return m_sub_hz; }

    /**
     * @brief One 48 kHz mono program-input frame: satellite band + sub band.
     * @param out Output PCM (int16, 48 kHz) [samples].
     * @param n   Samples (480 per 10 ms frame) [samples].
     */
    void frame48(int16_t* out, size_t n) {
        if (m_mode == Mode::SINE) {
            const float inc_sat = 6.28318531f * m_sat_hz / 48000.0f;
            const float inc_sub = 6.28318531f * m_sub_hz / 48000.0f;
            for (size_t i = 0; i < n; ++i) {
                out[i] = clamp16(m_sat_gain * std::sin(m_sat_phase) + m_sub_gain * std::sin(m_sub_phase));
                m_sat_phase += inc_sat;
                if (m_sat_phase > 6.28318531f) m_sat_phase -= 6.28318531f;
                m_sub_phase += inc_sub;
                if (m_sub_phase > 6.28318531f) m_sub_phase -= 6.28318531f;
            }
        } else if (m_mode == Mode::PINK) {
            for (size_t i = 0; i < n; ++i) {
                out[i] = clamp16(m_sat_gain * m_sat.next() + m_sub_gain * m_sub.next());
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
            return filter(static_cast<float>(static_cast<int32_t>(rng)) * (1.0f / 2147483648.0f));
        }
        /** @brief Pink filter + band-pass for one white-noise sample (linear, time-invariant). */
        float filter(float white) {
            p0 = 0.99765f * p0 + white * 0.0990460f;
            p1 = 0.96300f * p1 + white * 0.2965164f;
            p2 = 0.57000f * p2 + white * 1.0526913f;
            const float pink = p0 + p1 + p2 + white * 0.1848f;
            return lp2.run(lp1.run(hp2.run(hp1.run(pink))));
        }
        /**
         * @brief Expected output RMS for the white-noise input of next() (uniform in [-1, 1): variance 1/3).
         * Linear chain: RMS = sqrt(var * sum(h[n]^2)), h = impulse response of filter(). 8192 samples: the
         * slowest pole (0.99765, pink filter) has decayed to ~4e-9 by then. A few ms of CPU, no state change.
         */
        float outputRms() const {
            BandPink g = *this;  // same coefficients, own (cleared) state
            g.p0 = g.p1 = g.p2 = 0;
            for (Biquad* b : {&g.hp1, &g.hp2, &g.lp1, &g.lp2}) b->x1 = b->x2 = b->y1 = b->y2 = 0;
            float energy = 0.0f;
            for (int n = 0; n < 8192; ++n) {
                const float h = g.filter(n == 0 ? 1.0f : 0.0f);
                energy += h * h;
            }
            const float rms = std::sqrt(energy / 3.0f);
            return rms > 1e-9f ? rms : 1.0f;
        }
    };

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
