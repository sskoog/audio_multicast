#pragma once
/**
 * @file thdn_meter.hpp
 * @brief THD+N of a sine in 10 ms frames (SINK console `thdn`), integer-only so it is cheap on the C6.
 *
 * For a test frequency that is a multiple of 100 Hz, a 480-sample frame at 48 kHz holds a whole number of cycles,
 * so the sine and cosine at that frequency, and DC, are orthogonal over the frame. Per frame:
 *   fundamental energy = (S_s^2 + S_c^2) * 2 / N,   with S_s = sum x*sin, S_c = sum x*cos
 *   DC energy          = (sum x)^2 / N
 *   residual           = sum x^2 - fundamental - DC     (distortion + noise)
 * THD+N = 10 log10(residual / fundamental), accumulated over many frames. Frame-local fit, so phase drift
 * between frames (clock offset, sample edits) does not matter.
 */
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace Audio {

class ThdnMeter {
public:
    /** @brief Prepare for test frequency hz (multiple of 100 Hz, 100..20000) at 48 kHz, 480-sample frames. */
    bool configure(uint32_t hz) {
        if (hz < 100 || hz > 20000 || hz % 100 != 0) {
            return false;
        }
        for (int i = 0; i < N; ++i) {
            const double ph = 2.0 * 3.14159265358979 * hz * i / 48000.0;
            m_sin[i] = static_cast<int32_t>(std::lround(TBL * std::sin(ph)));
            m_cos[i] = static_cast<int32_t>(std::lround(TBL * std::cos(ph)));
        }
        reset();
        return true;
    }

    void reset() {
        m_fund = m_res = 0.0;
        m_frames = 0;
        m_peak = 0;
    }

    /**
     * @brief Add one frame of N samples, given as int32 at any scale (shift applied first to keep sums in range).
     * @param x      Samples (stride 1 or 2 for interleaved stereo).
     * @param stride Distance between samples.
     * @param shift  Right shift applied to each sample (e.g. 8 for 24-bit-in-32 data).
     */
    template <typename T>
    void addFrame(const T* x, int stride, int shift) {
        int64_t ss = 0, sc = 0, s1 = 0, s2 = 0;
        for (int i = 0; i < N; ++i) {
            const int32_t v = static_cast<int32_t>(x[i * stride]) >> shift;
            ss += static_cast<int64_t>(v) * m_sin[i];
            sc += static_cast<int64_t>(v) * m_cos[i];
            s1 += v;
            s2 += static_cast<int64_t>(v) * v;
            const int32_t a = v < 0 ? -v : v;
            if (a > m_peak) m_peak = a;
        }
        const double k = 1.0 / TBL;
        const double fs = ss * k, fc = sc * k;
        const double fund = (fs * fs + fc * fc) * 2.0 / N;
        const double dc = static_cast<double>(s1) * s1 / N;
        double res = static_cast<double>(s2) - fund - dc;
        if (res < 0) res = 0;
        m_fund += fund;
        m_res += res;
        m_frames++;
    }

    uint32_t frames() const { return m_frames; }
    int32_t peak() const { return m_peak; }
    /** @return THD+N in dB (residual / fundamental), or 0 with no data. */
    double thdnDb() const { return (m_frames && m_fund > 0) ? 10.0 * std::log10((m_res + 1e-12) / m_fund) : 0.0; }
    /** @return Fundamental RMS level in dB relative to full scale fs_peak (peak sample value of a full-scale sine). */
    double levelDbfs(double fs_peak) const {
        return (m_frames && m_fund > 0) ? 10.0 * std::log10(m_fund / (m_frames * N) / (fs_peak * fs_peak / 2.0)) : -200.0;
    }

    static constexpr int N = 480;

private:
    // Q24 tables: 16 / 24-bit samples x 2^24 x 480 stays within int64, and the table rounding (6e-8) keeps the
    // fundamental estimate accurate to ~-140 dB (Q15 tables made the residual go negative on clean signals)
    static constexpr double TBL = 16777216.0;
    int32_t  m_sin[N] = {}, m_cos[N] = {};
    double   m_fund = 0, m_res = 0;
    uint32_t m_frames = 0;
    int32_t  m_peak = 0;
};

}  // namespace Audio
