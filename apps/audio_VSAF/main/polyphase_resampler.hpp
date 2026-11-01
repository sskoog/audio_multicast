#pragma once
/**
 * @file polyphase_resampler.hpp
 * @brief Rational polyphase FIR resampler between the standard LC3 rates (8, 16, 24, 32, 48 kHz), integer only.
 *
 * Used for the subwoofer stream: on the SINK from the stream rate to the 48 kHz DAC rate (replacing the sample
 * repeat), on the SOURCE from the 8 kHz sub DSP rate to a higher sub stream rate (CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ).
 *
 * out / in = L / M (reduced). Prototype low-pass: Kaiser-windowed sinc (beta 7, about 70 dB stopband), L x 16 taps at
 * in x L, cutoff 0.45 x min(in, out). Each of the L polyphase branches is normalized to unity DC gain (no ripple on
 * steady signals). Coefficients Q20 (int32), 64-bit accumulation: no FPU needed in process() (C6), deterministic.
 * configure() designs the filter (float, a few ms on a C6) and must run outside the per-frame path or only on a rate
 * change; process() never allocates. One call = one 10 ms frame: n_in x L is a multiple of M for 10 ms frames of
 * standard rates, so every frame starts on output phase 0.
 */
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Audio {

template <size_t MaxIn>
class PolyphaseResampler {
public:
    static constexpr int kTapsPerPhase = 16;  // taps per polyphase branch
    static constexpr int kMaxL = 6;           // 8 -> 48 kHz

    /** @brief Greatest common divisor (rates in Hz). */
    static constexpr uint32_t gcd(uint32_t a, uint32_t b) { return b == 0 ? a : gcd(b, a % b); }

    /** @brief Standard rate this class supports: 8, 16, 24, 32 or 48 kHz. */
    static constexpr bool isStandardRate(uint32_t hz) {
        return hz == 8000 || hz == 16000 || hz == 24000 || hz == 32000 || hz == 48000;
    }

    /**
     * @brief Group delay of the filter for in -> out (us); 0 when in == out (pass-through).
     * @param in_hz  Input rate [Hz].
     * @param out_hz Output rate [Hz].
     */
    static float delayUsFor(uint32_t in_hz, uint32_t out_hz) {
        if (in_hz == out_hz || in_hz == 0 || out_hz == 0) return 0.0f;
        const uint32_t g = gcd(in_hz, out_hz);
        const uint32_t L = out_hz / g;
        const float n_taps = static_cast<float>(L * kTapsPerPhase);
        return (n_taps - 1.0f) * 0.5f / static_cast<float>(in_hz * L) * 1.0e6f;
    }

    /**
     * @brief Design the filter for in -> out and clear the history.
     * @param in_hz  Input rate, standard rate [Hz].
     * @param out_hz Output rate, standard rate [Hz].
     * @return false (and pass-through) if a rate is not supported.
     */
    bool configure(uint32_t in_hz, uint32_t out_hz) {
        in_hz_ = in_hz;
        out_hz_ = out_hz;
        reset();
        if (!isStandardRate(in_hz) || !isStandardRate(out_hz)) {
            L_ = M_ = 1;
            return false;
        }
        const uint32_t g = gcd(in_hz, out_hz);
        L_ = static_cast<int>(out_hz / g);
        M_ = static_cast<int>(in_hz / g);
        if (L_ == 1 && M_ == 1) return true;  // pass-through
        if (L_ > kMaxL) {
            L_ = M_ = 1;
            return false;
        }
        const int n = L_ * kTapsPerPhase;
        const double fs_up = static_cast<double>(in_hz) * L_;
        const double fc = 0.45 * static_cast<double>(std::min(in_hz, out_hz)) / fs_up;  // cycles per upsampled sample
        const double beta = 7.0;
        const double i0b = besselI0(beta);
        double h[kMaxL * kTapsPerPhase];
        for (int j = 0; j < n; ++j) {
            const double t = j - (n - 1) * 0.5;
            const double x = 2.0 * fc * t;
            const double sinc = (std::fabs(x) < 1e-12) ? 1.0 : std::sin(kPi * x) / (kPi * x);
            const double r = 2.0 * j / (n - 1) - 1.0;
            const double w = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
            h[j] = 2.0 * fc * sinc * w;
        }
        // Polyphase branch p = taps p, p + L, p + 2L, ...; each normalized to a DC gain of 1
        for (int p = 0; p < L_; ++p) {
            double sum = 0.0;
            for (int k = 0; k < kTapsPerPhase; ++k) sum += h[p + k * L_];
            const double s = (std::fabs(sum) > 1e-12) ? 1.0 / sum : 0.0;
            for (int k = 0; k < kTapsPerPhase; ++k) {
                coef_[p * kTapsPerPhase + k] = static_cast<int32_t>(std::lround(h[p + k * L_] * s * kOne));
            }
        }
        return true;
    }

    /** @brief Clear the history (stream start, channel change). */
    void reset() { std::memset(hist_, 0, sizeof(hist_)); }

    bool passthrough() const { return L_ == 1 && M_ == 1; }
    uint32_t inRate() const { return in_hz_; }
    uint32_t outRate() const { return out_hz_; }
    float delayUs() const { return passthrough() ? 0.0f : delayUsFor(in_hz_, out_hz_); }

    /**
     * @brief Resample one frame. in and out may be the same buffer (the input is copied first).
     * @param in    Input samples (n_in <= MaxIn) [PCM].
     * @param n_in  Input samples per frame [samples].
     * @param out   Output, room for n_in x L / M samples [PCM].
     * @param lo    Lowest output value (clip) [PCM].
     * @param hi    Highest output value (clip) [PCM].
     * @return Output samples written.
     */
    template <typename T>
    size_t process(const T* in, size_t n_in, T* out, int32_t lo, int32_t hi) {
        n_in = std::min(n_in, MaxIn);
        if (passthrough()) {
            if (out != in) std::memmove(out, in, n_in * sizeof(T));
            return n_in;
        }
        constexpr int H = kTapsPerPhase - 1;
        int32_t* x = hist_;  // x[0 .. H-1] = previous frame's last samples, x[H ..] = this frame
        for (size_t i = 0; i < n_in; ++i) x[H + i] = static_cast<int32_t>(in[i]);
        const size_t n_out = n_in * static_cast<size_t>(L_) / static_cast<size_t>(M_);
        for (size_t o = 0; o < n_out; ++o) {
            const size_t u = o * static_cast<size_t>(M_);   // position on the upsampled grid
            const size_t i = u / static_cast<size_t>(L_);   // newest input sample in reach
            const int32_t* c = &coef_[(u % static_cast<size_t>(L_)) * kTapsPerPhase];
            const int32_t* xs = &x[H + i];
            int64_t acc = 0;
            for (int k = 0; k < kTapsPerPhase; ++k) acc += static_cast<int64_t>(c[k]) * xs[-k];
            const int64_t y = (acc + (1 << (kShift - 1))) >> kShift;
            out[o] = static_cast<T>(std::clamp<int64_t>(y, lo, hi));
        }
        std::memmove(x, &x[n_in], H * sizeof(int32_t));  // keep the last H input samples
        return n_out;
    }

private:
    static constexpr int kShift = 20;
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kOne = static_cast<double>(1 << kShift);

    static double besselI0(double x) {  // modified Bessel function of the first kind, order 0 (series)
        double sum = 1.0, term = 1.0;
        for (int k = 1; k < 30; ++k) {
            term *= (x / (2.0 * k)) * (x / (2.0 * k));
            sum += term;
            if (term < 1e-12 * sum) break;
        }
        return sum;
    }

    int L_ = 1, M_ = 1;
    uint32_t in_hz_ = 0, out_hz_ = 0;
    int32_t coef_[kMaxL * kTapsPerPhase] = {};
    int32_t hist_[kTapsPerPhase - 1 + MaxIn] = {};
};

}  // namespace Audio
