#include "dsp_primitives.hpp"

#include <cmath>

namespace audio_dsp {

/**
 * Single RAM-resident 130-entry quarter-wave sine lookup table (0..128 + guard tap).
 * Placed in .dram1.data on ESP32-S3 via DSP_DRAM_ATTR so all table reads execute
 * from single-cycle internal SRAM with zero Flash cache-miss stalls.
 */
alignas(16) DSP_DRAM_ATTR float g_fast_sin_table[130] = {
    0.000000000e+00f, 1.227153838e-02f, 2.454122901e-02f, 3.680722415e-02f, 4.906767607e-02f, 6.132073700e-02f,
    7.356456667e-02f, 8.579730988e-02f, 9.801714122e-02f, 1.102222055e-01f, 1.224106699e-01f, 1.345807016e-01f,
    1.467304677e-01f, 1.588581502e-01f, 1.709618866e-01f, 1.830398887e-01f, 1.950903237e-01f, 2.071113735e-01f,
    2.191012353e-01f, 2.310581207e-01f, 2.429801822e-01f, 2.548656464e-01f, 2.667127550e-01f, 2.785196900e-01f,
    2.902846634e-01f, 3.020059466e-01f, 3.136817515e-01f, 3.253102899e-01f, 3.368898332e-01f, 3.484186828e-01f,
    3.598950505e-01f, 3.713172078e-01f, 3.826834559e-01f, 3.939920366e-01f, 4.052413106e-01f, 4.164295495e-01f,
    4.275550842e-01f, 4.386162460e-01f, 4.496113360e-01f, 4.605387151e-01f, 4.713967443e-01f, 4.821837842e-01f,
    4.928981662e-01f, 5.035383701e-01f, 5.141027570e-01f, 5.245897174e-01f, 5.349976420e-01f, 5.453249812e-01f,
    5.555702448e-01f, 5.657317638e-01f, 5.758082271e-01f, 5.857978463e-01f, 5.956993103e-01f, 6.055110097e-01f,
    6.152315736e-01f, 6.248595119e-01f, 6.343932748e-01f, 6.438315511e-01f, 6.531728506e-01f, 6.624158025e-01f,
    6.715589762e-01f, 6.806010008e-01f, 6.895405650e-01f, 6.983762383e-01f, 7.071067691e-01f, 7.157308459e-01f,
    7.242470980e-01f, 7.326542735e-01f, 7.409511209e-01f, 7.491364479e-01f, 7.572088838e-01f, 7.651672959e-01f,
    7.730104327e-01f, 7.807372212e-01f, 7.883464694e-01f, 7.958369255e-01f, 8.032075763e-01f, 8.104572296e-01f,
    8.175848126e-01f, 8.245893121e-01f, 8.314695954e-01f, 8.382247090e-01f, 8.448535800e-01f, 8.513551950e-01f,
    8.577286005e-01f, 8.639728427e-01f, 8.700870275e-01f, 8.760701418e-01f, 8.819212317e-01f, 8.876396418e-01f,
    8.932242990e-01f, 8.986744285e-01f, 9.039892554e-01f, 9.091680050e-01f, 9.142097831e-01f, 9.191138148e-01f,
    9.238795042e-01f, 9.285060763e-01f, 9.329927564e-01f, 9.373390079e-01f, 9.415440559e-01f, 9.456073642e-01f,
    9.495281577e-01f, 9.533060193e-01f, 9.569403529e-01f, 9.604305029e-01f, 9.637760520e-01f, 9.669764638e-01f,
    9.700312614e-01f, 9.729399681e-01f, 9.757021070e-01f, 9.783173800e-01f, 9.807852507e-01f, 9.831054807e-01f,
    9.852776527e-01f, 9.873014092e-01f, 9.891765118e-01f, 9.909026623e-01f, 9.924795628e-01f, 9.939069748e-01f,
    9.951847196e-01f, 9.963126183e-01f, 9.972904325e-01f, 9.981181026e-01f, 9.987954497e-01f, 9.993224144e-01f,
    9.996988177e-01f, 9.999247193e-01f, 1.000000000e+00f, 1.000000000e+00f
};

uint32_t FastSinLUT::deg_to_u32(float deg) noexcept {
    double turns = static_cast<double>(deg) / 360.0;
    turns -= std::floor(turns);
    const int64_t ticks = std::llround(turns * 4294967296.0);
    return static_cast<uint32_t>(ticks & 0xFFFFFFFFLL);
}

uint32_t FastSinLUT::rad_to_u32(float rad) noexcept {
    constexpr double kInvTwoPi = 1.0 / 6.28318530717958647692;
    double turns = static_cast<double>(rad) * kInvTwoPi;
    turns -= std::floor(turns);
    const int64_t ticks = std::llround(turns * 4294967296.0);
    return static_cast<uint32_t>(ticks & 0xFFFFFFFFLL);
}

uint32_t FastSinLUT::hz_to_inc_u32(float freq_hz, int fs) noexcept {
    if (fs <= 0) {
        return 0u;
    }
    const double scaled = static_cast<double>(freq_hz) * (4294967296.0 / static_cast<double>(fs));
    const int64_t rounded = std::llround(scaled);
    return static_cast<uint32_t>(static_cast<int32_t>(rounded));
}

/**
 * @brief Design a 1st-order bilinear Low-Pass filter section.
 *
 * @param cutoff_hz -3 dB cutoff frequency in Hertz [Hz].
 * @param fs Audio sampling rate in Hertz [Hz].
 * @return BiquadDF2T Configured DF2T filter section [dimensionless struct].
 */
BiquadDF2T BiquadDF2T::design_first_order_lpf(float cutoff_hz, int fs) noexcept {
    BiquadDF2T sec{};
    const float fc = std::clamp(cutoff_hz, 1.0f, 0.49f * static_cast<float>(fs));
    const double k = std::tan(3.14159265358979323846 * static_cast<double>(fc) / static_cast<double>(fs));
    const double norm = 1.0 / (1.0 + k);
    sec.b0 = static_cast<float>(k * norm);
    sec.b1 = sec.b0;
    sec.b2 = 0.0f;
    sec.a1 = static_cast<float>((k - 1.0) * norm);
    sec.a2 = 0.0f;
    return sec;
}

/**
 * @brief Design a 1st-order bilinear High-Pass filter section.
 *
 * @param cutoff_hz -3 dB cutoff frequency in Hertz [Hz].
 * @param fs Audio sampling rate in Hertz [Hz].
 * @return BiquadDF2T Configured DF2T filter section [dimensionless struct].
 */
BiquadDF2T BiquadDF2T::design_first_order_hpf(float cutoff_hz, int fs) noexcept {
    BiquadDF2T sec{};
    const float fc = std::clamp(cutoff_hz, 1.0f, 0.49f * static_cast<float>(fs));
    const double k = std::tan(3.14159265358979323846 * static_cast<double>(fc) / static_cast<double>(fs));
    const double norm = 1.0 / (1.0 + k);
    sec.b0 = static_cast<float>(norm);
    sec.b1 = static_cast<float>(-norm);
    sec.b2 = 0.0f;
    sec.a1 = static_cast<float>((k - 1.0) * norm);
    sec.a2 = 0.0f;
    return sec;
}

/**
 * @brief Design a 2nd-order Low-Pass biquad filter section with configurable Q.
 *
 * Uses the bilinear transform (RBJ / Butterworth formulation) matching scipy.signal.butter
 * when Q = 1 / sqrt(2) = 0.70710678.
 *
 * @param cutoff_hz -3 dB cutoff frequency in Hertz [Hz].
 * @param fs Audio sampling rate in Hertz [Hz].
 * @param q Filter quality factor (default 0.70710678f) [dimensionless].
 * @return BiquadDF2T Configured DF2T filter section [dimensionless struct].
 */
BiquadDF2T BiquadDF2T::design_second_order_lpf(float cutoff_hz, int fs, float q) noexcept {
    BiquadDF2T sec{};
    const float fc = std::clamp(cutoff_hz, 1.0f, 0.49f * static_cast<float>(fs));
    const float safe_q = (q > 0.05f) ? q : kDefaultQ;
    const double w0 = 2.0 * 3.14159265358979323846 * static_cast<double>(fc) / static_cast<double>(fs);
    const double cos_w0 = std::cos(w0);
    const double sin_w0 = std::sin(w0);
    const double alpha = sin_w0 / (2.0 * static_cast<double>(safe_q));
    const double a0 = 1.0 + alpha;
    const double inv_a0 = 1.0 / a0;

    sec.b0 = static_cast<float>(((1.0 - cos_w0) * 0.5) * inv_a0);
    sec.b1 = static_cast<float>((1.0 - cos_w0) * inv_a0);
    sec.b2 = sec.b0;
    sec.a1 = static_cast<float>((-2.0 * cos_w0) * inv_a0);
    sec.a2 = static_cast<float>((1.0 - alpha) * inv_a0);
    return sec;
}

/**
 * @brief Design a 2nd-order High-Pass biquad filter section with configurable Q.
 *
 * Uses the bilinear transform (RBJ / Butterworth formulation) matching scipy.signal.butter
 * when Q = 1 / sqrt(2) = 0.70710678.
 *
 * @param cutoff_hz -3 dB cutoff frequency in Hertz [Hz].
 * @param fs Audio sampling rate in Hertz [Hz].
 * @param q Filter quality factor (default 0.70710678f) [dimensionless].
 * @return BiquadDF2T Configured DF2T filter section [dimensionless struct].
 */
BiquadDF2T BiquadDF2T::design_second_order_hpf(float cutoff_hz, int fs, float q) noexcept {
    BiquadDF2T sec{};
    const float fc = std::clamp(cutoff_hz, 1.0f, 0.49f * static_cast<float>(fs));
    const float safe_q = (q > 0.05f) ? q : kDefaultQ;
    const double w0 = 2.0 * 3.14159265358979323846 * static_cast<double>(fc) / static_cast<double>(fs);
    const double cos_w0 = std::cos(w0);
    const double sin_w0 = std::sin(w0);
    const double alpha = sin_w0 / (2.0 * static_cast<double>(safe_q));
    const double a0 = 1.0 + alpha;
    const double inv_a0 = 1.0 / a0;

    sec.b0 = static_cast<float>(((1.0 + cos_w0) * 0.5) * inv_a0);
    sec.b1 = static_cast<float>(-(1.0 + cos_w0) * inv_a0);
    sec.b2 = sec.b0;
    sec.a1 = static_cast<float>((-2.0 * cos_w0) * inv_a0);
    sec.a2 = static_cast<float>((1.0 - alpha) * inv_a0);
    return sec;
}

HilbertIIRAllpass::HilbertIIRAllpass(int fs, float f_min_hz) noexcept {
    configure(fs, f_min_hz);
    reset();
}

/**
 * @brief Compute 8-section Niemitalo / HIIR elliptic IIR Dual All-Pass coefficients.
 *
 * Matches compute_iir_allpass_coeffs(f_min, fs) in Signal_processing/hilbert_helper.py
 * (lines 372-433) bit-for-bit.
 *
 * @param fs Audio sampling rate in Hertz [Hz].
 * @param f_min_hz Passband lower limit in Hertz (e.g. 100.0f) [Hz].
 */
void HilbertIIRAllpass::configure(int fs, float f_min_hz) noexcept {
    constexpr int kNumSectionsPerBranch = static_cast<int>(kSectionsPerBranch);
    constexpr int kTotalSections = 2 * kNumSectionsPerBranch;
    constexpr int kOrder = kTotalSections * 2 + 1;
    constexpr double kPiD = 3.14159265358979323846;

    const double safe_fs = (fs > 0) ? static_cast<double>(fs) : 48000.0;
    double transition = 2.0 * std::max(5.0, static_cast<double>(f_min_hz)) / safe_fs;
    transition = std::clamp(transition, 1.0e-5, 0.45);

    double k = std::tan((1.0 - transition * 2.0) * kPiD / 4.0);
    k = k * k;
    const double kksqrt = std::pow(1.0 - k * k, 0.25);
    const double e = 0.5 * (1.0 - kksqrt) / (1.0 + kksqrt);
    const double e4 = e * e * e * e;
    const double q = e * (1.0 + e4 * (2.0 + e4 * (15.0 + 150.0 * e4)));

    auto compute_acc_numden = [&](int c, bool den) noexcept -> double {
        int i = den ? 1 : 0;
        double sign = den ? -1.0 : 1.0;
        double sum_val = 0.0;
        while (true) {
            const int i2 = den ? i : (i + 1);
            const double angle =
                (static_cast<double>(i + i2) * static_cast<double>(c) * kPiD / static_cast<double>(kOrder)) +
                (den ? (kPiD * 0.5) : 0.0);
            const double q_ii = std::pow(q, static_cast<double>(i * i2)) * std::sin(angle) * sign;
            sum_val += q_ii;
            sign = -sign;
            ++i;
            if (std::fabs(q_ii) < 1.0e-15 || i > 100) {
                break;
            }
        }
        return sum_val;
    };

    for (int idx = 0; idx < kTotalSections; ++idx) {
        const int c = idx + 1;
        const double num = compute_acc_numden(c, false) * std::pow(q, 0.25);
        const double den = compute_acc_numden(c, true) + 0.5;
        const double ww = (num / den) * (num / den);
        const double x = std::sqrt((1.0 - ww * k) * (1.0 - ww / k)) / (1.0 + ww);
        const float coeff = static_cast<float>((1.0 - x) / (1.0 + x));

        if ((idx % 2) == 0) {
            c_in_[static_cast<std::size_t>(idx / 2)] = coeff;
        } else {
            c_quad_[static_cast<std::size_t>(idx / 2)] = coeff;
        }
    }
}

void HilbertIIRAllpass::reset() noexcept {
    in_x1_.fill(0.0f);
    in_x2_.fill(0.0f);
    in_y1_.fill(0.0f);
    in_y2_.fill(0.0f);
    quad_delay_ = 0.0f;
    q_x1_.fill(0.0f);
    q_x2_.fill(0.0f);
    q_y1_.fill(0.0f);
    q_y2_.fill(0.0f);
    match_x1_.fill(0.0f);
    match_x2_.fill(0.0f);
    match_y1_.fill(0.0f);
    match_y2_.fill(0.0f);
}

/**
 * @brief Process a block of samples through the 8-section Niemitalo Hilbert Dual All-Pass cascade.
 *
 * Unrolls the 4 in-phase sections and 4 quadrature sections in scalar registers to eliminate
 * memory traffic inside the inner sample loop on ESP32-S3.
 *
 * @param in Input audio sample array [normalized amplitude].
 * @param out_d Output in-phase branch array d[n] [normalized amplitude].
 * @param out_q Output quadrature branch array q[n] [normalized amplitude].
 * @param n_samples Number of samples in the block [samples].
 */
void HilbertIIRAllpass::process_block(
    const float* in,
    float* out_d,
    float* DSP_RESTRICT out_q,
    std::size_t n_samples) noexcept {
    const float ci0 = c_in_[0], ci1 = c_in_[1], ci2 = c_in_[2], ci3 = c_in_[3];
    const float cq0 = c_quad_[0], cq1 = c_quad_[1], cq2 = c_quad_[2], cq3 = c_quad_[3];

    float ix1_0 = in_x1_[0], ix2_0 = in_x2_[0], iy1_0 = in_y1_[0], iy2_0 = in_y2_[0];
    float ix1_1 = in_x1_[1], ix2_1 = in_x2_[1], iy1_1 = in_y1_[1], iy2_1 = in_y2_[1];
    float ix1_2 = in_x1_[2], ix2_2 = in_x2_[2], iy1_2 = in_y1_[2], iy2_2 = in_y2_[2];
    float ix1_3 = in_x1_[3], ix2_3 = in_x2_[3], iy1_3 = in_y1_[3], iy2_3 = in_y2_[3];

    float qd = quad_delay_;
    float qx1_0 = q_x1_[0], qx2_0 = q_x2_[0], qy1_0 = q_y1_[0], qy2_0 = q_y2_[0];
    float qx1_1 = q_x1_[1], qx2_1 = q_x2_[1], qy1_1 = q_y1_[1], qy2_1 = q_y2_[1];
    float qx1_2 = q_x1_[2], qx2_2 = q_x2_[2], qy1_2 = q_y1_[2], qy2_2 = q_y2_[2];
    float qx1_3 = q_x1_[3], qx2_3 = q_x2_[3], qy1_3 = q_y1_[3], qy2_3 = q_y2_[3];

    for (std::size_t i = 0; i < n_samples; ++i) {
        const float x = in[i];

        // In-phase 4-section cascade
        const float yd0 = ci0 * (x + iy2_0) - ix2_0;
        ix2_0 = ix1_0; ix1_0 = x; iy2_0 = iy1_0; iy1_0 = yd0;

        const float yd1 = ci1 * (yd0 + iy2_1) - ix2_1;
        ix2_1 = ix1_1; ix1_1 = yd0; iy2_1 = iy1_1; iy1_1 = yd1;

        const float yd2 = ci2 * (yd1 + iy2_2) - ix2_2;
        ix2_2 = ix1_2; ix1_2 = yd1; iy2_2 = iy1_2; iy1_2 = yd2;

        const float yd3 = ci3 * (yd2 + iy2_3) - ix2_3;
        ix2_3 = ix1_3; ix1_3 = yd2; iy2_3 = iy1_3; iy1_3 = yd3;

        out_d[i] = yd3;

        // Quadrature 1-sample delay + 4-section cascade
        const float xq = qd;
        qd = x;

        const float yq0 = cq0 * (xq + qy2_0) - qx2_0;
        qx2_0 = qx1_0; qx1_0 = xq; qy2_0 = qy1_0; qy1_0 = yq0;

        const float yq1 = cq1 * (yq0 + qy2_1) - qx2_1;
        qx2_1 = qx1_1; qx1_1 = yq0; qy2_1 = qy1_1; qy1_1 = yq1;

        const float yq2 = cq2 * (yq1 + qy2_2) - qx2_2;
        qx2_2 = qx1_2; qx1_2 = yq1; qy2_2 = qy1_2; qy1_2 = yq2;

        const float yq3 = cq3 * (yq2 + qy2_3) - qx2_3;
        qx2_3 = qx1_3; qx1_3 = yq2; qy2_3 = qy1_3; qy1_3 = yq3;

        out_q[i] = yq3;
    }

    in_x1_[0] = ix1_0; in_x2_[0] = ix2_0; in_y1_[0] = iy1_0; in_y2_[0] = iy2_0;
    in_x1_[1] = ix1_1; in_x2_[1] = ix2_1; in_y1_[1] = iy1_1; in_y2_[1] = iy2_1;
    in_x1_[2] = ix1_2; in_x2_[2] = ix2_2; in_y1_[2] = iy1_2; in_y2_[2] = iy2_2;
    in_x1_[3] = ix1_3; in_x2_[3] = ix2_3; in_y1_[3] = iy1_3; in_y2_[3] = iy2_3;

    quad_delay_ = qd;
    q_x1_[0] = qx1_0; q_x2_[0] = qx2_0; q_y1_[0] = qy1_0; q_y2_[0] = qy2_0;
    q_x1_[1] = qx1_1; q_x2_[1] = qx2_1; q_y1_[1] = qy1_1; q_y2_[1] = qy2_1;
    q_x1_[2] = qx1_2; q_x2_[2] = qx2_2; q_y1_[2] = qy1_2; q_y2_[2] = qy2_2;
    q_x1_[3] = qx1_3; q_x2_[3] = qx2_3; q_y1_[3] = qy1_3; q_y2_[3] = qy2_3;
}

/**
 * @brief Process a block of samples through the 4-section in-phase delay-matching cascade.
 *
 * Used for the Mid channel when no channel requests frequency shifting (dfs_shift_hz == 0).
 *
 * @param in Input audio sample array [normalized amplitude].
 * @param out_d Output phase-matched array d_M[n] [normalized amplitude].
 * @param n_samples Number of samples in the block [samples].
 */
void HilbertIIRAllpass::match_in_phase_block(
    const float* in,
    float* out_d,
    std::size_t n_samples) noexcept {
    const float ci0 = c_in_[0], ci1 = c_in_[1], ci2 = c_in_[2], ci3 = c_in_[3];

    float mx1_0 = match_x1_[0], mx2_0 = match_x2_[0], my1_0 = match_y1_[0], my2_0 = match_y2_[0];
    float mx1_1 = match_x1_[1], mx2_1 = match_x2_[1], my1_1 = match_y1_[1], my2_1 = match_y2_[1];
    float mx1_2 = match_x1_[2], mx2_2 = match_x2_[2], my1_2 = match_y1_[2], my2_2 = match_y2_[2];
    float mx1_3 = match_x1_[3], mx2_3 = match_x2_[3], my1_3 = match_y1_[3], my2_3 = match_y2_[3];

    for (std::size_t i = 0; i < n_samples; ++i) {
        const float x = in[i];

        const float y0 = ci0 * (x + my2_0) - mx2_0;
        mx2_0 = mx1_0; mx1_0 = x; my2_0 = my1_0; my1_0 = y0;

        const float y1 = ci1 * (y0 + my2_1) - mx2_1;
        mx2_1 = mx1_1; mx1_1 = y0; my2_1 = my1_1; my1_1 = y1;

        const float y2 = ci2 * (y1 + my2_2) - mx2_2;
        mx2_2 = mx1_2; mx1_2 = y1; my2_2 = my1_2; my1_2 = y2;

        const float y3 = ci3 * (y2 + my2_3) - mx2_3;
        mx2_3 = mx1_3; mx1_3 = y2; my2_3 = my1_3; my1_3 = y3;

        out_d[i] = y3;
    }

    match_x1_[0] = mx1_0; match_x2_[0] = mx2_0; match_y1_[0] = my1_0; match_y2_[0] = my2_0;
    match_x1_[1] = mx1_1; match_x2_[1] = mx2_1; match_y1_[1] = my1_1; match_y2_[1] = my2_1;
    match_x1_[2] = mx1_2; match_x2_[2] = mx2_2; match_y1_[2] = my1_2; match_y2_[2] = my2_2;
    match_x1_[3] = mx1_3; match_x2_[3] = mx2_3; match_y1_[3] = my1_3; match_y2_[3] = my2_3;
}

/**
 * @brief Synthesize a block of continuous-phase sine samples using coupled quadrature rotation.
 *
 * Matches RecursiveNCO.synthesize_block in Signal_processing/subharmonic_synth.py.
 * Applies periodic first-order Taylor inverse-square-root gain stabilization every 512 samples.
 *
 * @param freq_hz Target oscillator frequency in Hertz [Hz].
 * @param voiced True if pitch detector is voiced; false outputs silence while preserving phase [boolean].
 * @param out Output buffer receiving synthesized sinusoid samples [normalized amplitude].
 * @param n_samples Number of samples to generate (e.g. 80 at 8 kHz) [samples].
 */
void RecursiveNCO::synthesize_block(
    float freq_hz,
    bool voiced,
    float* DSP_RESTRICT out,
    std::size_t n_samples) noexcept {
    if (!voiced || freq_hz <= 0.0f) {
        std::fill(out, out + n_samples, 0.0f);
        return;
    }

    const float omega = kTwoPi * freq_hz / static_cast<float>(fs_);
    const float cos_w = std::cos(omega);
    const float sin_w = std::sin(omega);

    float u = u_;
    float v = v_;
    uint32_t step = step_counter_;

    for (std::size_t i = 0; i < n_samples; ++i) {
        out[i] = v;
        const float u_next = u * cos_w - v * sin_w;
        const float v_next = u * sin_w + v * cos_w;
        u = u_next;
        v = v_next;

        ++step;
        if ((step & 511u) == 0u) {
            const float r2 = u * u + v * v;
            const float g = 1.5f - 0.5f * r2;
            u *= g;
            v *= g;
        }
    }

    u_ = u;
    v_ = v;
    step_counter_ = step;
}

EnvelopeFollower::EnvelopeFollower(float attack_ms, float decay_ms, int fs) noexcept {
    configure(attack_ms, decay_ms, fs);
}

/**
 * @brief Configure attack and decay one-pole time constants of the envelope follower.
 *
 * @param attack_ms Attack time constant in milliseconds [ms].
 * @param decay_ms Decay time constant in milliseconds [ms].
 * @param fs Audio sampling rate in Hertz [Hz].
 */
void EnvelopeFollower::configure(float attack_ms, float decay_ms, int fs) noexcept {
    const float safe_fs = (fs > 0) ? static_cast<float>(fs) : static_cast<float>(kSubSampleRateHz);
    const float tau_att = std::max(0.1f, attack_ms) * 1.0e-3f;
    const float tau_dec = std::max(0.1f, decay_ms) * 1.0e-3f;
    alpha_att_ = std::exp(-1.0f / (tau_att * safe_fs));
    alpha_dec_ = std::exp(-1.0f / (tau_dec * safe_fs));
}

/**
 * @brief Track peak envelope across a block of input samples.
 *
 * @param in Input audio sample array [normalized amplitude].
 * @param out Output envelope sample array [normalized amplitude].
 * @param n_samples Number of samples in the block [samples].
 */
void EnvelopeFollower::process_block(
    const float* DSP_RESTRICT in,
    float* DSP_RESTRICT out,
    std::size_t n_samples) noexcept {
    float env = env_;
    const float a_att = alpha_att_;
    const float a_dec = alpha_dec_;
    for (std::size_t i = 0; i < n_samples; ++i) {
        const float abs_x = std::fabs(in[i]);
        const float alpha = (abs_x > env) ? a_att : a_dec;
        env = alpha * env + (1.0f - alpha) * abs_x;
        out[i] = env;
    }
    if (env < 1.0e-20f) env = 0.0f;
    env_ = env;
}

}  // namespace audio_dsp
