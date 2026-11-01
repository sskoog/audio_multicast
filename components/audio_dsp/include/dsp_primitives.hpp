#ifndef AUDIO_DSP_PRIMITIVES_HPP_
#define AUDIO_DSP_PRIMITIVES_HPP_

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define DSP_DRAM_ATTR DRAM_ATTR
#define DSP_IRAM_ATTR IRAM_ATTR
#else
#define DSP_DRAM_ATTR
#define DSP_IRAM_ATTR
#endif

#if defined(__GNUC__) || defined(__clang__)
#define DSP_RESTRICT __restrict__
#elif defined(_MSC_VER)
#define DSP_RESTRICT __restrict
#else
#define DSP_RESTRICT
#endif

namespace audio_dsp {

/// Primary full-band audio sample rate in Hertz (Hz).
inline constexpr int kSampleRateHz = 48000;

/// Number of samples per 10.0 ms LC3 frame at 48 kHz (multiple of 16 for SIMD).
inline constexpr std::size_t kFrameSamples = 480;

/// Decimation factor from 48 kHz to 8 kHz for the subwoofer subharmonic pipeline.
inline constexpr int kSubDecimFactor = 6;

/// Subwoofer subharmonic pipeline sample rate in Hertz (Hz).
inline constexpr int kSubSampleRateHz = 8000;

/// Number of samples per 10.0 ms LC3 frame at 8 kHz (multiple of 16 for SIMD).
inline constexpr std::size_t kSubFrameSamples = 80;

/// Minimum and maximum supported output channels for HilbertQuadratureMs.
inline constexpr std::size_t kMinQuadratureChannels = 2;
inline constexpr std::size_t kMaxQuadratureChannels = 8;

/// Mathematical constants.
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 6.28318530717958647692f;
inline constexpr float kDefaultQ = 0.7071067811865475f;

/// Quarter-turn (+90 deg) offset in uint32_t full-turn (2^32) angle representation.
inline constexpr uint32_t kAzimuthOffsetU32 = (1u << 30);

/**
 * @brief Single RAM-resident 130-entry quarter-wave sine lookup table (0..128 + guard tap).
 *
 * Placed in internal DRAM (.dram1.data on ESP32-S3) to avoid Flash cache misses.
 * Only one instance of this table exists in RAM across all channels and DSP objects.
 */
alignas(16) extern float g_fast_sin_table[130];

/**
 * @brief Lock-free double-buffered configuration holder for FreeRTOS thread safety.
 *
 * Control tasks write new parameter structs via set(), and the real-time audio
 * task reads the latest snapshot via get() without mutexes or dynamic allocation.
 */
template <typename T>
class AtomicConfig {
public:
    AtomicConfig() noexcept : slots_{T{}, T{}}, active_idx_(0u), generation_(0u) {}

    explicit AtomicConfig(const T& initial) noexcept
        : slots_{initial, initial}, active_idx_(0u), generation_(0u) {}

    /**
     * @brief Publish a new configuration from a non-real-time control task.
     *
     * @param cfg New configuration structure [dimensionless struct].
     * @return uint32_t Updated generation counter [count].
     */
    uint32_t set(const T& cfg) noexcept {
        const uint32_t cur = active_idx_.load(std::memory_order_relaxed);
        const uint32_t next = cur ^ 1u;
        slots_[next] = cfg;
        active_idx_.store(next, std::memory_order_release);
        return generation_.fetch_add(1u, std::memory_order_acq_rel) + 1u;
    }

    /**
     * @brief Read the active configuration from the real-time audio task.
     *
     * @param out_generation Optional pointer receiving the current generation counter [count].
     * @return T Copy of the active configuration structure [dimensionless struct].
     */
    T get(uint32_t* out_generation = nullptr) const noexcept {
        if (out_generation != nullptr) {
            *out_generation = generation_.load(std::memory_order_acquire);
        }
        const uint32_t idx = active_idx_.load(std::memory_order_acquire);
        return slots_[idx];
    }

    uint32_t generation() const noexcept {
        return generation_.load(std::memory_order_acquire);
    }

private:
    T slots_[2];
    std::atomic<uint32_t> active_idx_;
    std::atomic<uint32_t> generation_;
};

/**
 * @brief Fast quarter-wave sine/cosine evaluator indexed by uint32_t full-turn angles.
 *
 * Uses the single RAM-resident g_fast_sin_table[130] (512 effective full-wave steps).
 * Angle representation: 2^32 = 1 full turn = 360 degrees = 2*pi radians.
 * - Bit 31 selects sign (0 = positive, 1 = negative).
 * - Bit 30 selects quadrant mirroring (0 = ascending, 1 = descending).
 * - Bits 29..23 select table index (0..128).
 * - Bits 22..0 provide the 23-bit linear interpolation fraction (* 2^-23).
 */
class FastSinLUT {
public:
    static constexpr std::size_t kQuarterSize = 128;
    static constexpr std::size_t kTableSize = 130;

    /**
     * @brief Return pointer to the single shared RAM-resident lookup table.
     *
     * @return const float* Pointer to g_fast_sin_table[0] in RAM.
     */
    static inline const float* table_ptr() noexcept {
        return g_fast_sin_table;
    }

    /**
     * @brief Evaluate sine of a uint32_t modulo-2^32 angle using RAM LUT + linear interpolation.
     *
     * Exact 0.0f at 0 and 180 deg, exact +1.0f at 90 deg (0x40000000u),
     * exact -1.0f at 270 deg (0xC0000000u), and < -94.5 dB max error across all angles.
     *
     * @param thetaModulo2Pow32 Phase angle where 2^32 represents 1 full turn (360 deg) [uint32_t turns * 2^32].
     * @return float Approximated sine value in [-1.0f, 1.0f] [dimensionless].
     */
    static inline float sin_u32(uint32_t theta) noexcept {
        uint32_t q_phase = theta & 0x3FFFFFFFu;
        if ((theta & 0x40000000u) != 0u) {
            q_phase = 0x40000000u - q_phase;
        }
        const uint32_t idx = q_phase >> 23;
        const float frac = static_cast<float>(q_phase & 0x007FFFFFu) * 0x1.0p-23f;
        const float y0 = g_fast_sin_table[idx];
        const float y1 = g_fast_sin_table[idx + 1u];
        const float mag = y0 + frac * (y1 - y0);
        return ((theta & 0x80000000u) != 0u) ? -mag : mag;
    }

    /**
     * @brief Evaluate cosine of a uint32_t modulo-2^32 angle via phase shift (+2^30 = +90 deg).
     *
     * @param theta Phase angle where 2^32 represents 1 full turn (360 deg) [uint32_t turns * 2^32].
     * @return float Approximated cosine value in [-1.0f, 1.0f] [dimensionless].
     */
    static inline float cos_u32(uint32_t theta) noexcept {
        return sin_u32(theta + kAzimuthOffsetU32);
    }

    /**
     * @brief Simultaneously evaluate sine and cosine of a uint32_t modulo-2^32 angle.
     *
     * @param theta Phase angle where 2^32 represents 1 full turn (360 deg) [uint32_t turns * 2^32].
     * @param out_sin Output reference receiving sin(theta) [dimensionless].
     * @param out_cos Output reference receiving cos(theta) [dimensionless].
     */
    static inline void sin_cos_u32(uint32_t theta, float& out_sin, float& out_cos) noexcept {
        out_sin = sin_u32(theta);
        out_cos = sin_u32(theta + kAzimuthOffsetU32);
    }

    /**
     * @brief Convert an angle in degrees to uint32_t full-turn representation (2^32 = 360 deg).
     *
     * Handles negative and multi-turn angles seamlessly (e.g. -180 deg and +180 deg both
     * map to 0x80000000u; -90 deg maps to 0xC0000000u).
     *
     * @param deg Angle in degrees [deg].
     * @return uint32_t Modulo-2^32 phase angle [turns * 2^32].
     */
    static uint32_t deg_to_u32(float deg) noexcept;

    /**
     * @brief Convert an angle in radians to uint32_t full-turn representation (2^32 = 2*pi rad).
     *
     * @param rad Angle in radians [rad].
     * @return uint32_t Modulo-2^32 phase angle [turns * 2^32].
     */
    static uint32_t rad_to_u32(float rad) noexcept;

    /**
     * @brief Convert a frequency in Hz to a per-sample uint32_t phase increment: round(f / fs * 2^32).
     *
     * Negative frequencies wrap naturally via two's-complement arithmetic.
     *
     * @param freq_hz Oscillation or frequency-shift rate in Hertz [Hz].
     * @param fs Audio sampling frequency in Hertz [Hz].
     * @return uint32_t Phase increment per sample [turns * 2^32 / sample].
     */
    static uint32_t hz_to_inc_u32(float freq_hz, int fs = kSampleRateHz) noexcept;

    /**
     * @brief Signed shortest-arc angular difference (target - current) across the 0/360 deg boundary.
     *
     * @param target_u32 Target angle in uint32_t full-turn units [turns * 2^32].
     * @param current_u32 Current angle in uint32_t full-turn units [turns * 2^32].
     * @return int32_t Signed shortest-arc difference in [-2^31, 2^31 - 1] [turns * 2^32].
     */
    static inline int32_t shortest_arc_diff_s32(uint32_t target_u32, uint32_t current_u32) noexcept {
        return static_cast<int32_t>(target_u32 - current_u32);
    }

    /**
     * @brief Convenience radian wrapper for sin(rad).
     */
    static inline float sin_rad(float rad) noexcept {
        return sin_u32(rad_to_u32(rad));
    }

    /**
     * @brief Convenience radian wrapper for cos(rad).
     */
    static inline float cos_rad(float rad) noexcept {
        return cos_u32(rad_to_u32(rad));
    }
};

/**
 * @brief Single Direct-Form II Transposed (DF2T) biquad filter section.
 *
 * Difference equations:
 *   y[n]  = b0 * x[n] + z1
 *   z1[n] = b1 * x[n] - a1 * y[n] + z2
 *   z2[n] = b2 * x[n] - a2 * y[n]
 *
 * Also supports 1st-order sections by setting b2 = 0 and a2 = 0.
 */
struct BiquadDF2T {
    float b0 = 1.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
    float z1 = 0.0f;
    float z2 = 0.0f;

    inline void reset() noexcept {
        z1 = 0.0f;
        z2 = 0.0f;
    }

    inline float process_sample(float x) noexcept {
        const float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }

    void process_block(const float* in, float* out, std::size_t n_samples) noexcept {
        float s1 = z1;
        float s2 = z2;
        const float c_b0 = b0;
        const float c_b1 = b1;
        const float c_b2 = b2;
        const float c_a1 = a1;
        const float c_a2 = a2;
        for (std::size_t i = 0; i < n_samples; ++i) {
            const float x = in[i];
            const float y = c_b0 * x + s1;
            s1 = c_b1 * x - c_a1 * y + s2;
            s2 = c_b2 * x - c_a2 * y;
            out[i] = y;
        }
        if (std::fabs(s1) < 1.0e-20f) s1 = 0.0f;
        if (std::fabs(s2) < 1.0e-20f) s2 = 0.0f;
        z1 = s1;
        z2 = s2;
    }

    static BiquadDF2T design_first_order_lpf(float cutoff_hz, int fs) noexcept;
    static BiquadDF2T design_first_order_hpf(float cutoff_hz, int fs) noexcept;
    static BiquadDF2T design_second_order_lpf(float cutoff_hz, int fs, float q = kDefaultQ) noexcept;
    static BiquadDF2T design_second_order_hpf(float cutoff_hz, int fs, float q = kDefaultQ) noexcept;
};

/**
 * @brief Statically allocated cascade of up to MaxSections DF2T biquad sections.
 */
template <std::size_t MaxSections>
class BiquadCascade {
public:
    BiquadCascade() noexcept : num_active_(0) {}

    void reset() noexcept {
        for (std::size_t i = 0; i < MaxSections; ++i) {
            sections_[i].reset();
        }
    }

    void set_num_active(std::size_t n) noexcept {
        num_active_ = (n <= MaxSections) ? n : MaxSections;
    }

    std::size_t num_active() const noexcept { return num_active_; }

    BiquadDF2T& section(std::size_t idx) noexcept { return sections_[idx]; }
    const BiquadDF2T& section(std::size_t idx) const noexcept { return sections_[idx]; }

    inline float process_sample(float x) noexcept {
        float y = x;
        for (std::size_t s = 0; s < num_active_; ++s) {
            y = sections_[s].process_sample(y);
        }
        return y;
    }

    void process_block(const float* in, float* out, std::size_t n_samples) noexcept {
        if (num_active_ == 0) {
            if (in != out) {
                std::copy(in, in + n_samples, out);
            }
            return;
        }
        sections_[0].process_block(in, out, n_samples);
        for (std::size_t s = 1; s < num_active_; ++s) {
            sections_[s].process_block(out, out, n_samples);
        }
    }

    /**
     * @brief Configure cascade as a Butterworth Low-Pass filter of order 1, 2, 3, or 4.
     *
     * Order 4 uses true 4th-order Butterworth pole Q factors (Q1 = 0.54119610, Q2 = 1.30656296)
     * matching scipy.signal.butter(4, cutoff_hz, btype='low', fs=fs, output='sos').
     *
     * @param order Filter order in {1, 2, 3, 4} (0 = bypass) [dimensionless].
     * @param cutoff_hz -3 dB cutoff frequency in Hertz [Hz].
     * @param fs Audio sampling rate in Hertz [Hz].
     */
    void configure_butterworth_lpf(int order, float cutoff_hz, int fs) noexcept {
        if (order <= 0) {
            num_active_ = 0;
            return;
        }
        if (order == 1 && MaxSections >= 1) {
            sections_[0] = BiquadDF2T::design_first_order_lpf(cutoff_hz, fs);
            num_active_ = 1;
        } else if (order == 2 && MaxSections >= 1) {
            sections_[0] = BiquadDF2T::design_second_order_lpf(cutoff_hz, fs, kDefaultQ);
            num_active_ = 1;
        } else if (order == 3 && MaxSections >= 2) {
            sections_[0] = BiquadDF2T::design_first_order_lpf(cutoff_hz, fs);
            sections_[1] = BiquadDF2T::design_second_order_lpf(cutoff_hz, fs, 1.0f);
            num_active_ = 2;
        } else if (MaxSections >= 2) {
            constexpr float kQ4_0 = 0.541196100146197f;
            constexpr float kQ4_1 = 1.3065629648763766f;
            sections_[0] = BiquadDF2T::design_second_order_lpf(cutoff_hz, fs, kQ4_0);
            sections_[1] = BiquadDF2T::design_second_order_lpf(cutoff_hz, fs, kQ4_1);
            num_active_ = 2;
        }
    }

    /**
     * @brief Configure cascade as a High-Pass filter of order 1, 2, 3, or 4 with configurable Q.
     *
     * @param order Filter order in {0, 1, 2, 3, 4} (0 = bypass) [dimensionless].
     * @param cutoff_hz Cutoff frequency in Hertz [Hz].
     * @param fs Audio sampling rate in Hertz [Hz].
     * @param q Quality factor for 2nd-order sections (default 0.70710678f) [dimensionless].
     */
    void configure_hpf(int order, float cutoff_hz, int fs, float q = kDefaultQ) noexcept {
        if (order <= 0) {
            num_active_ = 0;
            return;
        }
        if (order == 1 && MaxSections >= 1) {
            sections_[0] = BiquadDF2T::design_first_order_hpf(cutoff_hz, fs);
            num_active_ = 1;
        } else if (order == 2 && MaxSections >= 1) {
            sections_[0] = BiquadDF2T::design_second_order_hpf(cutoff_hz, fs, q);
            num_active_ = 1;
        } else if (order == 3 && MaxSections >= 2) {
            sections_[0] = BiquadDF2T::design_first_order_hpf(cutoff_hz, fs);
            sections_[1] = BiquadDF2T::design_second_order_hpf(cutoff_hz, fs, q);
            num_active_ = 2;
        } else if (MaxSections >= 2) {
            sections_[0] = BiquadDF2T::design_second_order_hpf(cutoff_hz, fs, q);
            sections_[1] = BiquadDF2T::design_second_order_hpf(cutoff_hz, fs, q);
            num_active_ = 2;
        }
    }

private:
    std::array<BiquadDF2T, MaxSections> sections_{};
    std::size_t num_active_;
};

/**
 * @brief 8-Section Niemitalo Elliptic IIR Dual All-Pass Hilbert Transformer.
 *
 * Splits a real audio signal x[n] into an analytic quadrature pair (d[n], q[n])
 * maintaining a 90.0 deg (+/- 0.7 deg) phase difference across [f_min, fs/2 - f_min].
 * Each 2nd-order all-pass section implements:
 *   y[n] = c_k * (x[n] + y[n-2]) - x[n-2]
 * The quadrature branch includes a 1-sample delay before its 4 all-pass sections.
 */
class HilbertIIRAllpass {
public:
    static constexpr std::size_t kSectionsPerBranch = 4;

    explicit HilbertIIRAllpass(int fs = kSampleRateHz, float f_min_hz = 100.0f) noexcept;

    void configure(int fs = kSampleRateHz, float f_min_hz = 100.0f) noexcept;
    void reset() noexcept;

    /**
     * @brief Process a single sample through both the in-phase (d) and quadrature (q) branches.
     *
     * @param x Input audio sample [normalized amplitude].
     * @param out_d Output reference receiving in-phase branch sample d[n] [normalized amplitude].
     * @param out_q Output reference receiving quadrature branch sample q[n] [normalized amplitude].
     */
    inline void process_sample(float x, float& out_d, float& out_q) noexcept {
        float vd = x;
        for (std::size_t k = 0; k < kSectionsPerBranch; ++k) {
            const float y = c_in_[k] * (vd + in_y2_[k]) - in_x2_[k];
            in_x2_[k] = in_x1_[k];
            in_x1_[k] = vd;
            in_y2_[k] = in_y1_[k];
            in_y1_[k] = y;
            vd = y;
        }
        out_d = vd;

        float vq = quad_delay_;
        quad_delay_ = x;
        for (std::size_t k = 0; k < kSectionsPerBranch; ++k) {
            const float y = c_quad_[k] * (vq + q_y2_[k]) - q_x2_[k];
            q_x2_[k] = q_x1_[k];
            q_x1_[k] = vq;
            q_y2_[k] = q_y1_[k];
            q_y1_[k] = y;
            vq = y;
        }
        out_q = vq;
    }

    /**
     * @brief Pass a signal through an independent 4-section in-phase allpass cascade to match d[n] phase.
     *
     * @param x Input audio sample [normalized amplitude].
     * @return float Phase-aligned output sample [normalized amplitude].
     */
    inline float match_in_phase_sample(float x) noexcept {
        float v = x;
        for (std::size_t k = 0; k < kSectionsPerBranch; ++k) {
            const float y = c_in_[k] * (v + match_y2_[k]) - match_x2_[k];
            match_x2_[k] = match_x1_[k];
            match_x1_[k] = v;
            match_y2_[k] = match_y1_[k];
            match_y1_[k] = y;
            v = y;
        }
        return v;
    }

    void process_block(
        const float* in,
        float* out_d,
        float* DSP_RESTRICT out_q,
        std::size_t n_samples) noexcept;

    void match_in_phase_block(
        const float* in,
        float* out_d,
        std::size_t n_samples) noexcept;

    const std::array<float, kSectionsPerBranch>& c_in() const noexcept { return c_in_; }
    const std::array<float, kSectionsPerBranch>& c_quad() const noexcept { return c_quad_; }

private:
    std::array<float, kSectionsPerBranch> c_in_{};
    std::array<float, kSectionsPerBranch> c_quad_{};

    std::array<float, kSectionsPerBranch> in_x1_{};
    std::array<float, kSectionsPerBranch> in_x2_{};
    std::array<float, kSectionsPerBranch> in_y1_{};
    std::array<float, kSectionsPerBranch> in_y2_{};

    float quad_delay_ = 0.0f;
    std::array<float, kSectionsPerBranch> q_x1_{};
    std::array<float, kSectionsPerBranch> q_x2_{};
    std::array<float, kSectionsPerBranch> q_y1_{};
    std::array<float, kSectionsPerBranch> q_y2_{};

    std::array<float, kSectionsPerBranch> match_x1_{};
    std::array<float, kSectionsPerBranch> match_x2_{};
    std::array<float, kSectionsPerBranch> match_y1_{};
    std::array<float, kSectionsPerBranch> match_y2_{};
};

/**
 * @brief Continuous-phase coupled-form quadrature oscillator (RecursiveNCO).
 *
 * Rotates state vector (u + j*v) by (cos_w + j*sin_w) each sample and applies
 * periodic first-order Taylor inverse-square-root normalization every 512 samples.
 */
class RecursiveNCO {
public:
    explicit RecursiveNCO(int fs = kSubSampleRateHz) noexcept : fs_(fs) {}

    void set_sample_rate(int fs) noexcept {
        fs_ = (fs > 0) ? fs : kSubSampleRateHz;
    }

    void reset() noexcept {
        u_ = 1.0f;
        v_ = 0.0f;
        step_counter_ = 0u;
    }

    void synthesize_block(
        float freq_hz,
        bool voiced,
        float* DSP_RESTRICT out,
        std::size_t n_samples) noexcept;

private:
    int fs_ = kSubSampleRateHz;
    float u_ = 1.0f;
    float v_ = 0.0f;
    uint32_t step_counter_ = 0u;
};

/**
 * @brief Asymmetric first-order peak envelope follower (separate attack and decay time constants).
 */
class EnvelopeFollower {
public:
    EnvelopeFollower(
        float attack_ms = 20.0f,
        float decay_ms = 80.0f,
        int fs = kSubSampleRateHz) noexcept;

    void configure(float attack_ms, float decay_ms, int fs) noexcept;
    void reset() noexcept { env_ = 0.0f; }

    inline float process_sample(float x) noexcept {
        const float abs_x = std::fabs(x);
        const float alpha = (abs_x > env_) ? alpha_att_ : alpha_dec_;
        env_ = alpha * env_ + (1.0f - alpha) * abs_x;
        return env_;
    }

    void process_block(
        const float* DSP_RESTRICT in,
        float* DSP_RESTRICT out,
        std::size_t n_samples) noexcept;

private:
    float alpha_att_ = 0.0f;
    float alpha_dec_ = 0.0f;
    float env_ = 0.0f;
};

/**
 * @brief Convert 16-bit signed PCM samples to normalized float32 [-1.0f, 1.0f).
 */
inline void pcm16_to_float_block(
    const int16_t* DSP_RESTRICT in_pcm16,
    float* DSP_RESTRICT out_f32,
    std::size_t n_samples) noexcept {
    constexpr float kScale = 1.0f / 32768.0f;
    for (std::size_t i = 0; i < n_samples; ++i) {
        out_f32[i] = static_cast<float>(in_pcm16[i]) * kScale;
    }
}

/**
 * @brief Convert normalized float32 samples to 16-bit signed PCM with branchless saturation.
 *
 * Protects against integer wrap-around when side_gain up to 2.0 (+6 dB) causes peaks > 1.0f.
 */
inline void float_to_pcm16_block(
    const float* DSP_RESTRICT in_f32,
    int16_t* DSP_RESTRICT out_pcm16,
    std::size_t n_samples) noexcept {
    for (std::size_t i = 0; i < n_samples; ++i) {
        const float scaled = std::nearbyintf(in_f32[i] * 32768.0f);
        const float clamped = std::clamp(scaled, -32768.0f, 32767.0f);
        out_pcm16[i] = static_cast<int16_t>(clamped);
    }
}

/**
 * @brief Deinterleave stereo 16-bit PCM [L0, R0, L1, R1, ...] into aligned float32 L and R buffers.
 */
inline void pcm16_stereo_to_float_lr(
    const int16_t* DSP_RESTRICT stereo_in,
    float* DSP_RESTRICT left_out,
    float* DSP_RESTRICT right_out,
    std::size_t n_frames) noexcept {
    constexpr float kScale = 1.0f / 32768.0f;
    for (std::size_t i = 0; i < n_frames; ++i) {
        left_out[i] = static_cast<float>(stereo_in[2u * i]) * kScale;
        right_out[i] = static_cast<float>(stereo_in[2u * i + 1u]) * kScale;
    }
}

/**
 * @brief Linearly slew a parameter toward its target so that ramping across the full
 *        [min_val, max_val] range takes 1.0 second (max_step = (max_val - min_val) * dt_sec).
 *
 * @param current Current parameter value in effect [parameter units].
 * @param target Target parameter value [parameter units].
 * @param min_val Minimum defined value for this parameter [parameter units].
 * @param max_val Maximum defined value for this parameter [parameter units].
 * @param dt_sec Block duration in seconds (default 0.01f for a 10 ms frame) [seconds].
 * @return float Updated parameter value after one block step [parameter units].
 */
inline float slew_towards(
    float current,
    float target,
    float min_val,
    float max_val,
    float dt_sec = 0.01f) noexcept {
    if (current == target) {
        return current;
    }
    const float max_step = (max_val - min_val) * dt_sec;
    if (current < target) {
        return std::min(target, current + max_step);
    }
    return std::max(target, current - max_step);
}

}  // namespace audio_dsp

#endif  // AUDIO_DSP_PRIMITIVES_HPP_
