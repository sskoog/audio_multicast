#include "subharmonic_pre_stage.hpp"

#include <algorithm>

namespace audio_dsp {

SubharmonicPreStage::SubharmonicPreStage(const SubharmonicPreStageConfig& cfg) noexcept {
    init(cfg);
}

/**
 * @brief Initialize the shared 48 kHz -> 8 kHz downsampling pre-stage.
 *
 * Clamps the cutoff frequency to [70.0, 200.0] Hz, configures the 4th-order
 * Butterworth Low-Pass filter at 48 kHz, and clears internal state registers.
 *
 * @param cfg Pre-stage configuration structure [dimensionless struct].
 */
void SubharmonicPreStage::init(const SubharmonicPreStageConfig& cfg) noexcept {
    SubharmonicPreStageConfig sanitized = cfg;
    sanitized.lpf_cutoff_hz = std::clamp(cfg.lpf_cutoff_hz, kMinCutoffHz, kMaxCutoffHz);
    target_cutoff_hz_ = sanitized.lpf_cutoff_hz;
    active_cutoff_hz_ = sanitized.lpf_cutoff_hz;
    lpf_4th_.configure_butterworth_lpf(4, active_cutoff_hz_, kSampleRateHz);
    lpf_4th_.reset();
    applied_gen_ = config_Queue_.set(sanitized);
}

void SubharmonicPreStage::set_cutoff_hz(float cutoff_hz) noexcept {
    SubharmonicPreStageConfig cfg{};
    cfg.lpf_cutoff_hz = std::clamp(cutoff_hz, kMinCutoffHz, kMaxCutoffHz);
    config_Queue_.set(cfg);
}

void SubharmonicPreStage::reset() noexcept {
    lpf_4th_.reset();
}

/**
 * @brief Synchronize and slew pending cutoff frequency updates from AtomicConfig without blocking.
 */
void SubharmonicPreStage::sync_config_if_needed(float dt_sec) noexcept {
    uint32_t gen = 0u;
    const SubharmonicPreStageConfig cfg = config_Queue_.get(&gen);
    if (gen != applied_gen_) {
        applied_gen_ = gen;
        target_cutoff_hz_ = std::clamp(cfg.lpf_cutoff_hz, kMinCutoffHz, kMaxCutoffHz);
    }
    if (active_cutoff_hz_ != target_cutoff_hz_) {
        active_cutoff_hz_ = slew_towards(
            active_cutoff_hz_, target_cutoff_hz_, kMinCutoffHz, kMaxCutoffHz, dt_sec);
        const float z0_1 = lpf_4th_.section(0).z1;
        const float z0_2 = lpf_4th_.section(0).z2;
        const float z1_1 = lpf_4th_.section(1).z1;
        const float z1_2 = lpf_4th_.section(1).z2;
        lpf_4th_.configure_butterworth_lpf(4, active_cutoff_hz_, kSampleRateHz);
        lpf_4th_.section(0).z1 = z0_1;
        lpf_4th_.section(0).z2 = z0_2;
        lpf_4th_.section(1).z1 = z1_1;
        lpf_4th_.section(1).z2 = z1_2;
    }
}

/**
 * @brief Downmix deinterleaved 48 kHz stereo float32 to mono, apply 4th-order LP, and decimate 6:1 to 8 kHz.
 *
 * @param left_48k Input Left channel buffer at 48 kHz (length n_frames_48k) [normalized amplitude].
 * @param right_48k Input Right channel buffer at 48 kHz (length n_frames_48k) [normalized amplitude].
 * @param out_mono_8k Output mono buffer at 8 kHz (length n_frames_48k / 6) [normalized amplitude].
 * @param n_frames_48k Number of 48 kHz input frames (default 480) [samples].
 * @return std::size_t Number of 8 kHz output samples produced [samples].
 */
std::size_t SubharmonicPreStage::process_stereo_f32_to_8k(
    const float* DSP_RESTRICT left_48k,
    const float* DSP_RESTRICT right_48k,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    sync_config_if_needed(static_cast<float>(n_frames_48k) / static_cast<float>(kSampleRateHz));

    BiquadDF2T& s0 = lpf_4th_.section(0);
    BiquadDF2T& s1 = lpf_4th_.section(1);
    float s0_z1 = s0.z1, s0_z2 = s0.z2;
    float s1_z1 = s1.z1, s1_z2 = s1.z2;
    const float b0_0 = s0.b0, b1_0 = s0.b1, b2_0 = s0.b2, a1_0 = s0.a1, a2_0 = s0.a2;
    const float b0_1 = s1.b0, b1_1 = s1.b1, b2_1 = s1.b2, a1_1 = s1.a1, a2_1 = s1.a2;

    const std::size_t n_out = n_frames_48k / static_cast<std::size_t>(kSubDecimFactor);
    for (std::size_t g = 0; g < n_out; ++g) {
        const std::size_t base = g * static_cast<std::size_t>(kSubDecimFactor);
        for (std::size_t k = 0; k < static_cast<std::size_t>(kSubDecimFactor); ++k) {
            const float x = 0.5f * (left_48k[base + k] + right_48k[base + k]);
            const float y0 = b0_0 * x + s0_z1;
            s0_z1 = b1_0 * x - a1_0 * y0 + s0_z2;
            s0_z2 = b2_0 * x - a2_0 * y0;

            const float y1 = b0_1 * y0 + s1_z1;
            s1_z1 = b1_1 * y0 - a1_1 * y1 + s1_z2;
            s1_z2 = b2_1 * y0 - a2_1 * y1;

            if (k == 0u) {
                out_mono_8k[g] = y1;
            }
        }
    }

    s0.z1 = s0_z1; s0.z2 = s0_z2;
    s1.z1 = s1_z1; s1.z2 = s1_z2;
    return n_out;
}

/**
 * @brief Filter 48 kHz mono float32 through the 4th-order LP and decimate 6:1 to 8 kHz.
 *
 * @param mono_48k Input mono buffer at 48 kHz (length n_frames_48k) [normalized amplitude].
 * @param out_mono_8k Output mono buffer at 8 kHz (length n_frames_48k / 6) [normalized amplitude].
 * @param n_frames_48k Number of 48 kHz input frames (default 480) [samples].
 * @return std::size_t Number of 8 kHz output samples produced [samples].
 */
std::size_t SubharmonicPreStage::process_mono_f32_to_8k(
    const float* DSP_RESTRICT mono_48k,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    sync_config_if_needed(static_cast<float>(n_frames_48k) / static_cast<float>(kSampleRateHz));

    BiquadDF2T& s0 = lpf_4th_.section(0);
    BiquadDF2T& s1 = lpf_4th_.section(1);
    float s0_z1 = s0.z1, s0_z2 = s0.z2;
    float s1_z1 = s1.z1, s1_z2 = s1.z2;
    const float b0_0 = s0.b0, b1_0 = s0.b1, b2_0 = s0.b2, a1_0 = s0.a1, a2_0 = s0.a2;
    const float b0_1 = s1.b0, b1_1 = s1.b1, b2_1 = s1.b2, a1_1 = s1.a1, a2_1 = s1.a2;

    const std::size_t n_out = n_frames_48k / static_cast<std::size_t>(kSubDecimFactor);
    for (std::size_t g = 0; g < n_out; ++g) {
        const std::size_t base = g * static_cast<std::size_t>(kSubDecimFactor);
        for (std::size_t k = 0; k < static_cast<std::size_t>(kSubDecimFactor); ++k) {
            const float x = mono_48k[base + k];
            const float y0 = b0_0 * x + s0_z1;
            s0_z1 = b1_0 * x - a1_0 * y0 + s0_z2;
            s0_z2 = b2_0 * x - a2_0 * y0;

            const float y1 = b0_1 * y0 + s1_z1;
            s1_z1 = b1_1 * y0 - a1_1 * y1 + s1_z2;
            s1_z2 = b2_1 * y0 - a2_1 * y1;

            if (k == 0u) {
                out_mono_8k[g] = y1;
            }
        }
    }

    s0.z1 = s0_z1; s0.z2 = s0_z2;
    s1.z1 = s1_z1; s1.z2 = s1_z2;
    return n_out;
}

/**
 * @brief Downmix deinterleaved 48 kHz stereo int16 PCM to mono, apply 4th-order LP, and decimate 6:1 to 8 kHz float32.
 *
 * @param left_pcm16 Input Left 16-bit PCM buffer at 48 kHz [int16 PCM].
 * @param right_pcm16 Input Right 16-bit PCM buffer at 48 kHz [int16 PCM].
 * @param out_mono_8k Output mono float32 buffer at 8 kHz (length n_frames_48k / 6) [normalized amplitude].
 * @param n_frames_48k Number of 48 kHz input frames (default 480) [samples].
 * @return std::size_t Number of 8 kHz output samples produced [samples].
 */
std::size_t SubharmonicPreStage::process_stereo_pcm16_to_8k(
    const int16_t* DSP_RESTRICT left_pcm16,
    const int16_t* DSP_RESTRICT right_pcm16,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    sync_config_if_needed(static_cast<float>(n_frames_48k) / static_cast<float>(kSampleRateHz));

    BiquadDF2T& s0 = lpf_4th_.section(0);
    BiquadDF2T& s1 = lpf_4th_.section(1);
    float s0_z1 = s0.z1, s0_z2 = s0.z2;
    float s1_z1 = s1.z1, s1_z2 = s1.z2;
    const float b0_0 = s0.b0, b1_0 = s0.b1, b2_0 = s0.b2, a1_0 = s0.a1, a2_0 = s0.a2;
    const float b0_1 = s1.b0, b1_1 = s1.b1, b2_1 = s1.b2, a1_1 = s1.a1, a2_1 = s1.a2;

    constexpr float kHalfPcmScale = 0.5f / 32768.0f;
    const std::size_t n_out = n_frames_48k / static_cast<std::size_t>(kSubDecimFactor);
    for (std::size_t g = 0; g < n_out; ++g) {
        const std::size_t base = g * static_cast<std::size_t>(kSubDecimFactor);
        for (std::size_t k = 0; k < static_cast<std::size_t>(kSubDecimFactor); ++k) {
            const int32_t sum = static_cast<int32_t>(left_pcm16[base + k]) +
                                static_cast<int32_t>(right_pcm16[base + k]);
            const float x = static_cast<float>(sum) * kHalfPcmScale;

            const float y0 = b0_0 * x + s0_z1;
            s0_z1 = b1_0 * x - a1_0 * y0 + s0_z2;
            s0_z2 = b2_0 * x - a2_0 * y0;

            const float y1 = b0_1 * y0 + s1_z1;
            s1_z1 = b1_1 * y0 - a1_1 * y1 + s1_z2;
            s1_z2 = b2_1 * y0 - a2_1 * y1;

            if (k == 0u) {
                out_mono_8k[g] = y1;
            }
        }
    }

    s0.z1 = s0_z1; s0.z2 = s0_z2;
    s1.z1 = s1_z1; s1.z2 = s1_z2;
    return n_out;
}

/**
 * @brief Downmix interleaved 48 kHz stereo int16 PCM [L0, R0, L1, R1...] to mono, apply 4th-order LP, and decimate 6:1.
 *
 * @param stereo_pcm16 Interleaved stereo 16-bit PCM buffer at 48 kHz (2 * n_frames_48k elements) [int16 PCM].
 * @param out_mono_8k Output mono float32 buffer at 8 kHz (length n_frames_48k / 6) [normalized amplitude].
 * @param n_frames_48k Number of 48 kHz stereo frames (default 480) [samples].
 * @return std::size_t Number of 8 kHz output samples produced [samples].
 */
std::size_t SubharmonicPreStage::process_interleaved_pcm16_to_8k(
    const int16_t* DSP_RESTRICT stereo_pcm16,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    sync_config_if_needed(static_cast<float>(n_frames_48k) / static_cast<float>(kSampleRateHz));

    BiquadDF2T& s0 = lpf_4th_.section(0);
    BiquadDF2T& s1 = lpf_4th_.section(1);
    float s0_z1 = s0.z1, s0_z2 = s0.z2;
    float s1_z1 = s1.z1, s1_z2 = s1.z2;
    const float b0_0 = s0.b0, b1_0 = s0.b1, b2_0 = s0.b2, a1_0 = s0.a1, a2_0 = s0.a2;
    const float b0_1 = s1.b0, b1_1 = s1.b1, b2_1 = s1.b2, a1_1 = s1.a1, a2_1 = s1.a2;

    constexpr float kHalfPcmScale = 0.5f / 32768.0f;
    const std::size_t n_out = n_frames_48k / static_cast<std::size_t>(kSubDecimFactor);
    for (std::size_t g = 0; g < n_out; ++g) {
        const std::size_t base = g * static_cast<std::size_t>(kSubDecimFactor);
        for (std::size_t k = 0; k < static_cast<std::size_t>(kSubDecimFactor); ++k) {
            const std::size_t idx2 = 2u * (base + k);
            const int32_t sum = static_cast<int32_t>(stereo_pcm16[idx2]) +
                                static_cast<int32_t>(stereo_pcm16[idx2 + 1u]);
            const float x = static_cast<float>(sum) * kHalfPcmScale;

            const float y0 = b0_0 * x + s0_z1;
            s0_z1 = b1_0 * x - a1_0 * y0 + s0_z2;
            s0_z2 = b2_0 * x - a2_0 * y0;

            const float y1 = b0_1 * y0 + s1_z1;
            s1_z1 = b1_1 * y0 - a1_1 * y1 + s1_z2;
            s1_z2 = b2_1 * y0 - a2_1 * y1;

            if (k == 0u) {
                out_mono_8k[g] = y1;
            }
        }
    }

    s0.z1 = s0_z1; s0.z2 = s0_z2;
    s1.z1 = s1_z1; s1.z2 = s1_z2;
    return n_out;
}

}  // namespace audio_dsp

