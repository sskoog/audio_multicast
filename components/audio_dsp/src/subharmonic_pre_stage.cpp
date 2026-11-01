#include "subharmonic_pre_stage.hpp"

#include <algorithm>

namespace audio_dsp {

SubharmonicPreStage::SubharmonicPreStage(const SubharmonicPreStageConfig& cfg) noexcept {
    init(cfg);
}

/**
 * @brief Initialize the shared 48 kHz -> 8 kHz downsampling pre-stage.
 *
 * Clamps the cutoff frequency to [70.0, 200.0] Hz, maps the order to {2, 3, 4, 6}, configures the
 * Butterworth Low-Pass filter at 48 kHz, and clears internal state registers.
 *
 * @param cfg Pre-stage configuration structure [dimensionless struct].
 */
void SubharmonicPreStage::init(const SubharmonicPreStageConfig& cfg) noexcept {
    SubharmonicPreStageConfig sanitized = cfg;
    sanitized.lpf_cutoff_hz = std::clamp(cfg.lpf_cutoff_hz, kMinCutoffHz, kMaxCutoffHz);
    sanitized.lpf_order = normalize_order(cfg.lpf_order);
    target_cutoff_hz_ = sanitized.lpf_cutoff_hz;
    active_cutoff_hz_ = sanitized.lpf_cutoff_hz;
    active_order_ = sanitized.lpf_order;
    pending_order_.store(active_order_, std::memory_order_release);
    design_lpf();
    lpf_.reset();
    applied_gen_ = config_Queue_.set(sanitized);
}

void SubharmonicPreStage::set_cutoff_hz(float cutoff_hz) noexcept {
    SubharmonicPreStageConfig cfg{};
    cfg.lpf_cutoff_hz = std::clamp(cutoff_hz, kMinCutoffHz, kMaxCutoffHz);
    config_Queue_.set(cfg);  // only the cutoff is read from the queue; the order has its own atomic
}

void SubharmonicPreStage::reset() noexcept {
    lpf_.reset();
}

/**
 * @brief Coefficients for active_order_ / active_cutoff_hz_; sections the order does not use become
 *        pass-through (b0 = 1) so the fixed-length loops below can run them unconditionally.
 *        Filter state registers are kept (the caller decides whether to clear them).
 */
void SubharmonicPreStage::design_lpf() noexcept {
    lpf_.configure_butterworth_lpf(active_order_, active_cutoff_hz_, kSampleRateHz);
    for (std::size_t s = lpf_.num_active(); s < 3; ++s) {
        BiquadDF2T& b = lpf_.section(s);
        b.b0 = 1.0f;
        b.b1 = b.b2 = b.a1 = b.a2 = 0.0f;
        b.z1 = b.z2 = 0.0f;
    }
}

/**
 * @brief Synchronize pending cutoff (slewed) and order (switched) updates without blocking.
 */
void SubharmonicPreStage::sync_config_if_needed(float dt_sec) noexcept {
    uint32_t gen = 0u;
    const SubharmonicPreStageConfig cfg = config_Queue_.get(&gen);
    if (gen != applied_gen_) {
        applied_gen_ = gen;
        target_cutoff_hz_ = std::clamp(cfg.lpf_cutoff_hz, kMinCutoffHz, kMaxCutoffHz);
    }
    const int req_order = pending_order_.load(std::memory_order_acquire);
    if (req_order != active_order_) {
        active_order_ = req_order;
        design_lpf();
        lpf_.reset();  // new section structure: the old state does not apply
    }
    if (active_cutoff_hz_ != target_cutoff_hz_) {
        active_cutoff_hz_ = slew_towards(
            active_cutoff_hz_, target_cutoff_hz_, kMinCutoffHz, kMaxCutoffHz, dt_sec);
        float z[3][2];
        for (std::size_t s = 0; s < 3; ++s) {
            z[s][0] = lpf_.section(s).z1;
            z[s][1] = lpf_.section(s).z2;
        }
        design_lpf();
        for (std::size_t s = 0; s < lpf_.num_active(); ++s) {
            lpf_.section(s).z1 = z[s][0];
            lpf_.section(s).z2 = z[s][1];
        }
    }
}

/**
 * @brief LP-filter 48 kHz samples through the first NS sections and keep every 6th output (8 kHz).
 *
 * @param load_x       Callable size_t -> float returning input sample i (mono, normalized) [normalized amplitude].
 * @param out_mono_8k  Output at 8 kHz (n_frames_48k / 6 samples) [normalized amplitude].
 * @param n_frames_48k Input samples at 48 kHz [samples].
 * @return Output samples written [samples].
 */
template <std::size_t NS, typename LoadX>
std::size_t SubharmonicPreStage::run_lpf_decimate(
    LoadX load_x, float* DSP_RESTRICT out_mono_8k, std::size_t n_frames_48k) noexcept {
    float b0[NS], b1[NS], b2[NS], a1[NS], a2[NS], z1[NS], z2[NS];
    for (std::size_t s = 0; s < NS; ++s) {
        const BiquadDF2T& q = lpf_.section(s);
        b0[s] = q.b0; b1[s] = q.b1; b2[s] = q.b2; a1[s] = q.a1; a2[s] = q.a2;
        z1[s] = q.z1; z2[s] = q.z2;
    }
    const std::size_t n_out = n_frames_48k / static_cast<std::size_t>(kSubDecimFactor);
    for (std::size_t g = 0; g < n_out; ++g) {
        const std::size_t base = g * static_cast<std::size_t>(kSubDecimFactor);
        for (std::size_t k = 0; k < static_cast<std::size_t>(kSubDecimFactor); ++k) {
            float x = load_x(base + k);
            for (std::size_t s = 0; s < NS; ++s) {  // NS is a compile-time constant: fully unrolled
                const float y = b0[s] * x + z1[s];
                z1[s] = b1[s] * x - a1[s] * y + z2[s];
                z2[s] = b2[s] * x - a2[s] * y;
                x = y;
            }
            if (k == 0u) {
                out_mono_8k[g] = x;
            }
        }
    }
    for (std::size_t s = 0; s < NS; ++s) {
        lpf_.section(s).z1 = z1[s];
        lpf_.section(s).z2 = z2[s];
    }
    return n_out;
}

/** @brief 2 sections for order 2 / 3 / 4 (unused one is pass-through), 3 for order 6. */
template <typename LoadX>
std::size_t SubharmonicPreStage::lpf_decimate(
    LoadX load_x, float* DSP_RESTRICT out_mono_8k, std::size_t n_frames_48k) noexcept {
    sync_config_if_needed(static_cast<float>(n_frames_48k) / static_cast<float>(kSampleRateHz));
    if (lpf_.num_active() >= 3) {
        return run_lpf_decimate<3>(load_x, out_mono_8k, n_frames_48k);
    }
    return run_lpf_decimate<2>(load_x, out_mono_8k, n_frames_48k);
}

std::size_t SubharmonicPreStage::process_stereo_f32_to_8k(
    const float* DSP_RESTRICT left_48k,
    const float* DSP_RESTRICT right_48k,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    return lpf_decimate([&](std::size_t i) { return 0.5f * (left_48k[i] + right_48k[i]); },
                        out_mono_8k, n_frames_48k);
}

std::size_t SubharmonicPreStage::process_mono_f32_to_8k(
    const float* DSP_RESTRICT mono_48k,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    return lpf_decimate([&](std::size_t i) { return mono_48k[i]; }, out_mono_8k, n_frames_48k);
}

std::size_t SubharmonicPreStage::process_stereo_pcm16_to_8k(
    const int16_t* DSP_RESTRICT left_pcm16,
    const int16_t* DSP_RESTRICT right_pcm16,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    constexpr float kHalfPcmScale = 0.5f / 32768.0f;
    return lpf_decimate(
        [&](std::size_t i) {
            return static_cast<float>(static_cast<int32_t>(left_pcm16[i]) + static_cast<int32_t>(right_pcm16[i])) *
                   kHalfPcmScale;
        },
        out_mono_8k, n_frames_48k);
}

std::size_t SubharmonicPreStage::process_interleaved_pcm16_to_8k(
    const int16_t* DSP_RESTRICT stereo_pcm16,
    float* DSP_RESTRICT out_mono_8k,
    std::size_t n_frames_48k) noexcept {
    constexpr float kHalfPcmScale = 0.5f / 32768.0f;
    return lpf_decimate(
        [&](std::size_t i) {
            return static_cast<float>(static_cast<int32_t>(stereo_pcm16[2u * i]) +
                                      static_cast<int32_t>(stereo_pcm16[2u * i + 1u])) *
                   kHalfPcmScale;
        },
        out_mono_8k, n_frames_48k);
}

}  // namespace audio_dsp
