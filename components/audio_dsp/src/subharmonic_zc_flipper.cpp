#include "subharmonic_zc_flipper.hpp"

#include <algorithm>
#include <cmath>

namespace audio_dsp {

SubharmonicZcFlipper::SubharmonicZcFlipper(const SubharmonicZcFlipperConfig& cfg, int fs) noexcept {
    init(cfg, fs);
}

/**
 * @brief Initialize or reconfigure the 8 kHz Zero-Crossing Polarity Flipper and reset states.
 *
 * @param cfg Configuration parameters [dimensionless struct].
 * @param fs Operating sample rate in Hertz (default 8000 Hz) [Hz].
 */
void SubharmonicZcFlipper::init(const SubharmonicZcFlipperConfig& cfg, int fs) noexcept {
    fs_ = (fs > 0) ? fs : kSubSampleRateHz;
    apply_config_internal(cfg, true);
    target_cfg_ = active_cfg_;
    prev_dry_gain_ = active_cfg_.dry_gain;
    prev_sub_gain_ = active_cfg_.sub_gain;
    applied_gen_ = config_queue_.set(active_cfg_);
    reset();
}

void SubharmonicZcFlipper::set_config(const SubharmonicZcFlipperConfig& cfg) noexcept {
    config_queue_.set(cfg);
}

void SubharmonicZcFlipper::reset() noexcept {
    sign_ = 1.0f;
    armed_ = false;
    env_ = 0.0f;
    samples_since_toggle_ = max_period_samples_;
    post_lpf_.reset();
}

/**
 * @brief Sanitize configuration parameters and recompute filter/detector coefficients.
 *
 * @param cfg Requested configuration structure [dimensionless struct].
 * @param reset_states True to clear biquad delay lines, false to preserve state [boolean].
 */
void SubharmonicZcFlipper::apply_config_internal(
    const SubharmonicZcFlipperConfig& cfg,
    bool reset_states) noexcept {
    SubharmonicZcFlipperConfig sanitized = cfg;
    sanitized.post_lpf_cutoff_hz = std::clamp(cfg.post_lpf_cutoff_hz, kMinPostLpfHz, kMaxPostLpfHz);
    if (cfg.post_lpf_order <= 0) {
        sanitized.post_lpf_order = 0;
    } else if (cfg.post_lpf_order <= 2) {
        sanitized.post_lpf_order = 2;
    } else {
        sanitized.post_lpf_order = 4;
    }
    sanitized.hysteresis_ratio = std::clamp(cfg.hysteresis_ratio, kMinHysteresis, kMaxHysteresis);
    sanitized.release_ms = std::clamp(cfg.release_ms, kMinReleaseMs, kMaxReleaseMs);
    sanitized.min_floor_db = std::clamp(cfg.min_floor_db, kMinFloorDb, kMaxFloorDb);
    sanitized.static_thresh = std::max(0.0f, cfg.static_thresh);
    sanitized.guard_f_max_hz = std::clamp(cfg.guard_f_max_hz, kMinGuardFMaxHz, kMaxGuardFMaxHz);
    sanitized.guard_f_min_hz = std::clamp(cfg.guard_f_min_hz, kMinGuardFMinHz, sanitized.guard_f_max_hz);
    sanitized.dry_gain = std::clamp(cfg.dry_gain, kMinGain, kMaxGain);
    sanitized.sub_gain = std::clamp(cfg.sub_gain, kMinGain, kMaxGain);

    const float tau_rel = sanitized.release_ms * 1.0e-3f;
    alpha_rel_ = std::exp(-1.0f / (tau_rel * static_cast<float>(fs_)));
    min_floor_lin_ = std::pow(10.0f, sanitized.min_floor_db / 20.0f);

    const float f_max = std::max(20.0f, sanitized.guard_f_max_hz);
    const float f_min = std::clamp(sanitized.guard_f_min_hz, 10.0f, f_max);
    min_period_samples_ = static_cast<uint32_t>(std::max(1.0f, std::floor(static_cast<float>(fs_) / f_max)));
    max_period_samples_ = static_cast<uint32_t>(std::max(1.0f, std::ceil(static_cast<float>(fs_) / f_min)));

    const bool filter_changed =
        reset_states ||
        (sanitized.post_lpf_order != active_cfg_.post_lpf_order) ||
        (sanitized.post_lpf_cutoff_hz != active_cfg_.post_lpf_cutoff_hz);

    active_cfg_ = sanitized;

    if (filter_changed) {
        const float z0_1 = post_lpf_.section(0).z1;
        const float z0_2 = post_lpf_.section(0).z2;
        const float z1_1 = post_lpf_.section(1).z1;
        const float z1_2 = post_lpf_.section(1).z2;
        post_lpf_.configure_butterworth_lpf(active_cfg_.post_lpf_order, active_cfg_.post_lpf_cutoff_hz, fs_);
        if (!reset_states) {
            post_lpf_.section(0).z1 = z0_1;
            post_lpf_.section(0).z2 = z0_2;
            post_lpf_.section(1).z1 = z1_1;
            post_lpf_.section(1).z2 = z1_2;
        }
    }
}

void SubharmonicZcFlipper::sync_config_if_needed(float dt_sec) noexcept {
    prev_dry_gain_ = active_cfg_.dry_gain;
    prev_sub_gain_ = active_cfg_.sub_gain;

    uint32_t gen = 0u;
    const SubharmonicZcFlipperConfig cfg = config_queue_.get(&gen);
    if (gen != applied_gen_) {
        applied_gen_ = gen;
        target_cfg_ = cfg;
        target_cfg_.post_lpf_cutoff_hz = std::clamp(cfg.post_lpf_cutoff_hz, kMinPostLpfHz, kMaxPostLpfHz);
        target_cfg_.hysteresis_ratio = std::clamp(cfg.hysteresis_ratio, kMinHysteresis, kMaxHysteresis);
        target_cfg_.release_ms = std::clamp(cfg.release_ms, kMinReleaseMs, kMaxReleaseMs);
        target_cfg_.min_floor_db = std::clamp(cfg.min_floor_db, kMinFloorDb, kMaxFloorDb);
        target_cfg_.static_thresh = std::max(0.0f, cfg.static_thresh);
        target_cfg_.guard_f_max_hz = std::clamp(cfg.guard_f_max_hz, kMinGuardFMaxHz, kMaxGuardFMaxHz);
        target_cfg_.guard_f_min_hz = std::clamp(cfg.guard_f_min_hz, kMinGuardFMinHz, target_cfg_.guard_f_max_hz);
        target_cfg_.dry_gain = std::clamp(cfg.dry_gain, kMinGain, kMaxGain);
        target_cfg_.sub_gain = std::clamp(cfg.sub_gain, kMinGain, kMaxGain);
    }

    const bool needs_slew =
        (active_cfg_.post_lpf_cutoff_hz != target_cfg_.post_lpf_cutoff_hz) ||
        (active_cfg_.post_lpf_order != target_cfg_.post_lpf_order) ||
        (active_cfg_.use_adaptive_thresh != target_cfg_.use_adaptive_thresh) ||
        (active_cfg_.static_thresh != target_cfg_.static_thresh) ||
        (active_cfg_.hysteresis_ratio != target_cfg_.hysteresis_ratio) ||
        (active_cfg_.release_ms != target_cfg_.release_ms) ||
        (active_cfg_.min_floor_db != target_cfg_.min_floor_db) ||
        (active_cfg_.use_period_guard != target_cfg_.use_period_guard) ||
        (active_cfg_.guard_f_min_hz != target_cfg_.guard_f_min_hz) ||
        (active_cfg_.guard_f_max_hz != target_cfg_.guard_f_max_hz) ||
        (active_cfg_.dry_gain != target_cfg_.dry_gain) ||
        (active_cfg_.sub_gain != target_cfg_.sub_gain);

    if (needs_slew) {
        SubharmonicZcFlipperConfig next = target_cfg_;
        next.post_lpf_cutoff_hz = slew_towards(
            active_cfg_.post_lpf_cutoff_hz, target_cfg_.post_lpf_cutoff_hz, kMinPostLpfHz, kMaxPostLpfHz, dt_sec);
        next.hysteresis_ratio = slew_towards(
            active_cfg_.hysteresis_ratio, target_cfg_.hysteresis_ratio, kMinHysteresis, kMaxHysteresis, dt_sec);
        next.release_ms = slew_towards(
            active_cfg_.release_ms, target_cfg_.release_ms, kMinReleaseMs, kMaxReleaseMs, dt_sec);
        next.min_floor_db = slew_towards(
            active_cfg_.min_floor_db, target_cfg_.min_floor_db, kMinFloorDb, kMaxFloorDb, dt_sec);
        next.guard_f_min_hz = slew_towards(
            active_cfg_.guard_f_min_hz, target_cfg_.guard_f_min_hz, kMinGuardFMinHz, kMaxGuardFMinHz, dt_sec);
        next.guard_f_max_hz = slew_towards(
            active_cfg_.guard_f_max_hz, target_cfg_.guard_f_max_hz, kMinGuardFMaxHz, kMaxGuardFMaxHz, dt_sec);
        next.dry_gain = slew_towards(
            active_cfg_.dry_gain, target_cfg_.dry_gain, kMinGain, kMaxGain, dt_sec);
        next.sub_gain = slew_towards(
            active_cfg_.sub_gain, target_cfg_.sub_gain, kMinGain, kMaxGain, dt_sec);
        apply_config_internal(next, false);
    }
}

/**
 * @brief Process a block of 8 kHz float32 samples through the zero-crossing flipper and post-LPF.
 *
 * Matches flip_on_zero_crossings_adaptive (shr_experiments.ipynb) and flip_on_zero_crossings
 * (audio_filters.py) running at 8 kHz.
 *
 * @param in_8k Input 8 kHz LP-filtered mono bass buffer [normalized amplitude].
 * @param out_8k Output 8 kHz mono subwoofer buffer [normalized amplitude].
 * @param n_samples Number of 8 kHz samples to process (default 80, max 480) [samples].
 */
void SubharmonicZcFlipper::process_block_f32(
    const float* DSP_RESTRICT in_8k,
    float* DSP_RESTRICT out_8k,
    std::size_t n_samples) noexcept {
    const std::size_t count = std::min(n_samples, kSubFrameSamples);
    const float dt_sec = static_cast<float>(count) / static_cast<float>((fs_ > 0) ? fs_ : kSubSampleRateHz);
    sync_config_if_needed(dt_sec);

    float sign = sign_;
    bool armed = armed_;
    float env = env_;
    uint32_t period_cnt = samples_since_toggle_;

    const bool adaptive = active_cfg_.use_adaptive_thresh;
    const float alpha_rel = alpha_rel_;
    const float hyst = active_cfg_.hysteresis_ratio;
    const float min_floor = min_floor_lin_;
    const float static_th = active_cfg_.static_thresh;
    const bool use_guard = active_cfg_.use_period_guard;
    const uint32_t min_per = min_period_samples_;

    for (std::size_t i = 0; i < count; ++i) {
        const float x = in_8k[i];
        work_in_[i] = x;

        float dyn_thresh = static_th;
        if (adaptive) {
            const float abs_x = std::fabs(x);
            if (abs_x > env) {
                env = abs_x;
            } else {
                env *= alpha_rel;
            }
            dyn_thresh = std::max(min_floor, env * hyst);
        }

        ++period_cnt;

        if (x < -dyn_thresh) {
            armed = true;
        } else if (armed && x >= 0.0f) {
            if (!use_guard || period_cnt >= min_per) {
                sign = -sign;
                period_cnt = 0u;
            }
            armed = false;
        }

        flipped_buf_[i] = x * sign;
    }

    sign_ = sign;
    armed_ = armed;
    env_ = env;
    samples_since_toggle_ = period_cnt;

    post_lpf_.process_block(flipped_buf_, smooth_buf_, count);

    const float g_dry1 = active_cfg_.dry_gain;
    const float g_sub1 = active_cfg_.sub_gain;
    const float inv_n = (count > 0u) ? (1.0f / static_cast<float>(count)) : 0.0f;
    const float step_dry = (g_dry1 - prev_dry_gain_) * inv_n;
    const float step_sub = (g_sub1 - prev_sub_gain_) * inv_n;
    float g_dry = prev_dry_gain_;
    float g_sub = prev_sub_gain_;
    for (std::size_t i = 0; i < count; ++i) {
        g_dry += step_dry;
        g_sub += step_sub;
        out_8k[i] = g_dry * work_in_[i] + g_sub * smooth_buf_[i];
    }
    prev_dry_gain_ = g_dry1;
    prev_sub_gain_ = g_sub1;
}

/**
 * @brief Process a block of 8 kHz 16-bit PCM samples through the zero-crossing flipper.
 *
 * @param in_pcm16_8k Input 8 kHz 16-bit PCM mono buffer [int16 PCM].
 * @param out_pcm16_8k Output 8 kHz 16-bit PCM mono buffer [int16 PCM].
 * @param n_samples Number of 8 kHz samples to process (default 80) [samples].
 */
void SubharmonicZcFlipper::process_block_pcm16(
    const int16_t* DSP_RESTRICT in_pcm16_8k,
    int16_t* DSP_RESTRICT out_pcm16_8k,
    std::size_t n_samples) noexcept {
    const std::size_t count = std::min(n_samples, kSubFrameSamples);
    alignas(16) float in_f32[kSubFrameSamples]{};
    alignas(16) float out_f32[kSubFrameSamples]{};
    pcm16_to_float_block(in_pcm16_8k, in_f32, count);
    process_block_f32(in_f32, out_f32, count);
    float_to_pcm16_block(out_f32, out_pcm16_8k, count);
}

}  // namespace audio_dsp
