#include "subharmonic_nccf_synth.hpp"

#include <algorithm>
#include <cmath>

namespace audio_dsp {

SubharmonicNccfSynth::SubharmonicNccfSynth(const SubharmonicNccfConfig& cfg, int fs) noexcept
    : nco_(fs), env_follower_(cfg.env_attack_ms, cfg.env_decay_ms, fs) {
    init(cfg, fs);
}

/**
 * @brief Initialize or reconfigure the 8 kHz NCCF Subharmonic Synthesizer and clear states.
 *
 * @param cfg Configuration parameters [dimensionless struct].
 * @param fs Operating sample rate in Hertz (default 8000 Hz) [Hz].
 */
void SubharmonicNccfSynth::init(const SubharmonicNccfConfig& cfg, int fs) noexcept {
    fs_ = (fs > 0) ? fs : kSubSampleRateHz;
    nco_.set_sample_rate(fs_);
    apply_config_internal(cfg, true);
    target_cfg_ = active_cfg_;
    prev_dry_gain_ = active_cfg_.dry_gain;
    prev_sub_gain_ = active_cfg_.sub_gain;
    applied_gen_ = config_queue_.set(active_cfg_);
    reset();
}

void SubharmonicNccfSynth::set_config(const SubharmonicNccfConfig& cfg) noexcept {
    config_queue_.set(cfg);
}

/**
 * @brief Reset circular buffer, pitch tracker, NCO, envelope follower, and filter states.
 */
void SubharmonicNccfSynth::reset() noexcept {
    circ_buffer_.fill(0.0f);
    lag_energy_.fill(0.0f);
    nccf_raw_.fill(0.0f);
    nccf_weighted_.fill(0.0f);
    lag_evaluated_.fill(0u);

    is_voiced_ = false;
    hangover_counter_ = 0;
    last_valid_f0_ = 0.0f;
    pitch_history_.fill(0.0f);
    pitch_hist_count_ = 0;

    nco_.reset();
    env_follower_.reset();
    dry_hpf_.reset();
    last_diag_ = SubharmonicNccfDiagnostics{};
}

/**
 * @brief Sanitize configuration parameters and recompute lag bounds, ducking weights, and filter coefficients.
 *
 * @param cfg Requested configuration structure [dimensionless struct].
 * @param reset_states True to reset envelope and filter states [boolean].
 */
void SubharmonicNccfSynth::apply_config_internal(
    const SubharmonicNccfConfig& cfg,
    bool reset_states) noexcept {
    SubharmonicNccfConfig sanitized = cfg;
    sanitized.f_min_hz = std::clamp(cfg.f_min_hz, kMinFMinHz, kMaxFMinHz);
    sanitized.f_max_hz = std::clamp(cfg.f_max_hz, std::max(kMinFMaxHz, sanitized.f_min_hz + 10.0f), kMaxFMaxHz);
    sanitized.voicing_thresh_on = std::clamp(cfg.voicing_thresh_on, kMinVoicingThresh, kMaxVoicingThresh);
    sanitized.voicing_thresh_off = std::clamp(cfg.voicing_thresh_off, kMinVoicingThresh, sanitized.voicing_thresh_on);
    sanitized.hangover_frames = std::max(0, cfg.hangover_frames);
    sanitized.lag_first_peak_weight = std::clamp(cfg.lag_first_peak_weight, kMinLagWeight, kMaxLagWeight);
    sanitized.coarse_step = std::clamp(cfg.coarse_step, 1, 8);
    sanitized.fine_radius = std::clamp(cfg.fine_radius, 1, 8);
    sanitized.harmonic_3x_beta = std::clamp(cfg.harmonic_3x_beta, kMinHarmonicBeta, kMaxHarmonicBeta);
    sanitized.sub_duck_cutoff_hz = std::clamp(cfg.sub_duck_cutoff_hz, kMinDuckCutoffHz, kMaxDuckCutoffHz);
    sanitized.sub_duck_thresh_ratio = std::clamp(cfg.sub_duck_thresh_ratio, kMinDuckThreshRatio, kMaxDuckThreshRatio);
    sanitized.env_attack_ms = std::clamp(cfg.env_attack_ms, kMinEnvAttackMs, kMaxEnvAttackMs);
    sanitized.env_decay_ms = std::clamp(cfg.env_decay_ms, kMinEnvDecayMs, kMaxEnvDecayMs);
    sanitized.dry_hpf_hz = std::clamp(cfg.dry_hpf_hz, kMinDryHpfHz, kMaxDryHpfHz);
    sanitized.dry_gain = std::clamp(cfg.dry_gain, kMinGain, kMaxGain);
    sanitized.sub_gain = std::clamp(cfg.sub_gain, kMinGain, kMaxGain);

    const std::size_t raw_k_min = static_cast<std::size_t>(
        std::max(4.0f, std::floor(static_cast<float>(fs_) / sanitized.f_max_hz)));
    const std::size_t raw_k_max = static_cast<std::size_t>(
        std::min(static_cast<float>(kMaxLag), std::ceil(static_cast<float>(fs_) / sanitized.f_min_hz)));

    k_min_ = std::min(raw_k_min, kMaxLag - 4u);
    k_max_ = std::max(k_min_ + 2u, std::min(raw_k_max, kMaxLag));
    win_len_ = (kCircBufSize > k_max_ + 8u) ? (kCircBufSize - k_max_) : 40u;

    if (reset_states || sanitized.sub_duck_cutoff_hz != active_cfg_.sub_duck_cutoff_hz) {
        // Precompute Blackman-Tukey low-pass impulse response weights for sub-bass ducking
        const float wc = kTwoPi * sanitized.sub_duck_cutoff_hz / static_cast<float>(fs_);
        duck_weights_[0] = 2.0f * sanitized.sub_duck_cutoff_hz / static_cast<float>(fs_);
        for (std::size_t k = 1; k < kDuckLags; ++k) {
            const float fk = static_cast<float>(k);
            const float sinc = std::sin(wc * fk) / (kPi * fk);
            const float hamming = 0.54f + 0.46f * std::cos(kPi * fk / static_cast<float>(kDuckLags - 1u));
            duck_weights_[k] = sinc * hamming;
        }
    }

    if (reset_states ||
        sanitized.env_attack_ms != active_cfg_.env_attack_ms ||
        sanitized.env_decay_ms != active_cfg_.env_decay_ms) {
        env_follower_.configure(sanitized.env_attack_ms, sanitized.env_decay_ms, fs_);
    }

    const bool hpf_changed =
        reset_states ||
        (sanitized.apply_dry_hpf != active_cfg_.apply_dry_hpf) ||
        (sanitized.dry_hpf_hz != active_cfg_.dry_hpf_hz);

    active_cfg_ = sanitized;

    if (hpf_changed) {
        const float z1 = dry_hpf_.section(0).z1;
        const float z2 = dry_hpf_.section(0).z2;
        dry_hpf_.configure_hpf(active_cfg_.apply_dry_hpf ? 2 : 0, active_cfg_.dry_hpf_hz, fs_, kDefaultQ);
        if (!reset_states) {
            dry_hpf_.section(0).z1 = z1;
            dry_hpf_.section(0).z2 = z2;
        }
    }
}

void SubharmonicNccfSynth::sync_config_if_needed(float dt_sec) noexcept {
    prev_dry_gain_ = active_cfg_.dry_gain;
    prev_sub_gain_ = active_cfg_.sub_gain;

    uint32_t gen = 0u;
    const SubharmonicNccfConfig cfg = config_queue_.get(&gen);
    if (gen != applied_gen_) {
        applied_gen_ = gen;
        target_cfg_ = cfg;
        target_cfg_.f_min_hz = std::clamp(cfg.f_min_hz, kMinFMinHz, kMaxFMinHz);
        target_cfg_.f_max_hz = std::clamp(cfg.f_max_hz, std::max(kMinFMaxHz, target_cfg_.f_min_hz + 10.0f), kMaxFMaxHz);
        target_cfg_.voicing_thresh_on = std::clamp(cfg.voicing_thresh_on, kMinVoicingThresh, kMaxVoicingThresh);
        target_cfg_.voicing_thresh_off = std::clamp(cfg.voicing_thresh_off, kMinVoicingThresh, target_cfg_.voicing_thresh_on);
        target_cfg_.lag_first_peak_weight = std::clamp(cfg.lag_first_peak_weight, kMinLagWeight, kMaxLagWeight);
        target_cfg_.harmonic_3x_beta = std::clamp(cfg.harmonic_3x_beta, kMinHarmonicBeta, kMaxHarmonicBeta);
        target_cfg_.sub_duck_cutoff_hz = std::clamp(cfg.sub_duck_cutoff_hz, kMinDuckCutoffHz, kMaxDuckCutoffHz);
        target_cfg_.sub_duck_thresh_ratio = std::clamp(cfg.sub_duck_thresh_ratio, kMinDuckThreshRatio, kMaxDuckThreshRatio);
        target_cfg_.env_attack_ms = std::clamp(cfg.env_attack_ms, kMinEnvAttackMs, kMaxEnvAttackMs);
        target_cfg_.env_decay_ms = std::clamp(cfg.env_decay_ms, kMinEnvDecayMs, kMaxEnvDecayMs);
        target_cfg_.dry_hpf_hz = std::clamp(cfg.dry_hpf_hz, kMinDryHpfHz, kMaxDryHpfHz);
        target_cfg_.dry_gain = std::clamp(cfg.dry_gain, kMinGain, kMaxGain);
        target_cfg_.sub_gain = std::clamp(cfg.sub_gain, kMinGain, kMaxGain);
    }

    const bool needs_slew =
        (active_cfg_.f_min_hz != target_cfg_.f_min_hz) ||
        (active_cfg_.f_max_hz != target_cfg_.f_max_hz) ||
        (active_cfg_.voicing_thresh_on != target_cfg_.voicing_thresh_on) ||
        (active_cfg_.voicing_thresh_off != target_cfg_.voicing_thresh_off) ||
        (active_cfg_.hangover_frames != target_cfg_.hangover_frames) ||
        (active_cfg_.lag_first_peak_weight != target_cfg_.lag_first_peak_weight) ||
        (active_cfg_.use_hierarchical_search != target_cfg_.use_hierarchical_search) ||
        (active_cfg_.coarse_step != target_cfg_.coarse_step) ||
        (active_cfg_.fine_radius != target_cfg_.fine_radius) ||
        (active_cfg_.enable_3x_harmonic_check != target_cfg_.enable_3x_harmonic_check) ||
        (active_cfg_.harmonic_3x_beta != target_cfg_.harmonic_3x_beta) ||
        (active_cfg_.use_sub_ducking != target_cfg_.use_sub_ducking) ||
        (active_cfg_.sub_duck_cutoff_hz != target_cfg_.sub_duck_cutoff_hz) ||
        (active_cfg_.sub_duck_thresh_ratio != target_cfg_.sub_duck_thresh_ratio) ||
        (active_cfg_.env_attack_ms != target_cfg_.env_attack_ms) ||
        (active_cfg_.env_decay_ms != target_cfg_.env_decay_ms) ||
        (active_cfg_.apply_dry_hpf != target_cfg_.apply_dry_hpf) ||
        (active_cfg_.dry_hpf_hz != target_cfg_.dry_hpf_hz) ||
        (active_cfg_.dry_gain != target_cfg_.dry_gain) ||
        (active_cfg_.sub_gain != target_cfg_.sub_gain);

    if (needs_slew) {
        SubharmonicNccfConfig next = target_cfg_;
        next.f_min_hz = slew_towards(active_cfg_.f_min_hz, target_cfg_.f_min_hz, kMinFMinHz, kMaxFMinHz, dt_sec);
        next.f_max_hz = slew_towards(active_cfg_.f_max_hz, target_cfg_.f_max_hz, kMinFMaxHz, kMaxFMaxHz, dt_sec);
        next.voicing_thresh_on = slew_towards(active_cfg_.voicing_thresh_on, target_cfg_.voicing_thresh_on, kMinVoicingThresh, kMaxVoicingThresh, dt_sec);
        next.voicing_thresh_off = slew_towards(active_cfg_.voicing_thresh_off, target_cfg_.voicing_thresh_off, kMinVoicingThresh, kMaxVoicingThresh, dt_sec);
        next.lag_first_peak_weight = slew_towards(active_cfg_.lag_first_peak_weight, target_cfg_.lag_first_peak_weight, kMinLagWeight, kMaxLagWeight, dt_sec);
        next.harmonic_3x_beta = slew_towards(active_cfg_.harmonic_3x_beta, target_cfg_.harmonic_3x_beta, kMinHarmonicBeta, kMaxHarmonicBeta, dt_sec);
        next.sub_duck_cutoff_hz = slew_towards(active_cfg_.sub_duck_cutoff_hz, target_cfg_.sub_duck_cutoff_hz, kMinDuckCutoffHz, kMaxDuckCutoffHz, dt_sec);
        next.sub_duck_thresh_ratio = slew_towards(active_cfg_.sub_duck_thresh_ratio, target_cfg_.sub_duck_thresh_ratio, kMinDuckThreshRatio, kMaxDuckThreshRatio, dt_sec);
        next.env_attack_ms = slew_towards(active_cfg_.env_attack_ms, target_cfg_.env_attack_ms, kMinEnvAttackMs, kMaxEnvAttackMs, dt_sec);
        next.env_decay_ms = slew_towards(active_cfg_.env_decay_ms, target_cfg_.env_decay_ms, kMinEnvDecayMs, kMaxEnvDecayMs, dt_sec);
        next.dry_hpf_hz = slew_towards(active_cfg_.dry_hpf_hz, target_cfg_.dry_hpf_hz, kMinDryHpfHz, kMaxDryHpfHz, dt_sec);
        next.dry_gain = slew_towards(active_cfg_.dry_gain, target_cfg_.dry_gain, kMinGain, kMaxGain, dt_sec);
        next.sub_gain = slew_towards(active_cfg_.sub_gain, target_cfg_.sub_gain, kMinGain, kMaxGain, dt_sec);
        apply_config_internal(next, false);
    }
}

/**
 * @brief Evaluate the normalized cross-correlation at a single lag k using precomputed energy e1.
 *
 * @param k Candidate lag in samples at 8 kHz [samples].
 * @param x0 Pointer to the current analysis window of length win_len [normalized amplitude].
 * @param win_len Length of the correlation window in samples [samples].
 * @param e0 Energy of the current window x0 [normalized power].
 * @param e1 Energy of the lagged window x_k [normalized power].
 * @return float Unweighted NCCF coefficient R(k) in [-1.0, 1.0] [dimensionless].
 */
float SubharmonicNccfSynth::evaluate_single_lag(
    std::size_t k,
    const float* x0,
    std::size_t win_len,
    float e0,
    float e1) const noexcept {
    if (e1 <= 1.0e-12f) {
        return 0.0f;
    }
    const float* xk = &circ_buffer_[kCircBufSize - win_len - k];
    float cross = 0.0f;
    for (std::size_t i = 0; i < win_len; ++i) {
        cross += x0[i] * xk[i];
    }
    return cross / std::sqrt(e0 * e1 + 1.0e-12f);
}

/**
 * @brief Detect fundamental frequency f0 using NCCF with O(1) incremental lag energy,
 *        hierarchical coarse-to-fine search, 3*k_naive disambiguation, and parabolic refinement.
 *
 * Matches SubharmonicSynthesizer._compute_nccf and _detect_pitch in subharmonic_synth.py.
 *
 * @param out_f0_hz Output reference receiving interpolated fundamental frequency in Hertz [Hz].
 * @param out_r_peak Output reference receiving peak weighted NCCF correlation [dimensionless].
 */
void SubharmonicNccfSynth::detect_pitch_nccf(float& out_f0_hz, float& out_r_peak) noexcept {
    const std::size_t w = win_len_;
    const float* x0 = &circ_buffer_[kCircBufSize - w];

    float e0 = 0.0f;
    for (std::size_t i = 0; i < w; ++i) {
        e0 += x0[i] * x0[i];
    }
    if (e0 < 1.0e-7f) {
        out_f0_hz = 0.0f;
        out_r_peak = 0.0f;
        return;
    }

    // 1. O(1) incremental lag window energy across [k_min_, k_max_]
    float e1 = 0.0f;
    const float* x_first = &circ_buffer_[kCircBufSize - w - k_min_];
    for (std::size_t i = 0; i < w; ++i) {
        e1 += x_first[i] * x_first[i];
    }
    lag_energy_[k_min_] = std::max(0.0f, e1);

    for (std::size_t k = k_min_ + 1u; k <= k_max_; ++k) {
        const float entering = circ_buffer_[kCircBufSize - w - k];
        const float leaving = circ_buffer_[kCircBufSize - k];
        e1 = e1 + entering * entering - leaving * leaving;
        lag_energy_[k] = std::max(0.0f, e1);
    }

    const float lag_span = static_cast<float>(std::max<std::size_t>(1u, k_max_ - k_min_));
    const float alpha_w = active_cfg_.lag_first_peak_weight;

    auto eval_lag_cached = [&](std::size_t k) noexcept {
        if (k < k_min_ || k > k_max_ || lag_evaluated_[k] != 0u) {
            return;
        }
        const float raw = evaluate_single_lag(k, x0, w, e0, lag_energy_[k]);
        const float weight = 1.0f - alpha_w * static_cast<float>(k - k_min_) / lag_span;
        nccf_raw_[k] = raw;
        nccf_weighted_[k] = raw * weight;
        lag_evaluated_[k] = 1u;
    };

    for (std::size_t k = k_min_; k <= k_max_; ++k) {
        nccf_raw_[k] = 0.0f;
        nccf_weighted_[k] = 0.0f;
        lag_evaluated_[k] = 0u;
    }

    if (!active_cfg_.use_hierarchical_search || active_cfg_.coarse_step <= 1) {
        for (std::size_t k = k_min_; k <= k_max_; ++k) {
            eval_lag_cached(k);
        }
    } else {
        const std::size_t step = static_cast<std::size_t>(active_cfg_.coarse_step);
        const std::size_t radius = static_cast<std::size_t>(active_cfg_.fine_radius);

        std::size_t top1_k = k_min_;
        std::size_t top2_k = k_min_;
        float top1_val = -2.0f;
        float top2_val = -2.0f;

        for (std::size_t k = k_min_; k <= k_max_; k += step) {
            eval_lag_cached(k);
            const float val = nccf_weighted_[k];
            if (val > top1_val) {
                top2_val = top1_val;
                top2_k = top1_k;
                top1_val = val;
                top1_k = k;
            } else if (val > top2_val) {
                top2_val = val;
                top2_k = k;
            }
        }

        const std::size_t cands[2] = {top1_k, top2_k};
        for (std::size_t c_idx = 0; c_idx < 2u; ++c_idx) {
            const std::size_t kc = cands[c_idx];
            const std::size_t lo = (kc > k_min_ + radius) ? (kc - radius) : k_min_;
            const std::size_t hi = std::min(k_max_, kc + radius);
            for (std::size_t k = lo; k <= hi; ++k) {
                eval_lag_cached(k);
            }
        }
    }

    // Find naive peak on weighted NCCF
    std::size_t k_naive = k_min_;
    float r_naive = nccf_weighted_[k_min_];
    for (std::size_t k = k_min_ + 1u; k <= k_max_; ++k) {
        if (lag_evaluated_[k] != 0u && nccf_weighted_[k] > r_naive) {
            r_naive = nccf_weighted_[k];
            k_naive = k;
        }
    }

    std::size_t k_peak = k_naive;
    float r_peak = r_naive;

    // 3 * k_naive harmonic disambiguation check
    if (active_cfg_.enable_3x_harmonic_check && r_naive > 0.0f) {
        const std::size_t k_3x = 3u * k_naive;
        if (k_3x <= k_max_) {
            const std::size_t search_lo = (k_3x > k_min_ + 3u) ? (k_3x - 3u) : k_min_;
            const std::size_t search_hi = std::min(k_max_, k_3x + 3u);
            std::size_t best_3x_k = k_3x;
            float best_3x_val = -2.0f;
            for (std::size_t k = search_lo; k <= search_hi; ++k) {
                eval_lag_cached(k);
                if (nccf_weighted_[k] > best_3x_val) {
                    best_3x_val = nccf_weighted_[k];
                    best_3x_k = k;
                }
            }
            if (best_3x_val >= active_cfg_.harmonic_3x_beta * r_naive) {
                k_peak = best_3x_k;
                r_peak = best_3x_val;
            }
        }
    }

    // Ensure neighbors k_peak - 1 and k_peak + 1 are evaluated for 3-point parabolic refinement
    if (k_peak > k_min_) {
        eval_lag_cached(k_peak - 1u);
    }
    if (k_peak < k_max_) {
        eval_lag_cached(k_peak + 1u);
    }

    float k_interp = static_cast<float>(k_peak);
    if (k_peak > k_min_ && k_peak < k_max_) {
        const float y_prev = nccf_raw_[k_peak - 1u];
        const float y_curr = nccf_raw_[k_peak];
        const float y_next = nccf_raw_[k_peak + 1u];
        const float denom = y_prev - 2.0f * y_curr + y_next;
        if (std::fabs(denom) > 1.0e-9f) {
            const float delta = std::clamp(0.5f * (y_prev - y_next) / denom, -0.5f, 0.5f);
            k_interp += delta;
        }
    }

    out_f0_hz = static_cast<float>(fs_) / k_interp;
    out_r_peak = r_peak;
}

/**
 * @brief Compute sub-bass (< 45 Hz) ducking gain via Blackman-Tukey short-lag correlator.
 *
 * Matches SubBassEnergyCorrelator.compute_duck_gain in subharmonic_synth.py.
 *
 * @return float Linear ducking gain in [0.0f, 1.0f] [dimensionless].
 */
float SubharmonicNccfSynth::compute_sub_duck_gain() const noexcept {
    if (!active_cfg_.use_sub_ducking) {
        return 1.0f;
    }

    const std::size_t n = kCircBufSize;
    std::array<float, kDuckLags> r{};
    for (std::size_t k = 0; k < kDuckLags; ++k) {
        float acc = 0.0f;
        const std::size_t len = n - k;
        for (std::size_t i = 0; i < len; ++i) {
            acc += circ_buffer_[i + k] * circ_buffer_[i];
        }
        r[k] = acc / static_cast<float>(n);
    }

    if (r[0] < 1.0e-7f) {
        return 1.0f;
    }

    float e_sub = r[0] * duck_weights_[0];
    for (std::size_t k = 1; k < kDuckLags; ++k) {
        e_sub += 2.0f * r[k] * duck_weights_[k];
    }
    e_sub = std::max(0.0f, e_sub);

    const float ratio = e_sub / (r[0] + 1.0e-9f);
    const float gain = 1.0f - (ratio / active_cfg_.sub_duck_thresh_ratio);
    return std::clamp(gain, 0.0f, 1.0f);
}

/**
 * @brief Process a block of 8 kHz float32 samples through the NCCF subharmonic synthesizer.
 *
 * @param in_8k Input 8 kHz LP-filtered mono bass buffer [normalized amplitude].
 * @param out_8k Output 8 kHz mono subwoofer buffer [normalized amplitude].
 * @param n_samples Number of 8 kHz samples to process (default 80) [samples].
 */
void SubharmonicNccfSynth::process_block_f32(
    const float* DSP_RESTRICT in_8k,
    float* DSP_RESTRICT out_8k,
    std::size_t n_samples) noexcept {
    const std::size_t count = std::min(n_samples, kSubFrameSamples);
    const float dt_sec = static_cast<float>(count) / static_cast<float>((fs_ > 0) ? fs_ : kSubSampleRateHz);
    sync_config_if_needed(dt_sec);

    for (std::size_t i = 0; i < count; ++i) {
        work_in_[i] = in_8k[i];
    }

    // 1. Update 240-sample circular analysis buffer at 8 kHz
    if (count >= kCircBufSize) {
        std::copy(work_in_ + (count - kCircBufSize), work_in_ + count, circ_buffer_.begin());
    } else {
        std::move(circ_buffer_.begin() + count, circ_buffer_.end(), circ_buffer_.begin());
        std::copy(work_in_, work_in_ + count, circ_buffer_.end() - count);
    }

    // 2. NCCF Pitch Detection
    float f0_raw = 0.0f;
    float r_peak = 0.0f;
    detect_pitch_nccf(f0_raw, r_peak);

    // 3. Voicing Hysteresis & Hangover State Machine
    if (!is_voiced_) {
        if (r_peak >= active_cfg_.voicing_thresh_on) {
            is_voiced_ = true;
            hangover_counter_ = active_cfg_.hangover_frames;
            last_valid_f0_ = f0_raw;
        } else {
            f0_raw = 0.0f;
        }
    } else {
        if (r_peak >= active_cfg_.voicing_thresh_off) {
            hangover_counter_ = active_cfg_.hangover_frames;
            last_valid_f0_ = f0_raw;
        } else if (hangover_counter_ > 0) {
            --hangover_counter_;
            f0_raw = last_valid_f0_;
        } else {
            is_voiced_ = false;
            f0_raw = 0.0f;
            pitch_hist_count_ = 0;
        }
    }

    // Median-of-3 pitch history smoothing
    float f0_smooth = 0.0f;
    if (is_voiced_ && f0_raw > 0.0f) {
        if (pitch_hist_count_ < 3u) {
            pitch_history_[pitch_hist_count_++] = f0_raw;
        } else {
            pitch_history_[0] = pitch_history_[1];
            pitch_history_[1] = pitch_history_[2];
            pitch_history_[2] = f0_raw;
        }
        if (pitch_hist_count_ == 1u) {
            f0_smooth = pitch_history_[0];
        } else if (pitch_hist_count_ == 2u) {
            f0_smooth = 0.5f * (pitch_history_[0] + pitch_history_[1]);
        } else {
            std::array<float, 3> tmp = pitch_history_;
            std::sort(tmp.begin(), tmp.end());
            f0_smooth = tmp[1];
        }
    }

    // 4. Sub-bass ducking gain (< 45 Hz)
    const float duck_gain = compute_sub_duck_gain();

    // 5. Envelope tracking & RecursiveNCO synthesis at 8 kHz
    const float f_sub = 0.5f * f0_smooth;
    env_follower_.process_block(work_in_, env_buf_, count);
    nco_.synthesize_block(f_sub, is_voiced_, nco_buf_, count);

    if (active_cfg_.apply_dry_hpf && active_cfg_.dry_gain != 0.0f) {
        dry_hpf_.process_block(work_in_, dry_buf_, count);
    } else {
        std::copy(work_in_, work_in_ + count, dry_buf_);
    }

    const float g_dry1 = active_cfg_.dry_gain;
    const float g_sub1 = active_cfg_.sub_gain * duck_gain;
    const float g_sub0 = prev_sub_gain_ * duck_gain;
    const float inv_n = (count > 0u) ? (1.0f / static_cast<float>(count)) : 0.0f;
    const float step_dry = (g_dry1 - prev_dry_gain_) * inv_n;
    const float step_sub = (g_sub1 - g_sub0) * inv_n;
    float g_dry = prev_dry_gain_;
    float g_sub = g_sub0;
    for (std::size_t i = 0; i < count; ++i) {
        g_dry += step_dry;
        g_sub += step_sub;
        out_8k[i] = g_dry * dry_buf_[i] + g_sub * (nco_buf_[i] * env_buf_[i]);
    }
    prev_dry_gain_ = g_dry1;
    prev_sub_gain_ = active_cfg_.sub_gain;

    last_diag_.f0_hz = f0_smooth;
    last_diag_.f_sub_hz = f_sub;
    last_diag_.nccf_peak = r_peak;
    last_diag_.duck_gain = duck_gain;
    last_diag_.voiced = is_voiced_;
}

/**
 * @brief Process a block of 8 kHz 16-bit PCM samples through the NCCF subharmonic synthesizer.
 *
 * @param in_pcm16_8k Input 8 kHz 16-bit PCM mono buffer [int16 PCM].
 * @param out_pcm16_8k Output 8 kHz 16-bit PCM mono buffer [int16 PCM].
 * @param n_samples Number of 8 kHz samples to process (default 80) [samples].
 */
void SubharmonicNccfSynth::process_block_pcm16(
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

/**
 * @brief Initialize the unified SubharmonicPipeline (shared pre-stage + selectable 8 kHz algorithm).
 *
 * @param mode Initial active mode (kBypass, kZcFlipper, or kNccfSynth) [enum].
 * @param pre_lpf_cutoff_hz Shared 4th-order LP filter cutoff in [70.0, 200.0] Hz [Hz].
 * @param zc_cfg Configuration for Prototype 1 (Zero-Crossing Flipper) [struct].
 * @param nccf_cfg Configuration for Prototype 2 (NCCF Synthesizer) [struct].
 */
void SubharmonicPipeline::init(
    SubharmonicMode mode,
    float pre_lpf_cutoff_hz,
    const SubharmonicZcFlipperConfig& zc_cfg,
    const SubharmonicNccfConfig& nccf_cfg) noexcept {
    mode_.store(static_cast<uint8_t>(mode), std::memory_order_release);
    SubharmonicPreStageConfig pre_cfg{};
    pre_cfg.lpf_cutoff_hz = pre_lpf_cutoff_hz;
    pre_stage_.init(pre_cfg);
    zc_flipper_.init(zc_cfg, kSubSampleRateHz);
    nccf_synth_.init(nccf_cfg, kSubSampleRateHz);
}

void SubharmonicPipeline::set_mode(SubharmonicMode mode) noexcept {
    mode_.store(static_cast<uint8_t>(mode), std::memory_order_release);
}

void SubharmonicPipeline::reset() noexcept {
    pre_stage_.reset();
    zc_flipper_.reset();
    nccf_synth_.reset();
}

/**
 * @brief Process 48 kHz deinterleaved stereo float32 (480 frames) -> 8 kHz mono subwoofer float32 (80 samples).
 *
 * Executes the shared 48 kHz -> 8 kHz downsampling pre-stage and runs ONLY the currently enabled
 * subharmonic algorithm at 8 kHz.
 *
 * @param left_48k Input Left buffer at 48 kHz (480 samples) [normalized amplitude].
 * @param right_48k Input Right buffer at 48 kHz (480 samples) [normalized amplitude].
 * @param out_sub_8k Output mono subwoofer buffer at 8 kHz (80 samples) [normalized amplitude].
 * @param n_frames_48k Number of 48 kHz input stereo frames (default 480) [samples].
 * @return std::size_t Number of 8 kHz output samples produced (80) [samples].
 */
std::size_t SubharmonicPipeline::process_stereo_f32_to_8k(
    const float* DSP_RESTRICT left_48k,
    const float* DSP_RESTRICT right_48k,
    float* DSP_RESTRICT out_sub_8k,
    std::size_t n_frames_48k) noexcept {
    const std::size_t n_sub = pre_stage_.process_stereo_f32_to_8k(left_48k, right_48k, pre_out_8k_, n_frames_48k);
    const float dt_sec = static_cast<float>(n_frames_48k) / static_cast<float>(kSampleRateHz);
    const SubharmonicMode cur_mode = static_cast<SubharmonicMode>(mode_.load(std::memory_order_acquire));

    if (cur_mode == SubharmonicMode::kZcFlipper) {
        zc_flipper_.process_block_f32(pre_out_8k_, out_sub_8k, n_sub);
        nccf_synth_.sync_config_if_needed(dt_sec);
    } else if (cur_mode == SubharmonicMode::kNccfSynth) {
        zc_flipper_.sync_config_if_needed(dt_sec);
        nccf_synth_.process_block_f32(pre_out_8k_, out_sub_8k, n_sub);
    } else {
        zc_flipper_.sync_config_if_needed(dt_sec);
        nccf_synth_.sync_config_if_needed(dt_sec);
        std::copy(pre_out_8k_, pre_out_8k_ + n_sub, out_sub_8k);
    }
    return n_sub;
}

/**
 * @brief Process 48 kHz deinterleaved stereo int16 PCM (480 frames) -> 8 kHz mono subwoofer int16 PCM (80 samples).
 *
 * @param left_pcm16_48k Input Left 16-bit PCM buffer at 48 kHz (480 samples) [int16 PCM].
 * @param right_pcm16_48k Input Right 16-bit PCM buffer at 48 kHz (480 samples) [int16 PCM].
 * @param out_sub_pcm16_8k Output mono subwoofer 16-bit PCM buffer at 8 kHz (80 samples) [int16 PCM].
 * @param n_frames_48k Number of 48 kHz input stereo frames (default 480) [samples].
 * @return std::size_t Number of 8 kHz output samples produced (80) [samples].
 */
std::size_t SubharmonicPipeline::process_stereo_pcm16_to_8k(
    const int16_t* DSP_RESTRICT left_pcm16_48k,
    const int16_t* DSP_RESTRICT right_pcm16_48k,
    int16_t* DSP_RESTRICT out_sub_pcm16_8k,
    std::size_t n_frames_48k) noexcept {
    const std::size_t n_sub = pre_stage_.process_stereo_pcm16_to_8k(
        left_pcm16_48k, right_pcm16_48k, pre_out_8k_, n_frames_48k);
    const float dt_sec = static_cast<float>(n_frames_48k) / static_cast<float>(kSampleRateHz);
    const SubharmonicMode cur_mode = static_cast<SubharmonicMode>(mode_.load(std::memory_order_acquire));

    if (cur_mode == SubharmonicMode::kZcFlipper) {
        zc_flipper_.process_block_f32(pre_out_8k_, sub_out_8k_, n_sub);
        nccf_synth_.sync_config_if_needed(dt_sec);
        float_to_pcm16_block(sub_out_8k_, out_sub_pcm16_8k, n_sub);
    } else if (cur_mode == SubharmonicMode::kNccfSynth) {
        zc_flipper_.sync_config_if_needed(dt_sec);
        nccf_synth_.process_block_f32(pre_out_8k_, sub_out_8k_, n_sub);
        float_to_pcm16_block(sub_out_8k_, out_sub_pcm16_8k, n_sub);
    } else {
        zc_flipper_.sync_config_if_needed(dt_sec);
        nccf_synth_.sync_config_if_needed(dt_sec);
        float_to_pcm16_block(pre_out_8k_, out_sub_pcm16_8k, n_sub);
    }
    return n_sub;
}

}  // namespace audio_dsp
