#include "hilbert_quadrature_ms.hpp"

#include <algorithm>
#include <cmath>

namespace audio_dsp {

HilbertQuadratureMs::HilbertQuadratureMs(const HilbertQuadratureConfig& init_cfg) noexcept
    : hilbert_side_(init_cfg.fs, init_cfg.hilbert_f_min_hz),
      hilbert_mid_(init_cfg.fs, init_cfg.hilbert_f_min_hz) {
    init(init_cfg, nullptr, kMinQuadratureChannels);
}

/**
 * @brief Validate and clamp a single channel's spatial parameters to the allowed ranges.
 *
 * Allowed ranges:
 *   - dqs_deg: [-360.0f, +360.0f] deg
 *   - dqs_lfo_hz: [-5.0f, +5.0f] Hz
 *   - dqs_lfo_span_deg: [-360.0f, +360.0f] deg
 *   - dqs_lfo_offset_deg: [-360.0f, +360.0f] deg
 *   - dfs_shift_hz: [-20.0f, +20.0f] Hz
 *   - wet: [0.0f, 1.0f]
 *   - side_gain: [0.0f, 2.0f]
 *
 * @param p Input per-channel parameter structure [struct].
 * @return QuadratureChannelParams Clamped and sanitized parameter structure [struct].
 */
QuadratureChannelParams HilbertQuadratureMs::sanitize_channel_params(
    const QuadratureChannelParams& p) noexcept {
    QuadratureChannelParams out = p;
    out.dqs_deg = std::clamp(p.dqs_deg, kMinDqsDeg, kMaxDqsDeg);
    out.dqs_lfo_hz = std::clamp(p.dqs_lfo_hz, kMinLfoHz, kMaxLfoHz);
    out.dqs_lfo_span_deg = std::clamp(p.dqs_lfo_span_deg, kMinLfoSpanDeg, kMaxLfoSpanDeg);
    out.dqs_lfo_offset_deg = std::clamp(p.dqs_lfo_offset_deg, kMinLfoOffsetDeg, kMaxLfoOffsetDeg);
    out.dfs_shift_hz = std::clamp(p.dfs_shift_hz, kMinDfsShiftHz, kMaxDfsShiftHz);
    out.wet = std::clamp(p.wet, kMinWet, kMaxWet);
    out.side_gain = std::clamp(p.side_gain, kMinSideGain, kMaxSideGain);
    return out;
}

/**
 * @brief Initialize the spatial processor, configure the input HPF and Hilbert cascade, and reset states.
 *
 * @param init_cfg Initialization configuration (sample rate, HPF order/cutoff/Q, Hilbert f_min) [struct].
 * @param channels Initial per-channel parameter array (if nullptr, defaults to stereo -90/+90 deg) [pointer].
 * @param num_channels Number of active output channels (clamped to [1, kMaxQuadratureChannels]) [count].
 */
void HilbertQuadratureMs::init(
    const HilbertQuadratureConfig& init_cfg,
    const QuadratureChannelParams* channels,
    std::size_t num_channels) noexcept {
    init_cfg_ = init_cfg;
    init_cfg_.fs = (init_cfg.fs > 0) ? init_cfg.fs : kSampleRateHz;
    if (init_cfg.hpf_order <= 0) {
        init_cfg_.hpf_order = 0;
    } else {
        init_cfg_.hpf_order = std::clamp(init_cfg.hpf_order, 1, 4);
    }
    init_cfg_.hpf_cutoff_hz = std::clamp(init_cfg.hpf_cutoff_hz, kMinHpfCutoffHz, kMaxHpfCutoffHz);
    init_cfg_.hpf_q = (init_cfg.hpf_q > 0.05f) ? init_cfg.hpf_q : kDefaultQ;
    pending_hpf_cutoff_hz_.store(init_cfg_.hpf_cutoff_hz, std::memory_order_release);
    pending_hpf_order_.store(init_cfg_.hpf_order, std::memory_order_release);

    if (init_cfg_.angle_smooth_ms <= 0.0f) {
        angle_smooth_coeff_ = 1.0f;
    } else {
        const float tau = init_cfg_.angle_smooth_ms * 1.0e-3f;
        angle_smooth_coeff_ = 1.0f - std::exp(-1.0f / (tau * static_cast<float>(init_cfg_.fs)));
    }

    hpf_left_.configure_hpf(init_cfg_.hpf_order, init_cfg_.hpf_cutoff_hz, init_cfg_.fs, init_cfg_.hpf_q);
    hpf_right_.configure_hpf(init_cfg_.hpf_order, init_cfg_.hpf_cutoff_hz, init_cfg_.fs, init_cfg_.hpf_q);
    hilbert_side_.configure(init_cfg_.fs, init_cfg_.hilbert_f_min_hz);
    hilbert_mid_.configure(init_cfg_.fs, init_cfg_.hilbert_f_min_hz);

    QuadratureChannelParams default_stereo[kMaxQuadratureChannels]{};
    default_stereo[0].dqs_deg = -90.0f;
    default_stereo[1].dqs_deg = +90.0f;
    for (std::size_t c = 2; c < kMaxQuadratureChannels; ++c) {
        default_stereo[c].dqs_deg = 0.0f;
    }

    const std::size_t count = std::clamp<std::size_t>(num_channels, 1u, kMaxQuadratureChannels);
    const QuadratureChannelParams* src = (channels != nullptr) ? channels : default_stereo;

    staged_snapshot_.num_channels = count;
    staged_snapshot_.reset_phases = true;
    num_channels_ = count;

    for (std::size_t c = 0; c < kMaxQuadratureChannels; ++c) {
        const QuadratureChannelParams p = (c < count) ? sanitize_channel_params(src[c]) : default_stereo[c];
        staged_snapshot_.params[c] = p;
        active_params_[c] = p;
        update_channel_coefficients(c, p, p.dqs_deg, p.side_gain, true, true);
    }

    applied_gen_ = pending_config_.set(staged_snapshot_);
    reset();
}

/**
 * @brief Configure all active output channels (thread-safe from a non-real-time control task).
 *
 * @param channels Array of per-channel parameter structs [pointer].
 * @param num_channels Number of output channels to activate (clamped to [1, kMaxQuadratureChannels]) [count].
 * @param reset_phases If true, resets each channel's LFO and DFS phase accumulators to 0 [boolean].
 */
void HilbertQuadratureMs::set_channels(
    const QuadratureChannelParams* channels,
    std::size_t num_channels,
    bool reset_phases) noexcept {
    if (channels == nullptr) {
        return;
    }
    const std::size_t count = std::clamp<std::size_t>(num_channels, 1u, kMaxQuadratureChannels);
    staged_snapshot_.num_channels = count;
    staged_snapshot_.reset_phases = reset_phases;
    for (std::size_t c = 0; c < count; ++c) {
        staged_snapshot_.params[c] = sanitize_channel_params(channels[c]);
    }
    pending_config_.set(staged_snapshot_);
}

/**
 * @brief Update a single output channel's spatial parameters at runtime (thread-safe).
 *
 * @param channel_idx Output channel index in [0, kMaxQuadratureChannels - 1] [index].
 * @param params New spatial audio parameters for that speaker channel [struct].
 */
void HilbertQuadratureMs::set_channel_params(
    std::size_t channel_idx,
    const QuadratureChannelParams& params) noexcept {
    if (channel_idx >= kMaxQuadratureChannels) {
        return;
    }
    staged_snapshot_.params[channel_idx] = sanitize_channel_params(params);
    staged_snapshot_.reset_phases = false;
    pending_config_.set(staged_snapshot_);
}

/**
 * @brief Reset all filter delay lines and per-channel LFO/DFS phase accumulators to zero.
 */
void HilbertQuadratureMs::reset() noexcept {
    hpf_left_.reset();
    hpf_right_.reset();
    hilbert_side_.reset();
    hilbert_mid_.reset();
    for (std::size_t c = 0; c < kMaxQuadratureChannels; ++c) {
        states_[c].lfo_acc = 0u;
        states_[c].shift_acc = 0u;
        states_[c].current_dqs_u32 = states_[c].target_dqs_u32;
    }
}

/**
 * @brief Convert sanitized QuadratureChannelParams into uint32_t increments and gains for channel ch.
 *
 * @param ch Channel index in [0, kMaxQuadratureChannels - 1] [index].
 * @param p Sanitized channel parameters [struct].
 * @param dry_azimuth_deg Azimuth of the static dry Side reference in degrees [deg].
 * @param dry_gain Linear gain of the static dry Side reference [dimensionless].
 * @param snap_angle If true, immediately sets current_dqs_u32 = target_dqs_u32 [boolean].
 * @param reset_phase If true, resets lfo_acc and shift_acc to 0 [boolean].
 */
void HilbertQuadratureMs::update_channel_coefficients(
    std::size_t ch,
    const QuadratureChannelParams& p,
    float dry_azimuth_deg,
    float dry_gain,
    bool snap_angle,
    bool reset_phase) noexcept {
    ChannelState& st = states_[ch];
    const uint32_t target_u32 = FastSinLUT::deg_to_u32(p.dqs_deg);
    if (snap_angle) {
        st.current_dqs_u32 = target_u32;
        st.prev_wet = p.wet;
        st.prev_side_gain = p.side_gain;
    } else {
        st.current_dqs_u32 = st.target_dqs_u32;
        st.prev_wet = st.wet;
        st.prev_side_gain = st.side_gain;
    }
    st.target_dqs_u32 = target_u32;
    st.dry_dqs_u32 = FastSinLUT::deg_to_u32(dry_azimuth_deg);

    st.has_lfo = (p.dqs_lfo_hz != 0.0f) && (p.dqs_lfo_span_deg != 0.0f);
    st.lfo_inc_u32 = st.has_lfo ? FastSinLUT::hz_to_inc_u32(p.dqs_lfo_hz, init_cfg_.fs) : 0u;
    st.lfo_offset_u32 = FastSinLUT::deg_to_u32(p.dqs_lfo_offset_deg);

    // Scale half-span turns into S30 fixed-point (2^30 = 1073741824.0f) so hardware
    // float-to-int32 conversion (trunc.s) never overflows INT32_MAX at +/-360 deg span.
    st.lfo_half_span_s30 = (p.dqs_lfo_span_deg * 0.5f / 360.0f) * 1073741824.0f;

    st.has_dfs = (p.dfs_shift_hz != 0.0f);
    // Rotation angle subtracts 2*pi*dfs_shift_hz*t, so increment is for -dfs_shift_hz
    st.dfs_inc_u32 = st.has_dfs ? FastSinLUT::hz_to_inc_u32(-p.dfs_shift_hz, init_cfg_.fs) : 0u;

    st.wet = p.wet;
    st.side_gain = p.side_gain;
    st.dry_side_gain = dry_gain;

    if (reset_phase) {
        st.lfo_acc = 0u;
        st.shift_acc = 0u;
    }
}

void HilbertQuadratureMs::sync_pending_channels_if_needed(float dt_sec) noexcept {
    uint32_t gen = 0u;
    const PendingChannelSnapshot snap = pending_config_.get(&gen);
    const bool new_gen = (gen != applied_gen_);
    if (new_gen) {
        applied_gen_ = gen;
        num_channels_ = std::clamp<std::size_t>(snap.num_channels, 1u, kMaxQuadratureChannels);
    }

    for (std::size_t c = 0; c < num_channels_; ++c) {
        const QuadratureChannelParams& tp = snap.params[c];
        QuadratureChannelParams& ap = active_params_[c];
        if (new_gen && snap.reset_phases) {
            ap = tp;
            update_channel_coefficients(c, ap, ap.dqs_deg, ap.side_gain, true, true);
            continue;
        }
        const bool needs_slew =
            (ap.dqs_deg != tp.dqs_deg) ||
            (ap.dqs_lfo_hz != tp.dqs_lfo_hz) ||
            (ap.dqs_lfo_span_deg != tp.dqs_lfo_span_deg) ||
            (ap.dqs_lfo_offset_deg != tp.dqs_lfo_offset_deg) ||
            (ap.dfs_shift_hz != tp.dfs_shift_hz) ||
            (ap.wet != tp.wet) ||
            (ap.side_gain != tp.side_gain);
        if (needs_slew) {
            ap.dqs_deg = slew_towards(ap.dqs_deg, tp.dqs_deg, kMinDqsDeg, kMaxDqsDeg, dt_sec);
            ap.dqs_lfo_hz = slew_towards(ap.dqs_lfo_hz, tp.dqs_lfo_hz, kMinLfoHz, kMaxLfoHz, dt_sec);
            ap.dqs_lfo_span_deg = slew_towards(ap.dqs_lfo_span_deg, tp.dqs_lfo_span_deg, kMinLfoSpanDeg, kMaxLfoSpanDeg, dt_sec);
            ap.dqs_lfo_offset_deg = slew_towards(ap.dqs_lfo_offset_deg, tp.dqs_lfo_offset_deg, kMinLfoOffsetDeg, kMaxLfoOffsetDeg, dt_sec);
            ap.dfs_shift_hz = slew_towards(ap.dfs_shift_hz, tp.dfs_shift_hz, kMinDfsShiftHz, kMaxDfsShiftHz, dt_sec);
            ap.wet = slew_towards(ap.wet, tp.wet, kMinWet, kMaxWet, dt_sec);
            ap.side_gain = slew_towards(ap.side_gain, tp.side_gain, kMinSideGain, kMaxSideGain, dt_sec);
            update_channel_coefficients(c, ap, ap.dqs_deg, ap.side_gain, false, false);
        }
    }

    const float req_hpf = pending_hpf_cutoff_hz_.load(std::memory_order_acquire);
    if (req_hpf != init_cfg_.hpf_cutoff_hz) {
        init_cfg_.hpf_cutoff_hz = slew_towards(
            init_cfg_.hpf_cutoff_hz, req_hpf, kMinHpfCutoffHz, kMaxHpfCutoffHz, dt_sec);
        if (init_cfg_.hpf_order > 0) {
            const float l_z0_1 = hpf_left_.section(0).z1;
            const float l_z0_2 = hpf_left_.section(0).z2;
            const float l_z1_1 = hpf_left_.section(1).z1;
            const float l_z1_2 = hpf_left_.section(1).z2;
            const float r_z0_1 = hpf_right_.section(0).z1;
            const float r_z0_2 = hpf_right_.section(0).z2;
            const float r_z1_1 = hpf_right_.section(1).z1;
            const float r_z1_2 = hpf_right_.section(1).z2;
            hpf_left_.configure_hpf(init_cfg_.hpf_order, init_cfg_.hpf_cutoff_hz, init_cfg_.fs, init_cfg_.hpf_q);
            hpf_right_.configure_hpf(init_cfg_.hpf_order, init_cfg_.hpf_cutoff_hz, init_cfg_.fs, init_cfg_.hpf_q);
            hpf_left_.section(0).z1 = l_z0_1;
            hpf_left_.section(0).z2 = l_z0_2;
            hpf_left_.section(1).z1 = l_z1_1;
            hpf_left_.section(1).z2 = l_z1_2;
            hpf_right_.section(0).z1 = r_z0_1;
            hpf_right_.section(0).z2 = r_z0_2;
            hpf_right_.section(1).z1 = r_z1_1;
            hpf_right_.section(1).z2 = r_z1_2;
        }
    }

    // Order change: new section structure, so the old filter state does not apply (cleared)
    const int req_order = pending_hpf_order_.load(std::memory_order_acquire);
    if (req_order != init_cfg_.hpf_order) {
        init_cfg_.hpf_order = req_order;
        hpf_left_.configure_hpf(req_order, init_cfg_.hpf_cutoff_hz, init_cfg_.fs, init_cfg_.hpf_q);
        hpf_right_.configure_hpf(req_order, init_cfg_.hpf_cutoff_hz, init_cfg_.fs, init_cfg_.hpf_q);
        hpf_left_.reset();
        hpf_right_.reset();
    }
}

/**
 * @brief Generic templated per-channel rendering kernel reading shared (d_M, q_M, d_S, q_S) arrays.
 *
 * Dispatches to one of 3 specialized branchless kernels:
 *   1. Static Surround / Pure DQS ((!has_lfo && !has_dfs) || wet == 0.0f)
 *   2. DQS + LFO, No DFS (has_lfo && !has_dfs && wet > 0.0f)
 *   3. Full DQS + LFO + DFS (has_dfs && wet > 0.0f)
 *
 * @param st Isolated per-channel oscillator and gain state [struct].
 * @param n_samples Number of samples in the block [samples].
 * @param write_sample Functor invoked as write_sample(i, y_f32) for each sample i [callable].
 */
template <typename SampleWriter>
void HilbertQuadratureMs::render_channel_impl(
    ChannelState& st,
    std::size_t n_samples,
    SampleWriter&& write_sample) noexcept {
    if (bypass_active_) {
        // In bypass mode, d_m_ holds HPF(Left) and d_s_ holds HPF(Right).
        // Map speaker azimuth via cos/sin weights around front (0 deg = Mid, -90 deg = Left, +90 deg = Right).
        float sin_d = 0.0f, cos_d = 1.0f;
        FastSinLUT::sin_cos_u32(st.target_dqs_u32 + kAzimuthOffsetU32, sin_d, cos_d);
        const float w_l = 0.5f * (1.0f + cos_d);
        const float w_r = 0.5f * (1.0f - cos_d);
        for (std::size_t i = 0; i < n_samples; ++i) {
            write_sample(i, w_l * d_m_[i] + w_r * d_s_[i]);
        }
        st.lfo_acc += static_cast<uint32_t>(n_samples) * st.lfo_inc_u32;
        st.shift_acc += static_cast<uint32_t>(n_samples) * st.dfs_inc_u32;
        st.current_dqs_u32 = st.target_dqs_u32;
        return;
    }

    const float wet = st.wet;
    const float side_gain = st.side_gain;
    const float dry_gain = st.dry_side_gain;
    const bool settled = (st.current_dqs_u32 == st.target_dqs_u32) &&
                         (st.prev_wet == wet) &&
                         (st.prev_side_gain == side_gain);

    // Path 1: Static surround or wet == 0 (no per-sample trig needed)
    if ((!st.has_lfo && !st.has_dfs) || wet <= 0.0f) {
        float sin_d = 0.0f, cos_d = 1.0f;
        float sin_t1 = 0.0f, cos_t1 = 1.0f;
        FastSinLUT::sin_cos_u32(st.dry_dqs_u32 + kAzimuthOffsetU32, sin_d, cos_d);
        FastSinLUT::sin_cos_u32(st.target_dqs_u32 + kAzimuthOffsetU32, sin_t1, cos_t1);

        const float k_d1 = (1.0f - wet) * dry_gain * cos_d + wet * side_gain * cos_t1;
        const float k_q1 = (1.0f - wet) * dry_gain * sin_d + wet * side_gain * sin_t1;

        if (settled || n_samples == 0u) {
            for (std::size_t i = 0; i < n_samples; ++i) {
                write_sample(i, d_m_[i] + k_d1 * d_s_[i] + k_q1 * q_s_[i]);
            }
        } else {
            float sin_t0 = 0.0f, cos_t0 = 1.0f;
            FastSinLUT::sin_cos_u32(st.current_dqs_u32 + kAzimuthOffsetU32, sin_t0, cos_t0);
            const float w0 = st.prev_wet;
            const float sg0 = st.prev_side_gain;
            const bool track_dry = (st.dry_dqs_u32 == st.target_dqs_u32);
            const float sin_d0 = track_dry ? sin_t0 : sin_d;
            const float cos_d0 = track_dry ? cos_t0 : cos_d;
            const float dg0 = track_dry ? sg0 : dry_gain;

            const float k_d0 = (1.0f - w0) * dg0 * cos_d0 + w0 * sg0 * cos_t0;
            const float k_q0 = (1.0f - w0) * dg0 * sin_d0 + w0 * sg0 * sin_t0;
            const float inv_n = 1.0f / static_cast<float>(n_samples);
            const float step_kd = (k_d1 - k_d0) * inv_n;
            const float step_kq = (k_q1 - k_q0) * inv_n;
            float k_d = k_d0;
            float k_q = k_q0;

            for (std::size_t i = 0; i < n_samples; ++i) {
                k_d += step_kd;
                k_q += step_kq;
                write_sample(i, d_m_[i] + k_d * d_s_[i] + k_q * q_s_[i]);
            }
            st.current_dqs_u32 = st.target_dqs_u32;
            st.prev_wet = wet;
            st.prev_side_gain = side_gain;
        }

        // Advance background oscillators even if wet == 0 so phase remains continuous
        st.lfo_acc += static_cast<uint32_t>(n_samples) * st.lfo_inc_u32;
        st.shift_acc += static_cast<uint32_t>(n_samples) * st.dfs_inc_u32;
        return;
    }

    // Precompute static dry Side weights
    float sin_d = 0.0f, cos_d = 1.0f;
    FastSinLUT::sin_cos_u32(st.dry_dqs_u32 + kAzimuthOffsetU32, sin_d, cos_d);
    const float dc_dry = (1.0f - wet) * dry_gain * cos_d;
    const float qs_dry = (1.0f - wet) * dry_gain * sin_d;
    const float g_wet_side = wet * side_gain;

    uint32_t lfo_acc = st.lfo_acc;
    const uint32_t lfo_inc = st.lfo_inc_u32;
    const uint32_t lfo_off = st.lfo_offset_u32;
    const float lfo_span_s30 = st.lfo_half_span_s30;
    const uint32_t base_theta = st.current_dqs_u32 + kAzimuthOffsetU32;

    // Path 2: DQS + LFO active, no DFS (Mid is simply d_m_[i])
    // Evaluate slow LFO trig weights at 6 kHz control rate (every 8 samples) with linear interpolation.
    constexpr std::size_t kCtrlSubStep = 8u;
    constexpr float kInvCtrlSubStep = 1.0f / static_cast<float>(kCtrlSubStep);

    if (!st.has_dfs) {
        auto eval_lfo_weights = [&](uint32_t acc, float& kd_out, float& kq_out) noexcept {
            const float lfo_sin = FastSinLUT::sin_u32(acc + lfo_off);
            const uint32_t lfo_u32 =
                static_cast<uint32_t>(static_cast<int32_t>(lfo_sin * lfo_span_s30)) << 2;
            float sin_t = 0.0f, cos_t = 1.0f;
            FastSinLUT::sin_cos_u32(base_theta + lfo_u32, sin_t, cos_t);
            kd_out = dc_dry + g_wet_side * cos_t;
            kq_out = qs_dry + g_wet_side * sin_t;
        };

        float k_d = 0.0f, k_q = 0.0f;
        eval_lfo_weights(lfo_acc, k_d, k_q);

        std::size_t i = 0;
        for (; i + kCtrlSubStep <= n_samples; i += kCtrlSubStep) {
            lfo_acc += static_cast<uint32_t>(kCtrlSubStep) * lfo_inc;
            float k_d_next = 0.0f, k_q_next = 0.0f;
            eval_lfo_weights(lfo_acc, k_d_next, k_q_next);
            const float step_kd = (k_d_next - k_d) * kInvCtrlSubStep;
            const float step_kq = (k_q_next - k_q) * kInvCtrlSubStep;
            for (std::size_t j = 0; j < kCtrlSubStep; ++j) {
                const std::size_t idx = i + j;
                write_sample(idx, d_m_[idx] + k_d * d_s_[idx] + k_q * q_s_[idx]);
                k_d += step_kd;
                k_q += step_kq;
            }
            k_d = k_d_next;
            k_q = k_q_next;
        }
        for (; i < n_samples; ++i) {
            eval_lfo_weights(lfo_acc, k_d, k_q);
            lfo_acc += lfo_inc;
            write_sample(i, d_m_[i] + k_d * d_s_[i] + k_q * q_s_[i]);
        }
        st.lfo_acc = lfo_acc;
        st.current_dqs_u32 = st.target_dqs_u32;
        st.prev_wet = wet;
        st.prev_side_gain = side_gain;
        return;
    }

    // Path 3: Full DQS + optional LFO + DFS (both Mid and Side shifted)
    // Evaluate slow LFO + DFS trig weights at 6 kHz control rate (every 8 samples) with linear interpolation.
    uint32_t shift_acc = st.shift_acc;
    const uint32_t dfs_inc = st.dfs_inc_u32;
    const float g_dry_mid = 1.0f - wet;
    const float g_wet_mid = wet;
    const bool has_lfo = st.has_lfo;

    auto eval_dfs_weights = [&](uint32_t l_acc, uint32_t s_acc,
                                float& md_out, float& mq_out,
                                float& kd_out, float& kq_out) noexcept {
        uint32_t lfo_u32 = 0u;
        if (has_lfo) {
            const float lfo_sin = FastSinLUT::sin_u32(l_acc + lfo_off);
            lfo_u32 = static_cast<uint32_t>(static_cast<int32_t>(lfo_sin * lfo_span_s30)) << 2;
        }
        const uint32_t theta_u32 = base_theta + lfo_u32 + s_acc;
        float sin_t = 0.0f, cos_t = 1.0f;
        float sin_m = 0.0f, cos_m = 1.0f;
        FastSinLUT::sin_cos_u32(theta_u32, sin_t, cos_t);
        FastSinLUT::sin_cos_u32(s_acc, sin_m, cos_m);
        md_out = g_dry_mid + g_wet_mid * cos_m;
        mq_out = g_wet_mid * sin_m;
        kd_out = dc_dry + g_wet_side * cos_t;
        kq_out = qs_dry + g_wet_side * sin_t;
    };

    float m_d = 0.0f, m_q = 0.0f, k_d = 0.0f, k_q = 0.0f;
    eval_dfs_weights(lfo_acc, shift_acc, m_d, m_q, k_d, k_q);

    std::size_t i = 0;
    for (; i + kCtrlSubStep <= n_samples; i += kCtrlSubStep) {
        lfo_acc += static_cast<uint32_t>(kCtrlSubStep) * lfo_inc;
        shift_acc += static_cast<uint32_t>(kCtrlSubStep) * dfs_inc;
        float m_d_next = 0.0f, m_q_next = 0.0f, k_d_next = 0.0f, k_q_next = 0.0f;
        eval_dfs_weights(lfo_acc, shift_acc, m_d_next, m_q_next, k_d_next, k_q_next);
        const float step_md = (m_d_next - m_d) * kInvCtrlSubStep;
        const float step_mq = (m_q_next - m_q) * kInvCtrlSubStep;
        const float step_kd = (k_d_next - k_d) * kInvCtrlSubStep;
        const float step_kq = (k_q_next - k_q) * kInvCtrlSubStep;
        for (std::size_t j = 0; j < kCtrlSubStep; ++j) {
            const std::size_t idx = i + j;
            const float m_out = d_m_[idx] * m_d + q_m_[idx] * m_q;
            const float s_out = d_s_[idx] * k_d + q_s_[idx] * k_q;
            write_sample(idx, m_out + s_out);
            m_d += step_md;
            m_q += step_mq;
            k_d += step_kd;
            k_q += step_kq;
        }
        m_d = m_d_next;
        m_q = m_q_next;
        k_d = k_d_next;
        k_q = k_q_next;
    }
    for (; i < n_samples; ++i) {
        eval_dfs_weights(lfo_acc, shift_acc, m_d, m_q, k_d, k_q);
        lfo_acc += lfo_inc;
        shift_acc += dfs_inc;
        const float m_out = d_m_[i] * m_d + q_m_[i] * m_q;
        const float s_out = d_s_[i] * k_d + q_s_[i] * k_q;
        write_sample(i, m_out + s_out);
    }

    st.lfo_acc = lfo_acc;
    st.shift_acc = shift_acc;
    st.current_dqs_u32 = st.target_dqs_u32;
    st.prev_wet = wet;
    st.prev_side_gain = side_gain;
}

/**
 * @brief Render a single speaker output channel from the shared (d_M, q_M, d_S, q_S) block buffers.
 *
 * @param st Isolated per-channel oscillator and gain state [struct].
 * @param out_ch Output buffer for this channel (length n_samples) [normalized amplitude].
 * @param n_samples Number of samples in the block [samples].
 */
void HilbertQuadratureMs::render_channel_block(
    ChannelState& st,
    float* DSP_RESTRICT out_ch,
    std::size_t n_samples) noexcept {
    render_channel_impl(st, n_samples, [out_ch](std::size_t i, float y) noexcept {
        out_ch[i] = y;
    });
}

/**
 * @brief Step 1 of split dual-core processing: run input HPF, M/S split, and shared Mid/Side
 *        Niemitalo Hilbert IIR all-pass cascades for a 48 kHz float32 stereo block.
 *
 * @param left_in Input Left channel float32 buffer (n_samples elements) [normalized amplitude].
 * @param right_in Input Right channel float32 buffer (n_samples elements) [normalized amplitude].
 * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
 */
void HilbertQuadratureMs::prepare_block_f32(
    const float* left_in,
    const float* right_in,
    std::size_t n_samples) noexcept {
    const std::size_t count = std::min(n_samples, kFrameSamples);
    const float dt_sec = static_cast<float>(count) / static_cast<float>((init_cfg_.fs > 0) ? init_cfg_.fs : kSampleRateHz);
    sync_pending_channels_if_needed(dt_sec);
    bypass_active_ = bypass_.load(std::memory_order_acquire);

    // 1. Input High-Pass Filter on Left and Right into d_m_ and d_s_
    if (init_cfg_.hpf_order > 0) {
        hpf_left_.process_block(left_in, d_m_, count);
        hpf_right_.process_block(right_in, d_s_, count);
    } else {
        if (left_in != d_m_) {
            std::copy(left_in, left_in + count, d_m_);
        }
        if (right_in != d_s_) {
            std::copy(right_in, right_in + count, d_s_);
        }
    }

    if (bypass_active_) {
        return;
    }

    // 2. Stereo to Mid/Side in-place on d_m_ and d_s_ with 0.5 headroom factor
    for (std::size_t i = 0; i < count; ++i) {
        const float l = d_m_[i];
        const float r = d_s_[i];
        d_m_[i] = 0.5f * (l + r);
        d_s_[i] = 0.5f * (l - r);
    }

    // 3. Shared Hilbert Transform on Side and Mid (in-place for in-phase branch)
    hilbert_side_.process_block(d_s_, d_s_, q_s_, count);

    bool any_dfs = false;
    for (std::size_t c = 0; c < num_channels_; ++c) {
        if (states_[c].has_dfs && states_[c].wet > 0.0f) {
            any_dfs = true;
            break;
        }
    }

    if (any_dfs) {
        hilbert_mid_.process_block(d_m_, d_m_, q_m_, count);
    } else {
        hilbert_mid_.match_in_phase_block(d_m_, d_m_, count);
    }
}

/**
 * @brief Step 2a of split dual-core processing: render one output channel into float32.
 *
 * @param ch Output channel index in [0, num_channels - 1] [index].
 * @param out_ch Output float32 buffer (n_samples elements) [normalized amplitude].
 * @param n_samples Number of samples to render (default 480, max 480) [samples].
 */
void HilbertQuadratureMs::render_channel_f32(
    std::size_t ch,
    float* DSP_RESTRICT out_ch,
    std::size_t n_samples) noexcept {
    if (ch >= num_channels_ || out_ch == nullptr) {
        return;
    }
    const std::size_t count = std::min(n_samples, kFrameSamples);
    render_channel_block(states_[ch], out_ch, count);
}

/**
 * @brief Step 2b of split dual-core processing: render one output channel directly to 16-bit PCM
 *        with fused linear gain ramping (gain_start -> gain_end) and saturation clamping.
 *
 * @param ch Output channel index in [0, num_channels - 1] [index].
 * @param out_pcm16 Output int16_t PCM buffer (n_samples elements) [int16 PCM].
 * @param gain_start Linear gain at the start of the frame (1.0f = 0 dB) [dimensionless].
 * @param gain_end Linear gain at the end of the frame (1.0f = 0 dB) [dimensionless].
 * @param n_samples Number of samples to render (default 480, max 480) [samples].
 */
void HilbertQuadratureMs::render_channel_to_pcm16(
    std::size_t ch,
    int16_t* DSP_RESTRICT out_pcm16,
    float gain_start,
    float gain_end,
    std::size_t n_samples) noexcept {
    if (ch >= num_channels_ || out_pcm16 == nullptr) {
        return;
    }
    const std::size_t count = std::min(n_samples, kFrameSamples);
    if (gain_start == 0.0f && gain_end == 0.0f) {
        std::fill(out_pcm16, out_pcm16 + count, int16_t{0});
        states_[ch].lfo_acc += static_cast<uint32_t>(count) * states_[ch].lfo_inc_u32;
        states_[ch].shift_acc += static_cast<uint32_t>(count) * states_[ch].dfs_inc_u32;
        states_[ch].current_dqs_u32 = states_[ch].target_dqs_u32;
        return;
    }
    if (gain_start == 1.0f && gain_end == 1.0f) {
        render_channel_impl(states_[ch], count, [out_pcm16](std::size_t i, float y) noexcept {
            const float scaled = y * 32768.0f;
            out_pcm16[i] = static_cast<int16_t>(std::clamp(scaled, -32768.0f, 32767.0f));
        });
    } else {
        const float g0 = gain_start * 32768.0f;
        const float g1 = gain_end * 32768.0f;
        const float step = (g1 - g0) / static_cast<float>(count);
        float g = g0;
        render_channel_impl(states_[ch], count, [out_pcm16, &g, step](std::size_t i, float y) noexcept {
            g += step;
            const float scaled = y * g;
            out_pcm16[i] = static_cast<int16_t>(std::clamp(scaled, -32768.0f, 32767.0f));
        });
    }
}

/**
 * @brief Process a 48 kHz float32 stereo block (480 samples) into num_channels output buffers.
 *
 * Computes the adjustable input HPF, M/S split, and Hilbert pairs ONCE per block, then renders
 * each output channel using its own isolated LFO and DFS phase accumulators.
 *
 * @param left_in Input Left channel float32 buffer (n_samples elements) [normalized amplitude].
 * @param right_in Input Right channel float32 buffer (n_samples elements) [normalized amplitude].
 * @param out_channels Array of num_channels pointers to output float32 buffers [normalized amplitude].
 * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
 */
void HilbertQuadratureMs::process_block_f32(
    const float* DSP_RESTRICT left_in,
    const float* DSP_RESTRICT right_in,
    float* const* out_channels,
    std::size_t n_samples) noexcept {
    const std::size_t count = std::min(n_samples, kFrameSamples);
    prepare_block_f32(left_in, right_in, count);
    for (std::size_t c = 0; c < num_channels_; ++c) {
        if (out_channels[c] != nullptr) {
            render_channel_block(states_[c], out_channels[c], count);
        }
    }
}

/**
 * @brief Process a 48 kHz deinterleaved 16-bit PCM stereo block (480 samples) into num_channels int16 buffers.
 *
 * @param left_pcm16 Input Left channel 16-bit PCM buffer [int16 PCM].
 * @param right_pcm16 Input Right channel 16-bit PCM buffer [int16 PCM].
 * @param out_channels_pcm16 Array of num_channels pointers to output int16_t buffers [int16 PCM].
 * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
 */
void HilbertQuadratureMs::process_block_pcm16(
    const int16_t* DSP_RESTRICT left_pcm16,
    const int16_t* DSP_RESTRICT right_pcm16,
    int16_t* const* out_channels_pcm16,
    std::size_t n_samples) noexcept {
    const std::size_t count = std::min(n_samples, kFrameSamples);
    pcm16_to_float_block(left_pcm16, d_m_, count);
    pcm16_to_float_block(right_pcm16, d_s_, count);
    prepare_block_f32(d_m_, d_s_, count);
    for (std::size_t c = 0; c < num_channels_; ++c) {
        if (out_channels_pcm16[c] != nullptr) {
            render_channel_to_pcm16(c, out_channels_pcm16[c], 1.0f, 1.0f, count);
        }
    }
}

/**
 * @brief Process a 48 kHz interleaved 16-bit PCM stereo block [L0, R0, L1, R1...] into num_channels int16 buffers.
 *
 * @param stereo_pcm16 Interleaved stereo 16-bit PCM buffer (2 * n_samples elements) [int16 PCM].
 * @param out_channels_pcm16 Array of num_channels pointers to output int16_t buffers [int16 PCM].
 * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
 */
void HilbertQuadratureMs::process_interleaved_pcm16(
    const int16_t* DSP_RESTRICT stereo_pcm16,
    int16_t* const* out_channels_pcm16,
    std::size_t n_samples) noexcept {
    const std::size_t count = std::min(n_samples, kFrameSamples);
    pcm16_stereo_to_float_lr(stereo_pcm16, d_m_, d_s_, count);
    prepare_block_f32(d_m_, d_s_, count);
    for (std::size_t c = 0; c < num_channels_; ++c) {
        if (out_channels_pcm16[c] != nullptr) {
            render_channel_to_pcm16(c, out_channels_pcm16[c], 1.0f, 1.0f, count);
        }
    }
}

/**
 * @brief Convenience stereo-pair wrapper matching process_dqs in Signal_processing/hilbert_helper.py.
 *
 * Places Left at dqs_deg - 90 deg (with dry reference at -90 deg, gain 1.0) and
 * Right at dqs_deg + 90 deg (with dry reference at +90 deg, gain 1.0).
 *
 * @param left_in Input Left channel float32 buffer [normalized amplitude].
 * @param right_in Input Right channel float32 buffer [normalized amplitude].
 * @param left_out Output Left channel float32 buffer [normalized amplitude].
 * @param right_out Output Right channel float32 buffer [normalized amplitude].
 * @param params Shared DQS stereo parameters (dqs_deg, wet, side_gain, dqs_lfo_hz, span, offset) [struct].
 * @param n_samples Number of stereo frames to process (default 480) [samples].
 */
void HilbertQuadratureMs::process_dqs_stereo_f32(
    const float* DSP_RESTRICT left_in,
    const float* DSP_RESTRICT right_in,
    float* DSP_RESTRICT left_out,
    float* DSP_RESTRICT right_out,
    const QuadratureChannelParams& params,
    std::size_t n_samples) noexcept {
    QuadratureChannelParams p_left = params;
    p_left.dqs_deg = params.dqs_deg - 90.0f;
    p_left.dfs_shift_hz = 0.0f;
    p_left = sanitize_channel_params(p_left);

    QuadratureChannelParams p_right = params;
    p_right.dqs_deg = params.dqs_deg + 90.0f;
    p_right.dfs_shift_hz = 0.0f;
    p_right = sanitize_channel_params(p_right);

    num_channels_ = 2u;
    update_channel_coefficients(0u, p_left, -90.0f, 1.0f, true, false);
    update_channel_coefficients(1u, p_right, +90.0f, 1.0f, true, false);
    applied_gen_ = pending_config_.generation();

    float* outs[2] = {left_out, right_out};
    process_block_f32(left_in, right_in, outs, n_samples);
}

}  // namespace audio_dsp

