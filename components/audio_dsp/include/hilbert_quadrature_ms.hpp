#ifndef AUDIO_DSP_HILBERT_QUADRATURE_MS_HPP_
#define AUDIO_DSP_HILBERT_QUADRATURE_MS_HPP_

#include "dsp_primitives.hpp"

namespace audio_dsp {

/**
 * @brief Init-time configuration for HilbertQuadratureMs (including the adjustable input HPF).
 */
struct HilbertQuadratureConfig {
    /// Audio sampling rate in Hertz (default 48000 Hz) [Hz].
    int fs = kSampleRateHz;

    /// Input High-Pass filter order in {1, 2, 3, 4} (or 0 to bypass HPF; default 2) [dimensionless].
    int hpf_order = 2;

    /// Input High-Pass filter -3 dB cutoff frequency in Hertz (range [50.0, 200.0] Hz; default 100.0) [Hz].
    float hpf_cutoff_hz = 100.0f;

    /// Input High-Pass filter quality factor Q for 2nd-order sections (default 0.70710678f) [dimensionless].
    float hpf_q = kDefaultQ;

    /// Passband lower limit for the 8-section Niemitalo Hilbert IIR allpass cascade in Hertz [Hz].
    float hilbert_f_min_hz = 100.0f;

    /// Optional shortest-arc smoothing time constant for live dqs_deg updates in milliseconds (0 = instant) [ms].
    float angle_smooth_ms = 0.0f;
};

/**
 * @brief Parameter set of ONE output channel (speaker) of HilbertQuadratureMs.
 *
 * Matches QuadratureParams in Signal_processing/hilbert_helper.py and hilbert_part3.ipynb:
 * - dqs_deg: Speaker azimuth around the listener (-90 = left, 0 = front, +90 = right, 180 = rear).
 *            Allowed range: [-360.0f, +360.0f] deg (usable circle [-180.0f, +180.0f] deg).
 * - dqs_lfo_hz: Rate of sinusoidal angle oscillation around dqs_deg.
 *               Allowed range: [-5.0f, +5.0f] Hz.
 * - dqs_lfo_span_deg: Peak-to-peak LFO span (swings by +/- span / 2).
 *                     Allowed range: [-360.0f, +360.0f] deg.
 * - dqs_lfo_offset_deg: Starting phase of the channel's LFO sinusoid.
 *                       Allowed range: [-360.0f, +360.0f] deg.
 * - dfs_shift_hz: Frequency shift of the whole channel (Mid and Side), positive = up, 0 = off.
 *                 Allowed range: [-20.0f, +20.0f] Hz.
 * - wet: Effect depth in [0.0f, 1.0f], crossfading between static speaker placement (wet = 0)
 *        and the same placement with LFO and frequency shift (wet = 1).
 * - side_gain: Width gain of the rotated Side in both static and moving paths.
 *              Allowed range: [0.0f, 2.0f] (2.0f can amplify Side up to +6 dB).
 */
struct QuadratureChannelParams {
    float dqs_deg = 0.0f;
    float dqs_lfo_hz = 0.0f;
    float dqs_lfo_span_deg = 180.0f;
    float dqs_lfo_offset_deg = 0.0f;
    float dfs_shift_hz = 0.0f;
    float wet = 1.0f;
    float side_gain = 1.0f;
};

/**
 * @brief Multi-channel Mid/Side Hilbert Quadrature Spatial Processor (DQS + LFO + DFS).
 *
 * Optimized for ESP32-S3 execution (< 1.0 ms for 6 channels per 10.0 ms / 480-sample block):
 * - Computes the adjustable input HPF (Order 1..4, 50..200 Hz, Q=0.707), M/S matrix, and
 *   8-section Niemitalo IIR Dual All-Pass Hilbert pair ONCE per 480-sample block.
 * - Supports 2 to 8 independent output channels (speakers), each with its own isolated
 *   uint32_t full-turn (2^32) LFO and DFS phase accumulators (zero shared phase counters).
 * - Uses the single RAM-resident g_fast_sin_table[130] (DRAM_ATTR) with bit-shift indexing.
 */
class HilbertQuadratureMs {
public:
    static constexpr float kMinHpfCutoffHz = 70.0f;
    static constexpr float kMaxHpfCutoffHz = 200.0f;
    static constexpr float kMinDqsDeg = -360.0f;
    static constexpr float kMaxDqsDeg = 360.0f;
    static constexpr float kMinLfoHz = -5.0f;
    static constexpr float kMaxLfoHz = 5.0f;
    static constexpr float kMinLfoSpanDeg = -360.0f;
    static constexpr float kMaxLfoSpanDeg = 360.0f;
    static constexpr float kMinLfoOffsetDeg = -360.0f;
    static constexpr float kMaxLfoOffsetDeg = 360.0f;
    static constexpr float kMinDfsShiftHz = -20.0f;
    static constexpr float kMaxDfsShiftHz = 20.0f;
    static constexpr float kMinWet = 0.0f;
    static constexpr float kMaxWet = 1.0f;
    static constexpr float kMinSideGain = 0.0f;
    static constexpr float kMaxSideGain = 2.0f;

    explicit HilbertQuadratureMs(
        const HilbertQuadratureConfig& init_cfg = HilbertQuadratureConfig{}) noexcept;

    /**
     * @brief Initialize the spatial processor, configure the input HPF and Hilbert cascade, and reset states.
     *
     * @param init_cfg Initialization configuration (sample rate, HPF order/cutoff/Q, Hilbert f_min) [struct].
     * @param channels Initial per-channel parameter array (2 to 8 channels) [pointer].
     * @param num_channels Number of active output channels (2 to 8) [count].
     */
    void init(
        const HilbertQuadratureConfig& init_cfg = HilbertQuadratureConfig{},
        const QuadratureChannelParams* channels = nullptr,
        std::size_t num_channels = kMinQuadratureChannels) noexcept;

    /**
     * @brief Configure all active output channels (2 to 8 channels; thread-safe from control task).
     *
     * @param channels Array of per-channel parameter structs [pointer].
     * @param num_channels Number of output channels to activate (clamped to [1, kMaxQuadratureChannels]) [count].
     * @param reset_phases If true, resets each channel's LFO and DFS phase accumulators to 0 [boolean].
     */
    void set_channels(
        const QuadratureChannelParams* channels,
        std::size_t num_channels,
        bool reset_phases = false) noexcept;

    /**
     * @brief Update a single output channel's spatial parameters at runtime (thread-safe).
     *
     * Does not disturb the LFO or DFS phase accumulators of this or any other channel.
     *
     * @param channel_idx Output channel index in [0, num_channels - 1] [index].
     * @param params New spatial audio parameters for that speaker channel [struct].
     */
    void set_channel_params(std::size_t channel_idx, const QuadratureChannelParams& params) noexcept;

    /**
     * @brief Read back the currently active (slewed) parameters of a single output channel.
     *
     * @param channel_idx Output channel index in [0, kMaxQuadratureChannels - 1] [index].
     * @return QuadratureChannelParams Copy of the channel's active spatial parameters [struct].
     */
    QuadratureChannelParams channel_params(std::size_t channel_idx) const noexcept {
        return (channel_idx < kMaxQuadratureChannels) ? active_params_[channel_idx] : QuadratureChannelParams{};
    }

    /**
     * @brief Read back the target (staged) parameters of a single output channel.
     *
     * @param channel_idx Output channel index in [0, kMaxQuadratureChannels - 1] [index].
     * @return QuadratureChannelParams Copy of the channel's target spatial parameters [struct].
     */
    QuadratureChannelParams target_channel_params(std::size_t channel_idx) const noexcept {
        return (channel_idx < kMaxQuadratureChannels) ? staged_snapshot_.params[channel_idx] : QuadratureChannelParams{};
    }

    /**
     * @brief Update the input High-Pass Filter cutoff frequency at runtime (thread-safe, slewed).
     *
     * @param cutoff_hz Desired -3 dB cutoff frequency in [70.0, 200.0] Hz [Hz].
     */
    void set_hpf_cutoff_hz(float cutoff_hz) noexcept {
        pending_hpf_cutoff_hz_.store(std::clamp(cutoff_hz, kMinHpfCutoffHz, kMaxHpfCutoffHz), std::memory_order_release);
    }

    float hpf_cutoff_hz() const noexcept {
        return init_cfg_.hpf_cutoff_hz;
    }

    float target_hpf_cutoff_hz() const noexcept {
        return pending_hpf_cutoff_hz_.load(std::memory_order_acquire);
    }

    /**
     * @brief Enable or disable spatial upmix bypass (when bypassed, outputs HPF-filtered L/R/Mid).
     */
    void set_bypass(bool bypass) noexcept {
        bypass_.store(bypass, std::memory_order_release);
    }

    bool bypass() const noexcept {
        return bypass_.load(std::memory_order_acquire);
    }

    /**
     * @brief Return the currently active number of output channels.
     */
    std::size_t num_channels() const noexcept { return num_channels_; }

    /**
     * @brief Reset all filter delay lines and per-channel phase accumulators to zero.
     */
    void reset() noexcept;

    /**
     * @brief Step 1 of split dual-core processing: run input HPF, M/S split, and shared Mid/Side
     *        Niemitalo Hilbert IIR all-pass cascades for a 48 kHz float32 stereo block.
     *
     * After this call completes on Core 0, render_channel_f32() or render_channel_to_pcm16()
     * may be called concurrently on Core 0 and Core 1 for distinct channel indices ch.
     *
     * @param left_in Input Left channel float32 buffer (n_samples elements) [normalized amplitude].
     * @param right_in Input Right channel float32 buffer (n_samples elements) [normalized amplitude].
     * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
     */
    void prepare_block_f32(
        const float* left_in,
        const float* right_in,
        std::size_t n_samples = kFrameSamples) noexcept;

    /**
     * @brief Step 2a of split dual-core processing: render one output channel into float32.
     *
     * Thread-safe across cores for distinct ch indices after prepare_block_f32() has completed.
     *
     * @param ch Output channel index in [0, num_channels - 1] [index].
     * @param out_ch Output float32 buffer (n_samples elements) [normalized amplitude].
     * @param n_samples Number of samples to render (default 480, max 480) [samples].
     */
    void render_channel_f32(
        std::size_t ch,
        float* DSP_RESTRICT out_ch,
        std::size_t n_samples = kFrameSamples) noexcept;

    /**
     * @brief Step 2b of split dual-core processing: render one output channel directly to 16-bit PCM
     *        with fused linear gain ramping (gain_start -> gain_end) and saturation clamping.
     *
     * Thread-safe across cores for distinct ch indices after prepare_block_f32() has completed.
     *
     * @param ch Output channel index in [0, num_channels - 1] [index].
     * @param out_pcm16 Output int16_t PCM buffer (n_samples elements) [int16 PCM].
     * @param gain_start Linear gain at the start of the frame (1.0f = 0 dB) [dimensionless].
     * @param gain_end Linear gain at the end of the frame (1.0f = 0 dB) [dimensionless].
     * @param n_samples Number of samples to render (default 480, max 480) [samples].
     */
    void render_channel_to_pcm16(
        std::size_t ch,
        int16_t* DSP_RESTRICT out_pcm16,
        float gain_start = 1.0f,
        float gain_end = 1.0f,
        std::size_t n_samples = kFrameSamples) noexcept;

    /**
     * @brief Process a 48 kHz float32 stereo block (480 samples) into num_channels output buffers.
     *
     * Matches process_quadrature_ms in Signal_processing/hilbert_helper.py.
     *
     * @param left_in Input Left channel float32 buffer (n_samples elements) [normalized amplitude].
     * @param right_in Input Right channel float32 buffer (n_samples elements) [normalized amplitude].
     * @param out_channels Array of num_channels pointers, each pointing to an output float32 buffer
     *                     of length n_samples [normalized amplitude].
     * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
     */
    void process_block_f32(
        const float* DSP_RESTRICT left_in,
        const float* DSP_RESTRICT right_in,
        float* const* out_channels,
        std::size_t n_samples = kFrameSamples) noexcept;

    /**
     * @brief Process a 48 kHz deinterleaved 16-bit PCM stereo block (480 samples) into num_channels int16 buffers.
     *
     * @param left_pcm16 Input Left channel 16-bit PCM buffer (n_samples elements) [int16 PCM].
     * @param right_pcm16 Input Right channel 16-bit PCM buffer (n_samples elements) [int16 PCM].
     * @param out_channels_pcm16 Array of num_channels pointers, each pointing to an output int16_t buffer
     *                           of length n_samples [int16 PCM].
     * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
     */
    void process_block_pcm16(
        const int16_t* DSP_RESTRICT left_pcm16,
        const int16_t* DSP_RESTRICT right_pcm16,
        int16_t* const* out_channels_pcm16,
        std::size_t n_samples = kFrameSamples) noexcept;

    /**
     * @brief Process a 48 kHz interleaved 16-bit PCM stereo block [L0, R0, L1, R1...] into num_channels int16 buffers.
     *
     * @param stereo_pcm16 Interleaved stereo 16-bit PCM buffer (2 * n_samples elements) [int16 PCM].
     * @param out_channels_pcm16 Array of num_channels pointers to output int16_t buffers [int16 PCM].
     * @param n_samples Number of stereo frames to process (default 480, max 480) [samples].
     */
    void process_interleaved_pcm16(
        const int16_t* DSP_RESTRICT stereo_pcm16,
        int16_t* const* out_channels_pcm16,
        std::size_t n_samples = kFrameSamples) noexcept;

    /**
     * @brief Convenience stereo-pair wrapper matching process_dqs in Signal_processing/hilbert_helper.py.
     *
     * Places Left at dqs_deg - 90 deg and Right at dqs_deg + 90 deg, crossfaded against
     * unprocessed Left (-90 deg, gain 1.0) and Right (+90 deg, gain 1.0).
     *
     * @param left_in Input Left channel float32 buffer [normalized amplitude].
     * @param right_in Input Right channel float32 buffer [normalized amplitude].
     * @param left_out Output Left channel float32 buffer [normalized amplitude].
     * @param right_out Output Right channel float32 buffer [normalized amplitude].
     * @param params Shared DQS stereo parameters (dqs_deg, wet, side_gain, dqs_lfo_hz, span, offset) [struct].
     * @param n_samples Number of stereo frames to process (default 480) [samples].
     */
    void process_dqs_stereo_f32(
        const float* DSP_RESTRICT left_in,
        const float* DSP_RESTRICT right_in,
        float* DSP_RESTRICT left_out,
        float* DSP_RESTRICT right_out,
        const QuadratureChannelParams& params,
        std::size_t n_samples = kFrameSamples) noexcept;

    /**
     * @brief Inspect a channel's current LFO and DFS phase accumulators (for testing state isolation).
     */
    uint32_t channel_lfo_phase_u32(std::size_t ch) const noexcept {
        return (ch < kMaxQuadratureChannels) ? states_[ch].lfo_acc : 0u;
    }

    uint32_t channel_dfs_phase_u32(std::size_t ch) const noexcept {
        return (ch < kMaxQuadratureChannels) ? states_[ch].shift_acc : 0u;
    }

    static QuadratureChannelParams sanitize_channel_params(const QuadratureChannelParams& p) noexcept;

private:
    /**
     * @brief Strictly isolated per-channel oscillator and spatial parameter state.
     *
     * Each output channel owns its own independent lfo_acc and shift_acc phase counters.
     */
    struct ChannelState {
        uint32_t lfo_acc = 0u;
        uint32_t shift_acc = 0u;
        uint32_t current_dqs_u32 = 0u;
        uint32_t target_dqs_u32 = 0u;
        uint32_t dry_dqs_u32 = 0u;
        uint32_t lfo_inc_u32 = 0u;
        uint32_t lfo_offset_u32 = 0u;
        uint32_t dfs_inc_u32 = 0u;
        float lfo_half_span_s30 = 0.0f;
        float wet = 1.0f;
        float prev_wet = 1.0f;
        float side_gain = 1.0f;
        float prev_side_gain = 1.0f;
        float dry_side_gain = 1.0f;
        bool has_lfo = false;
        bool has_dfs = false;
    };

    struct PendingChannelSnapshot {
        std::array<QuadratureChannelParams, kMaxQuadratureChannels> params{};
        std::size_t num_channels = kMinQuadratureChannels;
        bool reset_phases = false;
    };

    void update_channel_coefficients(
        std::size_t ch,
        const QuadratureChannelParams& p,
        float dry_azimuth_deg,
        float dry_gain,
        bool snap_angle,
        bool reset_phase) noexcept;

    void sync_pending_channels_if_needed(float dt_sec) noexcept;

    template <typename SampleWriter>
    void render_channel_impl(
        ChannelState& st,
        std::size_t n_samples,
        SampleWriter&& write_sample) noexcept;

    void render_channel_block(
        ChannelState& st,
        float* DSP_RESTRICT out_ch,
        std::size_t n_samples) noexcept;

    HilbertQuadratureConfig init_cfg_{};
    float angle_smooth_coeff_ = 1.0f;
    std::atomic<float> pending_hpf_cutoff_hz_{100.0f};
    std::atomic<bool> bypass_{false};
    bool bypass_active_ = false;

    BiquadCascade<2> hpf_left_;
    BiquadCascade<2> hpf_right_;
    HilbertIIRAllpass hilbert_side_;
    HilbertIIRAllpass hilbert_mid_;

    std::size_t num_channels_ = kMinQuadratureChannels;
    std::array<ChannelState, kMaxQuadratureChannels> states_{};
    std::array<QuadratureChannelParams, kMaxQuadratureChannels> active_params_{};
    PendingChannelSnapshot staged_snapshot_{};
    AtomicConfig<PendingChannelSnapshot> pending_config_;
    uint32_t applied_gen_ = 0u;

    alignas(16) float d_s_[kFrameSamples]{};
    alignas(16) float q_s_[kFrameSamples]{};
    alignas(16) float d_m_[kFrameSamples]{};
    alignas(16) float q_m_[kFrameSamples]{};
};

}  // namespace audio_dsp

#endif  // AUDIO_DSP_HILBERT_QUADRATURE_MS_HPP_

