#ifndef AUDIO_DSP_SUBHARMONIC_NCCF_SYNTH_HPP_
#define AUDIO_DSP_SUBHARMONIC_NCCF_SYNTH_HPP_

#include "dsp_primitives.hpp"
#include "subharmonic_pre_stage.hpp"
#include "subharmonic_zc_flipper.hpp"

namespace audio_dsp {

/**
 * @brief Configuration parameters for Prototype 2: 8 kHz NCCF Pitch-Tracked Subharmonic Synthesizer.
 */
struct SubharmonicNccfConfig {
    /// Minimum detectable fundamental frequency in Hertz [Hz].
    float f_min_hz = 45.0f;

    /// Maximum detectable fundamental frequency in Hertz [Hz].
    float f_max_hz = 120.0f;

    /// Voicing activation threshold for weighted NCCF peak [dimensionless, 0.0 to 1.0].
    float voicing_thresh_on = 0.65f;

    /// Voicing deactivation hysteresis threshold for weighted NCCF peak [dimensionless, 0.0 to 1.0].
    float voicing_thresh_off = 0.50f;

    /// Number of hangover frames to sustain pitch after correlation drops [frames].
    int hangover_frames = 2;

    /// Linear lag preference weight suppressing sub-octave period doubling (2*T0) [dimensionless].
    float lag_first_peak_weight = 0.10f;

    /// Enable 2-stage coarse-to-fine hierarchical lag search [boolean].
    bool use_hierarchical_search = true;

    /// Coarse lag search stride in samples [samples].
    int coarse_step = 4;

    /// Fine lag refinement radius around coarse peak candidates in samples [samples].
    int fine_radius = 4;

    /// Enable 3*k_naive harmonic disambiguation check [boolean].
    bool enable_3x_harmonic_check = true;

    /// Relative correlation threshold for 3*k_naive disambiguation [dimensionless].
    float harmonic_3x_beta = 0.70f;

    /// Enable Blackman-Tukey sub-bass energy correlator ducking (< 45 Hz) [boolean].
    bool use_sub_ducking = true;

    /// Sub-bass cutoff frequency for ducking correlator in Hertz [Hz].
    float sub_duck_cutoff_hz = 45.0f;

    /// Sub-bass energy ratio at which synthesizer ducks to 0.0 [dimensionless].
    float sub_duck_thresh_ratio = 0.25f;

    /// Envelope follower attack time constant in milliseconds [ms].
    float env_attack_ms = 20.0f;

    /// Envelope follower decay time constant in milliseconds [ms].
    float env_decay_ms = 80.0f;

    /// Apply 2nd-order Butterworth HPF on the dry bass path before mixing [boolean].
    bool apply_dry_hpf = false;

    /// Dry path HPF cutoff frequency in Hertz [Hz].
    float dry_hpf_hz = 80.0f;

    /// Linear gain of the dry 8 kHz bass signal in the output mix [dimensionless].
    float dry_gain = 0.0f;

    /// Linear gain of the synthesized f0/2 subharmonic in the output mix [dimensionless].
    float sub_gain = 1.0f;
};

/**
 * @brief Per-frame telemetry diagnostics produced by SubharmonicNccfSynth.
 */
struct SubharmonicNccfDiagnostics {
    float f0_hz = 0.0f;
    float f_sub_hz = 0.0f;
    float nccf_peak = 0.0f;
    float duck_gain = 1.0f;
    bool voiced = false;
};

/**
 * @brief Prototype 2: Frame-based NCCF Pitch-Tracked Subharmonic Synthesizer at 8 kHz.
 *
 * Receives 80 mono samples per 10.0 ms block at 8 kHz from the shared SubharmonicPreStage,
 * estimates fundamental bass pitch f0 via Normalized Cross-Correlation Function (NCCF)
 * with O(1) incremental lag energy, 2-stage coarse-to-fine search, 3*k_naive disambiguation,
 * and 3-point parabolic interpolation, and synthesizes a continuous-phase f0/2 subharmonic
 * using RecursiveNCO and EnvelopeFollower natively at 8 kHz.
 */
class SubharmonicNccfSynth {
public:
    static constexpr std::size_t kCircBufSize = 240;  // 30.0 ms at 8 kHz
    static constexpr std::size_t kMaxLag = 200;       // Down to 40 Hz at 8 kHz
    static constexpr std::size_t kDuckLags = 12;      // Blackman-Tukey lags at 8 kHz

    static constexpr float kMinGain = 0.0f;
    static constexpr float kMaxGain = 2.0f;
    static constexpr float kMinFMinHz = 42.0f;
    static constexpr float kMaxFMinHz = 100.0f;
    static constexpr float kMinFMaxHz = 60.0f;
    static constexpr float kMaxFMaxHz = 200.0f;
    static constexpr float kMinVoicingThresh = 0.10f;
    static constexpr float kMaxVoicingThresh = 0.98f;
    static constexpr float kMinLagWeight = 0.0f;
    static constexpr float kMaxLagWeight = 0.50f;
    static constexpr float kMinHarmonicBeta = 0.30f;
    static constexpr float kMaxHarmonicBeta = 0.99f;
    static constexpr float kMinDuckCutoffHz = 20.0f;
    static constexpr float kMaxDuckCutoffHz = 80.0f;
    static constexpr float kMinDuckThreshRatio = 0.01f;
    static constexpr float kMaxDuckThreshRatio = 1.00f;
    static constexpr float kMinEnvAttackMs = 1.0f;
    static constexpr float kMaxEnvAttackMs = 100.0f;
    static constexpr float kMinEnvDecayMs = 10.0f;
    static constexpr float kMaxEnvDecayMs = 500.0f;
    static constexpr float kMinDryHpfHz = 40.0f;
    static constexpr float kMaxDryHpfHz = 200.0f;

    explicit SubharmonicNccfSynth(
        const SubharmonicNccfConfig& cfg = SubharmonicNccfConfig{},
        int fs = kSubSampleRateHz) noexcept;

    /**
     * @brief Initialize or reconfigure the NCCF subharmonic synthesizer and reset states.
     *
     * @param cfg Configuration parameters [dimensionless struct].
     * @param fs Operating sample rate in Hertz (default 8000 Hz) [Hz].
     */
    void init(
        const SubharmonicNccfConfig& cfg = SubharmonicNccfConfig{},
        int fs = kSubSampleRateHz) noexcept;

    /**
     * @brief Update configuration parameters from a non-real-time control task (thread-safe, slewed).
     *
     * @param cfg New configuration parameters [dimensionless struct].
     */
    void set_config(const SubharmonicNccfConfig& cfg) noexcept;

    /**
     * @brief Return the currently active (slewed) configuration parameters.
     */
    SubharmonicNccfConfig config() const noexcept { return active_cfg_; }

    /**
     * @brief Return the target (staged) configuration parameters.
     */
    SubharmonicNccfConfig target_config() const noexcept { return config_queue_.get(); }

    /**
     * @brief Advance 1-second full-range slew limiters for one processing block of duration dt_sec.
     */
    void sync_config_if_needed(float dt_sec = 0.01f) noexcept;

    /**
     * @brief Reset circular buffer, pitch tracker, NCO, envelope follower, and filter states.
     */
    void reset() noexcept;

    /**
     * @brief Process a block of 8 kHz float32 samples (typically 80 samples per 10 ms frame).
     *
     * @param in_8k Input 8 kHz LP-filtered mono bass buffer [normalized amplitude].
     * @param out_8k Output 8 kHz mono subwoofer buffer [normalized amplitude].
     * @param n_samples Number of 8 kHz samples to process (default 80) [samples].
     */
    void process_block_f32(
        const float* DSP_RESTRICT in_8k,
        float* DSP_RESTRICT out_8k,
        std::size_t n_samples = kSubFrameSamples) noexcept;

    /**
     * @brief Process a block of 8 kHz 16-bit PCM samples (typically 80 samples per 10 ms frame).
     *
     * @param in_pcm16_8k Input 8 kHz 16-bit PCM mono buffer [int16 PCM].
     * @param out_pcm16_8k Output 8 kHz 16-bit PCM mono buffer [int16 PCM].
     * @param n_samples Number of 8 kHz samples to process (default 80) [samples].
     */
    void process_block_pcm16(
        const int16_t* DSP_RESTRICT in_pcm16_8k,
        int16_t* DSP_RESTRICT out_pcm16_8k,
        std::size_t n_samples = kSubFrameSamples) noexcept;

    /**
     * @brief Return diagnostics from the most recently processed 10 ms frame.
     */
    SubharmonicNccfDiagnostics last_diagnostics() const noexcept { return last_diag_; }

private:
    void apply_config_internal(const SubharmonicNccfConfig& cfg, bool reset_states) noexcept;
    float evaluate_single_lag(
        std::size_t k,
        const float* x0,
        std::size_t win_len,
        float e0,
        float e1) const noexcept;
    void detect_pitch_nccf(float& out_f0_hz, float& out_r_peak) noexcept;
    float compute_sub_duck_gain() const noexcept;

    int fs_ = kSubSampleRateHz;
    AtomicConfig<SubharmonicNccfConfig> config_queue_;
    uint32_t applied_gen_ = 0u;
    SubharmonicNccfConfig target_cfg_{};
    SubharmonicNccfConfig active_cfg_{};
    float prev_dry_gain_ = 0.0f;
    float prev_sub_gain_ = 1.0f;

    std::size_t k_min_ = 53;
    std::size_t k_max_ = 177;
    std::size_t win_len_ = 63;

    alignas(16) std::array<float, kCircBufSize> circ_buffer_{};
    alignas(16) std::array<float, kMaxLag + 2> lag_energy_{};
    alignas(16) std::array<float, kMaxLag + 2> nccf_raw_{};
    alignas(16) std::array<float, kMaxLag + 2> nccf_weighted_{};
    std::array<uint8_t, kMaxLag + 2> lag_evaluated_{};
    std::array<float, kDuckLags> duck_weights_{};

    bool is_voiced_ = false;
    int hangover_counter_ = 0;
    float last_valid_f0_ = 0.0f;
    std::array<float, 3> pitch_history_{};
    std::size_t pitch_hist_count_ = 0;

    RecursiveNCO nco_;
    EnvelopeFollower env_follower_;
    BiquadCascade<1> dry_hpf_;

    alignas(16) float work_in_[kSubFrameSamples]{};
    alignas(16) float env_buf_[kSubFrameSamples]{};
    alignas(16) float nco_buf_[kSubFrameSamples]{};
    alignas(16) float dry_buf_[kSubFrameSamples]{};

    SubharmonicNccfDiagnostics last_diag_{};
};

/**
 * @brief Operating mode of the unified 48 kHz -> 8 kHz SubharmonicPipeline.
 *
 * Only one subharmonic algorithm is executed at a time.
 */
enum class SubharmonicMode : uint8_t {
    kBypass = 0,     ///< Output the 8 kHz LP-filtered mono bass without subharmonic synthesis.
    kZcFlipper = 1,  ///< Execute Prototype 1 (SubharmonicZcFlipper) at 8 kHz.
    kNccfSynth = 2   ///< Execute Prototype 2 (SubharmonicNccfSynth) at 8 kHz.
};

/**
 * @brief Unified Subharmonic Resonator Pipeline sharing the 48 kHz -> 8 kHz pre-stage.
 *
 * Feeds 48 kHz stereo audio (480 samples / 10.0 ms frame) through the shared
 * SubharmonicPreStage (stereo -> mono -> configurable 4th-order Butterworth LP in 70-200 Hz
 * -> 6:1 decimation to 8 kHz) and routes the resulting 80-sample 8 kHz block to whichever
 * subharmonic resonator mode is currently enabled (only one runs at a time).
 */
class SubharmonicPipeline {
public:
    SubharmonicPipeline() noexcept = default;

    /**
     * @brief Initialize the shared pre-stage and both 8 kHz subharmonic engines.
     *
     * @param mode Initial active mode (kBypass, kZcFlipper, or kNccfSynth) [enum].
     * @param pre_lpf_cutoff_hz Shared 4th-order LP filter cutoff in [70.0, 200.0] Hz [Hz].
     * @param zc_cfg Configuration for Prototype 1 (Zero-Crossing Flipper) [struct].
     * @param nccf_cfg Configuration for Prototype 2 (NCCF Synthesizer) [struct].
     */
    void init(
        SubharmonicMode mode = SubharmonicMode::kBypass,
        float pre_lpf_cutoff_hz = 100.0f,
        const SubharmonicZcFlipperConfig& zc_cfg = SubharmonicZcFlipperConfig{},
        const SubharmonicNccfConfig& nccf_cfg = SubharmonicNccfConfig{}) noexcept;

    /**
     * @brief Select which subharmonic algorithm is active (thread-safe).
     */
    void set_mode(SubharmonicMode mode) noexcept;

    SubharmonicMode mode() const noexcept {
        return static_cast<SubharmonicMode>(mode_.load(std::memory_order_acquire));
    }

    /**
     * @brief Set the cutoff frequency of the shared 4th-order LP pre-filter in [70.0, 200.0] Hz.
     */
    void set_pre_lpf_cutoff_hz(float cutoff_hz) noexcept {
        pre_stage_.set_cutoff_hz(cutoff_hz);
    }

    float pre_lpf_cutoff_hz() const noexcept {
        return pre_stage_.cutoff_hz();
    }

    /** @brief Pre-stage Butterworth LP order {2, 3, 4, 6} (thread-safe, applied at the next block). */
    void set_pre_lpf_order(int order) noexcept { pre_stage_.set_order(order); }
    int pre_lpf_order() const noexcept { return pre_stage_.order(); }

    SubharmonicPreStage& pre_stage() noexcept { return pre_stage_; }
    const SubharmonicPreStage& pre_stage() const noexcept { return pre_stage_; }
    SubharmonicZcFlipper& zc_flipper() noexcept { return zc_flipper_; }
    const SubharmonicZcFlipper& zc_flipper() const noexcept { return zc_flipper_; }
    SubharmonicNccfSynth& nccf_synth() noexcept { return nccf_synth_; }
    const SubharmonicNccfSynth& nccf_synth() const noexcept { return nccf_synth_; }

    void reset() noexcept;

    /**
     * @brief Process 48 kHz deinterleaved stereo float32 (480 frames) -> 8 kHz mono subwoofer float32 (80 samples).
     *
     * @param left_48k Input Left buffer at 48 kHz (480 samples) [normalized amplitude].
     * @param right_48k Input Right buffer at 48 kHz (480 samples) [normalized amplitude].
     * @param out_sub_8k Output mono subwoofer buffer at 8 kHz (80 samples) [normalized amplitude].
     * @param n_frames_48k Number of 48 kHz input stereo frames (default 480) [samples].
     * @return std::size_t Number of 8 kHz output samples produced (80) [samples].
     */
    std::size_t process_stereo_f32_to_8k(
        const float* DSP_RESTRICT left_48k,
        const float* DSP_RESTRICT right_48k,
        float* DSP_RESTRICT out_sub_8k,
        std::size_t n_frames_48k = kFrameSamples) noexcept;

    /**
     * @brief Process 48 kHz deinterleaved stereo int16 PCM (480 frames) -> 8 kHz mono subwoofer int16 PCM (80 samples).
     *
     * @param left_pcm16_48k Input Left 16-bit PCM buffer at 48 kHz (480 samples) [int16 PCM].
     * @param right_pcm16_48k Input Right 16-bit PCM buffer at 48 kHz (480 samples) [int16 PCM].
     * @param out_sub_pcm16_8k Output mono subwoofer 16-bit PCM buffer at 8 kHz (80 samples) [int16 PCM].
     * @param n_frames_48k Number of 48 kHz input stereo frames (default 480) [samples].
     * @return std::size_t Number of 8 kHz output samples produced (80) [samples].
     */
    std::size_t process_stereo_pcm16_to_8k(
        const int16_t* DSP_RESTRICT left_pcm16_48k,
        const int16_t* DSP_RESTRICT right_pcm16_48k,
        int16_t* DSP_RESTRICT out_sub_pcm16_8k,
        std::size_t n_frames_48k = kFrameSamples) noexcept;

private:
    std::atomic<uint8_t> mode_{static_cast<uint8_t>(SubharmonicMode::kBypass)};
    SubharmonicPreStage pre_stage_;
    SubharmonicZcFlipper zc_flipper_;
    SubharmonicNccfSynth nccf_synth_;
    alignas(16) float pre_out_8k_[kSubFrameSamples]{};
    alignas(16) float sub_out_8k_[kSubFrameSamples]{};
};

}  // namespace audio_dsp

#endif  // AUDIO_DSP_SUBHARMONIC_NCCF_SYNTH_HPP_

