#ifndef AUDIO_DSP_SUBHARMONIC_ZC_FLIPPER_HPP_
#define AUDIO_DSP_SUBHARMONIC_ZC_FLIPPER_HPP_

#include "dsp_primitives.hpp"

namespace audio_dsp {

/**
 * @brief Configuration parameters for Prototype 1: 8 kHz Zero-Crossing Polarity Flipper.
 */
struct SubharmonicZcFlipperConfig {
    /// Post-flip harmonic smoothing Butterworth LP cutoff frequency in Hertz [Hz] (range 50 - 200 Hz).
    float post_lpf_cutoff_hz = 100.0f;

    /// Post-flip Butterworth LP filter order in {2, 4} (0 = bypass) [dimensionless].
    int post_lpf_order = 2;

    /// Enable adaptive leaky-peak Schmitt trigger threshold (false = static_thresh) [boolean].
    bool use_adaptive_thresh = true;

    /// Static negative arming threshold when use_adaptive_thresh is false [normalized amplitude].
    float static_thresh = 1.0e-5f;

    /// Adaptive Schmitt trigger hysteresis fraction of envelope (0.05 to 0.50) [dimensionless].
    float hysteresis_ratio = 0.20f;

    /// Envelope release time constant in milliseconds [ms].
    float release_ms = 30.0f;

    /// Minimum noise floor threshold in dBFS [dB].
    float min_floor_db = -70.0f;

    /// Enable period timer guard between zero-crossing toggles [boolean].
    bool use_period_guard = false;

    /// Minimum valid fundamental frequency for period guard in Hertz [Hz].
    float guard_f_min_hz = 45.0f;

    /// Maximum valid fundamental frequency for period guard in Hertz [Hz].
    float guard_f_max_hz = 150.0f;

    /// Linear gain of the 8 kHz input bass signal in the output mix [dimensionless].
    float dry_gain = 0.0f;

    /// Linear gain of the generated f0/2 subharmonic in the output mix [dimensionless].
    float sub_gain = 1.0f;
};

/**
 * @brief Prototype 1: Time-domain Zero-Crossing Polarity Flipper operating at 8 kHz (mono subwoofer).
 *
 * Receives 80 mono samples per 10.0 ms block at 8 kHz from the shared SubharmonicPreStage
 * (which already applies the 4th-order 70-200 Hz LP filter and 6:1 decimation from 48 kHz),
 * toggles waveform polarity on rising zero-crossings to halve the instantaneous frequency
 * (f0 -> f0 / 2) while preserving the natural signal envelope, and smooths switching harmonics
 * with a post-flip Butterworth LP filter at 8 kHz.
 */
class SubharmonicZcFlipper {
public:
    static constexpr float kMinPostLpfHz = 50.0f;
    static constexpr float kMaxPostLpfHz = 200.0f;
    static constexpr float kMinGain = 0.0f;
    static constexpr float kMaxGain = 2.0f;
    static constexpr float kMinHysteresis = 0.05f;
    static constexpr float kMaxHysteresis = 0.50f;
    static constexpr float kMinReleaseMs = 5.0f;
    static constexpr float kMaxReleaseMs = 200.0f;
    static constexpr float kMinFloorDb = -90.0f;
    static constexpr float kMaxFloorDb = -20.0f;
    static constexpr float kMinGuardFMinHz = 20.0f;
    static constexpr float kMaxGuardFMinHz = 100.0f;
    static constexpr float kMinGuardFMaxHz = 80.0f;
    static constexpr float kMaxGuardFMaxHz = 250.0f;

    explicit SubharmonicZcFlipper(
        const SubharmonicZcFlipperConfig& cfg = SubharmonicZcFlipperConfig{},
        int fs = kSubSampleRateHz) noexcept;

    /**
     * @brief Initialize or reconfigure the zero-crossing flipper and clear states.
     *
     * @param cfg Configuration parameters [dimensionless struct].
     * @param fs Operating sample rate in Hertz (default 8000 Hz) [Hz].
     */
    void init(
        const SubharmonicZcFlipperConfig& cfg = SubharmonicZcFlipperConfig{},
        int fs = kSubSampleRateHz) noexcept;

    /**
     * @brief Update configuration parameters from a non-real-time control task (thread-safe, slewed).
     *
     * @param cfg New configuration parameters [dimensionless struct].
     */
    void set_config(const SubharmonicZcFlipperConfig& cfg) noexcept;

    /**
     * @brief Return the currently active (slewed) configuration parameters.
     */
    SubharmonicZcFlipperConfig config() const noexcept { return active_cfg_; }

    /**
     * @brief Return the target (staged) configuration parameters.
     */
    SubharmonicZcFlipperConfig target_config() const noexcept { return config_queue_.get(); }

    /**
     * @brief Advance 1-second full-range slew limiters for one processing block of duration dt_sec.
     */
    void sync_config_if_needed(float dt_sec = 0.01f) noexcept;

    /**
     * @brief Reset detector, polarity, and post-LPF filter states.
     */
    void reset() noexcept;

    /**
     * @brief Process a block of 8 kHz float32 samples (typically 80 samples per 10 ms frame).
     *
     * @param in_8k Input 8 kHz LP-filtered mono bass buffer [normalized amplitude].
     * @param out_8k Output 8 kHz mono subwoofer buffer [normalized amplitude].
     * @param n_samples Number of 8 kHz samples to process (default 80, max 80) [samples].
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

private:
    void apply_config_internal(const SubharmonicZcFlipperConfig& cfg, bool reset_states) noexcept;

    int fs_ = kSubSampleRateHz;
    AtomicConfig<SubharmonicZcFlipperConfig> config_queue_;
    uint32_t applied_gen_ = 0u;
    SubharmonicZcFlipperConfig target_cfg_{};
    SubharmonicZcFlipperConfig active_cfg_{};
    float prev_dry_gain_ = 0.0f;
    float prev_sub_gain_ = 1.0f;

    float alpha_rel_ = 0.0f;
    float min_floor_lin_ = 3.16227766e-4f;
    uint32_t min_period_samples_ = 0u;
    uint32_t max_period_samples_ = 0u;

    float sign_ = 1.0f;
    bool armed_ = false;
    float env_ = 0.0f;
    uint32_t samples_since_toggle_ = 0u;

    BiquadCascade<2> post_lpf_;
    alignas(16) float work_in_[kSubFrameSamples]{};
    alignas(16) float flipped_buf_[kSubFrameSamples]{};
    alignas(16) float smooth_buf_[kSubFrameSamples]{};
};

}  // namespace audio_dsp

#endif  // AUDIO_DSP_SUBHARMONIC_ZC_FLIPPER_HPP_

