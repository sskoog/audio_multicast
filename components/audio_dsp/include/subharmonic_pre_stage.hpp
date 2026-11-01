#ifndef AUDIO_DSP_SUBHARMONIC_PRE_STAGE_HPP_
#define AUDIO_DSP_SUBHARMONIC_PRE_STAGE_HPP_

#include <atomic>

#include "dsp_primitives.hpp"

namespace audio_dsp {

/**
 * @brief Configuration for the shared 48 kHz -> 8 kHz subwoofer downsampling pre-stage.
 */
struct SubharmonicPreStageConfig {
    /// Butterworth Low-Pass cutoff frequency in Hertz (clamped to [70.0f, 200.0f] Hz).
    float lpf_cutoff_hz = 100.0f;
    /// Butterworth Low-Pass order in {2, 3, 4, 6} (other values are mapped to the nearest valid one).
    int lpf_order = 4;
};

/**
 * @brief Common downsampling pre-stage shared by both subharmonic resonator pipelines.
 *
 * Pipeline:
 *   48 kHz Stereo (480 samples / 10 ms)
 *   -> Mono downmix: 0.5 * (L + R)
 *   -> Configurable Butterworth Low-Pass filter (order 2 / 3 / 4 / 6, 70 - 200 Hz); this is both the
 *      subwoofer side of the crossover and the anti-alias filter of the decimation
 *   -> 6:1 Decimation to 8 kHz (80 samples / 10 ms)
 *
 * Statically allocated (zero heap allocation in steady state) and thread-safe for runtime cutoff
 * (AtomicConfig, slewed) and order (atomic, switched at the next block) updates from a control task.
 */
class SubharmonicPreStage {
public:
    static constexpr float kMinCutoffHz = 70.0f;
    static constexpr float kMaxCutoffHz = 200.0f;

    explicit SubharmonicPreStage(const SubharmonicPreStageConfig& cfg = SubharmonicPreStageConfig{}) noexcept;

    /**
     * @brief Initialize or reconfigure the pre-stage and reset filter states.
     *
     * @param cfg Pre-stage configuration structure [dimensionless struct].
     */
    void init(const SubharmonicPreStageConfig& cfg = SubharmonicPreStageConfig{}) noexcept;

    /**
     * @brief Update the LP filter cutoff frequency (thread-safe from control task, slewed).
     *
     * @param cutoff_hz Desired -3 dB cutoff frequency in Hertz (clamped to [70.0, 200.0] Hz) [Hz].
     */
    void set_cutoff_hz(float cutoff_hz) noexcept;

    /**
     * @brief Get current active cutoff frequency in Hertz [Hz].
     */
    float cutoff_hz() const noexcept { return active_cutoff_hz_; }

    /**
     * @brief Change the LP filter order (thread-safe from control task; applied at the next block, filter
     *        state cleared because the section structure changes).
     *
     * @param order Butterworth order in {2, 3, 4, 6}; 1 -> 2, 5 and above -> 6 [dimensionless].
     */
    void set_order(int order) noexcept { pending_order_.store(normalize_order(order), std::memory_order_release); }

    /** @brief Requested LP filter order {2, 3, 4, 6}. */
    int order() const noexcept { return pending_order_.load(std::memory_order_acquire); }

    /** @brief Map any requested order to a supported one: <= 2 -> 2, 3, 4, >= 5 -> 6. */
    static constexpr int normalize_order(int order) noexcept {
        return order <= 2 ? 2 : (order >= 5 ? 6 : order);
    }

    /**
     * @brief Reset internal biquad filter state registers to zero.
     */
    void reset() noexcept;

    /**
     * @brief Downmix deinterleaved 48 kHz stereo float32 to mono, apply the LP, and decimate 6:1 to 8 kHz.
     *
     * @param left_48k Input Left channel buffer at 48 kHz (length n_frames_48k) [normalized amplitude].
     * @param right_48k Input Right channel buffer at 48 kHz (length n_frames_48k) [normalized amplitude].
     * @param out_mono_8k Output mono buffer at 8 kHz (length n_frames_48k / 6 = 80) [normalized amplitude].
     * @param n_frames_48k Number of 48 kHz input frames (default 480, must be multiple of 6) [samples].
     * @return std::size_t Number of 8 kHz output samples produced (80 for 480 input frames) [samples].
     */
    std::size_t process_stereo_f32_to_8k(
        const float* DSP_RESTRICT left_48k,
        const float* DSP_RESTRICT right_48k,
        float* DSP_RESTRICT out_mono_8k,
        std::size_t n_frames_48k = kFrameSamples) noexcept;

    /**
     * @brief Filter 48 kHz mono float32 through the LP and decimate 6:1 to 8 kHz.
     *
     * @param mono_48k Input mono buffer at 48 kHz (length n_frames_48k) [normalized amplitude].
     * @param out_mono_8k Output mono buffer at 8 kHz (length n_frames_48k / 6 = 80) [normalized amplitude].
     * @param n_frames_48k Number of 48 kHz input frames (default 480, must be multiple of 6) [samples].
     * @return std::size_t Number of 8 kHz output samples produced [samples].
     */
    std::size_t process_mono_f32_to_8k(
        const float* DSP_RESTRICT mono_48k,
        float* DSP_RESTRICT out_mono_8k,
        std::size_t n_frames_48k = kFrameSamples) noexcept;

    /**
     * @brief Downmix deinterleaved 48 kHz stereo int16 PCM to mono, apply the LP, and decimate 6:1 to 8 kHz float32.
     *
     * @param left_pcm16 Input Left 16-bit PCM buffer at 48 kHz [int16 PCM].
     * @param right_pcm16 Input Right 16-bit PCM buffer at 48 kHz [int16 PCM].
     * @param out_mono_8k Output mono float32 buffer at 8 kHz (length n_frames_48k / 6 = 80) [normalized amplitude].
     * @param n_frames_48k Number of 48 kHz input frames (default 480) [samples].
     * @return std::size_t Number of 8 kHz output samples produced [samples].
     */
    std::size_t process_stereo_pcm16_to_8k(
        const int16_t* DSP_RESTRICT left_pcm16,
        const int16_t* DSP_RESTRICT right_pcm16,
        float* DSP_RESTRICT out_mono_8k,
        std::size_t n_frames_48k = kFrameSamples) noexcept;

    /**
     * @brief Downmix interleaved 48 kHz stereo int16 PCM [L0, R0, L1, R1...] to mono, apply the LP, and decimate 6:1.
     *
     * @param stereo_pcm16 Interleaved stereo 16-bit PCM buffer at 48 kHz (2 * n_frames_48k elements) [int16 PCM].
     * @param out_mono_8k Output mono float32 buffer at 8 kHz (length n_frames_48k / 6 = 80) [normalized amplitude].
     * @param n_frames_48k Number of 48 kHz stereo frames (default 480) [samples].
     * @return std::size_t Number of 8 kHz output samples produced [samples].
     */
    std::size_t process_interleaved_pcm16_to_8k(
        const int16_t* DSP_RESTRICT stereo_pcm16,
        float* DSP_RESTRICT out_mono_8k,
        std::size_t n_frames_48k = kFrameSamples) noexcept;

private:
    void sync_config_if_needed(float dt_sec) noexcept;
    void design_lpf() noexcept;  // (re)compute the coefficients for active_order_ / active_cutoff_hz_

    template <std::size_t NS, typename LoadX>
    std::size_t run_lpf_decimate(LoadX load_x, float* DSP_RESTRICT out_mono_8k, std::size_t n_frames_48k) noexcept;
    template <typename LoadX>
    std::size_t lpf_decimate(LoadX load_x, float* DSP_RESTRICT out_mono_8k, std::size_t n_frames_48k) noexcept;

    AtomicConfig<SubharmonicPreStageConfig> config_Queue_;
    uint32_t applied_gen_ = 0u;
    float target_cutoff_hz_ = 100.0f;
    float active_cutoff_hz_ = 100.0f;
    std::atomic<int> pending_order_{4};
    int active_order_ = 4;
    BiquadCascade<3> lpf_;  // up to 6th order; unused sections are pass-through
};

}  // namespace audio_dsp

#endif  // AUDIO_DSP_SUBHARMONIC_PRE_STAGE_HPP_
