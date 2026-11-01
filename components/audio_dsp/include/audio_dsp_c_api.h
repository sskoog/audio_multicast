#ifndef AUDIO_DSP_C_API_H_
#define AUDIO_DSP_C_API_H_

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define AUDIO_DSP_API __declspec(dllexport)
#else
#define AUDIO_DSP_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float dqs_deg;
    float dqs_lfo_hz;
    float dqs_lfo_span_deg;
    float dqs_lfo_offset_deg;
    float dfs_shift_hz;
    float wet;
    float side_gain;
} AudioDspQuadratureParamC;

/**
 * @brief Evaluate FastSinLUT::sin_u32 for an array of uint32_t full-turn angles.
 */
AUDIO_DSP_API void audio_dsp_fast_sin_cos_u32_batch(
    const uint32_t* theta_u32,
    float* out_sin,
    float* out_cos,
    size_t n_samples);

/**
 * @brief Run HilbertQuadratureMs over a multi-block float32 stereo stream (480 samples/block).
 *
 * Processes total_samples in 480-sample (10.0 ms) blocks to verify block-boundary continuity
 * and numerical parity against Python's process_quadrature_ms.
 */
AUDIO_DSP_API void audio_dsp_run_quadrature_ms_f32(
    const float* left_in,
    const float* right_in,
    size_t total_samples,
    const AudioDspQuadratureParamC* params,
    size_t num_channels,
    int hpf_order,
    float hpf_cutoff_hz,
    float hpf_q,
    int fs,
    float* out_flattened);

/**
 * @brief Run the shared SubharmonicPreStage (48 kHz stereo -> mono -> 4th-order LP -> 8 kHz)
 *        followed by Prototype 1 (SubharmonicZcFlipper) at 8 kHz in 10.0 ms blocks.
 */
AUDIO_DSP_API size_t audio_dsp_run_subharmonic_zc_pipeline_f32(
    const float* left_48k,
    const float* right_48k,
    size_t total_frames_48k,
    float pre_lpf_cutoff_hz,
    float post_lpf_cutoff_hz,
    int post_lpf_order,
    int use_adaptive_thresh,
    float hysteresis_ratio,
    float release_ms,
    float min_floor_db,
    float dry_gain,
    float sub_gain,
    float* out_sub_8k);

/**
 * @brief Run the shared SubharmonicPreStage (48 kHz stereo -> mono -> 4th-order LP -> 8 kHz)
 *        followed by Prototype 2 (SubharmonicNccfSynth) at 8 kHz in 10.0 ms blocks.
 */
AUDIO_DSP_API size_t audio_dsp_run_subharmonic_nccf_pipeline_f32(
    const float* left_48k,
    const float* right_48k,
    size_t total_frames_48k,
    float pre_lpf_cutoff_hz,
    int use_hierarchical_search,
    int enable_3x_harmonic_check,
    int use_sub_ducking,
    float dry_gain,
    float sub_gain,
    float* out_sub_8k,
    float* out_f0_frames);

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_DSP_C_API_H_

