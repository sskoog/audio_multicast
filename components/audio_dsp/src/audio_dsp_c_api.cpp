#include "audio_dsp_c_api.h"

#include "dsp_primitives.hpp"
#include "hilbert_quadrature_ms.hpp"
#include "subharmonic_nccf_synth.hpp"
#include "subharmonic_pre_stage.hpp"
#include "subharmonic_zc_flipper.hpp"

#include <algorithm>

extern "C" {

/**
 * @brief Evaluate FastSinLUT::sin_cos_u32 for a batch of uint32_t full-turn angles.
 *
 * @param theta_u32 Input array of uint32_t angles (2^32 = 1 turn) [turns * 2^32].
 * @param out_sin Output array of sine values [dimensionless].
 * @param out_cos Output array of cosine values [dimensionless].
 * @param n_samples Number of angles to evaluate [count].
 */
AUDIO_DSP_API void audio_dsp_fast_sin_cos_u32_batch(
    const uint32_t* theta_u32,
    float* out_sin,
    float* out_cos,
    size_t n_samples) {
    if (theta_u32 == nullptr || out_sin == nullptr || out_cos == nullptr) {
        return;
    }
    for (size_t i = 0; i < n_samples; ++i) {
        audio_dsp::FastSinLUT::sin_cos_u32(theta_u32[i], out_sin[i], out_cos[i]);
    }
}

/**
 * @brief Run HilbertQuadratureMs over a multi-block float32 stereo stream (480 samples/block).
 *
 * @param left_in Input Left channel float32 array (length total_samples) [normalized amplitude].
 * @param right_in Input Right channel float32 array (length total_samples) [normalized amplitude].
 * @param total_samples Total number of stereo samples per channel [samples].
 * @param params Array of per-channel parameter structures (length num_channels) [struct].
 * @param num_channels Number of output channels (1 to 8) [count].
 * @param hpf_order Input High-Pass filter order in {0, 1, 2, 3, 4} [dimensionless].
 * @param hpf_cutoff_hz Input High-Pass filter cutoff frequency in Hertz [Hz].
 * @param hpf_q Input High-Pass filter Q factor [dimensionless].
 * @param fs Audio sampling rate in Hertz [Hz].
 * @param out_flattened Output row-major array of shape (num_channels, total_samples) [normalized amplitude].
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
    float* out_flattened) {
    if (left_in == nullptr || right_in == nullptr || params == nullptr || out_flattened == nullptr) {
        return;
    }
    const size_t n_ch = std::clamp<size_t>(num_channels, 1u, audio_dsp::kMaxQuadratureChannels);

    audio_dsp::HilbertQuadratureConfig init_cfg{};
    init_cfg.fs = fs;
    init_cfg.hpf_order = hpf_order;
    init_cfg.hpf_cutoff_hz = hpf_cutoff_hz;
    init_cfg.hpf_q = hpf_q;
    init_cfg.hilbert_f_min_hz = 100.0f;
    init_cfg.angle_smooth_ms = 0.0f;

    audio_dsp::QuadratureChannelParams cpp_params[audio_dsp::kMaxQuadratureChannels]{};
    for (size_t c = 0; c < n_ch; ++c) {
        cpp_params[c].dqs_deg = params[c].dqs_deg;
        cpp_params[c].dqs_lfo_hz = params[c].dqs_lfo_hz;
        cpp_params[c].dqs_lfo_span_deg = params[c].dqs_lfo_span_deg;
        cpp_params[c].dqs_lfo_offset_deg = params[c].dqs_lfo_offset_deg;
        cpp_params[c].dfs_shift_hz = params[c].dfs_shift_hz;
        cpp_params[c].wet = params[c].wet;
        cpp_params[c].side_gain = params[c].side_gain;
    }

    audio_dsp::HilbertQuadratureMs proc;
    proc.init(init_cfg, cpp_params, n_ch);

    size_t offset = 0;
    while (offset < total_samples) {
        const size_t chunk = std::min(audio_dsp::kFrameSamples, total_samples - offset);
        float* ch_ptrs[audio_dsp::kMaxQuadratureChannels]{};
        for (size_t c = 0; c < n_ch; ++c) {
            ch_ptrs[c] = out_flattened + (c * total_samples) + offset;
        }
        proc.process_block_f32(left_in + offset, right_in + offset, ch_ptrs, chunk);
        offset += chunk;
    }
}

/**
 * @brief Run the shared SubharmonicPreStage (48 kHz -> 8 kHz) + SubharmonicZcFlipper (8 kHz) in 10.0 ms blocks.
 *
 * @param left_48k Input Left buffer at 48 kHz [normalized amplitude].
 * @param right_48k Input Right buffer at 48 kHz [normalized amplitude].
 * @param total_frames_48k Total number of 48 kHz input frames [samples].
 * @param pre_lpf_cutoff_hz Shared 4th-order LP filter cutoff in [70.0, 200.0] Hz [Hz].
 * @param post_lpf_cutoff_hz Post-flip LP filter cutoff at 8 kHz in Hertz [Hz].
 * @param post_lpf_order Post-flip LP filter order in {2, 4} [dimensionless].
 * @param use_adaptive_thresh Non-zero for adaptive Schmitt trigger, 0 for static threshold [boolean].
 * @param hysteresis_ratio Schmitt trigger hysteresis fraction [dimensionless].
 * @param release_ms Envelope release time constant in milliseconds [ms].
 * @param min_floor_db Noise floor in dBFS [dB].
 * @param dry_gain Linear gain of 8 kHz dry bass [dimensionless].
 * @param sub_gain Linear gain of generated subharmonic [dimensionless].
 * @param out_sub_8k Output mono 8 kHz subwoofer buffer (capacity >= total_frames_48k / 6) [normalized amplitude].
 * @return size_t Total number of 8 kHz output samples written [samples].
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
    float* out_sub_8k) {
    if (left_48k == nullptr || right_48k == nullptr || out_sub_8k == nullptr) {
        return 0u;
    }

    audio_dsp::SubharmonicZcFlipperConfig zc_cfg{};
    zc_cfg.post_lpf_cutoff_hz = post_lpf_cutoff_hz;
    zc_cfg.post_lpf_order = post_lpf_order;
    zc_cfg.use_adaptive_thresh = (use_adaptive_thresh != 0);
    zc_cfg.hysteresis_ratio = hysteresis_ratio;
    zc_cfg.release_ms = release_ms;
    zc_cfg.min_floor_db = min_floor_db;
    zc_cfg.dry_gain = dry_gain;
    zc_cfg.sub_gain = sub_gain;

    audio_dsp::SubharmonicPipeline pipeline;
    pipeline.init(audio_dsp::SubharmonicMode::kZcFlipper, pre_lpf_cutoff_hz, zc_cfg, {});

    size_t offset_48k = 0;
    size_t offset_8k = 0;
    while (offset_48k + audio_dsp::kSubDecimFactor <= total_frames_48k) {
        const size_t rem = total_frames_48k - offset_48k;
        const size_t chunk_48k = std::min(audio_dsp::kFrameSamples,
            (rem / audio_dsp::kSubDecimFactor) * audio_dsp::kSubDecimFactor);
        const size_t written = pipeline.process_stereo_f32_to_8k(
            left_48k + offset_48k, right_48k + offset_48k, out_sub_8k + offset_8k, chunk_48k);
        offset_48k += chunk_48k;
        offset_8k += written;
    }
    return offset_8k;
}

/**
 * @brief Run the shared SubharmonicPreStage (48 kHz -> 8 kHz) + SubharmonicNccfSynth (8 kHz) in 10.0 ms blocks.
 *
 * @param left_48k Input Left buffer at 48 kHz [normalized amplitude].
 * @param right_48k Input Right buffer at 48 kHz [normalized amplitude].
 * @param total_frames_48k Total number of 48 kHz input frames [samples].
 * @param pre_lpf_cutoff_hz Shared 4th-order LP filter cutoff in [70.0, 200.0] Hz [Hz].
 * @param use_hierarchical_search Non-zero to enable 2-stage coarse-to-fine search [boolean].
 * @param enable_3x_harmonic_check Non-zero to enable 3*k_naive disambiguation [boolean].
 * @param use_sub_ducking Non-zero to enable < 45 Hz sub-bass ducking [boolean].
 * @param dry_gain Linear gain of 8 kHz dry bass [dimensionless].
 * @param sub_gain Linear gain of synthesized subharmonic [dimensionless].
 * @param out_sub_8k Output mono 8 kHz subwoofer buffer (capacity >= total_frames_48k / 6) [normalized amplitude].
 * @param out_f0_frames Optional output array receiving detected f0_hz per 10 ms frame [Hz].
 * @return size_t Total number of 8 kHz output samples written [samples].
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
    float* out_f0_frames) {
    if (left_48k == nullptr || right_48k == nullptr || out_sub_8k == nullptr) {
        return 0u;
    }

    audio_dsp::SubharmonicNccfConfig nccf_cfg{};
    nccf_cfg.use_hierarchical_search = (use_hierarchical_search != 0);
    nccf_cfg.enable_3x_harmonic_check = (enable_3x_harmonic_check != 0);
    nccf_cfg.use_sub_ducking = (use_sub_ducking != 0);
    nccf_cfg.dry_gain = dry_gain;
    nccf_cfg.sub_gain = sub_gain;

    audio_dsp::SubharmonicPipeline pipeline;
    pipeline.init(audio_dsp::SubharmonicMode::kNccfSynth, pre_lpf_cutoff_hz, {}, nccf_cfg);

    size_t offset_48k = 0;
    size_t offset_8k = 0;
    size_t frame_idx = 0;
    while (offset_48k + audio_dsp::kFrameSamples <= total_frames_48k) {
        const size_t written = pipeline.process_stereo_f32_to_8k(
            left_48k + offset_48k, right_48k + offset_48k, out_sub_8k + offset_8k, audio_dsp::kFrameSamples);
        if (out_f0_frames != nullptr) {
            out_f0_frames[frame_idx] = pipeline.nccf_synth().last_diagnostics().f0_hz;
        }
        offset_48k += audio_dsp::kFrameSamples;
        offset_8k += written;
        ++frame_idx;
    }
    return offset_8k;
}

}  // extern "C"

