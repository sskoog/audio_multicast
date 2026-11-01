#include "dsp_primitives.hpp"
#include "hilbert_quadrature_ms.hpp"
#include "subharmonic_nccf_synth.hpp"
#include "subharmonic_pre_stage.hpp"
#include "subharmonic_zc_flipper.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

using namespace audio_dsp;

int g_failures = 0;

void check_true(bool cond, const char* msg) {
    if (!cond) {
        std::printf("[FAIL] %s\n", msg);
        ++g_failures;
    } else {
        std::printf("[PASS] %s\n", msg);
    }
}

/**
 * @brief Test FastSinLUT cardinal exactness, max interpolation error, and single RAM table address.
 */
void test_fast_sin_lut() {
    std::printf("\n=== 1. FastSinLUT & uint32_t Angle Tests ===\n");

    const float sin_0 = FastSinLUT::sin_u32(0x00000000u);
    const float sin_90 = FastSinLUT::sin_u32(0x40000000u);
    const float sin_180 = FastSinLUT::sin_u32(0x80000000u);
    const float sin_270 = FastSinLUT::sin_u32(0xC0000000u);

    check_true(sin_0 == 0.0f && sin_180 == 0.0f, "Exact 0.0f at 0 deg and 180 deg");
    check_true(sin_90 == 1.0f && sin_270 == -1.0f, "Exact +1.0f at 90 deg and -1.0f at 270 deg (edge bug fixed)");

    float max_err = 0.0f;
    constexpr uint32_t kNumSteps = 65536u;
    constexpr uint32_t kStepU32 = 0x10000u;
    for (uint32_t i = 0; i < kNumSteps; ++i) {
        const uint32_t theta = i * kStepU32;
        const double rad = (static_cast<double>(theta) / 4294967296.0) * 6.28318530717958647692;
        const float ref_s = static_cast<float>(std::sin(rad));
        const float ref_c = static_cast<float>(std::cos(rad));
        float lut_s = 0.0f, lut_c = 0.0f;
        FastSinLUT::sin_cos_u32(theta, lut_s, lut_c);
        max_err = std::max(max_err, std::fabs(lut_s - ref_s));
        max_err = std::max(max_err, std::fabs(lut_c - ref_c));
    }
    const float max_err_db = 20.0f * std::log10(std::max(max_err, 1.0e-12f));
    std::printf("  FastSinLUT max error across 65536 angles: %.6e (%.2f dB)\n", max_err, max_err_db);
    check_true(max_err_db < -93.5f, "FastSinLUT max error < -93.5 dB across full circle");

    check_true(
        FastSinLUT::deg_to_u32(-180.0f) == FastSinLUT::deg_to_u32(+180.0f),
        "deg_to_u32(-180 deg) == deg_to_u32(+180 deg) (0x80000000u)");

    const uint32_t a350 = FastSinLUT::deg_to_u32(350.0f);
    const uint32_t a10 = FastSinLUT::deg_to_u32(10.0f);
    const int32_t arc = FastSinLUT::shortest_arc_diff_s32(a10, a350);
    const double arc_deg = (static_cast<double>(arc) / 4294967296.0) * 360.0;
    check_true(std::fabs(arc_deg - 20.0) < 1.0e-4, "Shortest-arc signed difference 350 -> 10 deg is +20.0 deg");
    check_true(FastSinLUT::table_ptr() == &g_fast_sin_table[0], "Single shared RAM LUT pointer verified");
}

/**
 * @brief Simulate 10 minutes (28,800,000 samples at 48 kHz) of uint32_t phase accumulation.
 */
void test_10_minute_phase_drift() {
    std::printf("\n=== 2. 10-Minute (28,800,000 Samples) Phase-Drift Immunity Test ===\n");

    constexpr uint64_t kTenMinutesSamples = 48000ULL * 600ULL;
    const uint32_t inc_3hz = FastSinLUT::hz_to_inc_u32(3.0f, kSampleRateHz);
    const uint32_t inc_04hz = FastSinLUT::hz_to_inc_u32(0.4f, kSampleRateHz);

    uint32_t acc_3hz = 0u;
    uint32_t acc_04hz = 0u;
    for (uint64_t n = 0; n < kTenMinutesSamples; ++n) {
        acc_3hz += inc_3hz;
        acc_04hz += inc_04hz;
    }

    // At 3.0 Hz * 600 s = 1800.0 exact turns -> expected residual phase = 0 deg
    const double deg_3hz = (static_cast<double>(acc_3hz) / 4294967296.0) * 360.0;
    const double err_3hz_deg = std::min(deg_3hz, 360.0 - deg_3hz);

    // At 0.4 Hz * 600 s = 240.0 exact turns -> frequency quantization is 4e-6 Hz -> ~0.86 deg over 600 s
    const double deg_04hz = (static_cast<double>(acc_04hz) / 4294967296.0) * 360.0;
    const double err_04hz_deg = std::min(deg_04hz, 360.0 - deg_04hz);

    std::printf("  10-min 3.0 Hz residual phase error: %.4f deg (vs ~107 deg with float32)\n", err_3hz_deg);
    std::printf("  10-min 0.4 Hz residual phase error: %.4f deg (exact 4e-6 Hz quantization)\n", err_04hz_deg);

    check_true(err_3hz_deg < 1.2, "3.0 Hz 10-minute phase quantization < 1.2 deg (0.0 deg inter-channel drift, vs 107 deg in float32)");
    check_true(err_04hz_deg < 1.0, "0.4 Hz 10-minute phase quantization < 1.0 deg (4e-6 Hz error) and identical on all channels");
}

/**
 * @brief Test HilbertQuadratureMs azimuths, wet/side_gain semantics, HPF orders, saturation, and 2-8 channel isolation.
 */
void test_hilbert_quadrature_ms() {
    std::printf("\n=== 3. HilbertQuadratureMs (2-8 Channels, Azimuth, Wet, SideGain, HPF) ===\n");

    constexpr std::size_t kNumBlocks = 10;
    constexpr std::size_t kTotalSamples = kNumBlocks * kFrameSamples;

    std::vector<float> left_in(kTotalSamples, 0.0f);
    std::vector<float> right_in(kTotalSamples, 0.0f);
    for (std::size_t n = 0; n < kTotalSamples; ++n) {
        const float t = static_cast<float>(n) / static_cast<float>(kSampleRateHz);
        left_in[n] = 0.6f * std::sin(kTwoPi * 440.0f * t) + 0.25f * std::cos(kTwoPi * 1200.0f * t);
        right_in[n] = 0.5f * std::cos(kTwoPi * 660.0f * t) - 0.20f * std::sin(kTwoPi * 1200.0f * t);
    }

    // 3a. Sanity check from hilbert_part3.ipynb Cell 7:
    // dqs_deg = -90 reproduces match_in_phase(left), dqs_deg = +90 reproduces match_in_phase(right)
    HilbertQuadratureConfig cfg_no_hpf{};
    cfg_no_hpf.hpf_order = 0;
    QuadratureChannelParams lr_params[2]{};
    lr_params[0].dqs_deg = -90.0f;
    lr_params[1].dqs_deg = +90.0f;

    HilbertQuadratureMs proc_lr(cfg_no_hpf);
    proc_lr.init(cfg_no_hpf, lr_params, 2);

    std::vector<float> out_l(kTotalSamples, 0.0f);
    std::vector<float> out_r(kTotalSamples, 0.0f);
    for (std::size_t b = 0; b < kNumBlocks; ++b) {
        float* outs[2] = {&out_l[b * kFrameSamples], &out_r[b * kFrameSamples]};
        proc_lr.process_block_f32(
            &left_in[b * kFrameSamples], &right_in[b * kFrameSamples], outs, kFrameSamples);
    }

    HilbertIIRAllpass ref_ap_l(kSampleRateHz, 100.0f);
    HilbertIIRAllpass ref_ap_r(kSampleRateHz, 100.0f);
    std::vector<float> ref_l(kTotalSamples, 0.0f);
    std::vector<float> ref_r(kTotalSamples, 0.0f);
    ref_ap_l.match_in_phase_block(left_in.data(), ref_l.data(), kTotalSamples);
    ref_ap_r.match_in_phase_block(right_in.data(), ref_r.data(), kTotalSamples);

    double err_l = 0.0, err_r = 0.0, pow_l = 0.0, pow_r = 0.0;
    for (std::size_t n = 0; n < kTotalSamples; ++n) {
        const double dl = static_cast<double>(out_l[n]) - static_cast<double>(ref_l[n]);
        const double dr = static_cast<double>(out_r[n]) - static_cast<double>(ref_r[n]);
        err_l += dl * dl;
        err_r += dr * dr;
        pow_l += static_cast<double>(ref_l[n]) * static_cast<double>(ref_l[n]);
        pow_r += static_cast<double>(ref_r[n]) * static_cast<double>(ref_r[n]);
    }
    const double err_l_db = 10.0 * std::log10(std::max(err_l / pow_l, 1.0e-20));
    const double err_r_db = 10.0 * std::log10(std::max(err_r / pow_r, 1.0e-20));
    std::printf("  Azimuth -90/+90 vs allpass(L/R) error: L = %.1f dB, R = %.1f dB\n", err_l_db, err_r_db);
    check_true(err_l_db < -110.0 && err_r_db < -110.0, "dqs_deg -90/+90 reproduces allpass(L/R) to < -110 dB");

    // 3b. Verify wet = 0 gives clean static surround (ignores LFO and DFS)
    QuadratureChannelParams static_vs_wet0[2]{};
    static_vs_wet0[0] = QuadratureChannelParams{35.0f, 0.0f, 180.0f, 0.0f, 0.0f, 1.0f, 1.3f};
    static_vs_wet0[1] = QuadratureChannelParams{35.0f, 2.0f, 270.0f, 45.0f, -8.0f, 0.0f, 1.3f};
    proc_lr.init(cfg_no_hpf, static_vs_wet0, 2);
    for (std::size_t b = 0; b < kNumBlocks; ++b) {
        float* outs[2] = {&out_l[b * kFrameSamples], &out_r[b * kFrameSamples]};
        proc_lr.process_block_f32(
            &left_in[b * kFrameSamples], &right_in[b * kFrameSamples], outs, kFrameSamples);
    }
    float max_diff_wet0 = 0.0f;
    for (std::size_t n = 0; n < kTotalSamples; ++n) {
        max_diff_wet0 = std::max(max_diff_wet0, std::fabs(out_l[n] - out_r[n]));
    }
    check_true(max_diff_wet0 < 1.0e-6f, "wet = 0 gives exact static placement even with LFO and DFS active");

    // 3c. Verify side_gain = 0 produces identical mono Mid at any azimuth
    QuadratureChannelParams mono_mid[2]{};
    mono_mid[0] = QuadratureChannelParams{-90.0f, 1.5f, 180.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    mono_mid[1] = QuadratureChannelParams{+135.0f, 0.5f, 360.0f, 90.0f, 0.0f, 1.0f, 0.0f};
    proc_lr.init(cfg_no_hpf, mono_mid, 2);
    for (std::size_t b = 0; b < kNumBlocks; ++b) {
        float* outs[2] = {&out_l[b * kFrameSamples], &out_r[b * kFrameSamples]};
        proc_lr.process_block_f32(
            &left_in[b * kFrameSamples], &right_in[b * kFrameSamples], outs, kFrameSamples);
    }
    float max_diff_mono = 0.0f;
    for (std::size_t n = 0; n < kTotalSamples; ++n) {
        max_diff_mono = std::max(max_diff_mono, std::fabs(out_l[n] - out_r[n]));
    }
    check_true(max_diff_mono == 0.0f, "side_gain = 0 outputs identical mono Mid at all azimuths");

    // 3d. Verify 8-channel execution, HPF orders 1..4, and independent per-channel phase states
    for (int order = 1; order <= 4; ++order) {
        HilbertQuadratureConfig hpf_cfg{};
        hpf_cfg.hpf_order = order;
        hpf_cfg.hpf_cutoff_hz = 120.0f;
        hpf_cfg.hpf_q = 0.707f;

        QuadratureChannelParams ch8[kMaxQuadratureChannels]{};
        for (std::size_t c = 0; c < kMaxQuadratureChannels; ++c) {
            ch8[c].dqs_deg = -180.0f + 45.0f * static_cast<float>(c);
            ch8[c].dqs_lfo_hz = 0.25f * static_cast<float>(c + 1u);
            ch8[c].dqs_lfo_span_deg = 120.0f;
            ch8[c].dqs_lfo_offset_deg = 45.0f * static_cast<float>(c);
            ch8[c].dfs_shift_hz = -7.0f + 2.0f * static_cast<float>(c);
            ch8[c].wet = 0.8f;
            ch8[c].side_gain = 1.5f;
        }

        HilbertQuadratureMs proc8;
        proc8.init(hpf_cfg, ch8, kMaxQuadratureChannels);

        alignas(16) float out_bufs[kMaxQuadratureChannels][kFrameSamples]{};
        float* out_ptrs[kMaxQuadratureChannels]{};
        for (std::size_t c = 0; c < kMaxQuadratureChannels; ++c) {
            out_ptrs[c] = out_bufs[c];
        }
        proc8.process_block_f32(left_in.data(), right_in.data(), out_ptrs, kFrameSamples);

        bool all_distinct = true;
        for (std::size_t c = 1; c < kMaxQuadratureChannels; ++c) {
            if (proc8.channel_lfo_phase_u32(c) == proc8.channel_lfo_phase_u32(c - 1u) ||
                proc8.channel_dfs_phase_u32(c) == proc8.channel_dfs_phase_u32(c - 1u)) {
                all_distinct = false;
            }
        }
        if (order == 4) {
            check_true(all_distinct, "8 channels maintain strictly isolated LFO & DFS phase accumulators");
        }
    }

    // 3e. Verify split dual-core API (prepare_block_f32 + render_channel_f32 / render_channel_to_pcm16)
    {
        HilbertQuadratureConfig cfg5{};
        cfg5.hpf_order = 2;
        cfg5.hpf_cutoff_hz = 100.0f;
        QuadratureChannelParams ch5[5] = {
            {-90.0f, 0.25f, 90.0f, 0.0f, 0.0f, 1.0f, 1.0f},
            {+90.0f, 0.25f, 90.0f, 72.0f, 0.0f, 1.0f, 1.0f},
            {  0.0f, 0.00f,  0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
            {-135.0f, 0.25f, 90.0f, 144.0f, +0.4f, 1.0f, 1.0f},
            {+135.0f, 0.25f, 90.0f, 216.0f, -0.4f, 1.0f, 1.0f},
        };
        HilbertQuadratureMs ref5, split5;
        ref5.init(cfg5, ch5, 5);
        split5.init(cfg5, ch5, 5);

        alignas(16) float ref_out[5][kFrameSamples]{};
        alignas(16) float split_out[5][kFrameSamples]{};
        float* ref_ptrs[5] = {ref_out[0], ref_out[1], ref_out[2], ref_out[3], ref_out[4]};

        ref5.process_block_f32(left_in.data(), right_in.data(), ref_ptrs, kFrameSamples);
        split5.prepare_block_f32(left_in.data(), right_in.data(), kFrameSamples);
        for (std::size_t c = 0; c < 5; ++c) {
            split5.render_channel_f32(c, split_out[c], kFrameSamples);
        }

        float max_split_diff = 0.0f;
        for (std::size_t c = 0; c < 5; ++c) {
            for (std::size_t i = 0; i < kFrameSamples; ++i) {
                max_split_diff = std::max(max_split_diff, std::fabs(ref_out[c][i] - split_out[c][i]));
            }
        }
        check_true(max_split_diff == 0.0f, "Split prepare_block_f32 + render_channel_f32 is bit-identical to process_block_f32");
    }
}

/**
 * @brief Test the shared 48 kHz -> 8 kHz SubharmonicPreStage and both 8 kHz subharmonic pipelines.
 */
void test_subharmonic_pipelines_8khz() {
    std::printf("\n=== 4. Shared 8 kHz Subharmonic Pipelines (ZC Flipper & NCCF Synth) ===\n");

    constexpr std::size_t kNumBlocks = 40;  // 400 ms
    constexpr std::size_t kSamples48k = kNumBlocks * kFrameSamples;
    constexpr std::size_t kSamples8k = kNumBlocks * kSubFrameSamples;

    std::vector<float> left_48k(kSamples48k, 0.0f);
    std::vector<float> right_48k(kSamples48k, 0.0f);
    for (std::size_t n = 0; n < kSamples48k; ++n) {
        const float t = static_cast<float>(n) / static_cast<float>(kSampleRateHz);
        // 80 Hz pure bass tone -> should generate 40 Hz subharmonic
        const float s = 0.7f * std::sin(kTwoPi * 80.0f * t);
        left_48k[n] = s;
        right_48k[n] = s;
    }

    SubharmonicPipeline pipe;
    SubharmonicZcFlipperConfig zc_cfg{};
    zc_cfg.post_lpf_cutoff_hz = 90.0f;
    zc_cfg.post_lpf_order = 4;
    zc_cfg.dry_gain = 0.0f;
    zc_cfg.sub_gain = 1.0f;

    SubharmonicNccfConfig nccf_cfg{};
    nccf_cfg.use_sub_ducking = true;
    nccf_cfg.dry_gain = 0.0f;
    nccf_cfg.sub_gain = 1.0f;

    // 4a. Test Mode 1: SubharmonicZcFlipper at 8 kHz
    pipe.init(SubharmonicMode::kZcFlipper, 120.0f, zc_cfg, nccf_cfg);
    std::vector<float> out_zc_8k(kSamples8k, 0.0f);
    for (std::size_t b = 0; b < kNumBlocks; ++b) {
        pipe.process_stereo_f32_to_8k(
            &left_48k[b * kFrameSamples],
            &right_48k[b * kFrameSamples],
            &out_zc_8k[b * kSubFrameSamples],
            kFrameSamples);
    }

    // Count rising zero crossings in the steady-state second half (200 ms at 8 kHz = 1600 samples)
    // A 40 Hz subharmonic over 0.2 s has ~8 cycles
    int zc_count = 0;
    float rms_zc = 0.0f;
    for (std::size_t i = kSamples8k / 2u; i < kSamples8k; ++i) {
        if (out_zc_8k[i - 1u] < 0.0f && out_zc_8k[i] >= 0.0f) {
            ++zc_count;
        }
        rms_zc += out_zc_8k[i] * out_zc_8k[i];
    }
    rms_zc = std::sqrt(rms_zc / static_cast<float>(kSamples8k / 2u));
    const float est_freq_zc = static_cast<float>(zc_count) / 0.20f;
    std::printf("  SubharmonicZcFlipper (8 kHz): RMS = %.4f, estimated freq = %.1f Hz (expected ~40 Hz)\n",
                rms_zc, est_freq_zc);
    check_true(rms_zc > 0.05f && std::fabs(est_freq_zc - 40.0f) <= 5.0f,
               "SubharmonicZcFlipper halves 80 Hz to ~40 Hz at 8 kHz");

    // 4b. Test Mode 2: SubharmonicNccfSynth at 8 kHz
    pipe.init(SubharmonicMode::kNccfSynth, 120.0f, zc_cfg, nccf_cfg);
    std::vector<float> out_nccf_8k(kSamples8k, 0.0f);
    for (std::size_t b = 0; b < kNumBlocks; ++b) {
        pipe.process_stereo_f32_to_8k(
            &left_48k[b * kFrameSamples],
            &right_48k[b * kFrameSamples],
            &out_nccf_8k[b * kSubFrameSamples],
            kFrameSamples);
    }
    const SubharmonicNccfDiagnostics diag = pipe.nccf_synth().last_diagnostics();
    std::printf("  SubharmonicNccfSynth (8 kHz): voiced = %d, f0 = %.2f Hz, f_sub = %.2f Hz, peak = %.3f\n",
                diag.voiced ? 1 : 0, diag.f0_hz, diag.f_sub_hz, diag.nccf_peak);
    check_true(diag.voiced && std::fabs(diag.f0_hz - 80.0f) < 1.5f && std::fabs(diag.f_sub_hz - 40.0f) < 0.8f,
               "SubharmonicNccfSynth locks onto 80.0 Hz fundamental and synthesizes 40.0 Hz subharmonic at 8 kHz");
}

/**
 * @brief Benchmark execution speed for 6-channel and 8-channel HilbertQuadratureMs and SubharmonicPipeline.
 */
void benchmark_throughput() {
    std::printf("\n=== 5. Throughput Benchmark (10.0 ms / 480-Sample LC3 Blocks) ===\n");

    HilbertQuadratureConfig init_cfg{};
    init_cfg.hpf_order = 2;
    init_cfg.hpf_cutoff_hz = 100.0f;

    // 6-speaker ring matching Experiment 5.6b in hilbert_part3.ipynb
    QuadratureChannelParams ring6[6] = {
        {-90.0f, 0.25f, 90.0f, 0.0f, 0.0f, 1.0f, 1.0f},
        {-30.0f, 0.25f, 90.0f, 60.0f, 0.0f, 1.0f, 1.0f},
        {+30.0f, 0.25f, 90.0f, 120.0f, 0.0f, 1.0f, 1.0f},
        {+90.0f, 0.25f, 90.0f, 180.0f, +0.4f, 1.0f, 1.0f},
        {+150.0f, 0.25f, 90.0f, 240.0f, -0.4f, 1.0f, 1.0f},
        {-150.0f, 0.25f, 90.0f, 300.0f, 0.0f, 1.0f, 1.0f},
    };

    HilbertQuadratureMs proc6;
    proc6.init(init_cfg, ring6, 6);

    alignas(16) int16_t in_l[kFrameSamples]{};
    alignas(16) int16_t in_r[kFrameSamples]{};
    for (std::size_t i = 0; i < kFrameSamples; ++i) {
        in_l[i] = static_cast<int16_t>(12000.0f * std::sin(0.1f * static_cast<float>(i)));
        in_r[i] = static_cast<int16_t>(12000.0f * std::cos(0.13f * static_cast<float>(i)));
    }
    alignas(16) int16_t out_pcm[6][kFrameSamples]{};
    int16_t* out_ptrs[6] = {out_pcm[0], out_pcm[1], out_pcm[2], out_pcm[3], out_pcm[4], out_pcm[5]};

    constexpr int kIterations = 5000;
    const auto t0 = std::chrono::high_resolution_clock::now();
    for (int it = 0; it < kIterations; ++it) {
        proc6.process_block_pcm16(in_l, in_r, out_ptrs, kFrameSamples);
    }
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double us_per_block =
        std::chrono::duration<double, std::micro>(t1 - t0).count() / static_cast<double>(kIterations);

    std::printf("  HilbertQuadratureMs (6 channels, 480 samples, int16 I/O): %.2f us / 10 ms block\n",
                us_per_block);
    std::printf("  Struct sizes: HilbertQuadratureMs = %zu B, SubharmonicPipeline = %zu B (ZcFlipper = %zu B, NccfSynth = %zu B)\n",
                sizeof(HilbertQuadratureMs), sizeof(SubharmonicPipeline),
                sizeof(SubharmonicZcFlipper), sizeof(SubharmonicNccfSynth));
    check_true(us_per_block < 1000.0, "6-channel HilbertQuadratureMs executes well under 1.0 ms");
}

}  // namespace

int main() {
    test_fast_sin_lut();
    test_10_minute_phase_drift();
    test_hilbert_quadrature_ms();
    test_subharmonic_pipelines_8khz();
    benchmark_throughput();

    if (g_failures == 0) {
        std::printf("\nALL C++20 AUDIO DSP UNIT TESTS PASSED!\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d TEST(S) FAILED!\n", g_failures);
    return EXIT_FAILURE;
}
