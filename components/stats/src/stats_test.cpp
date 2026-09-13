#include "stats_test.hpp"
#include "stats.hpp"
#include "esp_log.h"
#include <cmath>
#include <cassert>

static const char* TAG = "StatsTest";

extern "C" bool run_stats_self_test(void) {
    ESP_LOGI(TAG, "=== Running stats Component Self-Test Suite ===");

    // -------------------------------------------------------------
    // 1. Counter Tests (uint16, uint32, uint64)
    // -------------------------------------------------------------
    ESP_LOGI(TAG, "[1/4] Testing Counter templates...");
    {
        stats::Counter16 c16;
        if (c16.getValue() != 0) return false;
        c16.increment(5);
        if (c16.getValue() != 5) return false;
        c16 += 10;
        if (c16.getValue() != 15) return false;
        uint16_t prev16 = c16.getAndReset();
        if (prev16 != 15 || c16.getValue() != 0) return false;

        stats::Counter32 c32;
        c32.increment(1000);
        ++c32;
        if (c32.getValue() != 1001) return false;
        c32.reset();
        if (c32.getValue() != 0) return false;

        stats::Counter64 c64;
        c64.setValue(0x100000000ULL);
        c64.increment(0xFF);
        if (c64.getValue() != 0x1000000FFULL) return false;
    }
    ESP_LOGI(TAG, "  Counter tests PASSED");

    // -------------------------------------------------------------
    // 2. EMA Filter Tests
    // -------------------------------------------------------------
    ESP_LOGI(TAG, "[2/4] Testing EmaFilter...");
    {
        stats::EmaFilter<float> ema(0.5f);
        if (ema.isInitialized()) return false;

        // First sample should set state immediately (no ramp from 0)
        ema.update(10.0f);
        if (!ema.isInitialized()) return false;
        if (std::fabs(ema.getValue() - 10.0f) > 0.001f) return false;

        // Second sample: 0.5 * 20.0 + 0.5 * 10.0 = 15.0
        ema.update(20.0f);
        if (std::fabs(ema.getValue() - 15.0f) > 0.001f) return false;

        // Third sample: 0.5 * 30.0 + 0.5 * 15.0 = 22.5
        ema.update(30.0f);
        if (std::fabs(ema.getValue() - 22.5f) > 0.001f) return false;

        ema.reset();
        if (ema.isInitialized()) return false;
    }
    ESP_LOGI(TAG, "  EmaFilter tests PASSED");

    // -------------------------------------------------------------
    // 3. StatsRingBuffer Tests
    // -------------------------------------------------------------
    ESP_LOGI(TAG, "[3/4] Testing StatsRingBuffer...");
    {
        stats::StatsRingBuffer<float, 5> rb;
        if (!rb.isEmpty()) return false;
        if (rb.getCount() != 0) return false;
        if (rb.getCapacity() != 5) return false;

        // Push: [1, 2, 3, 4, 5]
        rb.push(1.0f);
        rb.push(2.0f);
        rb.push(3.0f);
        rb.push(4.0f);
        rb.push(5.0f);

        if (!rb.isFull()) return false;
        if (rb.getCount() != 5) return false;

        // Mean: (1+2+3+4+5)/5 = 3.0
        if (std::fabs(rb.getAvg() - 3.0f) > 0.001f) return false;

        // Odd median of sorted [1,2,3,4,5] = 3.0
        if (std::fabs(rb.getMedian() - 3.0f) > 0.001f) return false;

        // Min, Max, Range
        if (std::fabs(rb.getMin() - 1.0f) > 0.001f) return false;
        if (std::fabs(rb.getMax() - 5.0f) > 0.001f) return false;
        if (std::fabs(rb.getRange() - 4.0f) > 0.001f) return false;

        // Partial window: most recent 2 samples are [4, 5] -> avg = 4.5
        if (std::fabs(rb.getAvg(2) - 4.5f) > 0.001f) return false;
        if (std::fabs(rb.getRange(2) - 1.0f) > 0.001f) return false;

        // Wraparound test: push 6.0f -> buffer contains [2, 3, 4, 5, 6]
        rb.push(6.0f);
        if (rb.getCount() != 5) return false;
        if (std::fabs(rb.getMin() - 2.0f) > 0.001f) return false;
        if (std::fabs(rb.getMax() - 6.0f) > 0.001f) return false;
        if (std::fabs(rb.getAvg() - 4.0f) > 0.001f) return false;
        if (std::fabs(rb.getMedian() - 4.0f) > 0.001f) return false;

        // Even median test with 4 samples: [10, 20, 30, 40]
        stats::StatsRingBuffer<int32_t, 4> rb_int;
        rb_int.push(10);
        rb_int.push(20);
        rb_int.push(30);
        rb_int.push(40);
        // Even median: (20 + 30) / 2 = 25.0
        if (std::fabs(rb_int.getMedian() - 25.0f) > 0.001f) return false;

        // Standard deviation: [10, 20, 30, 40], mean = 25
        // diffs^2 = 225 + 25 + 25 + 225 = 500, sample std = sqrt(500/3) = 12.9099
        if (std::fabs(rb_int.getStd() - 12.9099f) > 0.05f) return false;

        // RMS test: [2, -2, 2, -2] -> squares = 4 -> RMS = 2.0
        stats::StatsRingBuffer<float, 4> rb_rms;
        rb_rms.push(2.0f);
        rb_rms.push(-2.0f);
        rb_rms.push(2.0f);
        rb_rms.push(-2.0f);
        if (std::fabs(rb_rms.getRMS() - 2.0f) > 0.001f) return false;

        // resetTo test
        rb_rms.resetTo(100.0f);
        if (rb_rms.getCount() != 4) return false;
        if (std::fabs(rb_rms.getAvg() - 100.0f) > 0.001f) return false;
        if (std::fabs(rb_rms.getRange() - 0.0f) > 0.001f) return false;
    }
    ESP_LOGI(TAG, "  StatsRingBuffer tests PASSED");

    // -------------------------------------------------------------
    // 4. Domain Trackers
    // -------------------------------------------------------------
    ESP_LOGI(TAG, "[4/4] Testing domain-specific helpers...");
    {
        // HealthCounters
        stats::HealthCounters health;
        health.dma_udr.increment(3);
        health.plc_count.increment(7);
        auto snap = health.getDeltasAndReset();
        if (snap.dma_udr != 3 || snap.plc_count != 7 || snap.fifo_ovf != 0) return false;
        if (health.dma_udr.getValue() != 0) return false;

        // TimeSyncTracker
        stats::TimeSyncTracker sync(0.1f);
        sync.update(1000);  // 1.0 ms
        sync.update(2000);  // 2.0 ms
        sync.update(3000);  // 3.0 ms
        if (std::fabs(sync.getMedianOffsetMs(3) - 2.0f) > 0.01f) return false;
        if (std::fabs(sync.getJitterRangeMs(3) - 2.0f) > 0.01f) return false; // 3.0 - 1.0 = 2.0ms

        // AudioVolumeTracker: Mono & Stereo Tests
        // 1) Test Mono instantaneous & 1-second windowed (100 elements @ 10 ms)
        stats::AudioVolumeTrackerMono mono_audio;
        int16_t silent_frame[100] = {0};
        mono_audio.processFrame(silent_frame, 100);
        if (mono_audio.getInstantaneousPeak() != 0) return false;
        if (mono_audio.getInstantaneousRms() != 0) return false;
        if (mono_audio.getInstantaneousPeakDbfs() > -90.0f) return false;
        if (mono_audio.getInstantaneousRmsDbfs() > -90.0f) return false;

        // Full scale square wave frame (+32767, -32767)
        int16_t full_scale[4] = {32767, -32767, 32767, -32767};
        mono_audio.processFrame(full_scale, 4);
        if (mono_audio.getInstantaneousPeak() != 32767) return false;
        if (std::fabs(mono_audio.getInstantaneousPeakDbfs() - 0.0f) > 0.01f) return false; // 0 dBFS
        if (mono_audio.getInstantaneousRms() != 32767) return false;
        if (std::fabs(mono_audio.getInstantaneousRmsDbfs() - 0.0f) > 0.02f) return false;

        // Simulate 100 frames (1 second @ 10 ms) with half-scale DC: 16384 (~ -6.02 dBFS)
        int16_t half_scale[10] = {16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384, 16384};
        for (int f = 0; f < 100; ++f) {
            mono_audio.processFrame(half_scale, 10);
        }
        if (mono_audio.getWindowedPeak() != 16384) return false;
        if (std::fabs(mono_audio.getWindowedPeakDbfs() - (-6.0206f)) > 0.1f) return false;
        if (mono_audio.getWindowedRms() != 16384) return false;
        if (std::fabs(mono_audio.getWindowedRmsDbfs() - (-6.0206f)) > 0.1f) return false;

        // 2) Test Stereo interleaved channel separation
        stats::AudioVolumeTrackerStereo stereo_audio;
        // Interleaved L/R: Left = full scale (32767), Right = silence (0)
        int16_t stereo_frame[4] = {32767, 0, -32767, 0};
        stereo_audio.processFrame(stereo_frame, 4);
        if (stereo_audio.getInstantaneousPeak(0) != 32767) return false;
        if (stereo_audio.getInstantaneousPeak(1) != 0) return false;
        if (std::fabs(stereo_audio.getInstantaneousPeakDbfs(0) - 0.0f) > 0.01f) return false;
        if (stereo_audio.getInstantaneousPeakDbfs(1) > -90.0f) return false;
    }
    ESP_LOGI(TAG, "  Domain helper tests PASSED");

    ESP_LOGI(TAG, "=== ALL STATS COMPONENT TESTS PASSED SUCCESSFULLY! ===");
    return true;
}
