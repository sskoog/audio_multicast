#include "lc3_benchmark.hpp"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <numeric>

static const char* TAG = "LC3_BENCH";

#define BENCH_MAGIC 0x4C433342 // "LC3B"

struct ClipEntry {
    uint32_t sample_rate;
    uint32_t offset;
    uint32_t size_bytes;
    uint32_t sample_count;
};

struct PartitionHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t clip_count;
    uint32_t header_size;
    ClipEntry entries[8];
};

namespace Benchmark {

Lc3BenchmarkSuite::Lc3BenchmarkSuite(Codec::Lc3CodecEngine& codec)
    : m_codec(codec) {}

esp_err_t Lc3BenchmarkSuite::init() {
    m_initialized = true;
    return ESP_OK;
}

esp_err_t Lc3BenchmarkSuite::runSingleBenchmark(uint32_t sample_rate, uint32_t frame_duration_us, uint16_t octets_per_frame,
                                                uint32_t flash_offset, uint32_t total_samples, BenchmarkResult& out_result) {
    const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (!part) {
        ESP_LOGE(TAG, "Storage partition not found!");
        return ESP_ERR_NOT_FOUND;
    }

    // Reconfigure encoder first
    esp_err_t err = m_codec.reconfigureEncoder(sample_rate, octets_per_frame, frame_duration_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconfigure encoder for SR=%lu, Octets=%u, Dur=%lu: %d",
                 sample_rate, octets_per_frame, frame_duration_us, err);
        return err;
    }

    size_t pcm_samples_per_frame = m_codec.getEncoderRequiredPcmSamples();
    if (pcm_samples_per_frame == 0) {
        pcm_samples_per_frame = Codec::calculateRequiredPcmSamples(sample_rate, frame_duration_us);
    }
    if (pcm_samples_per_frame == 0) {
        ESP_LOGE(TAG, "Invalid samples per frame for SR=%lu, Dur=%lu", sample_rate, frame_duration_us);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t total_frames = total_samples / pcm_samples_per_frame;
    if (total_frames == 0) {
        ESP_LOGE(TAG, "Not enough samples for even 1 frame!");
        return ESP_ERR_INVALID_SIZE;
    }

    std::vector<uint32_t> durations_us;
    durations_us.reserve(total_frames);

    std::vector<int16_t> pcm_buf(pcm_samples_per_frame);
    uint8_t out_lc3_buf[512];
    size_t out_bytes = 0;

    size_t frame_bytes = pcm_samples_per_frame * sizeof(int16_t);
    uint32_t curr_flash_addr = flash_offset;

    // Warm-up pass (5 frames)
    for (int w = 0; w < 5 && (curr_flash_addr + frame_bytes) <= (flash_offset + total_samples * sizeof(int16_t)); ++w) {
        esp_partition_read(part, curr_flash_addr, pcm_buf.data(), frame_bytes);
        m_codec.encodeFrame(pcm_buf.data(), pcm_samples_per_frame, out_lc3_buf, sizeof(out_lc3_buf), &out_bytes);
        curr_flash_addr += frame_bytes;
    }

    // Benchmark loop
    curr_flash_addr = flash_offset;
    for (uint32_t f = 0; f < total_frames; ++f) {
        if ((curr_flash_addr + frame_bytes) > (flash_offset + total_samples * sizeof(int16_t))) {
            break;
        }

        esp_partition_read(part, curr_flash_addr, pcm_buf.data(), frame_bytes);
        curr_flash_addr += frame_bytes;

        int64_t t_start = esp_timer_get_time();
        err = m_codec.encodeFrame(pcm_buf.data(), pcm_samples_per_frame, out_lc3_buf, sizeof(out_lc3_buf), &out_bytes);
        int64_t t_dur = esp_timer_get_time() - t_start;

        if (err == ESP_OK) {
            durations_us.push_back(static_cast<uint32_t>(t_dur));
        }
    }

    if (durations_us.empty()) {
        ESP_LOGE(TAG, "No successful encoded frames!");
        return ESP_FAIL;
    }

    // Calculate Statistics
    std::sort(durations_us.begin(), durations_us.end());

    uint64_t sum_us = std::accumulate(durations_us.begin(), durations_us.end(), 0ULL);
    size_t n = durations_us.size();

    out_result.sample_rate = sample_rate;
    out_result.frame_duration_us = frame_duration_us;
    out_result.octets_per_frame = octets_per_frame;
    out_result.bitrate_bps = (octets_per_frame * 8000000ULL) / frame_duration_us;
    out_result.total_frames = n;
    out_result.min_ms = static_cast<float>(durations_us.front()) / 1000.0f;
    out_result.max_ms = static_cast<float>(durations_us.back()) / 1000.0f;
    out_result.avg_ms = (static_cast<float>(sum_us) / static_cast<float>(n)) / 1000.0f;
    out_result.median_ms = static_cast<float>(durations_us[n / 2]) / 1000.0f;
    out_result.p95_ms = static_cast<float>(durations_us[static_cast<size_t>(n * 0.95f)]) / 1000.0f;

    float frame_dur_ms = static_cast<float>(frame_duration_us) / 1000.0f;
    out_result.cpu_load_pct = (out_result.avg_ms / frame_dur_ms) * 100.0f;
    out_result.realtime_factor = frame_dur_ms / out_result.avg_ms;

    return ESP_OK;
}

void Lc3BenchmarkSuite::printResultsTable(const std::vector<BenchmarkResult>& results) {
    printf("\n\n");
    printf("+======================================================================================================================+\n");
    printf("|                             ESP32-C6 HARDWARE LC3 ENCODER BENCHMARK RESULTS (10-SECOND REAL MUSIC)                   |\n");
    printf("+======================================================================================================================+\n");
    printf("|  Sample Rate | Frame Dur | Frame Octets |  Bitrate  | Total Frames | Min (ms) | Avg (ms) | P95 (ms) | Max (ms) |  CPU %%  |  RT Factor |\n");
    printf("+--------------+-----------+--------------+-----------+--------------+----------+----------+----------+----------+---------+------------+\n");

    for (const auto& r : results) {
        printf("| %6.1f kHz  | %4.1f ms  |   %3u B/fr   | %3lu kbps |   %5lu fr   |  %5.2f   |  %5.2f   |  %5.2f   |  %5.2f   |  %5.1f%% |   %5.2fx   |\n",
               r.sample_rate / 1000.0f,
               r.frame_duration_us / 1000.0f,
               r.octets_per_frame,
               r.bitrate_bps / 1000UL,
               (unsigned long)r.total_frames,
               r.min_ms,
               r.avg_ms,
               r.p95_ms,
               r.max_ms,
               r.cpu_load_pct,
               r.realtime_factor);
    }
    printf("+======================================================================================================================+\n\n");
}

void Lc3BenchmarkSuite::runAllBenchmarks() {
    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, " Starting Full LC3 Encoder Benchmark Suite on ESP32-C6  ");
    ESP_LOGI(TAG, "========================================================");

    // Disable Wi-Fi to eliminate RF interrupts and ensure 100% CPU dedication
    ESP_LOGI(TAG, "Stopping Wi-Fi radio during benchmark execution...");
    esp_wifi_stop();

    const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (!part) {
        ESP_LOGE(TAG, "Partition 'storage' not found! Make sure benchmark clips are flashed to 0x310000.");
        return;
    }

    PartitionHeader hdr;
    esp_err_t err = esp_partition_read(part, 0, &hdr, sizeof(hdr));
    if (err != ESP_OK || hdr.magic != BENCH_MAGIC) {
        ESP_LOGE(TAG, "Invalid benchmark partition header (magic: 0x%08lX, expected 0x%08lX)",
                 (unsigned long)hdr.magic, (unsigned long)BENCH_MAGIC);
        return;
    }

    ESP_LOGI(TAG, "Found valid benchmark clips container: %lu clips present", (unsigned long)hdr.clip_count);

    std::vector<BenchmarkResult> all_results;

    // Test cases: For each clip, test 10.0 ms and 7.5 ms
    for (uint32_t i = 0; i < hdr.clip_count; ++i) {
        const auto& entry = hdr.entries[i];
        ESP_LOGI(TAG, "Testing clip: SR = %lu Hz, Samples = %lu, Offset = 0x%lX",
                 entry.sample_rate, entry.sample_count, entry.offset);

        // Standard bitrates according to LC3 / LE Audio specs
        uint16_t octets_10ms = 80;
        uint16_t octets_75ms = 60;
        if (entry.sample_rate == 48000) { octets_10ms = 80; octets_75ms = 60; }
        else if (entry.sample_rate == 44100) { octets_10ms = 80; octets_75ms = 60; }
        else if (entry.sample_rate == 32000) { octets_10ms = 60; octets_75ms = 45; }
        else if (entry.sample_rate == 24000) { octets_10ms = 45; octets_75ms = 35; }
        else if (entry.sample_rate == 16000) { octets_10ms = 30; octets_75ms = 23; }
        else if (entry.sample_rate == 8000)  { octets_10ms = 20; octets_75ms = 20; }

        // Test 10.0 ms frame duration
        BenchmarkResult res10;
        if (runSingleBenchmark(entry.sample_rate, 10000, octets_10ms, entry.offset, entry.sample_count, res10) == ESP_OK) {
            all_results.push_back(res10);
            ESP_LOGI(TAG, "  -> 10.0ms: Avg = %.2f ms, P95 = %.2f ms, CPU = %.1f%%",
                     res10.avg_ms, res10.p95_ms, res10.cpu_load_pct);
        }

        // Test 7.5 ms frame duration
        BenchmarkResult res75;
        if (runSingleBenchmark(entry.sample_rate, 7500, octets_75ms, entry.offset, entry.sample_count, res75) == ESP_OK) {
            all_results.push_back(res75);
            ESP_LOGI(TAG, "  ->  7.5ms: Avg = %.2f ms, P95 = %.2f ms, CPU = %.1f%%",
                     res75.avg_ms, res75.p95_ms, res75.cpu_load_pct);
        }
    }

    // === 3 DEFINED PROJECT REFERENCE ENCODING LEVELS ===
    ESP_LOGI(TAG, ">>> RUNNING 3 PROJECT REFERENCE ENCODING LEVELS");
    for (uint32_t i = 0; i < hdr.clip_count; ++i) {
        const auto& entry = hdr.entries[i];
        if (entry.sample_rate == 48000) {
            // HIGH: 48.0 kHz | 7.5 ms | 120 B | 128 kbps
            BenchmarkResult resHigh;
            if (runSingleBenchmark(48000, 7500, 120, entry.offset, entry.sample_count, resHigh) == ESP_OK) {
                all_results.push_back(resHigh);
                ESP_LOGI(TAG, "  [REF HIGH]   48.0 kHz | 7.5 ms | 120 B (128 kbps) -> Avg: %.2f ms, P95: %.2f ms, CPU: %.1f%%",
                         resHigh.avg_ms, resHigh.p95_ms, resHigh.cpu_load_pct);
            }
        } else if (entry.sample_rate == 32000) {
            // MEDIUM: 32.0 kHz | 10.0 ms | 80 B | 64 kbps
            BenchmarkResult resMed;
            if (runSingleBenchmark(32000, 10000, 80, entry.offset, entry.sample_count, resMed) == ESP_OK) {
                all_results.push_back(resMed);
                ESP_LOGI(TAG, "  [REF MEDIUM] 32.0 kHz | 10.0 ms | 80 B (64 kbps)  -> Avg: %.2f ms, P95: %.2f ms, CPU: %.1f%%",
                         resMed.avg_ms, resMed.p95_ms, resMed.cpu_load_pct);
            }
        } else if (entry.sample_rate == 16000) {
            // LOW: 16.0 kHz | 10.0 ms | 40 B | 32 kbps
            BenchmarkResult resLow;
            if (runSingleBenchmark(16000, 10000, 40, entry.offset, entry.sample_count, resLow) == ESP_OK) {
                all_results.push_back(resLow);
                ESP_LOGI(TAG, "  [REF LOW]    16.0 kHz | 10.0 ms | 40 B (32 kbps)  -> Avg: %.2f ms, P95: %.2f ms, CPU: %.1f%%",
                         resLow.avg_ms, resLow.p95_ms, resLow.cpu_load_pct);
            }
        }
    }

    printResultsTable(all_results);
}

} // namespace Benchmark
