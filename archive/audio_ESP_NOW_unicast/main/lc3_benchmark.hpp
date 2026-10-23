#pragma once

#include "lc3_codec.hpp"
#include "esp_err.h"
#include <cstdint>
#include <vector>

namespace Benchmark {

struct BenchmarkResult {
    uint32_t sample_rate;
    uint32_t frame_duration_us;
    uint16_t octets_per_frame;
    uint32_t bitrate_bps;
    uint32_t total_frames;
    float    min_ms;
    float    avg_ms;
    float    median_ms;
    float    p95_ms;
    float    max_ms;
    float    cpu_load_pct;
    float    realtime_factor; // frame_duration_ms / avg_ms (> 1.0 means faster than real-time)
};

class Lc3BenchmarkSuite {
public:
    explicit Lc3BenchmarkSuite(Codec::Lc3CodecEngine& codec);
    ~Lc3BenchmarkSuite() = default;

    esp_err_t init();
    void runAllBenchmarks();

private:
    esp_err_t runSingleBenchmark(uint32_t sample_rate, uint32_t frame_duration_us, uint16_t octets_per_frame,
                                 uint32_t flash_offset, uint32_t total_samples, BenchmarkResult& out_result);

    void printResultsTable(const std::vector<BenchmarkResult>& results);

    Codec::Lc3CodecEngine& m_codec;
    bool                   m_initialized = false;
};

} // namespace Benchmark
