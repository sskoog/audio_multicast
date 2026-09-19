#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

enum class CodecEngineMode {
    LIBLC3_LTPF_ON,     // Google liblc3 (Hardware FPU) with LTPF active
    LIBLC3_LTPF_OFF,    // Google liblc3 (Hardware FPU) with LTPF disabled (lc3_encoder_disable_ltpf)
    FIXED_POINT_ESP     // Espressif fixed-point LC3 (libesp_audio_codec)
};

struct BenchmarkQualityProfile {
    const char* name;           // "VHQ", "HQ", "MQ", "LW", "SUB1", "SUB2"
    uint32_t sample_rate_hz;    // 48000, 32000, 24000, 8000
    uint32_t octets;            // 160, 120, 80, 60
};

struct BenchmarkRunResult {
    const char* profile_name;
    CodecEngineMode mode;
    bool is_iram;
    uint32_t sample_rate_hz;
    float frame_duration_ms;
    uint32_t frame_octets;
    uint32_t bitrate_bps;
    uint32_t num_frames;

    // Encode metrics
    uint32_t enc_total_us;
    float enc_avg_us;
    float enc_avg_ms;
    float enc_cpu_pct;
    float enc_rt_factor;

    // Decode metrics
    uint32_t dec_total_us;
    float dec_avg_us;
    float dec_avg_ms;
    float dec_cpu_pct;
    float dec_rt_factor;
};

class Lc3BenchmarkRunner {
public:
    Lc3BenchmarkRunner();
    ~Lc3BenchmarkRunner();

    bool init();
    void runFullSuite();
    void printResultsTable();
    void printCsvData();
    const std::vector<BenchmarkRunResult>& getResults() const { return m_results; }

private:
    bool runSinglePass(
        const char* profile_name,
        CodecEngineMode mode,
        uint32_t sample_rate_hz,
        float frame_duration_ms,
        uint32_t octets,
        BenchmarkRunResult& out_res
    );

    bool readPcmClip(uint32_t sample_rate, const int16_t*& out_samples, uint32_t& out_count);

    const void* m_mmap_handle = nullptr;
    const uint8_t* m_flash_base = nullptr;
    std::vector<BenchmarkRunResult> m_results;
    bool m_is_iram_detected = false;

    // Dynamically allocated buffers in internal RAM
    int16_t* m_pcm_in_ram = nullptr;
    uint8_t* m_encoded_frames_ram = nullptr;
};
