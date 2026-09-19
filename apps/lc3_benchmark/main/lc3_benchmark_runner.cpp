#include "lc3_benchmark_runner.hpp"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>

// Espressif fixed-point LC3 codec
#include "esp_lc3_enc.h"
#include "esp_lc3_dec.h"

// Google floating-point LC3 codec with hardware FPU
#include "lc3.h"

static const char* TAG = "LC3_BENCH";

#define BENCH_MAGIC 0x4C433342
#define NUM_BENCHMARK_FRAMES 100
#define NUM_PCM_RAM_FRAMES 25
#define MAX_FRAME_OCTETS 256
#define MAX_FRAME_SAMPLES 480

struct ClipHeaderEntry {
    uint32_t sample_rate;
    uint32_t offset_bytes;
    uint32_t size_bytes;
    uint32_t sample_count;
};

struct BinaryStorageHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t clip_count;
    uint32_t header_size;
    ClipHeaderEntry entries[16];
};

Lc3BenchmarkRunner::Lc3BenchmarkRunner() {}

Lc3BenchmarkRunner::~Lc3BenchmarkRunner() {
    if (m_mmap_handle) {
        esp_partition_munmap((esp_partition_mmap_handle_t)(uintptr_t)m_mmap_handle);
        m_mmap_handle = nullptr;
    }
    if (m_pcm_in_ram) {
        free(m_pcm_in_ram);
        m_pcm_in_ram = nullptr;
    }
    if (m_encoded_frames_ram) {
        free(m_encoded_frames_ram);
        m_encoded_frames_ram = nullptr;
    }
}

bool Lc3BenchmarkRunner::init() {
    const esp_partition_t* part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    
    if (!part) {
        ESP_LOGE(TAG, "Storage partition 'storage' not found in partition table!");
        return false;
    }

    esp_partition_mmap_handle_t mmap_h;
    const void* mapped_ptr = nullptr;
    esp_err_t err = esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, &mapped_ptr, &mmap_h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to memory-map 'storage' partition: %s", esp_err_to_name(err));
        return false;
    }

    m_mmap_handle = (const void*)(uintptr_t)mmap_h;
    m_flash_base = (const uint8_t*)mapped_ptr;

    const BinaryStorageHeader* hdr = (const BinaryStorageHeader*)m_flash_base;
    if (hdr->magic != BENCH_MAGIC) {
        ESP_LOGE(TAG, "Invalid storage partition magic: 0x%08lX (Expected 0x%08X)", (unsigned long)hdr->magic, BENCH_MAGIC);
        return false;
    }

    // Allocate benchmark working buffers in internal RAM (25 frames of PCM + 100 encoded frames)
    size_t pcm_bytes = MAX_FRAME_SAMPLES * NUM_PCM_RAM_FRAMES * sizeof(int16_t);
    m_pcm_in_ram = (int16_t*)heap_caps_malloc(pcm_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!m_pcm_in_ram) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes for PCM RAM buffer", (unsigned)pcm_bytes);
        return false;
    }

    size_t enc_bytes = NUM_BENCHMARK_FRAMES * MAX_FRAME_OCTETS;
    m_encoded_frames_ram = (uint8_t*)heap_caps_malloc(enc_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!m_encoded_frames_ram) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes for encoded frames RAM buffer", (unsigned)enc_bytes);
        return false;
    }

    // Check code placement address of lc3_encode
    uintptr_t fn_addr = (uintptr_t)&lc3_encode;
    m_is_iram_detected = (fn_addr >= 0x40370000 && fn_addr < 0x403E0000);

    ESP_LOGI(TAG, "Storage mapped at %p (%lu clips). Code Placement: %s (&lc3_encode=%p)",
             m_flash_base, (unsigned long)hdr->clip_count,
             m_is_iram_detected ? "IRAM (Internal RAM)" : "SPI Flash (XIP Cache)",
             (void*)fn_addr);
    return true;
}

bool Lc3BenchmarkRunner::readPcmClip(uint32_t sample_rate, const int16_t*& out_samples, uint32_t& out_count) {
    if (!m_flash_base) return false;
    const BinaryStorageHeader* hdr = (const BinaryStorageHeader*)m_flash_base;
    for (uint32_t i = 0; i < hdr->clip_count; ++i) {
        if (hdr->entries[i].sample_rate == sample_rate) {
            out_samples = (const int16_t*)(m_flash_base + hdr->entries[i].offset_bytes);
            out_count = hdr->entries[i].sample_count;
            return true;
        }
    }
    return false;
}

bool Lc3BenchmarkRunner::runSinglePass(
    const char* profile_name,
    CodecEngineMode mode,
    uint32_t sample_rate_hz,
    float frame_duration_ms,
    uint32_t octets,
    BenchmarkRunResult& out_res)
{
    const int16_t* pcm_flash = nullptr;
    uint32_t total_samples = 0;
    if (!readPcmClip(sample_rate_hz, pcm_flash, total_samples)) {
        ESP_LOGE(TAG, "No PCM clip found for sample rate %lu Hz", (unsigned long)sample_rate_hz);
        return false;
    }

    int dt_us = (int)(frame_duration_ms * 1000.0f + 0.5f);
    uint32_t frame_samples = (sample_rate_hz * dt_us) / 1000000;
    uint32_t required_pcm_samples = frame_samples * NUM_PCM_RAM_FRAMES;

    if (total_samples < required_pcm_samples) {
        ESP_LOGE(TAG, "PCM clip only has %lu samples, need %lu for %d RAM frames",
                 (unsigned long)total_samples, (unsigned long)required_pcm_samples, NUM_PCM_RAM_FRAMES);
        return false;
    }

    // 1. Read raw mono audio from flash to internal RAM
    std::memcpy(m_pcm_in_ram, pcm_flash, required_pcm_samples * sizeof(int16_t));


    void* esp_enc_handle = nullptr;
    void* esp_dec_handle = nullptr;
    lc3_encoder_t google_encoder = nullptr;
    lc3_decoder_t google_decoder = nullptr;
    void* google_enc_mem = nullptr;
    void* google_dec_mem = nullptr;

    uint32_t bitrate_bps = (uint32_t)((float)(octets * 8) * (1000.0f / frame_duration_ms));

    // Initialize Encoder
    if (mode == CodecEngineMode::FIXED_POINT_ESP) {
        esp_lc3_enc_config_t cfg = {};
        cfg.sample_rate = sample_rate_hz;
        cfg.bits_per_sample = 16;
        cfg.channel = 1;
        cfg.frame_dms = (uint8_t)(frame_duration_ms * 10.0f + 0.5f);
        cfg.nbyte = (uint16_t)octets;
        cfg.len_prefixed = false;

        esp_audio_err_t err = esp_lc3_enc_open(&cfg, sizeof(cfg), &esp_enc_handle);
        if (err != ESP_AUDIO_ERR_OK || !esp_enc_handle) {
            ESP_LOGE(TAG, "Failed to open esp_lc3 encoder: %d", err);
            return false;
        }
    } else {
        unsigned mem_size = lc3_encoder_size(dt_us, sample_rate_hz);
        google_enc_mem = heap_caps_malloc(mem_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!google_enc_mem) return false;
        google_encoder = lc3_setup_encoder(dt_us, sample_rate_hz, 0, google_enc_mem);
        if (!google_encoder) {
            free(google_enc_mem);
            return false;
        }
        if (mode == CodecEngineMode::LIBLC3_LTPF_OFF) {
            lc3_encoder_disable_ltpf(google_encoder);
        }
    }

    // Warm-up encode (2 frames)
    for (int w = 0; w < 2; ++w) {
        const int16_t* in_pcm = m_pcm_in_ram + (w * frame_samples);
        uint8_t* out_ptr = m_encoded_frames_ram + (w * MAX_FRAME_OCTETS);
        if (mode == CodecEngineMode::FIXED_POINT_ESP) {
            esp_audio_enc_in_frame_t in_f = {};
            in_f.buffer = (uint8_t*)in_pcm;
            in_f.len = frame_samples * sizeof(int16_t);
            esp_audio_enc_out_frame_t out_f = {};
            out_f.buffer = out_ptr;
            out_f.len = MAX_FRAME_OCTETS;
            esp_lc3_enc_process(esp_enc_handle, &in_f, &out_f);
        } else {
            lc3_encode(google_encoder, LC3_PCM_FORMAT_S16, in_pcm, 1, octets, out_ptr);
        }
    }

    // 2. Record encode_start time
    int64_t t_enc_start = esp_timer_get_time();

    // 3. Encode 100 LC3-frames, store results in RAM
    for (int f = 0; f < NUM_BENCHMARK_FRAMES; ++f) {
        int pcm_idx = f % NUM_PCM_RAM_FRAMES;
        const int16_t* in_pcm = m_pcm_in_ram + (pcm_idx * frame_samples);
        uint8_t* out_ptr = m_encoded_frames_ram + (f * MAX_FRAME_OCTETS);
        if (mode == CodecEngineMode::FIXED_POINT_ESP) {
            esp_audio_enc_in_frame_t in_f = {};
            in_f.buffer = (uint8_t*)in_pcm;
            in_f.len = frame_samples * sizeof(int16_t);
            esp_audio_enc_out_frame_t out_f = {};
            out_f.buffer = out_ptr;
            out_f.len = MAX_FRAME_OCTETS;
            esp_lc3_enc_process(esp_enc_handle, &in_f, &out_f);
        } else {
            lc3_encode(google_encoder, LC3_PCM_FORMAT_S16, in_pcm, 1, octets, out_ptr);
        }
    }

    // 4. Save encode_duration_us
    int64_t t_enc_end = esp_timer_get_time();
    uint32_t enc_duration_us = (uint32_t)(t_enc_end - t_enc_start);

    // Clean up encoder
    if (esp_enc_handle) esp_lc3_enc_close(esp_enc_handle);
    if (google_enc_mem) free(google_enc_mem);

    // Initialize Decoder
    if (mode == CodecEngineMode::FIXED_POINT_ESP) {
        esp_lc3_dec_cfg_t dec_cfg = {};
        dec_cfg.sample_rate = sample_rate_hz;
        dec_cfg.bits_per_sample = 16;
        dec_cfg.channel = 1;
        dec_cfg.frame_dms = (uint8_t)(frame_duration_ms * 10.0f + 0.5f);
        dec_cfg.nbyte = (uint16_t)octets;
        dec_cfg.is_cbr = true;
        dec_cfg.len_prefixed = false;
        dec_cfg.enable_plc = true;

        esp_audio_err_t err = esp_lc3_dec_open(&dec_cfg, sizeof(dec_cfg), &esp_dec_handle);
        if (err != ESP_AUDIO_ERR_OK || !esp_dec_handle) {
            ESP_LOGE(TAG, "Failed to open esp_lc3 decoder: %d", err);
            return false;
        }
    } else {
        unsigned dec_size = lc3_decoder_size(dt_us, sample_rate_hz);
        google_dec_mem = heap_caps_malloc(dec_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!google_dec_mem) return false;
        google_decoder = lc3_setup_decoder(dt_us, sample_rate_hz, 0, google_dec_mem);
        if (!google_decoder) {
            free(google_dec_mem);
            return false;
        }
    }

    int16_t scratch_pcm[MAX_FRAME_SAMPLES];

    // Warm-up decode (2 frames)
    for (int w = 0; w < 2; ++w) {
        uint8_t* in_ptr = m_encoded_frames_ram + (w * MAX_FRAME_OCTETS);
        if (mode == CodecEngineMode::FIXED_POINT_ESP) {
            esp_audio_dec_in_raw_t in_raw = {};
            in_raw.buffer = in_ptr;
            in_raw.len = octets;
            in_raw.frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE;
            esp_audio_dec_out_frame_t out_f = {};
            out_f.buffer = (uint8_t*)scratch_pcm;
            out_f.len = sizeof(scratch_pcm);
            esp_audio_dec_info_t dec_info;
            esp_lc3_dec_decode(esp_dec_handle, &in_raw, &out_f, &dec_info);
        } else {
            lc3_decode(google_decoder, in_ptr, octets, LC3_PCM_FORMAT_S16, scratch_pcm, 1);
        }
    }

    // 5. Record decode_start time
    int64_t t_dec_start = esp_timer_get_time();

    // 6. Decode 100 LC3-frames from RAM. PCM-output is discarded.
    for (int f = 0; f < NUM_BENCHMARK_FRAMES; ++f) {
        uint8_t* in_ptr = m_encoded_frames_ram + (f * MAX_FRAME_OCTETS);
        if (mode == CodecEngineMode::FIXED_POINT_ESP) {
            esp_audio_dec_in_raw_t in_raw = {};
            in_raw.buffer = in_ptr;
            in_raw.len = octets;
            in_raw.frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE;
            esp_audio_dec_out_frame_t out_f = {};
            out_f.buffer = (uint8_t*)scratch_pcm;
            out_f.len = sizeof(scratch_pcm);
            esp_audio_dec_info_t dec_info;
            esp_lc3_dec_decode(esp_dec_handle, &in_raw, &out_f, &dec_info);
        } else {
            lc3_decode(google_decoder, in_ptr, octets, LC3_PCM_FORMAT_S16, scratch_pcm, 1);
        }
    }

    // 7. Save decode_duration_us
    int64_t t_dec_end = esp_timer_get_time();
    uint32_t dec_duration_us = (uint32_t)(t_dec_end - t_dec_start);

    // Clean up decoder
    if (esp_dec_handle) esp_lc3_dec_close(esp_dec_handle);
    if (google_dec_mem) free(google_dec_mem);

    // Fill Output Result
    out_res.profile_name = profile_name;
    out_res.mode = mode;
    out_res.is_iram = m_is_iram_detected;
    out_res.sample_rate_hz = sample_rate_hz;
    out_res.frame_duration_ms = frame_duration_ms;
    out_res.frame_octets = octets;
    out_res.bitrate_bps = bitrate_bps;
    out_res.num_frames = NUM_BENCHMARK_FRAMES;

    out_res.enc_total_us = enc_duration_us;
    out_res.enc_avg_us = (float)enc_duration_us / (float)NUM_BENCHMARK_FRAMES;
    out_res.enc_avg_ms = out_res.enc_avg_us / 1000.0f;
    out_res.enc_cpu_pct = (out_res.enc_avg_ms / frame_duration_ms) * 100.0f;
    out_res.enc_rt_factor = out_res.enc_avg_ms / frame_duration_ms; // CPU-time / frame_duration

    out_res.dec_total_us = dec_duration_us;
    out_res.dec_avg_us = (float)dec_duration_us / (float)NUM_BENCHMARK_FRAMES;
    out_res.dec_avg_ms = out_res.dec_avg_us / 1000.0f;
    out_res.dec_cpu_pct = (out_res.dec_avg_ms / frame_duration_ms) * 100.0f;
    out_res.dec_rt_factor = out_res.dec_avg_ms / frame_duration_ms; // CPU-time / frame_duration

    return true;
}

void Lc3BenchmarkRunner::runFullSuite() {
    m_results.clear();

    ESP_LOGI(TAG, "===============================================================================");
    ESP_LOGI(TAG, "STARTING ESP32-S3 LC3 ENCODE & DECODE BENCHMARK SUITE REV 2 (240 MHz, Core %d)", xPortGetCoreID());
    ESP_LOGI(TAG, "Code Placement: %s (&lc3_encode=%p)",
             m_is_iram_detected ? "IRAM (Internal RAM)" : "SPI Flash (XIP Cache)",
             (void*)&lc3_encode);
    ESP_LOGI(TAG, "===============================================================================");

    static const BenchmarkQualityProfile s_profiles[] = {
        {"VHQ",  48000, 160},
        {"HQ",   48000, 120},
        {"MQ",   32000,  80},
        {"LW",   24000,  60},
        {"SUB1",  8000,  80},
        {"SUB2",  8000, 120}
    };

    static const float s_durations[] = {10.0f, 7.5f};

    static const CodecEngineMode s_modes[] = {
        CodecEngineMode::LIBLC3_LTPF_ON,
        CodecEngineMode::LIBLC3_LTPF_OFF,
        CodecEngineMode::FIXED_POINT_ESP
    };

    for (const auto& mode : s_modes) {
        const char* mode_name = (mode == CodecEngineMode::LIBLC3_LTPF_ON)  ? "Google liblc3 (Float+FPU, LTPF ON)" :
                                (mode == CodecEngineMode::LIBLC3_LTPF_OFF) ? "Google liblc3 (Float+FPU, LTPF OFF)" :
                                                                             "Espressif Fixed-Point (esp_audio_codec)";
        ESP_LOGI(TAG, ">>> SERIES: %s", mode_name);

        for (float dur : s_durations) {
            for (const auto& prof : s_profiles) {
                BenchmarkRunResult res = {};
                if (runSinglePass(prof.name, mode, prof.sample_rate_hz, dur, prof.octets, res)) {
                    m_results.push_back(res);
                    ESP_LOGI(TAG, "  [%4s] %5lu Hz | %4.1f ms | %3lu B (%3lu kbps) -> Enc: %5.2f ms (RT factor: %.3f) | Dec: %5.2f ms (RT factor: %.3f)",
                             res.profile_name, (unsigned long)res.sample_rate_hz, res.frame_duration_ms,
                             (unsigned long)res.frame_octets, (unsigned long)(res.bitrate_bps / 1000),
                             res.enc_avg_ms, res.enc_rt_factor, res.dec_avg_ms, res.dec_rt_factor);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    printResultsTable();
    printCsvData();
}

void Lc3BenchmarkRunner::printResultsTable() {
    printf("\n\n+====================================================================================================================================================+\n");
    printf("|                         ESP32-S3 LC3 CODEC BENCHMARK RESULTS REV 2 (240 MHz, Core %d, %s)                                  |\n",
           xPortGetCoreID(), m_is_iram_detected ? "IRAM PLACEMENT" : "FLASH PLACEMENT");
    printf("+====================================================================================================================================================+\n");
    printf("| Profile | Engine / Mode            | Rate (kHz)| Dur (ms)| Octets | Bitrate | Enc Time/Frame | Enc RT Factor | Dec Time/Frame | Dec RT Factor |\n");
    printf("+---------+--------------------------+-----------+---------+--------+---------+----------------+---------------+----------------+---------------+\n");

    for (const auto& r : m_results) {
        const char* mode_str = (r.mode == CodecEngineMode::LIBLC3_LTPF_ON)  ? "liblc3 (LTPF ON) " :
                               (r.mode == CodecEngineMode::LIBLC3_LTPF_OFF) ? "liblc3 (LTPF OFF)" :
                                                                              "esp_codec (FixP) ";
        printf("| %-7s | %-24s | %8.1f  | %6.1f  | %6lu | %4lu k |  %6.3f ms/fr   |   %6.3f      |  %6.3f ms/fr   |   %6.3f     |\n",
               r.profile_name,
               mode_str,
               r.sample_rate_hz / 1000.0f,
               r.frame_duration_ms,
               (unsigned long)r.frame_octets,
               (unsigned long)(r.bitrate_bps / 1000),
               r.enc_avg_ms,
               r.enc_rt_factor,
               r.dec_avg_ms,
               r.dec_rt_factor);
    }
    printf("+====================================================================================================================================================+\n\n");
}

void Lc3BenchmarkRunner::printCsvData() {
    printf("=== CSV START ===\n");
    printf("profile,engine,ltpf,placement,sample_rate,frame_duration_ms,octets,bitrate_bps,frames,enc_avg_us,enc_avg_ms,enc_rt_factor,dec_avg_us,dec_avg_ms,dec_rt_factor\n");
    for (const auto& r : m_results) {
        const char* eng_str = (r.mode == CodecEngineMode::FIXED_POINT_ESP) ? "esp_audio_codec" : "liblc3";
        const char* ltpf_str = (r.mode == CodecEngineMode::LIBLC3_LTPF_ON) ? "ON" :
                               (r.mode == CodecEngineMode::LIBLC3_LTPF_OFF) ? "OFF" : "N/A";
        const char* place_str = r.is_iram ? "IRAM" : "FLASH";

        printf("%s,%s,%s,%s,%lu,%.1f,%lu,%lu,%lu,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f\n",
               r.profile_name, eng_str, ltpf_str, place_str,
               (unsigned long)r.sample_rate_hz, r.frame_duration_ms, (unsigned long)r.frame_octets,
               (unsigned long)r.bitrate_bps, (unsigned long)r.num_frames,
               r.enc_avg_us, r.enc_avg_ms, r.enc_rt_factor,
               r.dec_avg_us, r.dec_avg_ms, r.dec_rt_factor);
    }
    printf("=== CSV END ===\n");
}
