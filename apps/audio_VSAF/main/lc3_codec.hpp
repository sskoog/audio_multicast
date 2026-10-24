#pragma once

#ifndef AUDIO_SAMPLE_RATE_HZ
#define AUDIO_SAMPLE_RATE_HZ 32000
#endif

#ifndef AUDIO_CHANNELS_NUM
#define AUDIO_CHANNELS_NUM 1
#endif

#ifndef AUDIO_FRAME_DURATION_US
#define AUDIO_FRAME_DURATION_US 10000
#endif

#ifndef AUDIO_LC3_OCTETS_PER_FRAME
#define AUDIO_LC3_OCTETS_PER_FRAME 80
#endif

#include "esp_audio_enc.h"
#include "esp_audio_dec.h"
#include "esp_lc3_enc.h"
#include "esp_lc3_dec.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <atomic>

namespace Codec {

static inline size_t calculateRequiredPcmSamples(uint32_t sample_rate, uint32_t frame_duration_us) {
    if (frame_duration_us == 7500) {
        switch (sample_rate) {
            case 8000:  return 60;
            case 16000: return 120;
            case 24000: return 180;
            case 32000: return 240;
            case 48000: return 360;
            case 96000: return 720;
            default:    return (sample_rate * 75) / 10000;
        }
    } else { // 10000 us default
        switch (sample_rate) {
            case 8000:  return 80;
            case 16000: return 160;
            case 24000: return 240;
            case 32000: return 320;
            case 48000: return 480;
            case 96000: return 960;
            default:    return (sample_rate * 10) / 1000;
        }
    }
}

class Lc3CodecEngine {
public:
    Lc3CodecEngine();
    ~Lc3CodecEngine();

    esp_err_t initEncoder(uint32_t sample_rate_hz = AUDIO_SAMPLE_RATE_HZ, uint8_t channels = AUDIO_CHANNELS_NUM,
                          uint32_t frame_duration_us = AUDIO_FRAME_DURATION_US, uint16_t octets_per_frame = AUDIO_LC3_OCTETS_PER_FRAME);
    esp_err_t initDecoder(uint32_t sample_rate_hz = AUDIO_SAMPLE_RATE_HZ, uint8_t channels = AUDIO_CHANNELS_NUM,
                          uint32_t frame_duration_us = AUDIO_FRAME_DURATION_US, uint16_t octets_per_frame = AUDIO_LC3_OCTETS_PER_FRAME);

    esp_err_t reconfigureEncoder(uint32_t sample_rate_hz, uint16_t octets_per_frame, uint32_t frame_duration_us = 10000);
    esp_err_t reconfigureDecoder(uint32_t sample_rate_hz, uint16_t octets_per_frame, uint32_t frame_duration_us = 10000);

    esp_err_t encodeFrame(const int16_t* pcm_in, size_t pcm_samples, uint8_t* out_lc3_buf, size_t max_out_bytes, size_t* actual_out_bytes, uint8_t channel_idx = 0, int stride = 1);
    esp_err_t decodeFrame(const uint8_t* in_lc3_buf, size_t in_bytes, int16_t* pcm_out, size_t max_pcm_samples,
                          size_t* actual_pcm_samples, uint32_t stream_sample_rate = 0, uint32_t stream_duration_us = 0);
    esp_err_t decodeFrame(const uint8_t* in_lc3_buf, size_t in_bytes, int32_t* pcm_out, size_t max_pcm_samples,
                          size_t* actual_pcm_samples, uint32_t stream_sample_rate = 0, uint32_t stream_duration_us = 0);

    uint32_t getSampleRate() const { return m_sample_rate; }
    uint32_t getFrameDurationUs() const { return m_frame_duration_us; }
    uint16_t getOctetsPerFrame() const { return m_octets_per_frame; }
    uint32_t getBitrateKbps() const;
    size_t   getEncoderRequiredPcmSamples() const;
    void incrementPlcCount() { m_plc_count.fetch_add(1, std::memory_order_relaxed); }
    uint32_t getAndResetPlcCount() { return m_plc_count.exchange(0, std::memory_order_relaxed); }
    uint32_t getPlcCount() const { return m_plc_count.load(std::memory_order_relaxed); }
    void resetPlcCount() { m_plc_count.store(0, std::memory_order_relaxed); }

    /**
     * @brief Encode one satellite PCM frame at HQ and MQ size with one shared analysis (S3, liblc3_dual).
     * @param pcm_in   480 (48 kHz) int16 samples, stride 1.
     * @param out_hq   HQ frame buffer of hq_bytes (120).
     * @param out_mq   MQ redundancy frame buffer of mq_bytes (60).
     * @param sat_ch   Satellite channel 0..4 (selects encoder slot sat_ch and its MQ rate state).
     * @return ESP_OK, ESP_ERR_INVALID_ARG or ESP_FAIL. Not available on C6 (returns ESP_ERR_NOT_SUPPORTED).
     */
    esp_err_t encodeFrameDual(const int16_t* pcm_in, uint8_t* out_hq, size_t hq_bytes, uint8_t* out_mq,
                              size_t mq_bytes, uint8_t sat_ch);

    /**
     * @brief Two-phase form of encodeFrameDual(): phase 1 = shared analysis + HQ frame, MQ frame left pending.
     * @param mq_bytes MQ frame size (60) used by the matching encodeFrameDualEnd().
     */
    esp_err_t encodeFrameDualBegin(const int16_t* pcm_in, uint8_t* out_hq, size_t hq_bytes, size_t mq_bytes,
                                   uint8_t sat_ch);
    /**
     * @brief Phase 2: quantize and write the pending MQ frame of satellite `sat_ch` (after its Begin()).
     */
    esp_err_t encodeFrameDualEnd(uint8_t* out_mq, uint8_t sat_ch);

    // SOURCE encoder bank (S3, liblc3_dual). Every LC3 encoder keeps per-stream state (MDCT overlap), and
    // the two cores encode different channels in parallel, so each channel has its own instance.
    //   0..4  : satellite ch 0..4 HQ encoder (48 kHz, 120 B), full liblc3 encoder
    //   5..9  : satellite ch 0..4 MQ rate state (60 B redundancy) paired with encoder 0..4; it reuses
    //           that encoder's analysis (lc3_encode_dual), only ~2 KB instead of a 5.4 KB encoder
    //   10    : subwoofer ch 5 (8 kHz, 60 B), full encoder
    static constexpr size_t NUM_ENCODERS = 11;
    static constexpr size_t ENC_HQ_BASE = 0;
    static constexpr size_t ENC_MQ_BASE = 5;
    static constexpr size_t SUB_ENCODER_INDEX = 10;

private:
    void* m_enc_handle = nullptr;
    void* m_dec_handle = nullptr;
    void* m_google_encoder[NUM_ENCODERS] = {};
    void* m_google_enc_mem[NUM_ENCODERS] = {};
    void* m_google_decoder = nullptr;
    void* m_google_dec_mem = nullptr;
    uint32_t m_sample_rate = AUDIO_SAMPLE_RATE_HZ;
    uint8_t  m_channels = AUDIO_CHANNELS_NUM;
    uint32_t m_frame_duration_us = AUDIO_FRAME_DURATION_US;
    uint16_t m_octets_per_frame = AUDIO_LC3_OCTETS_PER_FRAME;
    bool m_encoder_ready = false;
    bool m_decoder_ready = false;
    std::atomic<uint32_t> m_plc_count{0};
};

} // namespace Codec
