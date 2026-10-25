#include "lc3_codec.hpp"
#include "esp_log.h"
#include "lc3.h"
#include "esp_heap_caps.h"
#include <algorithm>
#include <cstring>
#include <cmath>

static const char* TAG = "ESP_LC3";

namespace Codec {

Lc3CodecEngine::Lc3CodecEngine() {}

Lc3CodecEngine::~Lc3CodecEngine() {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    for (size_t i = 0; i < NUM_ENCODERS; ++i) {
        if (m_google_enc_mem[i]) {
            free(m_google_enc_mem[i]);
            m_google_enc_mem[i] = nullptr;
        }
        m_google_encoder[i] = nullptr;
    }
    if (m_google_dec_mem) {
        free(m_google_dec_mem);
        m_google_dec_mem = nullptr;
    }
    m_google_decoder = nullptr;
#endif
    if (m_enc_handle) {
        esp_lc3_enc_close(m_enc_handle);
        m_enc_handle = nullptr;
    }
    if (m_dec_handle) {
        esp_lc3_dec_close(m_dec_handle);
        m_dec_handle = nullptr;
    }
}

esp_err_t Lc3CodecEngine::initEncoder(uint32_t sample_rate_hz, uint8_t channels, uint32_t frame_duration_us, uint16_t octets_per_frame) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    for (size_t i = 0; i < NUM_ENCODERS; ++i) {
        if (m_google_enc_mem[i]) {
            free(m_google_enc_mem[i]);
            m_google_enc_mem[i] = nullptr;
        }
        m_google_encoder[i] = nullptr;
    }
    m_encoder_ready = false;

    m_sample_rate = sample_rate_hz;
    // 'channels' = number of encoder instances to create (the SOURCE uses the full bank of NUM_ENCODERS)
    m_channels = static_cast<uint8_t>(std::min<size_t>(std::max<uint8_t>(channels, 1), NUM_ENCODERS));
    m_frame_duration_us = 10000;
    m_octets_per_frame = octets_per_frame;

    size_t total_bytes = 0;
    for (uint8_t i = 0; i < m_channels; ++i) {
        uint32_t ch_sr = (i == SUB_ENCODER_INDEX) ? 8000 : m_sample_rate;
        const bool is_mq_rate = (i >= ENC_MQ_BASE && i < ENC_MQ_BASE + 5);
        if (is_mq_rate) {
            // MQ slot: small second-rate state paired with HQ encoder (i - ENC_MQ_BASE), see lc3_encode_dual()
            void* primary = m_google_encoder[i - ENC_MQ_BASE];
            unsigned rate_size = lc3_encoder_rate_size(m_frame_duration_us, ch_sr);
            m_google_enc_mem[i] = primary ? heap_caps_malloc(rate_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) : nullptr;
            total_bytes += rate_size;
            m_google_encoder[i] = m_google_enc_mem[i]
                ? lc3_setup_encoder_rate(static_cast<lc3_encoder_t>(primary), m_google_enc_mem[i]) : nullptr;
            if (!m_google_encoder[i]) {
                ESP_LOGE(TAG, "Failed to set up MQ rate state %u (%u B)", i, rate_size);
                return ESP_ERR_NO_MEM;
            }
            continue;
        }
        unsigned mem_size = lc3_encoder_size(m_frame_duration_us, ch_sr);
        // Encoder state is touched on every frame: force internal SRAM (never PSRAM on PSRAM boards)
        m_google_enc_mem[i] = heap_caps_malloc(mem_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        total_bytes += mem_size;
        if (!m_google_enc_mem[i]) {
            ESP_LOGE(TAG, "Failed to allocate %u bytes for Google liblc3 encoder ch %u", mem_size, i);
            return ESP_ERR_NO_MEM;
        }

        m_google_encoder[i] = lc3_setup_encoder(m_frame_duration_us, ch_sr, ch_sr, m_google_enc_mem[i]);
        if (!m_google_encoder[i]) {
            ESP_LOGE(TAG, "Failed to setup Google liblc3 encoder ch %u (%lu Hz, %.1f ms)", i, (unsigned long)ch_sr, m_frame_duration_us / 1000.0f);
            free(m_google_enc_mem[i]);
            m_google_enc_mem[i] = nullptr;
            return ESP_FAIL;
        }

        // Disable LTPF (Long Term Pitch Filter) analysis for significant encoding speedup & clean subwoofer bass
        lc3_encoder_disable_ltpf(static_cast<lc3_encoder_t>(m_google_encoder[i]));
    }

    m_encoder_ready = true;
    ESP_LOGI(TAG, "liblc3_dual (IRAM code, FPU, no LTPF) encoder bank: %u slots, %u B internal SRAM "
                  "(0..4 HQ %lu Hz, 5..9 MQ rate states %lu Hz sharing the HQ analysis, 10 sub 8000 Hz)",
             m_channels, (unsigned)total_bytes, (unsigned long)m_sample_rate, (unsigned long)m_sample_rate);
    return ESP_OK;
#else
    if (m_enc_handle) {
        esp_lc3_enc_close(m_enc_handle);
        m_enc_handle = nullptr;
        m_encoder_ready = false;
    }

    m_sample_rate = sample_rate_hz;
    m_channels = channels;
    m_frame_duration_us = (frame_duration_us == 7500) ? 7500 : 10000;
    m_octets_per_frame = octets_per_frame;

    esp_lc3_enc_config_t enc_cfg = {
        .sample_rate = m_sample_rate,
        .bits_per_sample = 16,
        .channel = m_channels,
        .frame_dms = static_cast<uint8_t>(m_frame_duration_us / 100), // 75 for 7.5ms, 100 for 10ms
        .nbyte = m_octets_per_frame,
        .len_prefixed = false,
    };

    esp_audio_err_t ret = esp_lc3_enc_open(&enc_cfg, sizeof(enc_cfg), &m_enc_handle);
    if (ret != ESP_AUDIO_ERR_OK || !m_enc_handle) {
        ESP_LOGE(TAG, "Failed to open Espressif fixed-point LC3 encoder: error %d", ret);
        return ESP_FAIL;
    }

    m_encoder_ready = true;
    ESP_LOGI(TAG, "Espressif Fixed-Point LC3 Encoder Initialized: %lu Hz, %u-ch, %.1f ms duration, %u octets/frame (%lu kbps)",
             (unsigned long)m_sample_rate, m_channels, m_frame_duration_us / 1000.0f, m_octets_per_frame, (unsigned long)getBitrateKbps());
    return ESP_OK;
#endif
}

esp_err_t Lc3CodecEngine::initDecoder(uint32_t sample_rate_hz, uint8_t channels, uint32_t frame_duration_us, uint16_t octets_per_frame) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (m_google_dec_mem) {
        free(m_google_dec_mem);
        m_google_dec_mem = nullptr;
    }
    m_google_decoder = nullptr;
    m_decoder_ready = false;

    m_sample_rate = sample_rate_hz;
    m_channels = channels;
    m_frame_duration_us = (frame_duration_us == 7500) ? 7500 : 10000;
    m_octets_per_frame = octets_per_frame;

    unsigned mem_size = lc3_decoder_size(m_frame_duration_us, m_sample_rate);
    m_google_dec_mem = malloc(mem_size);
    if (!m_google_dec_mem) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes for Google liblc3 decoder", mem_size);
        return ESP_ERR_NO_MEM;
    }

    m_google_decoder = lc3_setup_decoder(m_frame_duration_us, m_sample_rate, 0, m_google_dec_mem);
    if (!m_google_decoder) {
        ESP_LOGE(TAG, "Failed to setup Google liblc3 decoder (%lu Hz, %.1f ms)", (unsigned long)m_sample_rate, m_frame_duration_us / 1000.0f);
        free(m_google_dec_mem);
        m_google_dec_mem = nullptr;
        return ESP_FAIL;
    }

    m_decoder_ready = true;
    ESP_LOGI(TAG, "Google liblc3 (Hardware FPU, IRAM) Decoder Initialized: %lu Hz, %u-ch, %.1f ms, %u octets/frame (24-bit native)",
             (unsigned long)m_sample_rate, m_channels, m_frame_duration_us / 1000.0f, m_octets_per_frame);
    return ESP_OK;
#else
    if (m_dec_handle) {
        esp_lc3_dec_close(m_dec_handle);
        m_dec_handle = nullptr;
        m_decoder_ready = false;
    }
    m_sample_rate = sample_rate_hz;
    m_channels = channels;
    m_frame_duration_us = (frame_duration_us == 7500) ? 7500 : 10000;
    m_octets_per_frame = octets_per_frame;

    esp_lc3_dec_cfg_t dec_cfg = {
        .sample_rate = m_sample_rate,
        .channel = m_channels,
        .bits_per_sample = 16,
        .frame_dms = static_cast<uint8_t>(m_frame_duration_us / 100), // 75 for 7.5ms, 100 for 10ms
        // VBR: each call decodes in_raw.len bytes, so HQ (120 B), redundancy and sub (60 B) frames all decode.
        // With is_cbr = true every frame had to be exactly nbyte long: all 60 B frames failed and came out as
        // concealment (the C6 sub never played; C6 satellite redundancy frames were not real recoveries).
        .nbyte = std::max<uint16_t>(m_octets_per_frame, 120),
        .is_cbr = false,
        .len_prefixed = false,
        .enable_plc = true,
    };

    esp_audio_err_t ret = esp_lc3_dec_open(&dec_cfg, sizeof(dec_cfg), &m_dec_handle);
    if (ret != ESP_AUDIO_ERR_OK || !m_dec_handle) {
        ESP_LOGE(TAG, "Failed to open Espressif fixed-point LC3 decoder: error %d", ret);
        return ESP_FAIL;
    }

    m_decoder_ready = true;
    ESP_LOGD(TAG, "Espressif Fixed-Point LC3 Decoder Initialized: %lu Hz, %u-ch, %.1f ms duration, %u octets/frame",
             (unsigned long)m_sample_rate, m_channels, m_frame_duration_us / 1000.0f, m_octets_per_frame);
    return ESP_OK;
#endif
}

esp_err_t Lc3CodecEngine::reconfigureEncoder(uint32_t sample_rate_hz, uint16_t octets_per_frame, uint32_t frame_duration_us) {
    uint32_t target_dur = (frame_duration_us == 7500) ? 7500 : 10000;
    if (m_sample_rate == sample_rate_hz && m_octets_per_frame == octets_per_frame &&
        m_frame_duration_us == target_dur && m_encoder_ready) {
        return ESP_OK;
    }
    return initEncoder(sample_rate_hz, m_channels, target_dur, octets_per_frame);
}

esp_err_t Lc3CodecEngine::reconfigureDecoder(uint32_t sample_rate_hz, uint16_t octets_per_frame, uint32_t frame_duration_us) {
    uint32_t target_dur = (frame_duration_us == 7500) ? 7500 : 10000;
    if (m_sample_rate == sample_rate_hz && m_octets_per_frame == octets_per_frame &&
        m_frame_duration_us == target_dur && m_decoder_ready) {
        return ESP_OK;
    }
    return initDecoder(sample_rate_hz, m_channels, target_dur, octets_per_frame);
}

uint32_t Lc3CodecEngine::getBitrateKbps() const {
    if (m_frame_duration_us == 0) return 0;
    return (static_cast<uint32_t>(m_octets_per_frame) * 8 * 1000000ULL) / (m_frame_duration_us * 1000ULL);
}

size_t Lc3CodecEngine::getEncoderRequiredPcmSamples() const {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    return Codec::calculateRequiredPcmSamples(m_sample_rate, m_frame_duration_us);
#else
    if (m_enc_handle) {
        int in_sz = 0, out_sz = 0;
        if (esp_lc3_enc_get_frame_size(m_enc_handle, &in_sz, &out_sz) == ESP_AUDIO_ERR_OK && in_sz > 0) {
            return static_cast<size_t>(in_sz) / sizeof(int16_t);
        }
    }
    return Codec::calculateRequiredPcmSamples(m_sample_rate, m_frame_duration_us);
#endif
}

esp_err_t Lc3CodecEngine::encodeFrameDual(const int16_t* pcm_in, uint8_t* out_hq, size_t hq_bytes, uint8_t* out_mq,
                                           size_t mq_bytes, uint8_t sat_ch) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const size_t mq_slot = ENC_MQ_BASE + sat_ch;
    if (!m_encoder_ready || sat_ch >= 5 || mq_slot >= m_channels || !m_google_encoder[sat_ch] ||
        !m_google_encoder[mq_slot] || !pcm_in || !out_hq || !out_mq) {
        return ESP_ERR_INVALID_ARG;
    }
    int ret = lc3_encode_dual(static_cast<lc3_encoder_t>(m_google_encoder[sat_ch]),
                              static_cast<lc3_encoder_rate_t>(m_google_encoder[mq_slot]), LC3_PCM_FORMAT_S16, pcm_in,
                              1, static_cast<int>(hq_bytes), out_hq, static_cast<int>(mq_bytes), out_mq);
    if (ret != 0) {
        ESP_LOGE(TAG, "liblc3_dual encode error: %d on ch %u", ret, sat_ch);
        return ESP_FAIL;
    }
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t Lc3CodecEngine::encodeFrameDualBegin(const int16_t* pcm_in, uint8_t* out_hq, size_t hq_bytes,
                                                size_t mq_bytes, uint8_t sat_ch) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const size_t mq_slot = ENC_MQ_BASE + sat_ch;
    if (!m_encoder_ready || sat_ch >= 5 || mq_slot >= m_channels || !m_google_encoder[sat_ch] ||
        !m_google_encoder[mq_slot] || !pcm_in || !out_hq) {
        return ESP_ERR_INVALID_ARG;
    }
    int ret = lc3_encode_dual_begin(static_cast<lc3_encoder_t>(m_google_encoder[sat_ch]),
                                    static_cast<lc3_encoder_rate_t>(m_google_encoder[mq_slot]), LC3_PCM_FORMAT_S16,
                                    pcm_in, 1, static_cast<int>(hq_bytes), out_hq, static_cast<int>(mq_bytes));
    return ret == 0 ? ESP_OK : ESP_FAIL;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t Lc3CodecEngine::encodeFrameDualEnd(uint8_t* out_mq, uint8_t sat_ch) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const size_t mq_slot = ENC_MQ_BASE + sat_ch;
    if (!m_encoder_ready || sat_ch >= 5 || mq_slot >= m_channels || !m_google_encoder[mq_slot] || !out_mq) {
        return ESP_ERR_INVALID_ARG;
    }
    int ret = lc3_encode_dual_end(static_cast<lc3_encoder_rate_t>(m_google_encoder[mq_slot]), out_mq);
    return ret == 0 ? ESP_OK : ESP_FAIL;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t Lc3CodecEngine::encodeFrame(const int16_t* pcm_in, size_t pcm_samples, uint8_t* out_lc3_buf, size_t max_out_bytes, size_t* actual_out_bytes, uint8_t channel_idx, int stride) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (channel_idx >= ENC_MQ_BASE && channel_idx < ENC_MQ_BASE + 5) {
        return ESP_ERR_INVALID_ARG;  // MQ slots are rate states, only usable through encodeFrameDual()
    }
    if (!m_encoder_ready || channel_idx >= m_channels || !m_google_encoder[channel_idx] || !pcm_in || !out_lc3_buf || !actual_out_bytes) {
        ESP_LOGW(TAG, "encodeFrame: invalid arg (ready=%d, ch_idx=%u, max_ch=%u, enc=%p)",
                 m_encoder_ready, channel_idx, m_channels, m_google_encoder[channel_idx < NUM_ENCODERS ? channel_idx : 0]);
        return ESP_ERR_INVALID_ARG;
    }
    size_t target_nbytes = max_out_bytes;
    if (target_nbytes < 20) {
        target_nbytes = m_octets_per_frame;
    }

    int ret = lc3_encode(static_cast<lc3_encoder_t>(m_google_encoder[channel_idx]), LC3_PCM_FORMAT_S16,
                         pcm_in, stride, static_cast<int>(target_nbytes), out_lc3_buf);
    if (ret != 0) {
        ESP_LOGE(TAG, "liblc3 encode error: %d on ch %u", ret, channel_idx);
        return ESP_FAIL;
    }

    *actual_out_bytes = target_nbytes;
    return ESP_OK;
#else
    if (!m_encoder_ready || !m_enc_handle || !pcm_in || !out_lc3_buf || !actual_out_bytes) {
        return ESP_ERR_INVALID_ARG;
    }
    if (max_out_bytes < 20) {
        return ESP_ERR_NO_MEM;
    }

    esp_audio_enc_in_frame_t in_f = {
        .buffer = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(pcm_in)),
        .len = static_cast<uint32_t>(pcm_samples * sizeof(int16_t)),
    };
    esp_audio_enc_out_frame_t out_f = {
        .buffer = out_lc3_buf,
        .len = static_cast<uint32_t>(max_out_bytes),
        .encoded_bytes = 0,
    };

    esp_audio_err_t ret = esp_lc3_enc_process(m_enc_handle, &in_f, &out_f);
    if (ret != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "esp_lc3_enc_process error: %d", ret);
        return ESP_FAIL;
    }

    *actual_out_bytes = out_f.encoded_bytes;
    return ESP_OK;
#endif
}

esp_err_t Lc3CodecEngine::decodeFrame(const uint8_t* in_lc3_buf, size_t in_bytes, int16_t* pcm_out, size_t max_pcm_samples,
                                      size_t* actual_pcm_samples, uint32_t stream_sample_rate, uint32_t stream_duration_us) {
    if (!pcm_out || !actual_pcm_samples) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t target_rate = (stream_sample_rate > 0) ? stream_sample_rate : m_sample_rate;
    uint32_t target_dur = (stream_duration_us > 0) ? stream_duration_us : m_frame_duration_us;

    /* Dynamically adapt decoder ONLY if sample rate or frame duration change on-the-fly (LC3 frame octet length is dynamic per frame) */
    if (target_rate != m_sample_rate || target_dur != m_frame_duration_us || !m_decoder_ready) {
        ESP_LOGI(TAG, "Dynamic decoder adaptation: %lu Hz (%.1f ms) -> %lu Hz (%.1f ms)",
                 (unsigned long)m_sample_rate, m_frame_duration_us / 1000.0f,
                 (unsigned long)target_rate, target_dur / 1000.0f);
        initDecoder(target_rate, m_channels, target_dur, m_octets_per_frame);
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!m_decoder_ready || !m_google_decoder) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t required_samples = calculateRequiredPcmSamples(m_sample_rate, m_frame_duration_us);
    if (max_pcm_samples < required_samples) {
        return ESP_ERR_NO_MEM;
    }

    bool is_plc = (in_lc3_buf == nullptr || in_bytes == 0);
    if (is_plc) {
        incrementPlcCount();
    }

    int ret = lc3_decode(static_cast<lc3_decoder_t>(m_google_decoder),
                         in_lc3_buf, (is_plc ? 0 : static_cast<int>(in_bytes)),
                         LC3_PCM_FORMAT_S16, pcm_out, 1);
    if (ret == 1 && !is_plc) {
        incrementPlcCount();
    } else if (ret < 0) {
        if (!is_plc) incrementPlcCount();
        lc3_decode(static_cast<lc3_decoder_t>(m_google_decoder), nullptr, 0, LC3_PCM_FORMAT_S16, pcm_out, 1);
    }

    *actual_pcm_samples = required_samples;
    return ESP_OK;
#else
    if (!m_decoder_ready || !m_dec_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t required_samples = calculateRequiredPcmSamples(m_sample_rate, m_frame_duration_us);
    if (max_pcm_samples < required_samples) {
        return ESP_ERR_NO_MEM;
    }

    bool is_plc = (in_lc3_buf == nullptr || in_bytes == 0);
    if (is_plc) {
        incrementPlcCount();
    }

    esp_audio_dec_in_raw_t in_raw = {
        .buffer = const_cast<uint8_t*>(in_lc3_buf),
        .len = static_cast<uint32_t>(in_bytes),
        .frame_recover = is_plc ? ESP_AUDIO_DEC_RECOVERY_PLC : ESP_AUDIO_DEC_RECOVERY_NONE,
    };

    esp_audio_dec_out_frame_t out_f = {
        .buffer = reinterpret_cast<uint8_t*>(pcm_out),
        .len = static_cast<uint32_t>(max_pcm_samples * sizeof(int16_t)),
        .needed_size = 0,
        .decoded_size = 0,
    };

    esp_audio_dec_info_t dec_info;
    esp_audio_err_t ret = esp_lc3_dec_decode(m_dec_handle, &in_raw, &out_f, &dec_info);
    if (ret != ESP_AUDIO_ERR_OK) {
        static uint32_t s_dec_errors = 0;
        if (!is_plc && (s_dec_errors++ % 1000) == 0) {
            ESP_LOGW(TAG, "LC3 decode error %d (%u bytes, %lu Hz): concealed (%lu errors so far)", (int)ret,
                     (unsigned)in_bytes, (unsigned long)m_sample_rate, (unsigned long)s_dec_errors);
        }
        // Fallback PLC
        if (!is_plc) {
            incrementPlcCount();
        }
        in_raw.buffer = nullptr;
        in_raw.len = 0;
        in_raw.frame_recover = ESP_AUDIO_DEC_RECOVERY_PLC;
        esp_lc3_dec_decode(m_dec_handle, &in_raw, &out_f, &dec_info);
    }

    *actual_pcm_samples = out_f.decoded_size / sizeof(int16_t);
    if (*actual_pcm_samples == 0) {
        *actual_pcm_samples = required_samples;
    }
    return ESP_OK;
#endif
}

esp_err_t Lc3CodecEngine::decodeFrame(const uint8_t* in_lc3_buf, size_t in_bytes, int32_t* pcm_out, size_t max_pcm_samples,
                                      size_t* actual_pcm_samples, uint32_t stream_sample_rate, uint32_t stream_duration_us) {
    if (!pcm_out || !actual_pcm_samples) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t target_rate = (stream_sample_rate > 0) ? stream_sample_rate : m_sample_rate;
    uint32_t target_dur = (stream_duration_us > 0) ? stream_duration_us : m_frame_duration_us;

    /* Dynamically adapt decoder ONLY if sample rate or frame duration change on-the-fly (LC3 frame octet length is dynamic per frame) */
    if (target_rate != m_sample_rate || target_dur != m_frame_duration_us || !m_decoder_ready) {
        ESP_LOGI(TAG, "Dynamic decoder adaptation: %lu Hz (%.1f ms) -> %lu Hz (%.1f ms)",
                 (unsigned long)m_sample_rate, m_frame_duration_us / 1000.0f,
                 (unsigned long)target_rate, target_dur / 1000.0f);
        initDecoder(target_rate, m_channels, target_dur, m_octets_per_frame);
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!m_decoder_ready || !m_google_decoder) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t required_samples = calculateRequiredPcmSamples(m_sample_rate, m_frame_duration_us);
    if (max_pcm_samples < required_samples) {
        return ESP_ERR_NO_MEM;
    }

    bool is_plc = (in_lc3_buf == nullptr || in_bytes == 0);
    if (is_plc) {
        incrementPlcCount();
    }

    int ret = lc3_decode(static_cast<lc3_decoder_t>(m_google_decoder),
                         in_lc3_buf, (is_plc ? 0 : static_cast<int>(in_bytes)),
                         LC3_PCM_FORMAT_S24, pcm_out, 1);
    if (ret == 1 && !is_plc) {
        incrementPlcCount();
    } else if (ret < 0) {
        if (!is_plc) incrementPlcCount();
        lc3_decode(static_cast<lc3_decoder_t>(m_google_decoder), nullptr, 0, LC3_PCM_FORMAT_S24, pcm_out, 1);
    }

    *actual_pcm_samples = required_samples;
    return ESP_OK;
#else
    static int16_t tmp_16[480];
    esp_err_t err = decodeFrame(in_lc3_buf, in_bytes, tmp_16, 480, actual_pcm_samples, stream_sample_rate, stream_duration_us);
    if (err == ESP_OK) {
        for (size_t i = 0; i < *actual_pcm_samples; ++i) {
            pcm_out[i] = static_cast<int32_t>(tmp_16[i]) << 8;
        }
    }
    return err;
#endif
}

} // namespace Codec

