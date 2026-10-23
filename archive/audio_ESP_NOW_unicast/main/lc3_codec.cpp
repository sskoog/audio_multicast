#include "lc3_codec.hpp"
#include "esp_log.h"
#include "lc3.h"
#include <cstring>
#include <cmath>

static const char* TAG = "ESP_LC3";

namespace Codec {

Lc3CodecEngine::Lc3CodecEngine() {}

Lc3CodecEngine::~Lc3CodecEngine() {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (m_google_enc_mem) {
        free(m_google_enc_mem);
        m_google_enc_mem = nullptr;
    }
    m_google_encoder = nullptr;
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
    if (m_google_enc_mem) {
        free(m_google_enc_mem);
        m_google_enc_mem = nullptr;
    }
    m_google_encoder = nullptr;
    m_encoder_ready = false;

    m_sample_rate = sample_rate_hz;
    m_channels = channels;
    m_frame_duration_us = (frame_duration_us == 7500) ? 7500 : 10000;
    m_octets_per_frame = octets_per_frame;

    unsigned mem_size = lc3_encoder_size(m_frame_duration_us, m_sample_rate);
    m_google_enc_mem = malloc(mem_size);
    if (!m_google_enc_mem) {
        ESP_LOGE(TAG, "Failed to allocate %u bytes for Google liblc3 encoder", mem_size);
        return ESP_ERR_NO_MEM;
    }

    m_google_encoder = lc3_setup_encoder(m_frame_duration_us, m_sample_rate, m_sample_rate, m_google_enc_mem);
    if (!m_google_encoder) {
        ESP_LOGE(TAG, "Failed to setup Google liblc3 encoder (%lu Hz, %.1f ms)", (unsigned long)m_sample_rate, m_frame_duration_us / 1000.0f);
        free(m_google_enc_mem);
        m_google_enc_mem = nullptr;
        return ESP_FAIL;
    }

    m_encoder_ready = true;
    ESP_LOGI(TAG, "Google liblc3 (Hardware FPU) Encoder Initialized: %lu Hz, %u-ch, %.1f ms duration, %u octets/frame (%lu kbps)",
             (unsigned long)m_sample_rate, m_channels, m_frame_duration_us / 1000.0f, m_octets_per_frame, (unsigned long)getBitrateKbps());
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
        .nbyte = m_octets_per_frame,
        .is_cbr = true,
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

esp_err_t Lc3CodecEngine::encodeFrame(const int16_t* pcm_in, size_t pcm_samples, uint8_t* out_lc3_buf, size_t max_out_bytes, size_t* actual_out_bytes) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    if (!m_encoder_ready || !m_google_encoder || !pcm_in || !out_lc3_buf || !actual_out_bytes) {
        return ESP_ERR_INVALID_ARG;
    }
    if (max_out_bytes < m_octets_per_frame) {
        return ESP_ERR_NO_MEM;
    }

    int ret = lc3_encode(static_cast<lc3_encoder_t>(m_google_encoder), LC3_PCM_FORMAT_S16,
                         pcm_in, 1 /* stride */, m_octets_per_frame, out_lc3_buf);
    if (ret != 0) {
        ESP_LOGE(TAG, "liblc3 encode error: %d", ret);
        return ESP_FAIL;
    }

    *actual_out_bytes = m_octets_per_frame;
    return ESP_OK;
#else
    if (!m_encoder_ready || !m_enc_handle || !pcm_in || !out_lc3_buf || !actual_out_bytes) {
        return ESP_ERR_INVALID_ARG;
    }
    if (max_out_bytes < m_octets_per_frame) {
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
    uint16_t target_octets = (in_bytes >= 20 && in_bytes <= 120) ? static_cast<uint16_t>(in_bytes) : m_octets_per_frame;

    /* Dynamically adapt decoder if stream sample rate, duration, or octets per frame change on-the-fly */
    if (target_rate != m_sample_rate || target_dur != m_frame_duration_us || target_octets != m_octets_per_frame || !m_decoder_ready) {
        ESP_LOGI(TAG, "Dynamic decoder adaptation: %lu Hz (%.1f ms, %u oct) -> %lu Hz (%.1f ms, %u oct)",
                 (unsigned long)m_sample_rate, m_frame_duration_us / 1000.0f, m_octets_per_frame,
                 (unsigned long)target_rate, target_dur / 1000.0f, target_octets);
        initDecoder(target_rate, m_channels, target_dur, target_octets);
    }
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
}

} // namespace Codec
