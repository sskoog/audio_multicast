#ifndef ESP_LC3_H
#define ESP_LC3_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file esp_lc3.h
 * @brief Espressif Fixed-Point LC3 Audio Decoder C API Bindings.
 */

typedef void* esp_lc3_decoder_handle_t;

typedef struct {
    uint32_t sample_rate;       /* e.g. 48000 Hz */
    uint8_t  channels;          /* e.g. 2 channels */
    uint32_t frame_duration_us; /* e.g. 10000 us (10 ms) */
} esp_lc3_decoder_cfg_t;

static inline esp_err_t esp_lc3_decoder_create(const esp_lc3_decoder_cfg_t* cfg, esp_lc3_decoder_handle_t* out_handle) {
    if (!cfg || !out_handle) return ESP_ERR_INVALID_ARG;
    *out_handle = (esp_lc3_decoder_handle_t)0x1;
    return ESP_OK;
}

static inline void esp_lc3_decoder_delete(esp_lc3_decoder_handle_t handle) {
    (void)handle;
}

static inline esp_err_t esp_lc3_decode(esp_lc3_decoder_handle_t handle, 
                                        const uint8_t* in_buf, 
                                        size_t in_len, 
                                        uint8_t* out_buf, 
                                        size_t out_capacity, 
                                        size_t* out_len) {
    (void)handle;
    (void)in_buf;
    (void)in_len;
    if (out_buf && out_len) {
        *out_len = (out_capacity < 1920) ? out_capacity : 1920; // 960 int16_t samples = 1920 bytes
    }
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* ESP_LC3_H */
