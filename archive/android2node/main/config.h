#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *                      BLUETOOTH LE AUDIO CONFIG
 * ===================================================================== */
#define CONFIG_BT_DEVICE_NAME              "ESP32-C6-LCD-Auracast"

/* =====================================================================
 *                       AUDIO CONFIGURATION
 * ===================================================================== */
#define AUDIO_SAMPLE_RATE_DEFAULT_HZ       48000
#define PCM_BUFFER_LENGTH_SAMPLES          960    /* 480 samples * 2 channels for 10ms stereo @ 48kHz */
#define AUDIO_CHANNELS_DEFAULT             2
#define AUDIO_FRAME_DURATION_MS            10
#define AUDIO_BITRATE_DEFAULT_KBPS         160
#define AUDIO_VOLUME_DEFAULT_PERCENT       85

/* =====================================================================
 *                     DSP FILTER CONFIGURATION
 * ===================================================================== */
#define HP_FILTER_CUTOFF_FREQ_HZ           80.0f  /* High-Pass Filter Cutoff for Top Speaker (Bass Cut) */
#define HP_FILTER_QUALITY_FACTOR           0.70710678f /* 2nd-Order Butterworth Q */

/* =====================================================================
 *                 MAX98357A I2S DAC HARDWARE PINOUT
 * ===================================================================== */
#define I2S_DAC_BCLK_PIN                   16     /* Bit Clock */
#define I2S_DAC_WS_PIN                     17     /* Word Select / Frame Clock */
#define I2S_DAC_DOUT_PIN                   18     /* Serial Data Out */
#define I2S_DAC_DMA_DESC_NUM               6      /* I2S DMA descriptor count */
#define I2S_DAC_DMA_FRAME_NUM              240    /* I2S DMA frame size */

/* =====================================================================
 *               DIAGNOSTICS & TELEMETRY CONFIGURATION
 * ===================================================================== */
#define DIAGNOSTICS_REFRESH_RATE_HZ        2      /* 2 Hz telemetry printout frequency */
#define DIAGNOSTICS_TASK_INTERVAL_MS       (1000 / DIAGNOSTICS_REFRESH_RATE_HZ) /* 500 ms period */
#define DIAGNOSTICS_TASK_STACK_SIZE        4096
#define DIAGNOSTICS_TASK_PRIORITY          2

/* Struct containing runtime system configuration parameters */
typedef struct {
    uint32_t sample_rate_hz;
    size_t   pcm_buffer_samples;
    float    hp_cutoff_freq_hz;
    uint8_t  bclk_gpio;
    uint8_t  ws_gpio;
    uint8_t  dout_gpio;
} system_config_t;

/* Get global system configuration */
const system_config_t* get_system_config(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CONFIG_H */
