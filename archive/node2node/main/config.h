#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *                       NODE ROLE SELECTION
 * ===================================================================== */
#define NODE_ROLE_SINK                  1   /* Audio SINK (Receiver + I2S DAC) */
#define NODE_ROLE_SOURCE                2   /* Audio SOURCE (Tone Gen, Broadcaster) */

/* Set active role here (can be overridden via compiler flags -DCONFIG_NODE_ROLE=...) */
#ifndef CONFIG_ACTIVE_NODE_ROLE
#define CONFIG_ACTIVE_NODE_ROLE         NODE_ROLE_SINK
#endif

/* =====================================================================
 *                   HARDWARE PROFILE DEFINITIONS
 * ===================================================================== */
#define BOARD_WAVESHARE_LCD             1   /* 1.47" ST7789 LCD + WS2812 RGB LED (Node20 on COM20) */
#define BOARD_ESP32C6_WROOM             2   /* Generic ESP32-C6-WROOM-1 DevKit (Node21 on COM21/22) */
#define BOARD_WAVESHARE_ZERO            3   /* Waveshare ESP32-C6-Zero 18-Pin Mini (Node23/24 on COM23/24) */
#define BOARD_HEEMOL_MINI               4   /* Heemol ESP32-C6 Mini 20-Pin (Node25/26 on COM25/26) */

/* Set active board type here (can be overridden via -DCONFIG_ACTIVE_BOARD_TYPE=...) */
#ifndef CONFIG_ACTIVE_BOARD_TYPE
#if CONFIG_ACTIVE_NODE_ROLE == NODE_ROLE_SINK
#define ACTIVE_BOARD_TYPE               BOARD_WAVESHARE_ZERO
#else
#define ACTIVE_BOARD_TYPE               BOARD_ESP32C6_WROOM
#endif
#endif

/* Node Device Name */
#ifndef NODE_DEVICE_NAME
#if ACTIVE_BOARD_TYPE == BOARD_WAVESHARE_LCD
#define NODE_DEVICE_NAME                "ESP32-C6-20"
#elif ACTIVE_BOARD_TYPE == BOARD_WAVESHARE_ZERO
#define NODE_DEVICE_NAME                "ESP32-C6-23"
#elif ACTIVE_BOARD_TYPE == BOARD_HEEMOL_MINI
#define NODE_DEVICE_NAME                "ESP32-C6-25"
#else
#define NODE_DEVICE_NAME                "ESP32-C6-21"
#endif
#endif

/* =====================================================================
 *                    AUDIO & CODEC CONFIGURATION
 * ===================================================================== */
#define AUDIO_SAMPLE_RATE_HZ            48000
#define AUDIO_BIT_DEPTH                 16
#define AUDIO_CHANNELS_NUM              1      /* Mono BIS */
#define AUDIO_FRAME_DURATION_MS         10     /* 10 ms ISO frame */
#define AUDIO_SAMPLES_PER_FRAME         (AUDIO_SAMPLE_RATE_HZ * AUDIO_FRAME_DURATION_MS / 1000) /* 441 samples */
#define AUDIO_PCM_FRAME_BYTES           (AUDIO_SAMPLES_PER_FRAME * sizeof(int16_t))             /* 882 bytes */
#define AUDIO_LC3_OCTETS_PER_FRAME      80     /* 80 octets per 10ms frame = 64 kbps mono */
#define AUDIO_BITRATE_KBPS              ((AUDIO_LC3_OCTETS_PER_FRAME * 8 * 1000) / (AUDIO_FRAME_DURATION_MS * 1000)) /* 64 kbps */
#define AUDIO_SYNC_TIMEOUT_MS           500    /* 0.5 s sync loss timeout */
#define AUDIO_TASK_STACK_SIZE           8192   /* 8 KB stack for audio decoding & I2S DMA write */

/* =====================================================================
 *             VCO / VFO AUDIO TEST TONE GENERATOR CONFIG
 * ===================================================================== */
#define VCO_NOMINAL_FREQ_HZ             220.0f
#define VCO_MIN_FREQ_HZ                 110.0f
#define VCO_MAX_FREQ_HZ                 440.0f
#define VCO_AMPLITUDE_PERCENT           30.0f
#define VCO_PEAK_AMPLITUDE_INT16        ((int16_t)(32767.0f * (VCO_AMPLITUDE_PERCENT / 100.0f))) /* ~9830 */
#define VFO_MIN_MOD_RATE_HZ             0.5f
#define VFO_MAX_MOD_RATE_HZ             2.0f

/* =====================================================================
 *                 MAX98357A I2S DAC HARDWARE PINOUT
 * ===================================================================== */
#if ACTIVE_BOARD_TYPE == BOARD_WAVESHARE_ZERO
/* Waveshare ESP32-C6-Zero 18-Pin Mini (Left Header GP1, GP2, GP3, GP0) */
#define I2S_DAC_BCLK_PIN                1      /* Bit Clock (GP1) */
#define I2S_DAC_WS_PIN                  2      /* Word Select / Frame Clock (GP2) */
#define I2S_DAC_DOUT_PIN                3      /* Serial Data Out (GP3) */
#define I2S_DAC_GAIN_PIN                0      /* Tri-state Hardware Gain Control (GP0) */
#define STATUS_LED_PIN                  8      /* Onboard WS2812 RGB LED (GP8) */
#else
/* Waveshare LCD (Node20) / Generic WROOM (Node21) / Heemol Mini */
#define I2S_DAC_BCLK_PIN                16     /* Bit Clock */
#define I2S_DAC_WS_PIN                  17     /* Word Select / Frame Clock */
#define I2S_DAC_DOUT_PIN                18     /* Serial Data Out */
#define I2S_DAC_GAIN_PIN                -1     /* Unassigned */
#define STATUS_LED_PIN                  8      /* Onboard WS2812 RGB LED */
#endif

#define I2S_DAC_DMA_DESC_NUM            6
#define I2S_DAC_DMA_FRAME_NUM           240

/* =====================================================================
 *               DIAGNOSTICS & TELEMETRY CONFIGURATION
 * ===================================================================== */
#define DIAGNOSTICS_REFRESH_RATE_HZ     4      /* 4 Hz periodic printout */
#define DIAGNOSTICS_TASK_INTERVAL_MS    (1000 / DIAGNOSTICS_REFRESH_RATE_HZ) /* Calculate period time in milliseconds */
#define DIAGNOSTICS_TASK_STACK_SIZE     8192
#define DIAGNOSTICS_TASK_PRIORITY       2

/* =====================================================================
 *             GATT LE AUDIO & VCS OSCILLATOR CONFIGURATION
 * ===================================================================== */
#define MAX_GATT_SINK_NODES             9      /* Track up to 9 SINKs */
#define VCS_DEFAULT_VOLUME_PERCENT      50     /* Default 30% volume */
#define VCS_VOLUME_MIN_PCT              10.0f  /* Min sine volume: 10% */
#define VCS_VOLUME_MAX_PCT              50.0f  /* Max sine volume: 50% */
#define VCS_SINE_MOD_FREQ_HZ            0.10f  /* 0.1 Hz LFO modulation */
#define VCS_UPDATE_RATE_HZ              4      /* 4 Hz transmission rate */
#define VCS_UPDATE_INTERVAL_MS          (1000 / VCS_UPDATE_RATE_HZ)

/* Standard Bluetooth SIG 16-bit Service UUIDs */
#define BLE_GATT_SVC_PACS_UUID16        0x184E /* Published Audio Capabilities Service */
#define BLE_GATT_SVC_BASS_UUID16        0x184F /* Broadcast Audio Scan Service */
#define BLE_GATT_SVC_VCS_UUID16         0x1844 /* Volume Control Service */
#define BLE_GATT_SVC_CAS_UUID16         0x1853 /* Common Audio Service */

/* Standard Bluetooth SIG 16-bit Characteristic UUIDs */
#define BLE_GATT_CHR_PACS_SINK_PAC      0x2BC9 /* Sink PAC */
#define BLE_GATT_CHR_PACS_SINK_LOC      0x2BCA /* Sink Audio Locations */
#define BLE_GATT_CHR_PACS_SINK_CTX      0x2BCE /* Sink Audio Contexts */

#define BLE_GATT_CHR_BASS_RECV_STATE    0x2BC8 /* Broadcast Receive State */
#define BLE_GATT_CHR_BASS_CP            0x2BC7 /* Broadcast Audio Scan Control Point */

#define BLE_GATT_CHR_VCS_STATE          0x2B7D /* Volume State */
#define BLE_GATT_CHR_VCS_CP             0x2B7E /* Volume Control Point */
#define BLE_GATT_CHR_VCS_FLAGS          0x2B7F /* Volume Flags */

/* Standard Bluetooth SIG Appearance */
#define BLE_GAP_APPEARANCE_HEADPHONES   0x0841 /* Stereo Headphones (LE Audio Sink) */

/* System configuration structure */
typedef struct {
    uint8_t  node_role;
    uint8_t  board_type;
    const char* device_name;
    uint32_t sample_rate_hz;
    uint8_t  channels;
    uint8_t  bit_depth;
    uint32_t bitrate_kbps;
} system_config_t;

const system_config_t* get_system_config(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CONFIG_H */
