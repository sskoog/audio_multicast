#pragma once

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_ROLE_SINK   0
#define NODE_ROLE_SOURCE 1

#ifndef CONFIG_ACTIVE_NODE_ID
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define CONFIG_ACTIVE_NODE_ID 16
#else
#define CONFIG_ACTIVE_NODE_ID 23
#endif
#endif

#ifndef CONFIG_ACTIVE_NODE_ROLE
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define CONFIG_ACTIVE_NODE_ROLE NODE_ROLE_SOURCE
#else
#define CONFIG_ACTIVE_NODE_ROLE NODE_ROLE_SINK
#endif
#endif

#ifndef CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES
#define CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES 5 // Number of 10ms LC3 frames needed before leaving SCANNING to PREFILL (5 = 50ms)
#endif

#ifndef CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES
#define CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES 20 // Consecutive missing frames / PLC before falling back from STREAMING to SCANNING (20 = 200ms)
#endif

#ifndef CONFIG_ESPNOW_SAMPLE_RATE_HZ
#define CONFIG_ESPNOW_SAMPLE_RATE_HZ 48000 // Default sample rate (16000, 24000, 32000, 44100, 48000 Hz)
#endif

#ifndef CONFIG_ESPNOW_FRAME_LEN_OCTETS
#define CONFIG_ESPNOW_FRAME_LEN_OCTETS 120 // Default LC3 octets per frame (120 octets = 128 kbps @ 7.5ms, 96 kbps @ 10ms)
#endif

#define MAX_LC3_FRAME_OCTETS 120 // Maximum LC3 frame size supported over ESP-NOW (up to 96 kbps @ 10ms)
#define MAX_PCM_FRAME_SAMPLES 480 // Maximum PCM samples per 10ms frame (480 @ 48 kHz)

typedef struct {
    uint8_t  node_id;
    uint8_t  node_role;
    const char* device_name;
    int      i2s_bclk_gpio;
    int      i2s_ws_gpio;
    int      i2s_dout_gpio;
    int      status_led_gpio;
    int      status_led_num;
    int      user_button_gpio; // GP9 (BOOT button)
    bool     has_display;
    uint8_t  default_channel;
    int8_t   max98357a_gain_db;
} system_config_t;

const system_config_t* get_system_config(void);
void set_node_role(uint8_t new_role);

#ifdef __cplusplus
}
#endif
