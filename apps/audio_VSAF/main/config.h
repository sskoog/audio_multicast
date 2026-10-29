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
#define CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES 5 // Number of 10ms LC3 frames needed before leaving SCANNING to PREFILL (5 = 50ms cushion: 2 in DMA, 3 in FIFO)
#endif

#ifndef CONFIG_VSAF_PREFERRED_WIFI_CHANNEL
// SOURCE boot channel. 13 is legal in the EU and does not overlap channel 6, where most neighbouring APs sit
// here. Measured 2026-10-24 at MCS3: ch 13 ~35-70 TX-done timeouts/s and ~2.8 ms TX sweep, ch 1 ~150-300/s and
// ~4.5 ms, although the decoded-frame survey ranked ch 1 cleaner. So the survey does not override this by default.
#define CONFIG_VSAF_PREFERRED_WIFI_CHANNEL 13
#endif

#ifndef CONFIG_VSAF_BOOT_CHANNEL_SCAN
// 0 (default): SOURCE boots straight onto CONFIG_VSAF_PREFERRED_WIFI_CHANNEL, no survey.
// 1: run the passive RF survey at boot and leave the preferred channel if its adjacency-weighted score
//    exceeds CONFIG_VSAF_CHANNEL_BUSY_SCORE. The console 'scan' command works either way.
#define CONFIG_VSAF_BOOT_CHANNEL_SCAN 0
#endif

#ifndef CONFIG_VSAF_CHANNEL_BUSY_SCORE
// Adjacency-weighted survey score above which the preferred channel is abandoned for the cleanest one.
// Same scale as the survey table: < 50 EXCELLENT, < 150 GOOD, < 350 MODERATE, else CONGESTED.
#define CONFIG_VSAF_CHANNEL_BUSY_SCORE 150
#endif

#ifndef CONFIG_VSAF_BOOT_SCAN_DWELL_MS
#define CONFIG_VSAF_BOOT_SCAN_DWELL_MS 210 // Per-channel dwell of the boot survey (>= 2 beacon intervals of 102.4 ms)
#endif

#ifndef CONFIG_VSAF_TX_POWER_QDBM
// Default Wi-Fi TX power for every node, in 0.25 dBm units (esp_wifi_set_max_tx_power). 40 = +10 dBm requested.
// ESP-IDF quantizes the setting (e.g. 8..19 -> 2 dBm, 34..43 -> 8.5 dBm, 44..51 -> 11 dBm on the documented
// table), and esp_wifi_get_max_tx_power() returns the requested value, not the quantized one. Measured
// 2026-10-25: S3 roughly follows the setting; the C6 transmits far below nominal at the lowest settings
// (2 -> 8.5 dBm = +14.5 dB uplink) and saturates above ~11 dBm. Kept moderate on purpose: SOURCE heating
// (it transmits 600 packets/s) and future battery-powered SINKs.
#define CONFIG_VSAF_TX_POWER_QDBM 40
#endif

#ifndef CONFIG_VSAF_PLAYOUT_LATENCY_US
// Presentation time of frame N = SOURCE frame time t0(N) + this latency (+ per-channel trim). Must cover SOURCE
// encode + TX (<= 9.5 ms), redundancy recovery (t-2 arrives ~20 ms after t0), decode and the DMA queue (2 x 5 ms descriptors + staging, ~12 ms).
// Sent to the SINKs in the control frame; the SOURCE console 'latency <ms>' changes it at runtime.
#define CONFIG_VSAF_PLAYOUT_LATENCY_US 50000
#endif

#ifndef CONFIG_VSAF_SYNC_WAIT_MS
// SINK: how long to wait for a valid time sync before starting playback unsynchronized (FIFO-level drift control)
#define CONFIG_VSAF_SYNC_WAIT_MS 1500
#endif

#ifndef CONFIG_VSAF_DAC_DELAY_US
// SINK: fixed delay from the I2S output to the analog output (DAC filter), added to the predicted play time.
// Not calibrated yet (needs an acoustic / line-in measurement); 0 on both DAC types.
#define CONFIG_VSAF_DAC_DELAY_US 0
#endif

#ifndef CONFIG_VSAF_C6_DAC_BITS
// C6 SINKs (MAX98357A): DAC word length 16 or 24. 24: the LC3 decoder outputs 32-bit PCM and the SINK uses the
// same 32-bit sample path as the S3 nodes.
#define CONFIG_VSAF_C6_DAC_BITS 24
#endif

#ifndef CONFIG_VSAF_C6_I2S_SLOT_BITS
// C6 SINKs with 24-bit DAC words: I2S slot width. 24: BCLK = 48 x fs (2.304 MHz at 48 kHz), 3-byte samples in DMA,
// MCLK = 384 x fs; 32: BCLK = 64 x fs (3.072 MHz), like the S3 / PCM5102A nodes. The MAX98357A accepts BCLK =
// 32, 48 or 64 x fs; long jumper wires favour the lower clock.
#define CONFIG_VSAF_C6_I2S_SLOT_BITS 24
#endif

// SINK sample path width: 32-bit (24-bit audio) on the S3 and on 24-bit C6 nodes, 16-bit otherwise
#if defined(CONFIG_IDF_TARGET_ESP32S3) || (defined(CONFIG_IDF_TARGET_ESP32C6) && CONFIG_VSAF_C6_DAC_BITS == 24)
#define VSAF_SINK_PCM32 1
#else
#define VSAF_SINK_PCM32 0
#endif

#ifndef CONFIG_VSAF_APPLY_USB_HOST_VOLUME
// SOURCE: apply the UAC1 feature-unit volume / mute from the USB host. Windows treats the device as having
// hardware volume: it sends the slider as SET_CUR requests and does not scale the PCM itself.
#define CONFIG_VSAF_APPLY_USB_HOST_VOLUME 1
#endif

#ifndef CONFIG_VSAF_DMA_DESC_PER_FRAME
// SINK: I2S DMA descriptors per 10 ms audio frame (2 descriptors in the ring either way). 2 = 5 ms descriptors:
// the SINK decodes a frame only ~5-10 ms + staging before it plays instead of ~20-25 ms, which leaves that much
// more of the playout latency for the radio (the t-2 / t-3 redundant copies arrive 20 / 30 ms after t0).
// Output goes through a staging buffer, so descriptors need not line up with LC3 frames.
#define CONFIG_VSAF_DMA_DESC_PER_FRAME 2
#endif

#ifndef CONFIG_VSAF_JIT_MARGIN_US
// SINK: the wait for a late packet (FIFO empty) ends this long before the DMA queue runs dry (decode + write time)
#define CONFIG_VSAF_JIT_MARGIN_US 2500
#endif

#ifndef CONFIG_VSAF_SCAN_DWELL_MS
// SINK channel search: dwell per channel. The SOURCE sends 600 packets/s, so 30 ms sees ~18 of them; a full
// sweep of 13 channels takes ~0.4 s. SINKs start on CONFIG_VSAF_PREFERRED_WIFI_CHANNEL and hop upward with wrap.
#define CONFIG_VSAF_SCAN_DWELL_MS 30
#endif

#ifndef CONFIG_VSAF_SCAN_HOLD_MS
// SINK: after the stream stops, keep listening on the same channel this long before sweeping again
// (covers a SOURCE reboot, which comes back on the same channel).
#define CONFIG_VSAF_SCAN_HOLD_MS 3000
#endif

#ifndef CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES
#define CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES 6 // Consecutive missing frames / PLC before falling back from STREAM to SCANNING (6 = 60ms)
#endif

#ifndef CONFIG_ESPNOW_SAMPLE_RATE_HZ
#define CONFIG_ESPNOW_SAMPLE_RATE_HZ 48000 // Default sample rate (16000, 24000, 32000, 44100, 48000 Hz)
#endif

#ifndef CONFIG_ESPNOW_FRAME_LEN_OCTETS
#define CONFIG_ESPNOW_FRAME_LEN_OCTETS 120 // Default LC3 octets per frame (120 octets = 128 kbps @ 7.5ms, 96 kbps @ 10ms)
#endif

#ifndef CONFIG_ESPNOW_PRESENTATION_DELAY_US
#define CONFIG_ESPNOW_PRESENTATION_DELAY_US 50000 // 50 ms Presentation Delay
#endif

// Subwoofer Channel Configuration
#ifndef CONFIG_ESPNOW_SUB_LP_HZ
#define CONFIG_ESPNOW_SUB_LP_HZ 200.0f // 4th-order Linkwitz-Riley LP cutoff frequency in Hz
#endif

#ifndef CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ
#define CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ 8000 // Subwoofer downsampled rate (lowest LC3-supported rate)
#endif

#ifndef CONFIG_ESPNOW_SUB_FRAME_LEN_OCTETS
#define CONFIG_ESPNOW_SUB_FRAME_LEN_OCTETS 80 // 80 octets per 10ms frame (64 kbps @ 8 kHz)
#endif

#define SUB_CHANNEL_ID 5 // Channel ID 5 (0: Left, 1: Right, 2: Center, 3: L-Surr, 4: R-Surr, 5: Sub)

// SINK Volume Control & Slew Limiter Constants
#ifndef CONFIG_VOLUME_MIN_DB
#define CONFIG_VOLUME_MIN_DB -96.0f // Minimum volume in dB (step 1)
#endif

#ifndef CONFIG_VOLUME_MAX_DB
#define CONFIG_VOLUME_MAX_DB -6.0f   // Maximum volume in dB
#endif

#ifndef CONFIG_VOLUME_DEFAULT_U8
#define CONFIG_VOLUME_DEFAULT_U8 220 // Default power-on volume
#endif

#ifndef CONFIG_VOLUME_SLEW_RATE_DB_PER_SEC
#define CONFIG_VOLUME_SLEW_RATE_DB_PER_SEC 100.0f // Volume Slew Rate in dB/second (96 dB/s)
#endif

#define MAX_LC3_FRAME_OCTETS 200 // Maximum LC3 frame size supported over ESP-NOW
#define MAX_PCM_FRAME_SAMPLES 480 // Maximum PCM samples per 10ms frame (480 @ 48 kHz)

typedef struct {
    uint8_t  node_id;
    uint8_t  node_role;
    const char* device_name;
    int      i2s_bclk_gpio;
    int      i2s_ws_gpio;
    int      i2s_dout_gpio;
    int      amp_mute_gpio;    // Power-amp Mute / Shutdown / Enable pin (-1 if unused)
    int      status_led_gpio;
    int      status_led_num;
    int      user_button_gpio; // GP9 on C6 / GP0 on S3 (BOOT button)
    bool     has_display;
    uint8_t  default_channel;
    int8_t   max98357a_gain_db;
    bool     is_pcm5102a;
} system_config_t;

const system_config_t* get_system_config(void);
void set_node_role(uint8_t new_role);

#ifdef __cplusplus
}
#endif
