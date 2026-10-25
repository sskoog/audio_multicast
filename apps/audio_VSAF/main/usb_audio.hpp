#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// Initialize TinyUSB with Composite CDC + UAC descriptor
void usb_audio_init(void);

// Read PCM data from the USB UAC FIFO (called by LC3 encoder task or Main loop)
// Returns the number of bytes read
size_t usb_audio_read_pcm(void* dest, size_t max_bytes);

// Returns true if the host is actively streaming audio (Alt != 0 and recent packets received)
bool usb_audio_is_streaming(void);

// Clears the internal audio stream buffer
void usb_audio_clear_buffer(void);

// Linear gains for left / right from the host's volume and mute controls (UAC1 Feature Unit, master + channel).
// The host (Windows) treats this as hardware volume and does not scale the PCM itself.
void usb_audio_get_gain(float* gain_l, float* gain_r);

// Raw feature-unit state from the host: mute[3] and volume[3] (1/256 dB) for master, ch1, ch2; set_count = SET_CUR requests
void usb_audio_get_fu_state(int8_t mute[3], int16_t volume[3], uint32_t* set_count);

// Reboot straight into the ROM download bootloader (COM16) via a ROM system reset (no esp_restart()
// shutdown handlers, which intermittently hung). Does not return.
void usb_reboot_to_rom_download(void);

#else
// Non-S3 targets (e.g. ESP32-C6 SINK) do not have USB-OTG / TinyUSB
static inline void usb_audio_init(void) {}
static inline size_t usb_audio_read_pcm(void* dest, size_t max_bytes) { (void)dest; (void)max_bytes; return 0; }
static inline bool usb_audio_is_streaming(void) { return false; }
static inline void usb_audio_clear_buffer(void) {}
static inline void usb_audio_get_gain(float* gain_l, float* gain_r) { *gain_l = 1.0f; *gain_r = 1.0f; }
static inline void usb_audio_get_fu_state(int8_t mute[3], int16_t volume[3], uint32_t* set_count) {
    for (int i = 0; i < 3; ++i) { mute[i] = 0; volume[i] = 0; } *set_count = 0; }
#endif

#ifdef __cplusplus
}
#endif
