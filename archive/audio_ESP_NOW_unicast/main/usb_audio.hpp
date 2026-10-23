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

#else
// Non-S3 targets (e.g. ESP32-C6 SINK) do not have USB-OTG / TinyUSB
static inline void usb_audio_init(void) {}
static inline size_t usb_audio_read_pcm(void* dest, size_t max_bytes) { (void)dest; (void)max_bytes; return 0; }
static inline bool usb_audio_is_streaming(void) { return false; }
static inline void usb_audio_clear_buffer(void) {}
#endif

#ifdef __cplusplus
}
#endif
