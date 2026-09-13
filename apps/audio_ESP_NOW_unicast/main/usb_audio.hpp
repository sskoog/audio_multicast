#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize TinyUSB with Composite CDC + UAC descriptor
void usb_audio_init(void);

// Read PCM data from the USB UAC FIFO (called by LC3 encoder task or Main loop)
// Returns the number of bytes read
size_t usb_audio_read_pcm(void* dest, size_t max_bytes);

// Returns true if the host is actively streaming audio (Alt != 0 and recent packets received)
bool usb_audio_is_streaming(void);

// Clears the internal audio stream buffer
void usb_audio_clear_buffer(void);

#ifdef __cplusplus
}
#endif

