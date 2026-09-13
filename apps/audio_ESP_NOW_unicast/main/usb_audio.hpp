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

#ifdef __cplusplus
}
#endif
