#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Thread-safe formatted console printout to stdout, UART0, and TinyUSB CDC (COM116).
 *
 * @param format Printf-style format string (const char*, input)
 * @param ... Format arguments
 */
void print_console(const char* format, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif
