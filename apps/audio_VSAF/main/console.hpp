#pragma once
#include <cstdint>
#include <cstddef>

namespace AudioNet {
    class EspNowBroadcastEngine;
    using EspNowUnicastEngine = EspNowBroadcastEngine;
}

// namespace Benchmark {
//     class Lc3BenchmarkSuite;
// }

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

/**
 * @brief Parse standard 6-byte colon or hyphen separated MAC address string.
 *
 * @param str Input string (e.g. "AA:BB:CC:DD:EE:FF" or "aa-bb-cc-dd-ee-ff")
 * @param out_mac Pointer to 6-byte output buffer (uint8_t[6])
 * @return true if successfully parsed 6 bytes, false otherwise
 */
bool parse_mac_address(const char* str, uint8_t* out_mac);

/**
 * @brief Parse and execute an ASCII CLI command line.
 *
 * @param raw_line Null-terminated ASCII command string
 */
void handle_ascii_command(const char* raw_line);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * @brief Register active engine for console command dispatch.
 *
 * @param unicast_engine Pointer to active EspNowUnicastEngine instance
 */
void console_init(AudioNet::EspNowUnicastEngine* unicast_engine);
#endif

