#include "console.hpp"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "tusb.h"
#endif

static SemaphoreHandle_t s_console_mutex = nullptr;
static StaticSemaphore_t s_console_mutex_buf;

static char s_raw_buf[512];
static char s_out_buf[1024];

/**
 * @brief Thread-safe formatted console printing routing to stdout, UART0, and TinyUSB CDC ACM.
 *
 * Expands standalone newline characters ('\n') to CRLF ("\r\n") for standard serial terminal compatibility,
 * and writes simultaneously to:
 * 1. Standard stdout (VFS console)
 * 2. Hardware UART0 (if driver is installed)
 * 3. TinyUSB CDC ACM virtual serial port (COM116 on ESP32-S3 Node 16) when endpoints are ready.
 *
 * @param format Printf-style format string (const char*, input)
 * @param ... Variable arguments matching format
 */
void print_console(const char* format, ...) {
    if (!s_console_mutex) {
        s_console_mutex = xSemaphoreCreateMutexStatic(&s_console_mutex_buf);
    }

    if (s_console_mutex) {
        xSemaphoreTake(s_console_mutex, portMAX_DELAY);
    }

    va_list args;
    va_start(args, format);
    int raw_len = vsnprintf(s_raw_buf, sizeof(s_raw_buf), format, args);
    va_end(args);

    if (raw_len > 0) {
        // Expand standalone '\n' to "\r\n" for clean serial terminal display without staircasing
        int out_len = 0;
        for (int i = 0; i < raw_len && out_len < static_cast<int>(sizeof(s_out_buf) - 2); ++i) {
            if (s_raw_buf[i] == '\n' && (i == 0 || s_raw_buf[i - 1] != '\r')) {
                s_out_buf[out_len++] = '\r';
            }
            s_out_buf[out_len++] = s_raw_buf[i];
        }
        s_out_buf[out_len] = '\0';

        // 1. Stdout / VFS console
        fwrite(s_out_buf, 1, out_len, stdout);
        fflush(stdout);

        // 2. Hardware UART0 if driver is active
        if (uart_is_driver_installed(UART_NUM_0)) {
            uart_write_bytes(UART_NUM_0, s_out_buf, out_len);
        }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        // 3. TinyUSB CDC ACM (COM116 on ESP32-S3 Node 16)
        if (tud_cdc_ready()) {
            tud_cdc_write(s_out_buf, static_cast<uint32_t>(out_len));
            tud_cdc_write_flush();
        }
#endif
    }

    if (s_console_mutex) {
        xSemaphoreGive(s_console_mutex);
    }
}
