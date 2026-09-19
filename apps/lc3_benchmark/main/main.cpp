#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lc3_benchmark_runner.hpp"

static const char *TAG = "APP_MAIN";

static Lc3BenchmarkRunner s_runner;

static void benchmark_task(void *pvParameters) {
    ESP_LOGI(TAG, "Benchmark task started on Core %d (Priority %d)", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    
    // Allow system and NVS initialization to settle
    vTaskDelay(pdMS_TO_TICKS(500));

    if (!s_runner.init()) {
        ESP_LOGE(TAG, "Benchmark runner initialization failed!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Running automated LC3 Benchmark Suite Rev 2...");
    s_runner.runFullSuite();

    ESP_LOGI(TAG, "Benchmark completed. Type 'bench' in console to re-run.");

    // Serial command listening loop
    while (true) {
        int c = getchar();
        if (c != EOF) {
            static char line_buf[64];
            static size_t line_len = 0;
            if (c == '\n' || c == '\r') {
                line_buf[line_len] = '\0';
                if (strcmp(line_buf, "bench") == 0) {
                    ESP_LOGI(TAG, "Re-running benchmark suite upon user request...");
                    s_runner.runFullSuite();
                }
                line_len = 0;
            } else if (line_len + 1 < sizeof(line_buf)) {
                line_buf[line_len++] = (char)c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== ESP32-S3 LC3 CODEC BENCHMARK FIRMWARE REV 2 ===");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Pin benchmark task strictly to CORE 1 (Xtensa LX7 Core 1 dedicated to codec)
    xTaskCreatePinnedToCore(
        benchmark_task,
        "lc3_bench_task",
        16384,
        NULL,
        5,
        NULL,
        1 // Core 1
    );
}
