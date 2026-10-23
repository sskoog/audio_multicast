#include "ota_manager.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char* TAG = "OTA_MANAGER";

namespace System {

OtaManager::OtaManager() = default;

esp_err_t OtaManager::initWifi() {
    ESP_LOGI(TAG, "Initializing Wi-Fi 6 Station Mode for ESP32-C6...");
    m_status = OtaStatus::WIFI_CONNECTING;

    // 1. Initialize NVS Flash (Required for RF calibration, Wi-Fi & BLE stack)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize TCP/IP Netif & Default Event Loop Architecture
    ESP_ERROR_CHECK(esp_netif_init());
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }

    // 3. Create Default Wi-Fi Station Network Interface
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    (void)sta_netif;

    // 2. Initialize Wi-Fi driver with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 3. Register Event Handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &OtaManager::wifiEventHandler, this, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &OtaManager::wifiEventHandler, this, nullptr));

    // 4. Configure Wi-Fi Credentials from wifi_credentials.h
    wifi_config_t wifi_config = {};
    snprintf(reinterpret_cast<char*>(wifi_config.sta.ssid), sizeof(wifi_config.sta.ssid), "%s", CONFIG_WIFI_SSID);
    snprintf(reinterpret_cast<char*>(wifi_config.sta.password), sizeof(wifi_config.sta.password), "%s", CONFIG_WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID: '%s'...", CONFIG_WIFI_SSID);
    return ESP_OK;
}

void OtaManager::wifiEventHandler(void* arg, const char* event_base, int32_t event_id, void* event_data) {
    auto* self = static_cast<OtaManager*>(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        self->m_wifi_connected = false;
        self->m_status = OtaStatus::WIFI_FAILED;
        ESP_LOGW(TAG, "Wi-Fi disconnected! Retrying connection...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(event_data);
        self->m_wifi_connected = true;
        self->m_status = OtaStatus::WIFI_CONNECTED;
        ESP_LOGI(TAG, "Wi-Fi Connected! Obtained IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

esp_err_t OtaManager::startOtaUpdate(const char* ota_url) {
    if (!m_wifi_connected) {
        ESP_LOGE(TAG, "Cannot start OTA update: Wi-Fi is not connected!");
        m_status = OtaStatus::ERROR_FAILED;
        return ESP_ERR_INVALID_STATE;
    }

    const char* url = (ota_url && strlen(ota_url) > 0) ? ota_url : CONFIG_OTA_FIRMWARE_URL;
    ESP_LOGI(TAG, "Starting HTTP/HTTPS OTA Firmware Update from: %s", url);
    m_status = OtaStatus::DOWNLOADING_FIRMWARE;

    esp_http_client_config_t http_config = {};
    http_config.url = url;
    http_config.timeout_ms = 10000;
    http_config.keep_alive_enable = true;

    esp_https_ota_config_t ota_config = {};
    ota_config.http_config = &http_config;

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        m_status = OtaStatus::SUCCESS_REBOOTING;
        ESP_LOGI(TAG, "OTA Firmware Update Succeeded! Rebooting system in 2 seconds...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        m_status = OtaStatus::ERROR_FAILED;
        ESP_LOGE(TAG, "OTA Firmware Update Failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

const char* OtaManager::getStatusStr() const {
    switch (m_status) {
        case OtaStatus::IDLE: return "IDLE";
        case OtaStatus::WIFI_CONNECTING: return "CONNECTING WIFI";
        case OtaStatus::WIFI_CONNECTED: return "WIFI CONNECTED";
        case OtaStatus::WIFI_FAILED: return "WIFI FAILED";
        case OtaStatus::DOWNLOADING_FIRMWARE: return "DOWNLOADING FIRMWARE";
        case OtaStatus::FLASHING: return "FLASHING FIRMWARE";
        case OtaStatus::SUCCESS_REBOOTING: return "SUCCESS (REBOOTING)";
        case OtaStatus::ERROR_FAILED: return "OTA ERROR";
        default: return "UNKNOWN";
    }
}

} // namespace System
