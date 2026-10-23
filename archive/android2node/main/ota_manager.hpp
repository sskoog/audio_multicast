#pragma once

#include <cstdint>
#include "esp_err.h"
#include "wifi_credentials.h"

namespace System {

enum class OtaStatus {
    IDLE,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_FAILED,
    DOWNLOADING_FIRMWARE,
    FLASHING,
    SUCCESS_REBOOTING,
    ERROR_FAILED
};

/**
 * @brief Wi-Fi Connection & Over-The-Air (OTA) Firmware Update Manager Class.
 * 
 * Connects the ESP32-C6 to Wi-Fi 6 (802.11ax) network using credentials in wifi_credentials.h
 * and performs HTTP/HTTPS Over-The-Air (OTA) firmware updates.
 */
class OtaManager {
public:
    OtaManager();
    ~OtaManager() = default;

    /**
     * @brief Initialize ESP32-C6 Wi-Fi 6 Station mode and connect to AP.
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t initWifi();

    /**
     * @brief Trigger HTTP/HTTPS OTA Firmware Update from specified URL.
     * @param ota_url Firmware binary HTTP/HTTPS URL (defaults to CONFIG_OTA_FIRMWARE_URL)
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t startOtaUpdate(const char* ota_url = CONFIG_OTA_FIRMWARE_URL);

    /**
     * @brief Check if Wi-Fi is connected.
     */
    bool isWifiConnected() const { return m_wifi_connected; }

    /**
     * @brief Get current OTA status.
     */
    OtaStatus getStatus() const { return m_status; }
    const char* getStatusStr() const;

private:
    bool m_wifi_connected{false};
    OtaStatus m_status{OtaStatus::IDLE};

    static void wifiEventHandler(void* arg, const char* event_base, int32_t event_id, void* event_data);
};

} // namespace System
