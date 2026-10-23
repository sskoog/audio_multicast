#include "wifi_manager.hpp"
#include "wifi_credentials.h"
#include "config.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cstring>

static const char* TAG = "WIFI_MGR";

namespace Network {

WifiManager& WifiManager::getInstance() {
    static WifiManager instance;
    return instance;
}

esp_err_t WifiManager::init() {
    /* 1. Initialize NVS Flash */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. Initialize TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    m_sta_netif = esp_netif_create_default_wifi_sta();
    m_ap_netif  = esp_netif_create_default_wifi_ap();

    /* 3. Initialize Wi-Fi driver */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 4. Register Event Handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &WifiManager::wifiEventHandler,
                                                        this,
                                                        nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &WifiManager::ipEventHandler,
                                                        this,
                                                        nullptr));

    /* 5. Configure AP+STA Mode:
     * - SoftAP is isolated on 192.168.4.1/24 to prevent subnet collision with 192.168.68.0/24.
     * - Station connects to your router and acquires the reserved 192.168.68.21 IP.
     */
    wifi_config_t sta_config = {};
    strncpy((char*)sta_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(sta_config.sta.ssid) - 1);
    strncpy((char*)sta_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(sta_config.sta.password) - 1);
    sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "ESP32-Audio-Source");
    strcpy((char*)ap_config.ap.password, "forestchirp");
    ap_config.ap.ssid_len = strlen("ESP32-Audio-Source");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    m_is_softap = true;
    m_is_connected = false;
    m_ssid = "ESP32-Audio-Source";
    m_ip_address = "192.168.4.1";

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  Connecting to Station SSID: '%s'...", CONFIG_WIFI_SSID);
    ESP_LOGI(TAG, "  Hotspot: 'ESP32-Audio-Source' (pass: forestchirp) @ http://192.168.4.1");
    ESP_LOGI(TAG, "==================================================");

    return ESP_OK;
}

int8_t WifiManager::getRssi() {
    wifi_ap_record_t ap_info = {};
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0;
}

void WifiManager::wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi Station Started, connecting to AP...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi Station Disconnected, reconnecting...");
        esp_wifi_connect();
    }
}

void WifiManager::ipEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    auto* self = static_cast<WifiManager*>(arg);
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(event_data);
        char ip_str[32];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
        self->m_ip_address = ip_str;
        self->m_ssid = CONFIG_WIFI_SSID;
        self->m_is_softap = false;
        self->m_is_connected = true;
        ESP_LOGI(TAG, "==================================================");
        ESP_LOGI(TAG, "  Wi-Fi Station CONNECTED! SSID: '%s'", CONFIG_WIFI_SSID);
        ESP_LOGI(TAG, "  Dashboard Station URL: http://%s", ip_str);
        ESP_LOGI(TAG, "==================================================");
    }
}

} // namespace Network
