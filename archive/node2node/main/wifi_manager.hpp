#ifndef WIFI_MANAGER_HPP
#define WIFI_MANAGER_HPP

#include <string>
#include <cstdint>
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"

namespace Network {

class WifiManager {
public:
    static WifiManager& getInstance();

    esp_err_t init();
    bool isConnected() const { return m_is_connected; }
    bool isSoftAp() const { return m_is_softap; }
    std::string getIpAddress() const { return m_ip_address; }
    std::string getSsid() const { return m_ssid; }
    int8_t getRssi();

private:
    WifiManager() = default;
    ~WifiManager() = default;
    WifiManager(const WifiManager&) = delete;
    WifiManager& operator=(const WifiManager&) = delete;

    static void wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    static void ipEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    void startSoftAp();

    bool m_is_connected = false;
    bool m_is_softap = false;
    int  m_retry_count = 0;
    std::string m_ip_address = "0.0.0.0";
    std::string m_ssid = "";

    esp_netif_t* m_sta_netif = nullptr;
    esp_netif_t* m_ap_netif = nullptr;
};

} // namespace Network

#endif /* WIFI_MANAGER_HPP */
