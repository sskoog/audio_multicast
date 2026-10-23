#ifndef WEB_DASHBOARD_HPP
#define WEB_DASHBOARD_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace Web {

struct TrackedSinkInfo {
    std::string name;
    uint16_t conn_handle;
    float volume_percent;
    bool is_synced;
    int8_t rssi_dbm;
    uint32_t age_ms;
};

struct DashboardTelemetry {
    std::string node_name;
    std::string bt_state;
    int cpu_mean_pct;
    int cpu_peak_pct;
    int cpu_temp_c;
    uint32_t free_heap_kb;
    uint32_t uptime_sec;
    float vco_freq_hz;
    float vfo_mod_rate_hz;
    float tone_gain_pct;
    uint32_t packets_count;
    bool lfo_enabled;
    uint8_t manual_volume_pct;
    std::vector<TrackedSinkInfo> sinks;
};

typedef std::function<void(bool)> LfoToggleCallback;
typedef std::function<void(uint8_t)> VolumeChangeCallback;
typedef std::function<void(float)> GainChangeCallback;
typedef std::function<void(float)> VcoFreqCallback;

class WebDashboard {
public:
    static WebDashboard& getInstance();

    esp_err_t start(uint16_t port = 80);
    void stop();

    void broadcastTelemetry(const DashboardTelemetry& data);

    void setLfoToggleCallback(LfoToggleCallback cb) { m_lfo_cb = cb; }
    void setVolumeChangeCallback(VolumeChangeCallback cb) { m_vol_cb = cb; }
    void setGainChangeCallback(GainChangeCallback cb) { m_gain_cb = cb; }
    void setVcoFreqCallback(VcoFreqCallback cb) { m_vco_cb = cb; }

    size_t getActiveClientCount() const { return m_client_fds.size(); }

private:
    WebDashboard() = default;
    ~WebDashboard() = default;
    WebDashboard(const WebDashboard&) = delete;
    WebDashboard& operator=(const WebDashboard&) = delete;

    static esp_err_t indexGetHandler(httpd_req_t* req);
    static esp_err_t wsHandler(httpd_req_t* req);

    void sendInitialData(int client_fd);
    void handleIncomingAction(const char* json_str, size_t len);

    httpd_handle_t m_server = nullptr;
    std::vector<int> m_client_fds;
    
    TickType_t m_last_broadcast_tick = 0;
    TickType_t m_last_chart_sample_tick = 0;

    std::vector<int> m_mean_cpu_hist;
    std::vector<int> m_peak_cpu_hist;

    LfoToggleCallback m_lfo_cb = nullptr;
    VolumeChangeCallback m_vol_cb = nullptr;
    GainChangeCallback m_gain_cb = nullptr;
    VcoFreqCallback m_vco_cb = nullptr;
};

} // namespace Web

#endif /* WEB_DASHBOARD_HPP */
