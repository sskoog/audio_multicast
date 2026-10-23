#include "web_dashboard.hpp"
#include "dashboard_html_gz.h"
#include "esp_log.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>

static const char* TAG = "WEB_DASH";

namespace Web {

WebDashboard& WebDashboard::getInstance() {
    static WebDashboard instance;
    return instance;
}

esp_err_t WebDashboard::start(uint16_t port) {
    if (m_server) return ESP_OK;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.max_open_sockets = 4;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;
    config.task_priority = 3; /* Low priority: Audio tasks (18-20) take strict precedence */

    ESP_LOGI(TAG, "Starting Native Web Dashboard Server on port %d...", port);
    esp_err_t ret = httpd_start(&m_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 1. Register Index GET handler */
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = WebDashboard::indexGetHandler,
        .user_ctx  = this,
        .is_websocket = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(m_server, &index_uri);

    /* 2. Register WebSocket URI handler */
    httpd_uri_t ws_uri = {
        .uri       = "/ws",
        .method    = HTTP_GET,
        .handler   = WebDashboard::wsHandler,
        .user_ctx  = this,
        .is_websocket = true,
        .handle_ws_control_frames = true,
        .supported_subprotocol = nullptr
    };
    httpd_register_uri_handler(m_server, &ws_uri);

    ESP_LOGI(TAG, "Web Dashboard HTTP & WebSocket endpoints registered successfully.");
    return ESP_OK;
}

void WebDashboard::stop() {
    if (m_server) {
        httpd_stop(m_server);
        m_server = nullptr;
        m_client_fds.clear();
        ESP_LOGI(TAG, "Web Dashboard stopped.");
    }
}

esp_err_t WebDashboard::indexGetHandler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000");

    httpd_resp_send(req, (const char*)dashboard_html_gz, DASHBOARD_HTML_GZ_LEN);
    return ESP_OK;
}

esp_err_t WebDashboard::wsHandler(httpd_req_t* req) {
    auto* self = static_cast<WebDashboard*>(req->user_ctx);

    if (req->method == HTTP_GET) {
        int client_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket Handshake Success! Client FD: %d", client_fd);

        if (self->m_client_fds.size() >= 2) {
            ESP_LOGW(TAG, "Max WebSocket clients reached. Rejecting client FD %d", client_fd);
            return ESP_FAIL;
        }

        if (std::find(self->m_client_fds.begin(), self->m_client_fds.end(), client_fd) == self->m_client_fds.end()) {
            self->m_client_fds.push_back(client_fd);
        }

        self->sendInitialData(client_fd);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {};
    uint8_t buf[512] = {0};
    ws_pkt.payload = buf;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "httpd_ws_recv_frame failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        buf[ws_pkt.len] = '\0';
        self->handleIncomingAction((const char*)buf, ws_pkt.len);
    } else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        int client_fd = httpd_req_to_sockfd(req);
        auto it = std::find(self->m_client_fds.begin(), self->m_client_fds.end(), client_fd);
        if (it != self->m_client_fds.end()) {
            self->m_client_fds.erase(it);
        }
        ESP_LOGI(TAG, "WebSocket Client Closed FD: %d (Remaining: %u)", client_fd, (unsigned int)self->m_client_fds.size());
    }

    return ESP_OK;
}

void WebDashboard::sendInitialData(int client_fd) {
    static const char init_manifest[] = 
    "{"
      "\"type\":\"init\","
      "\"title\":\"Auracast Audio Source\","
      "\"subtitle\":\"ESP32-C6 Dual-Node BLE 5.3 Control & Telemetry\","
      "\"groups\":["
        "{\"id\":\"grp_sys\",\"title\":\"System\",\"cardIds\":[\"card_node\",\"card_heap\",\"card_temp\",\"card_uptime\",\"card_cpu_chart\"]},"
        "{\"id\":\"grp_audio\",\"title\":\"Audio\",\"cardIds\":[\"ctrl_vco\",\"card_vfo\",\"ctrl_lfo\",\"ctrl_gain\",\"ctrl_vol\",\"card_codec\",\"card_channels\",\"card_samplerate\",\"card_bitdepth\"]},"
        "{\"id\":\"grp_bt\",\"title\":\"Bluetooth\",\"cardIds\":[\"card_bt_id\",\"card_bt_devs\",\"card_pkts\"]},"
        "{\"id\":\"grp_sinks\",\"title\":\"Connected SINK Nodes\",\"cardIds\":[\"card_sink_0\",\"card_sink_1\",\"card_sink_2\",\"card_sink_3\",\"card_sink_4\",\"card_sink_5\",\"card_sink_6\",\"card_sink_7\",\"card_sink_8\"]}"
      "],"
      "\"cards\":["
        "{\"id\":\"card_node\",\"type\":\"stat\",\"weight\":1,\"config\":{\"title\":\"Node Name\",\"value\":\"ESP32-C6-21\",\"unit\":\"SOURCE\",\"compact\":true,\"icon\":\"radio\"}},"
        "{\"id\":\"card_heap\",\"type\":\"stat\",\"weight\":2,\"config\":{\"title\":\"Free DRAM\",\"value\":\"221 KB\",\"compact\":true,\"icon\":\"database\"}},"
        "{\"id\":\"card_temp\",\"type\":\"stat\",\"weight\":3,\"config\":{\"title\":\"CPU Temp\",\"value\":\"30 °C\",\"compact\":true,\"icon\":\"thermometer\"}},"
        "{\"id\":\"card_uptime\",\"type\":\"stat\",\"weight\":4,\"config\":{\"title\":\"Uptime\",\"value\":\"0 s\",\"compact\":true,\"icon\":\"clock\"}},"
        "{\"id\":\"card_cpu_chart\",\"type\":\"chart\",\"weight\":5,\"config\":{\"title\":\"CPU Load (2 Min History)\",\"chartType\":\"line\",\"sizeX\":2,\"sizeY\":2,\"min\":0,\"max\":100,\"series\":[{\"name\":\"Mean CPU %\",\"color\":\"primary\",\"data\":[]},{\"name\":\"Peak CPU %\",\"color\":\"danger\",\"data\":[]}]}},"
        "{\"id\":\"ctrl_vco\",\"type\":\"slider\",\"weight\":11,\"config\":{\"title\":\"VCO Tone Freq\",\"value\":440,\"min\":220,\"max\":880,\"step\":10,\"unit\":\"Hz\"}},"
        "{\"id\":\"card_vfo\",\"type\":\"stat\",\"weight\":12,\"config\":{\"title\":\"VFO Mod Rate\",\"value\":\"1.00 Hz\",\"compact\":true,\"icon\":\"activity\"}},"
        "{\"id\":\"ctrl_lfo\",\"type\":\"toggle\",\"weight\":13,\"config\":{\"title\":\"VCS LFO On/Off (0.10 Hz Sine)\",\"value\":true}},"
        "{\"id\":\"ctrl_gain\",\"type\":\"slider\",\"weight\":14,\"config\":{\"title\":\"Tone Generator Gain\",\"value\":30,\"min\":0,\"max\":100,\"step\":5,\"unit\":\"%\"}},"
        "{\"id\":\"ctrl_vol\",\"type\":\"slider\",\"weight\":15,\"config\":{\"title\":\"Manual SINK Volume\",\"value\":30,\"min\":10,\"max\":100,\"step\":1,\"unit\":\"%\"}},"
        "{\"id\":\"card_codec\",\"type\":\"stat\",\"weight\":16,\"config\":{\"title\":\"Streaming Codec & Bitrate\",\"value\":\"LC3 fixp @ 64 kbps\",\"compact\":true,\"icon\":\"disc\"}},"
        "{\"id\":\"card_channels\",\"type\":\"stat\",\"weight\":17,\"config\":{\"title\":\"Audio Channels\",\"value\":\"1 (Mono)\",\"compact\":true,\"icon\":\"volume-2\"}},"
        "{\"id\":\"card_samplerate\",\"type\":\"stat\",\"weight\":18,\"config\":{\"title\":\"Sampling Frequency\",\"value\":\"44.1 kHz\",\"compact\":true,\"icon\":\"layers\"}},"
        "{\"id\":\"card_bitdepth\",\"type\":\"stat\",\"weight\":19,\"config\":{\"title\":\"Bit Depth\",\"value\":\"16-bit PCM\",\"compact\":true,\"icon\":\"hash\"}},"
        "{\"id\":\"card_bt_id\",\"type\":\"stat\",\"weight\":21,\"config\":{\"title\":\"Broadcast BIS Channels (1-9)\",\"value\":\"BIS #1: Active (0x123456)\",\"unit\":\"BIS #2-9: Standby\",\"icon\":\"bluetooth\"}},"
        "{\"id\":\"card_bt_devs\",\"type\":\"stat\",\"weight\":22,\"config\":{\"title\":\"Connected Devices\",\"value\":\"0 / 9\",\"compact\":true,\"icon\":\"users\"}},"
        "{\"id\":\"card_pkts\",\"type\":\"stat\",\"weight\":23,\"config\":{\"title\":\"Transmitted BIS-Frames\",\"value\":\"0\",\"compact\":true,\"icon\":\"send\"}},"
        "{\"id\":\"card_sink_0\",\"type\":\"status\",\"weight\":31,\"config\":{\"title\":\"Connected SINK\",\"sizeX\":2,\"icon\":\"headphones\",\"variant\":\"warning\",\"label\":\"Waiting for SINK...\",\"message\":\"No SINK nodes connected. Audio BIS #1 is streaming.\"}}"
      "]"
    "}";

    httpd_ws_frame_t ws_pkt = {};
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t*)init_manifest;
    ws_pkt.len = strlen(init_manifest);

    httpd_ws_send_frame_async(m_server, client_fd, &ws_pkt);
    ESP_LOGI(TAG, "Sent Initial Grouped Dashboard Manifest to Client FD %d", client_fd);
}

void WebDashboard::handleIncomingAction(const char* json_str, size_t len) {
    ESP_LOGI(TAG, "Received WebSocket Action: %s", json_str);

    if (strstr(json_str, "ctrl_lfo")) {
        bool state = (strstr(json_str, "true") != nullptr);
        if (m_lfo_cb) m_lfo_cb(state);
        ESP_LOGI(TAG, "Action: LFO Toggle set to %s", state ? "ON" : "OFF");
    } else if (strstr(json_str, "ctrl_vol")) {
        const char* val_ptr = strstr(json_str, "\"value\":");
        if (val_ptr) {
            int vol = atoi(val_ptr + 8);
            if (vol < 0) vol = 0;
            if (vol > 100) vol = 100;
            if (m_vol_cb) m_vol_cb(static_cast<uint8_t>(vol));
            ESP_LOGI(TAG, "Action: Manual Volume Override set to %d%%", vol);
        }
    } else if (strstr(json_str, "ctrl_gain")) {
        const char* val_ptr = strstr(json_str, "\"value\":");
        if (val_ptr) {
            float gain = (float)atof(val_ptr + 8);
            if (gain < 0.0f) gain = 0.0f;
            if (gain > 100.0f) gain = 100.0f;
            if (m_gain_cb) m_gain_cb(gain);
            ESP_LOGI(TAG, "Action: Tone Gain set to %.1f%%", gain);
        }
    } else if (strstr(json_str, "ctrl_vco")) {
        const char* val_ptr = strstr(json_str, "\"value\":");
        if (val_ptr) {
            float freq = (float)atof(val_ptr + 8);
            if (freq < 100.0f) freq = 100.0f;
            if (freq > 2000.0f) freq = 2000.0f;
            if (m_vco_cb) m_vco_cb(freq);
            ESP_LOGI(TAG, "Action: VCO Nominal Freq set to %.1f Hz", freq);
        }
    }
}

void WebDashboard::broadcastTelemetry(const DashboardTelemetry& data) {
    if (m_client_fds.empty() || !m_server) return;

    TickType_t now = xTaskGetTickCount();
    /* Rate limit to 3 Hz (every 333 ms) */
    if ((now - m_last_broadcast_tick) < pdMS_TO_TICKS(333)) {
        return;
    }
    m_last_broadcast_tick = now;

    // Sample chart points every 2 seconds for a 2-minute (60 points) history window
    if ((now - m_last_chart_sample_tick) >= pdMS_TO_TICKS(2000) || m_mean_cpu_hist.empty()) {
        m_last_chart_sample_tick = now;
        m_mean_cpu_hist.push_back(data.cpu_mean_pct);
        m_peak_cpu_hist.push_back(data.cpu_peak_pct);
        if (m_mean_cpu_hist.size() > 60) {
            m_mean_cpu_hist.erase(m_mean_cpu_hist.begin());
            m_peak_cpu_hist.erase(m_peak_cpu_hist.begin());
        }
    }

    auto sendFrame = [this](const char* payload, size_t len) {
        httpd_ws_frame_t ws_pkt = {};
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        ws_pkt.payload = (uint8_t*)payload;
        ws_pkt.len = len;

        for (auto it = m_client_fds.begin(); it != m_client_fds.end(); ) {
            esp_err_t ret = httpd_ws_send_frame_async(m_server, *it, &ws_pkt);
            if (ret != ESP_OK) {
                it = m_client_fds.erase(it);
            } else {
                ++it;
            }
        }
    };

    auto sendCardUpdate = [&sendFrame](const char* card_id, const char* val_str) {
        static char payload[256];
        int len = snprintf(payload, sizeof(payload), 
                           "{\"type\":\"update\",\"cardId\":\"%s\",\"data\":{\"value\":\"%s\"}}",
                           card_id, val_str);
        sendFrame(payload, len);
    };

    static char buf[1024];

    // 1. CPU Chart Update (2-minute window)
    static char mean_str[384]; mean_str[0] = '\0';
    static char peak_str[384]; peak_str[0] = '\0';
    int mean_offset = 0;
    int peak_offset = 0;
    for (size_t i = 0; i < m_mean_cpu_hist.size(); ++i) {
        mean_offset += snprintf(mean_str + mean_offset, sizeof(mean_str) - mean_offset, "%s%d", (i > 0 ? "," : ""), m_mean_cpu_hist[i]);
        peak_offset += snprintf(peak_str + peak_offset, sizeof(peak_str) - peak_offset, "%s%d", (i > 0 ? "," : ""), m_peak_cpu_hist[i]);
    }
    int chart_len = snprintf(buf, sizeof(buf),
        "{\"type\":\"update\",\"cardId\":\"card_cpu_chart\",\"data\":{\"series\":[{\"name\":\"Mean CPU %%\",\"color\":\"primary\",\"data\":[%s]},{\"name\":\"Peak CPU %%\",\"color\":\"danger\",\"data\":[%s]}]}}",
        mean_str, peak_str);
    sendFrame(buf, chart_len);

    // 2. System Stats
    snprintf(buf, sizeof(buf), "%lu KB", data.free_heap_kb);
    sendCardUpdate("card_heap", buf);

    snprintf(buf, sizeof(buf), "%d °C", data.cpu_temp_c);
    sendCardUpdate("card_temp", buf);

    snprintf(buf, sizeof(buf), "%lu s", data.uptime_sec);
    sendCardUpdate("card_uptime", buf);

    // 3. Audio Stats
    snprintf(buf, sizeof(buf), "%.2f Hz", data.vfo_mod_rate_hz);
    sendCardUpdate("card_vfo", buf);

    // 4. Bluetooth Stats
    snprintf(buf, sizeof(buf), "%u / 9", (unsigned int)data.sinks.size());
    sendCardUpdate("card_bt_devs", buf);

    snprintf(buf, sizeof(buf), "%lu", data.packets_count);
    sendCardUpdate("card_pkts", buf);

    // 5. Dynamic SINK Cards Update
    if (data.sinks.empty()) {
        static char sink_payload[384];
        int s_len = snprintf(sink_payload, sizeof(sink_payload),
            "{\"type\":\"update\",\"cardId\":\"card_sink_0\",\"data\":{\"title\":\"Connected SINK\",\"icon\":\"headphones\",\"variant\":\"warning\",\"label\":\"Waiting for SINK...\",\"message\":\"No SINK nodes connected. Audio BIS #1 is streaming.\"}}");
        sendFrame(sink_payload, s_len);
    } else {
        for (size_t i = 0; i < data.sinks.size() && i < 9; ++i) {
            const auto& sink = data.sinks[i];
            char sink_id[32];
            snprintf(sink_id, sizeof(sink_id), "card_sink_%u", (unsigned int)i);

            static char sink_payload[384];
            int s_len = snprintf(sink_payload, sizeof(sink_payload),
                "{\"type\":\"update\",\"cardId\":\"%s\",\"data\":{\"title\":\"%s\",\"icon\":\"headphones\",\"variant\":\"success\",\"label\":\"Volume: %.0f%%\",\"message\":\"Device: SINK Headset | RSSI: %d dBm | Handle: %u | %s | Age: %lu ms\"}}",
                sink_id,
                sink.name.c_str(),
                sink.volume_percent,
                sink.rssi_dbm,
                sink.conn_handle,
                sink.is_synced ? "SYNCED (BIS #1)" : "CONNECTED",
                sink.age_ms);
            sendFrame(sink_payload, s_len);
        }
    }
}

} // namespace Web
