#include "vsaf_broadcast_engine.hpp"
#include "status_led.hpp"
#include "console.hpp"
#include "usb_audio.hpp"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_mac.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "esp_netif.h"
#include <cmath>

static const char* TAG = "VSAF_BCAST";

namespace AudioNet {

uint8_t EspNowBroadcastEngine::s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
SemaphoreHandle_t EspNowBroadcastEngine::s_tx_done_sem = nullptr;
std::atomic<esp_now_send_status_t> EspNowBroadcastEngine::s_last_tx_status{ESP_NOW_SEND_SUCCESS};
static EspNowBroadcastEngine* s_engine_instance = nullptr;

// Promiscuous Sniffer Statistics Trackers
static volatile uint32_t s_promis_pkt_count = 0;
static volatile uint32_t s_promis_byte_count = 0;
static volatile int8_t   s_promis_max_rssi = -128;
static volatile int64_t  s_promis_sum_rssi = 0;
static volatile uint32_t s_promis_40mhz_count = 0;

static void IRAM_ATTR wifi_promiscuous_sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (!buf) return;
    const wifi_promiscuous_pkt_t* pkt = reinterpret_cast<const wifi_promiscuous_pkt_t*>(buf);
    s_promis_pkt_count = s_promis_pkt_count + 1;
    s_promis_byte_count = s_promis_byte_count + pkt->rx_ctrl.sig_len;
    int8_t rssi = pkt->rx_ctrl.rssi;
    if (rssi > s_promis_max_rssi) {
        s_promis_max_rssi = rssi;
    }
    s_promis_sum_rssi = s_promis_sum_rssi + rssi;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    if (pkt->rx_ctrl.second != 0) {
        s_promis_40mhz_count = s_promis_40mhz_count + 1;
    }
#else
    if (pkt->rx_ctrl.cwb == 1) {
        s_promis_40mhz_count = s_promis_40mhz_count + 1;
    }
#endif
}

void IRAM_ATTR EspNowBroadcastEngine::frameTimerCb(void* arg) {
    auto* engine = static_cast<EspNowBroadcastEngine*>(arg);
    if (engine && engine->m_audio_dsp_task_handle) {
        xTaskNotifyGive(engine->m_audio_dsp_task_handle);
    }
}

static void IRAM_ATTR onEspNowSendCb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
    EspNowBroadcastEngine::s_last_tx_status.store(status, std::memory_order_relaxed);
    if (EspNowBroadcastEngine::s_tx_done_sem) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(EspNowBroadcastEngine::s_tx_done_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void IRAM_ATTR onEspNowRecvCb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
    if (!s_engine_instance || !data || data_len < 2) return;
    int8_t rssi = (esp_now_info->rx_ctrl) ? esp_now_info->rx_ctrl->rssi : -127;
    uint8_t rate = (esp_now_info->rx_ctrl) ? esp_now_info->rx_ctrl->rate : 0;
    s_engine_instance->onPacketReceived(esp_now_info->src_addr, data, data_len, rssi, rate);
}

EspNowBroadcastEngine::EspNowBroadcastEngine(Codec::Lc3CodecEngine& primary_codec,
                                             Audio::ToneGenerator* primary_tone_gen,
                                             Hardware::I2sAudioDriver* i2s_dac)
    : m_lc3_codec(primary_codec),
      m_tone_gen(primary_tone_gen),
      m_i2s_dac(i2s_dac),
      m_node_role(NODE_ROLE_SINK),
      m_node_id(0),
      m_wifi_channel(1),
      m_target_channel(0),
      m_state(NetworkState::OFF),
      m_tone_test_mode(false),
      m_is_stereo(true),
      m_tx_phy_mode(WIFI_PHY_MODE_HT20),
      m_tx_phy_rate(WIFI_PHY_RATE_MCS3_LGI),
      m_peer_count(MAX_SINK_NODES),
      m_seq(0),
      m_octets_per_frame(CONFIG_ESPNOW_FRAME_LEN_OCTETS),
      m_frame_duration_us(10000),
      m_target_volume_u8(255),
      m_target_gain_db(0.0f),
      m_current_gain_db(0.0f),
      m_post_gain_db(0.0f),
      m_instant_vol_update(false),
      m_audio_dsp_task_handle(nullptr),
      m_sink_task_handle(nullptr),
      m_ema_time_offset_ms(0.0f),
      m_last_master_time_us(0),
      m_last_local_time_us(0),
      m_last_rssi(-127),
      m_plc_count(0),
      m_fifo_underflows(0),
      m_fifo_overflows(0),
      m_redundancy_recovered_packets(0),
      m_last_rx_seq(0xFF),
      m_first_packet_received(false),
      m_expected_seq(0),
      m_has_expected_seq(false),
      m_sink_fifo_head(0),
      m_sink_fifo_tail(0),
      m_sink_fifo_count(0),
      m_tx_packets_this_sec(0),
      m_rx_acks_this_sec(0),
      m_stats_packets_per_sec(0),
      m_stats_acks_per_sec(0),
      m_stats_ack_percentage(100.0f)
{
    s_engine_instance = this;
    m_peer_lock = portMUX_INITIALIZER_UNLOCKED;
    m_sink_fifo_lock = portMUX_INITIALIZER_UNLOCKED;

    m_hpf_stereo.initHighPass(100.0f, static_cast<float>(m_telemetry.sample_rate));
    m_sub_decimator.init(100.0f, static_cast<float>(m_telemetry.sample_rate));

    memset(m_prev1_encoded_sat, 0, sizeof(m_prev1_encoded_sat));
    memset(m_prev2_encoded_sat, 0, sizeof(m_prev2_encoded_sat));
    memset(m_prev1_sat_valid, 0, sizeof(m_prev1_sat_valid));
    memset(m_prev2_sat_valid, 0, sizeof(m_prev2_sat_valid));

    memset(m_prev1_encoded_sub, 0, sizeof(m_prev1_encoded_sub));
    memset(m_prev2_encoded_sub, 0, sizeof(m_prev2_encoded_sub));
    memset(m_prev3_encoded_sub, 0, sizeof(m_prev3_encoded_sub));
    m_prev1_sub_valid = false;
    m_prev2_sub_valid = false;
    m_prev3_sub_valid = false;

    // Initialize 6 SINK peer configurations
    const char* default_names[MAX_SINK_NODES] = {
        "Sink-Left", "Sink-Right", "Sink-Center",
        "Sink-LSurr", "Sink-RSurr", "Sink-Sub"
    };

    for (size_t i = 0; i < MAX_SINK_NODES; ++i) {
        m_peers[i].channel_id = static_cast<uint8_t>(i);
        strncpy(m_peers[i].name, default_names[i], sizeof(m_peers[i].name) - 1);
        m_peers[i].is_enabled = true; // All 6 channels broadcast unconditionally in CAST
        m_peers[i].status = PeerStatus::OFFLINE; // Online when Soft-ACK received
        m_peers[i].session_start_time_us = 0;
        m_peers[i].last_rx_telemetry_time_us = 0;
        m_peers[i].packets_sent = 0;
        m_peers[i].acks_received = 0;
        m_peers[i].ack_failures = 0;
        m_peers[i].consecutive_ack_fails = 0;
        m_peers[i].arq_retries = 0;
        m_peers[i].arq_successes = 0;
        m_peers[i].last_rssi = -127;
        m_peers[i].downlink_rssi = -127;
        m_peers[i].last_rtt_us = 0;
        m_peers[i].avg_rtt_us = 0;
        m_peers[i].last_total_rtt_us = 0;
        m_peers[i].last_dwell_us = 0;
        m_peers[i].last_net_rtt_us = 0;
        m_peers[i].clock_offset_us = 0;
        m_peers[i].fifo_fill = 50;
        m_sink_acked[i] = false;
    }
}

EspNowBroadcastEngine::~EspNowBroadcastEngine() {
    stop();
    s_engine_instance = nullptr;
}

esp_err_t EspNowBroadcastEngine::init(uint8_t role, uint8_t node_id, uint8_t wifi_channel) {
    m_node_role = role;
    m_node_id = node_id;
    m_wifi_channel = (wifi_channel >= 1 && wifi_channel <= 13) ? wifi_channel : 1;

    if (!s_tx_done_sem) {
        s_tx_done_sem = xSemaphoreCreateBinary();
        xSemaphoreGive(s_tx_done_sem);
    }

    // 0. Initialize TCP/IP and default event loop (required for Wi-Fi driver event dispatching)
    esp_err_t netif_err = esp_netif_init();
    if (netif_err != ESP_OK && netif_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(netif_err));
    }
    esp_err_t loop_err = esp_event_loop_create_default();
    if (loop_err != ESP_OK && loop_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(loop_err));
    }

    // 1. Initialize Wi-Fi Station
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.static_rx_buf_num = 32;
    cfg.dynamic_rx_buf_num = 64;
    cfg.static_tx_buf_num = 16;
    cfg.dynamic_tx_buf_num = 64;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE));

    // Standard 2.4 GHz protocols (11b/g/n on S3, 11b/g/n/ax on C6)
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX));
#else
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(12)); // +3.00 dBm (12 * 0.25 dBm)

    // 2. Wi-Fi RF Sniffer Scan on SOURCE Node (Auto-select cleanest channel at bootup)
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_wifi_channel = scanAndSelectBestChannel(120, true, true);
    }

    // 3. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(onEspNowSendCb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(onEspNowRecvCb));

    // 4. Register Single Broadcast Peer (FF:FF:FF:FF:FF:FF) with Primary Default PHY
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, s_broadcast_mac, 6);
    peer_info.channel = 0; // 0 = follow current STA interface channel
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to register broadcast peer: %s", esp_err_to_name(ret));
        return ret;
    }

    if (m_node_role == NODE_ROLE_SOURCE) {
        setWifiPhyProfile(WifiPhyProfile::PRIMARY_HT20_MCS3);
    } else {
        setWifiPhyProfile(WifiPhyProfile::TERTIARY_HT20_MCS0); // Robust MCS0 for SINK Soft-ACK telemetry replies
    }

    // 5. Initialize Codec (SOURCE: 5 encoders - Left HQ 48k, Right HQ 48k, Left Red 48k, Right Red 48k, Sub 8k)
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.initEncoder(m_telemetry.sample_rate, 5, m_frame_duration_us, m_octets_per_frame);
    } else {
        m_lc3_codec.initDecoder(m_telemetry.sample_rate, 1, m_frame_duration_us, m_octets_per_frame);
    }

    transitionTo(NetworkState::IDLE);
    ESP_LOGI(TAG, "EspNowBroadcastEngine initialized successfully (Role: %s, Node ID: %d, Ch: %d, Rate: %s)",
             (m_node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK", m_node_id, m_wifi_channel, getWifiPhyRateString());
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::start() {
    if (m_running.load(std::memory_order_acquire)) {
        return ESP_OK;
    }
    m_running.store(true, std::memory_order_release);

    if (m_node_role == NODE_ROLE_SOURCE) {
        transitionTo(NetworkState::CAST);
        if (s_tx_done_sem) {
            xSemaphoreTake(s_tx_done_sem, 0);
            xSemaphoreGive(s_tx_done_sem);
        }

        // 1. Create Audio DSP, LC3 Encoder, and Broadcast TX Task on Core 1 (Priority 6, 8KB stack, Hardware FPU)
        xTaskCreatePinnedToCore(audioDspTaskTrampoline, "audio_dsp_task", 8192, this, 6, &m_audio_dsp_task_handle, 1);

        // 2. Configure hardware periodic frame timer (10.0 ms / 7.5 ms)
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = frameTimerCb;
        timer_args.arg = this;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "vsaf_tx_timer";
        timer_args.skip_unhandled_events = true;

        esp_err_t err = esp_timer_create(&timer_args, &m_frame_timer);
        if (err == ESP_OK) {
            esp_timer_start_periodic(m_frame_timer, m_frame_duration_us);
            ESP_LOGI(TAG, "Hardware frame timer started (%lu us periodic)", (unsigned long)m_frame_duration_us);
        } else {
            ESP_LOGE(TAG, "Failed to create frame timer: %s", esp_err_to_name(err));
        }
    } else {
        transitionTo(NetworkState::SCANNING);
        xTaskCreatePinnedToCore(sinkTaskTrampoline, "bcast_snk_task", 16384, this, 6, &m_sink_task_handle, 0);
    }
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::stop() {
    m_running.store(false, std::memory_order_release);
    transitionTo(NetworkState::IDLE);
    if (m_frame_timer) {
        esp_timer_stop(m_frame_timer);
        esp_timer_delete(m_frame_timer);
        m_frame_timer = nullptr;
    }
    if (m_audio_dsp_task_handle) {
        vTaskDelete(m_audio_dsp_task_handle);
        m_audio_dsp_task_handle = nullptr;
    }
    if (m_sink_task_handle) {
        vTaskDelete(m_sink_task_handle);
        m_sink_task_handle = nullptr;
    }
    return ESP_OK;
}

/**
 * @brief Transitions the broadcast/unicast engine state machine to a new network state.
 *
 * Automatically manages dynamic Wi-Fi power save modes:
 * - IDLE, SCANNING, OFF: Sets WIFI_PS_MIN_MODEM (default power save during inactive/search phases)
 * - PREFILL, STREAM, CAST: Sets WIFI_PS_NONE (zero sleep latency and 100% active baseband for streaming)
 *
 * @param new_state Target network state (NetworkState enum: IDLE, SCANNING, PREFILL, STREAM, CAST, OFF)
 */
void EspNowBroadcastEngine::transitionTo(NetworkState new_state) {
    m_state.store(new_state, std::memory_order_release);

    // Dynamic Wi-Fi Power Save Management:
    // - IDLE, OFF: WIFI_PS_MIN_MODEM (default modem sleep when inactive or paused)
    // - SCANNING, PREFILL, STREAM, CAST: WIFI_PS_NONE (continuous full-power radio for scanning packet discovery and streaming)
    if (new_state == NetworkState::IDLE || new_state == NetworkState::OFF) {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    } else if (new_state == NetworkState::SCANNING || new_state == NetworkState::PREFILL ||
               new_state == NetworkState::STREAM || new_state == NetworkState::CAST) {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }

    // Synchronize Status LED with network state
    switch (new_state) {
        case NetworkState::OFF:
            Hardware::getStatusLed().off();
            break;
        case NetworkState::IDLE:
            Hardware::getStatusLed().setSystemState(Hardware::SystemState::IDLE);
            break;
        case NetworkState::SCANNING:
            Hardware::getStatusLed().setSystemState(Hardware::SystemState::SCANNING);
            break;
        case NetworkState::PREFILL:
            // SINK jitter buffer prefill: brief transitional phase
            break;
        case NetworkState::STREAM:
            Hardware::getStatusLed().setSystemState(Hardware::SystemState::STREAM);
            break;
        case NetworkState::CAST:
            if (m_node_role == NODE_ROLE_SOURCE) {
                Hardware::getStatusLed().setSystemState(Hardware::SystemState::BROADCASTING_TONE);
            }
            break;
    }

    if (m_node_role == NODE_ROLE_SOURCE) {
        if (new_state == NetworkState::CAST) {
            usb_audio_clear_buffer();
        }
    }

    if (m_node_role == NODE_ROLE_SINK) {
        if (new_state == NetworkState::IDLE || new_state == NetworkState::SCANNING) {
            if (m_i2s_dac) {
                m_i2s_dac->stop();
            }
            portENTER_CRITICAL(&m_sink_fifo_lock);
            m_sink_fifo_count = 0;
            m_sink_fifo_head = 0;
            m_sink_fifo_tail = 0;
            portEXIT_CRITICAL(&m_sink_fifo_lock);
            m_has_expected_seq = false;
            m_first_packet_received = false;

            // Reset all counters when transitioning out from streaming
            resetStreamingCounters();
        }

        if (m_sink_task_handle) {
            xTaskNotifyGive(m_sink_task_handle);
        }
    }
}

const char* EspNowBroadcastEngine::getStateString() const {
    switch (m_state.load(std::memory_order_acquire)) {
        case NetworkState::OFF:      return "OFF";
        case NetworkState::IDLE:     return "IDLE";
        case NetworkState::SCANNING: return "SCAN";
        case NetworkState::PREFILL:  return "PREFILL";
        case NetworkState::STREAM:   return "STREAM";
        case NetworkState::CAST:     return "CAST";
        default:                     return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
// SINK Peer Management (Channels 0 to 5)
// ---------------------------------------------------------------------------

bool EspNowBroadcastEngine::addPeer(uint8_t channel_id, const char* name) {
    if (channel_id >= MAX_SINK_NODES) return false;
    portENTER_CRITICAL(&m_peer_lock);
    m_peers[channel_id].is_enabled = true;
    m_peers[channel_id].status = PeerStatus::ONLINE;
    if (name) {
        strncpy(m_peers[channel_id].name, name, sizeof(m_peers[channel_id].name) - 1);
    }
    portEXIT_CRITICAL(&m_peer_lock);
    return true;
}

bool EspNowBroadcastEngine::removePeer(uint8_t channel_id) {
    if (channel_id >= MAX_SINK_NODES) return false;
    portENTER_CRITICAL(&m_peer_lock);
    m_peers[channel_id].is_enabled = false;
    m_peers[channel_id].status = PeerStatus::DISABLED;
    portEXIT_CRITICAL(&m_peer_lock);
    return true;
}

bool EspNowBroadcastEngine::setPeerEnabled(uint8_t channel_id, bool enabled) {
    if (channel_id >= MAX_SINK_NODES) return false;
    portENTER_CRITICAL(&m_peer_lock);
    m_peers[channel_id].is_enabled = enabled;
    m_peers[channel_id].status = enabled ? PeerStatus::ONLINE : PeerStatus::DISABLED;
    portEXIT_CRITICAL(&m_peer_lock);
    return true;
}

const SinkPeerConfig* EspNowBroadcastEngine::getPeer(int index) const {
    if (index < 0 || index >= static_cast<int>(MAX_SINK_NODES)) return nullptr;
    return &m_peers[index];
}

void EspNowBroadcastEngine::resetPeerStats() {
    portENTER_CRITICAL(&m_peer_lock);
    for (size_t i = 0; i < MAX_SINK_NODES; ++i) {
        m_peers[i].packets_sent = 0;
        m_peers[i].acks_received = 0;
        m_peers[i].ack_failures = 0;
        m_peers[i].consecutive_ack_fails = 0;
        m_peers[i].arq_retries = 0;
        m_peers[i].arq_successes = 0;
    }
    portEXIT_CRITICAL(&m_peer_lock);
}

void EspNowBroadcastEngine::getNodeStatusString(char* out_buf, size_t max_len) const {
    if (!out_buf || max_len < 7) return;
    for (size_t i = 0; i < 6; ++i) {
        if (!m_peers[i].is_enabled) {
            out_buf[i] = '-';
        } else if (m_peers[i].status == PeerStatus::ONLINE) {
            out_buf[i] = '1';
        } else {
            out_buf[i] = 'O'; // Offline
        }
    }
    out_buf[6] = '\0';
}

void EspNowBroadcastEngine::setTargetChannel(uint8_t channel_id) {
    m_target_channel = channel_id % MAX_SINK_NODES;
    ESP_LOGI(TAG, "SINK Target Channel set to %d", m_target_channel);
}

// ---------------------------------------------------------------------------
// Volume Control
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::setVolume(uint8_t vol_u8, bool instant) {
    m_target_volume_u8.store(vol_u8, std::memory_order_relaxed);
    if (vol_u8 == 0) {
        m_target_gain_db = -96.0f;
    } else {
        m_target_gain_db = (static_cast<float>(vol_u8 - 255) / 255.0f) * 48.0f; // 0..-48 dB
    }
    if (instant) {
        m_current_gain_db = m_target_gain_db;
    }
}

float EspNowBroadcastEngine::getTargetVolumeDb() const {
    return m_target_gain_db;
}

void EspNowBroadcastEngine::sendVolumeCommand(uint8_t channel_id, uint8_t vol_u8, bool instant) {
    setVolume(vol_u8, instant);
}

// ---------------------------------------------------------------------------
// Dynamic Stream Configuration
// ---------------------------------------------------------------------------

esp_err_t EspNowBroadcastEngine::setSampleRate(uint32_t sample_rate_hz) {
    m_telemetry.sample_rate = sample_rate_hz;
    m_hpf_stereo.initHighPass(100.0f, static_cast<float>(sample_rate_hz));
    m_sub_decimator.init(100.0f, static_cast<float>(sample_rate_hz));
    m_lc3_codec.reconfigureDecoder(sample_rate_hz, m_octets_per_frame, m_frame_duration_us);
    if (m_i2s_dac) {
        m_i2s_dac->reconfigureSampleRate(sample_rate_hz, m_frame_duration_us);
    }
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::setBitDepth(uint8_t bit_depth) {
    m_telemetry.bit_depth = bit_depth;
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::setFrameLen(uint16_t frame_len_octets) {
    m_octets_per_frame = frame_len_octets;
    m_telemetry.frame_len = frame_len_octets;
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::setFrameDuration(uint32_t frame_duration_us) {
    m_frame_duration_us = frame_duration_us;
    m_telemetry.frame_duration_us = frame_duration_us;
    if (m_frame_timer && m_running.load(std::memory_order_acquire) && m_node_role == NODE_ROLE_SOURCE) {
        esp_timer_stop(m_frame_timer);
        esp_timer_start_periodic(m_frame_timer, m_frame_duration_us);
    }
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::setWifiChannel(uint8_t channel) {
    if (channel < 1 || channel > 13) return ESP_ERR_INVALID_ARG;
    m_wifi_channel = channel;
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_channel(%d) failed: %s", channel, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::setWifiTxPower(int8_t power_0_25dbm) {
    if (power_0_25dbm < 8) power_0_25dbm = 8;
    if (power_0_25dbm > 84) power_0_25dbm = 84;
    return esp_wifi_set_max_tx_power(power_0_25dbm);
}

int8_t EspNowBroadcastEngine::getWifiTxPower() const {
    int8_t pwr = 0;
    if (esp_wifi_get_max_tx_power(&pwr) == ESP_OK) {
        return pwr;
    }
    return 78;
}

uint8_t EspNowBroadcastEngine::scanAndSelectBestChannel(uint32_t dwell_ms_per_ch, bool auto_apply, bool print_results) {
    WifiChannelScanResult results[14]; // Index 1..13
    uint8_t best_ch = 1;
    uint32_t lowest_score = 0xFFFFFFFF;

    wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_sniffer_cb);
    esp_wifi_set_promiscuous(true);

    if (print_results) {
        print_console("\n[RF SNIFFER] Passive spectrum survey across channels 1..13 (%lu ms/ch)...\n",
                      (unsigned long)dwell_ms_per_ch);
    }

    for (uint8_t ch = 1; ch <= 13; ++ch) {
        s_promis_pkt_count = 0;
        s_promis_byte_count = 0;
        s_promis_max_rssi = -128;
        s_promis_sum_rssi = 0;
        s_promis_40mhz_count = 0;

        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms_per_ch));

        results[ch].channel = ch;
        results[ch].packet_count = s_promis_pkt_count;
        results[ch].byte_count = s_promis_byte_count;
        results[ch].max_rssi = (s_promis_pkt_count > 0) ? s_promis_max_rssi : -100;
        results[ch].avg_rssi = (s_promis_pkt_count > 0) ? static_cast<int8_t>(s_promis_sum_rssi / s_promis_pkt_count) : -100;
        results[ch].has_40mhz = (s_promis_40mhz_count > 0);

        // Scoring Formula: lower is cleaner
        uint32_t score = (results[ch].packet_count * 2) + (results[ch].byte_count / 512);
        if (results[ch].packet_count > 0) {
            if (results[ch].max_rssi > -60) {
                score += (60 + results[ch].max_rssi) * 5;
            } else if (results[ch].max_rssi > -75) {
                score += (75 + results[ch].max_rssi) * 2;
            }
        }
        if (results[ch].has_40mhz) {
            score += 150;
        }
        // Non-overlapping channel bonus (1, 6, 11)
        if ((ch == 1 || ch == 6 || ch == 11) && score > 5) {
            score -= 5;
        }
        results[ch].score = score;

        if (score < lowest_score) {
            lowest_score = score;
            best_ch = ch;
        }
    }

    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);

    if (print_results) {
        print_console("\n================================= 802.11 RF SPECTRUM SURVEY =================================\n");
        print_console(" Ch | Packets |   Bytes | Max RSSI | Avg RSSI | 40MHz | Score | Cleanliness Rating\n");
        print_console("----+---------+---------+----------+----------+-------+-------+------------------------------\n");
        for (uint8_t ch = 1; ch <= 13; ++ch) {
            const char* rating = "CLEAN";
            if (results[ch].score == 0) rating = "PERFECT (Silent)";
            else if (results[ch].score < 50) rating = "EXCELLENT";
            else if (results[ch].score < 150) rating = "GOOD";
            else if (results[ch].score < 350) rating = "MODERATE NOISE";
            else rating = "CONGESTED";

            char sel_mark[20] = "";
            if (ch == best_ch) {
                snprintf(sel_mark, sizeof(sel_mark), " <-- SELECTED");
            }
            print_console(" %2u | %7lu | %6luB |  %4d dBm |  %4d dBm |  %3s  | %5lu | %-16s%s\n",
                          ch,
                          (unsigned long)results[ch].packet_count,
                          (unsigned long)results[ch].byte_count,
                          results[ch].max_rssi,
                          results[ch].avg_rssi,
                          results[ch].has_40mhz ? "YES" : "No",
                          (unsigned long)results[ch].score,
                          rating,
                          sel_mark);
        }
        print_console("==============================================================================================\n");
        print_console("[RF SNIFFER] Cleanest Wi-Fi channel: Channel %u (Score: %lu)\n\n",
                      best_ch, (unsigned long)lowest_score);
    }

    if (auto_apply) {
        setWifiChannel(best_ch);
    } else {
        esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE);
    }

    return best_ch;
}

esp_err_t EspNowBroadcastEngine::setWifiPhyProfile(WifiPhyProfile profile) {
    switch (profile) {
        case WifiPhyProfile::PRIMARY_HT20_MCS3:
            return setWifiPhyRate(WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS3_LGI);
        case WifiPhyProfile::SECONDARY_OFDM_12M:
            return setWifiPhyRate(WIFI_PHY_MODE_11G, WIFI_PHY_RATE_12M);
        case WifiPhyProfile::TERTIARY_HT20_MCS0:
            return setWifiPhyRate(WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS0_LGI);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t EspNowBroadcastEngine::setWifiPhyRate(wifi_phy_mode_t phymode, wifi_phy_rate_t rate) {
    m_tx_phy_mode = phymode;
    m_tx_phy_rate = rate;
    esp_now_rate_config_t rate_cfg = {
        .phymode = phymode,
        .rate = rate,
        .ersu = false,
        .dcm = false
    };
    return esp_now_set_peer_rate_config(s_broadcast_mac, &rate_cfg);
}

const char* EspNowBroadcastEngine::getWifiPhyRateString() const {
    switch (m_tx_phy_rate) {
        case WIFI_PHY_RATE_MCS3_LGI:
        case WIFI_PHY_RATE_MCS3_SGI: return "HT3";
        case WIFI_PHY_RATE_12M:      return "OFD";
        case WIFI_PHY_RATE_MCS0_LGI:
        case WIFI_PHY_RATE_MCS0_SGI: return "HT0";
        case WIFI_PHY_RATE_MCS1_LGI:
        case WIFI_PHY_RATE_MCS1_SGI: return "HT1";
        case WIFI_PHY_RATE_MCS2_LGI:
        case WIFI_PHY_RATE_MCS2_SGI: return "HT2";
        case WIFI_PHY_RATE_MCS4_LGI:
        case WIFI_PHY_RATE_MCS4_SGI: return "HT4";
        case WIFI_PHY_RATE_MCS5_LGI:
        case WIFI_PHY_RATE_MCS5_SGI: return "HT5";
        case WIFI_PHY_RATE_MCS6_LGI:
        case WIFI_PHY_RATE_MCS6_SGI: return "HT6";
        case WIFI_PHY_RATE_MCS7_LGI:
        case WIFI_PHY_RATE_MCS7_SGI: return "HT7";
        case WIFI_PHY_RATE_24M:      return "24M";
        case WIFI_PHY_RATE_18M:      return "18M";
        case WIFI_PHY_RATE_36M:      return "36M";
        case WIFI_PHY_RATE_48M:      return "48M";
        case WIFI_PHY_RATE_54M:      return "54M";
        case WIFI_PHY_RATE_6M:       return "6M";
        case WIFI_PHY_RATE_9M:       return "9M";
        case WIFI_PHY_RATE_1M_L:     return "1M";
        default:                     return "PHY";
    }
}

// ---------------------------------------------------------------------------
// Packet Callbacks
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::onPacketSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    // Handled via ISR semaphore give
}

void EspNowBroadcastEngine::onPacketReceived(const uint8_t* mac_addr, const uint8_t* data, int data_len, int8_t rssi, uint8_t rate) {
    if (!data || data_len < 2) return;
    m_raw_espnow_rx_count++;
    int64_t t_now_us = esp_timer_get_time();

    uint16_t type_id = *reinterpret_cast<const uint16_t*>(data);

    // =======================================================================
    // SINK NODE LOGIC: Receive Audio Broadcast
    // =======================================================================
    if (m_node_role == NODE_ROLE_SINK) {
        if (m_state.load(std::memory_order_acquire) == NetworkState::IDLE) {
            return;
        }

        if ((type_id == VSAF_TYPE_AUDIO_SATELLITE && data_len >= static_cast<int>(sizeof(vsaf_audio_packet_t))) ||
            (type_id == VSAF_TYPE_AUDIO_SUBWOOFER && data_len >= static_cast<int>(sizeof(vsaf_sub_packet_t)))) {
            m_raw_audio_pkt_count++;
            m_channel_locked.store(true, std::memory_order_release);
            m_last_rx_audio_pkt_us.store(t_now_us, std::memory_order_release);

            uint8_t flags = data[2];
            uint8_t rx_id = get_flags_rx_id(flags);
            m_last_seen_rx_id.store(rx_id, std::memory_order_relaxed);
            // Instant filter: Reject if not for our channel and not wildcard broadcast (7)
            if (rx_id != m_target_channel && rx_id != NODE_ID_BROADCAST) {
                return;
            }
            handleAudioPacket(data, data_len, rssi, t_now_us);
        }
        return;
    }

    // =======================================================================
    // SOURCE NODE LOGIC: Receive Round-Robin SINK Telemetry Feedback
    // =======================================================================
    if (m_node_role == NODE_ROLE_SOURCE) {
        if (type_id == VSAF_TYPE_SINK_TELEMETRY && data_len >= static_cast<int>(sizeof(vsaf_sink_telemetry_t))) {
            const auto* ack = reinterpret_cast<const vsaf_sink_telemetry_t*>(data);

            // Verify CRC-16 (over first 12 bytes of telemetry structure)
            uint16_t computed_crc = calc_crc16(data, sizeof(vsaf_sink_telemetry_t) - 4);
            if (computed_crc != ack->crc16) {
                return;
            }

            uint8_t sink_id = ack->sink_id;
            if (sink_id < MAX_SINK_NODES) {
                // Compute RTT metrics
                int64_t total_rtt = t_now_us - ack->t_tx1_echo;
                int64_t dwell_us = ack->t_dwell_us;
                int64_t net_rtt = total_rtt - dwell_us;
                if (total_rtt >= 0 && total_rtt < 30000) {
                    m_peers[sink_id].last_total_rtt_us = static_cast<uint32_t>(total_rtt);
                    m_peers[sink_id].last_dwell_us = static_cast<uint32_t>(dwell_us);
                    m_peers[sink_id].last_net_rtt_us = (net_rtt >= 0) ? static_cast<uint32_t>(net_rtt) : 0;
                    m_peers[sink_id].last_rtt_us = m_peers[sink_id].last_net_rtt_us;
                    m_peers[sink_id].avg_rtt_us = (m_peers[sink_id].avg_rtt_us == 0)
                        ? m_peers[sink_id].last_net_rtt_us
                        : ((m_peers[sink_id].avg_rtt_us * 7 + m_peers[sink_id].last_net_rtt_us) / 8);
                }

                // Dual-Way PTP Time Sync calculation
                int64_t offset = ((ack->t_dwell_us) - (total_rtt)) / 2;
                m_peers[sink_id].clock_offset_us = static_cast<int32_t>(offset);
                m_time_offset_buf.push(offset / 1000.0f);
                m_ema_time_offset_ms = (m_ema_time_offset_ms * 0.9f) + (offset / 1000.0f * 0.1f);

                // Update signal quality & status
                m_peers[sink_id].last_rssi = rssi; // Uplink RSSI
                m_peers[sink_id].downlink_rssi = ack->downlink_rssi; // Downlink RSSI
                m_peers[sink_id].fifo_fill = ack->fifo_fill;
                m_peers[sink_id].acks_received++;
                m_peers[sink_id].consecutive_ack_fails = 0;
                m_peers[sink_id].last_rx_telemetry_time_us = t_now_us;
                m_peers[sink_id].status = PeerStatus::ONLINE;

                m_rx_acks_this_sec++;
                m_tx_acks_total++;
                m_tx_acks_sec++;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// SINK Internal Helpers
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::handleAudioPacket(const uint8_t* data, size_t data_len, int8_t rssi, int64_t t_rx1_us) {
    uint16_t type_id = *reinterpret_cast<const uint16_t*>(data);
    bool is_sub = (type_id == VSAF_TYPE_AUDIO_SUBWOOFER);

    uint8_t flags = data[2];
    uint8_t seq = data[3];
    uint32_t t_tx1_us = *reinterpret_cast<const uint32_t*>(data + 4);

    m_channel_locked.store(true, std::memory_order_release);
    m_last_rssi = rssi;
    m_last_master_time_us = t_tx1_us;
    m_last_local_time_us = static_cast<uint32_t>(t_rx1_us);

    // Compute instantaneous clock offset between master and local SINK clock
    int32_t instant_offset = static_cast<int32_t>(t_tx1_us - static_cast<uint32_t>(t_rx1_us));
    float offset_ms = instant_offset / 1000.0f;
    m_time_offset_buf.push(offset_ms);
    if (m_ema_time_offset_ms == 0.0f) {
        m_ema_time_offset_ms = offset_ms;
    } else {
        m_ema_time_offset_ms = (m_ema_time_offset_ms * 0.95f) + (offset_ms * 0.05f);
    }

    const vsaf_audio_packet_t* sat_pkt = reinterpret_cast<const vsaf_audio_packet_t*>(data);
    const vsaf_sub_packet_t*   sub_pkt = reinterpret_cast<const vsaf_sub_packet_t*>(data);

    // Sequence tracking & multi-tier redundancy recovery
    int8_t seq_diff = static_cast<int8_t>(static_cast<uint8_t>(seq - m_last_rx_seq));

    if (!m_first_packet_received || seq_diff < -64 || seq_diff > 64) {
        // Initial synchronization or major sequence discontinuity
        m_first_packet_received = true;
        m_last_rx_seq = seq;

        portENTER_CRITICAL(&m_sink_fifo_lock);
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = seq;
            m_sink_fifo[m_sink_fifo_head].len = is_sub ? LC3_FRAME_OCTETS_RED : LC3_FRAME_OCTETS_HQ;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = false;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t0 : sat_pkt->data_t0, m_sink_fifo[m_sink_fifo_head].len);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        m_rx_packets_total++;
        m_rx_packets_sec++;
    } else if (seq_diff == 1) {
        // Consecutive frame (normal flow): push t0
        m_last_rx_seq = seq;
        portENTER_CRITICAL(&m_sink_fifo_lock);
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = seq;
            m_sink_fifo[m_sink_fifo_head].len = is_sub ? LC3_FRAME_OCTETS_RED : LC3_FRAME_OCTETS_HQ;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = false;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t0 : sat_pkt->data_t0, m_sink_fifo[m_sink_fifo_head].len);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        } else {
            m_fifo_overflows++;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        m_rx_packets_total++;
        m_rx_packets_sec++;
    } else if (seq_diff == 2) {
        // EXACTLY 1 packet dropped in RF: recover t-1 (60B), then push t0
        m_last_rx_seq = seq;
        portENTER_CRITICAL(&m_sink_fifo_lock);
        // Push t-1
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = static_cast<uint8_t>(seq - 1);
            m_sink_fifo[m_sink_fifo_head].len = LC3_FRAME_OCTETS_RED;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = true;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t_prev1 : sat_pkt->data_t_prev1, LC3_FRAME_OCTETS_RED);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        }
        // Push t0
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = seq;
            m_sink_fifo[m_sink_fifo_head].len = is_sub ? LC3_FRAME_OCTETS_RED : LC3_FRAME_OCTETS_HQ;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = false;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t0 : sat_pkt->data_t0, m_sink_fifo[m_sink_fifo_head].len);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        } else {
            m_fifo_overflows++;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        m_rx_packets_total += 2;
        m_rx_packets_sec += 2;
    } else if (seq_diff == 3) {
        // BURST of 2 packets dropped in RF: recover t-2 (60B), then t-1 (60B), then push t0
        m_last_rx_seq = seq;
        portENTER_CRITICAL(&m_sink_fifo_lock);
        // Push t-2
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = static_cast<uint8_t>(seq - 2);
            m_sink_fifo[m_sink_fifo_head].len = LC3_FRAME_OCTETS_RED;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = true;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t_prev2 : sat_pkt->data_t_prev2, LC3_FRAME_OCTETS_RED);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        }
        // Push t-1
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = static_cast<uint8_t>(seq - 1);
            m_sink_fifo[m_sink_fifo_head].len = LC3_FRAME_OCTETS_RED;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = true;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t_prev1 : sat_pkt->data_t_prev1, LC3_FRAME_OCTETS_RED);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        }
        // Push t0
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = seq;
            m_sink_fifo[m_sink_fifo_head].len = is_sub ? LC3_FRAME_OCTETS_RED : LC3_FRAME_OCTETS_HQ;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = false;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t0 : sat_pkt->data_t0, m_sink_fifo[m_sink_fifo_head].len);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        } else {
            m_fifo_overflows++;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        m_rx_packets_total += 3;
        m_rx_packets_sec += 3;
    } else if (seq_diff >= 4) {
        // BURST of 3+ packets dropped: recover available history (up to t-3 for sub, t-2 for sat)
        m_last_rx_seq = seq;
        portENTER_CRITICAL(&m_sink_fifo_lock);
        if (is_sub) {
            // Push t-3 for Subwoofer
            if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
                m_sink_fifo[m_sink_fifo_head].seq = static_cast<uint8_t>(seq - 3);
                m_sink_fifo[m_sink_fifo_head].len = LC3_FRAME_OCTETS_RED;
                m_sink_fifo[m_sink_fifo_head].flags = flags;
                m_sink_fifo[m_sink_fifo_head].is_redundant = true;
                memcpy(m_sink_fifo[m_sink_fifo_head].data, sub_pkt->data_t_prev3, LC3_FRAME_OCTETS_RED);
                m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
                m_sink_fifo_count++;
            }
        }
        // Push t-2
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = static_cast<uint8_t>(seq - 2);
            m_sink_fifo[m_sink_fifo_head].len = LC3_FRAME_OCTETS_RED;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = true;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t_prev2 : sat_pkt->data_t_prev2, LC3_FRAME_OCTETS_RED);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        }
        // Push t-1
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = static_cast<uint8_t>(seq - 1);
            m_sink_fifo[m_sink_fifo_head].len = LC3_FRAME_OCTETS_RED;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = true;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t_prev1 : sat_pkt->data_t_prev1, LC3_FRAME_OCTETS_RED);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        }
        // Push t0
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = seq;
            m_sink_fifo[m_sink_fifo_head].len = is_sub ? LC3_FRAME_OCTETS_RED : LC3_FRAME_OCTETS_HQ;
            m_sink_fifo[m_sink_fifo_head].flags = flags;
            m_sink_fifo[m_sink_fifo_head].is_redundant = false;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, is_sub ? sub_pkt->data_t0 : sat_pkt->data_t0, m_sink_fifo[m_sink_fifo_head].len);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        } else {
            m_fifo_overflows++;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        m_rx_packets_total += (is_sub ? 4 : 3);
        m_rx_packets_sec += (is_sub ? 4 : 3);
    }
    // (If seq_diff <= 0, duplicate/stale frame: ignore)

    // Wake up SINK audio task if waiting
    if (m_sink_task_handle) {
        vTaskNotifyGiveFromISR(m_sink_task_handle, nullptr);
    }

    // SINK Reply on Command Only:
    // Only transmit telemetry reply if SOURCE explicitly requested it in packet_flags (bit 7)
    if (get_flags_req_ack(flags)) {
        sendSinkTelemetry(seq, t_tx1_us, t_rx1_us, rssi);
    }
}

void EspNowBroadcastEngine::sendSinkTelemetry(uint8_t ack_seq, uint32_t t_tx1_echo, int64_t t_rx1_us, int8_t rssi) {
    int64_t t_tx2 = esp_timer_get_time();

    vsaf_sink_telemetry_t telem = {};
    telem.type_id = VSAF_TYPE_SINK_TELEMETRY;
    telem.sink_id = m_target_channel;
    telem.ack_seq = ack_seq;
    telem.t_tx1_echo = t_tx1_echo;
    telem.t_dwell_us = static_cast<uint16_t>(t_tx2 - t_rx1_us);
    telem.downlink_rssi = rssi;

    portENTER_CRITICAL(&m_sink_fifo_lock);
    telem.fifo_fill = static_cast<uint8_t>((m_sink_fifo_count * 100) / SINK_FIFO_PACKETS);
    portEXIT_CRITICAL(&m_sink_fifo_lock);

    telem.crc16 = calc_crc16(reinterpret_cast<const uint8_t*>(&telem), sizeof(vsaf_sink_telemetry_t) - 4);
    telem.reserved = 0;

    esp_now_send(s_broadcast_mac, reinterpret_cast<const uint8_t*>(&telem), sizeof(vsaf_sink_telemetry_t));
}

// ---------------------------------------------------------------------------
// SOURCE Audio Encoder Task (Core 0, Priority 3): Pre-encodes next frame
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SOURCE Audio DSP & Dual LC3 Encoder Task (Core 1, Priority 6, Hardware FPU)
// Pops USB/Synth audio, runs 4th-order Linkwitz-Riley crossover + decimation,
// hands over ch1 (Left) to Core 0, then encodes ch2 (Right) & ch3 (Subwoofer 8k)
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::audioDspTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runAudioDspLoop();
}

void EspNowBroadcastEngine::runAudioDspLoop() {
    ESP_LOGI(TAG, "SOURCE Audio Pipeline (DSP + Dual LC3 + Broadcast TX) started on Core 1 (Priority 6, Hardware FPU)");

    static float s_stereo_in_f32[480 * 2];
    static float s_stereo_hp_f32[480 * 2];
    static float s_mono_in_f32[480];
    static float s_sub_8k_f32[80];

    constexpr int64_t SWEEP_HARD_DEADLINE_US = 9500; // 9.5 ms: tightened from 9.4 ms (us-precision polling gives headroom)

    while (m_running.load(std::memory_order_acquire)) {
        // Hardware timer event pacing: exact 10.0 ms / 7.5 ms wakeups from esp_timer task
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
            continue;
        }

        if (m_state.load(std::memory_order_acquire) != NetworkState::CAST) {
            continue;
        }

        int64_t frame_start_us = esp_timer_get_time();

        size_t samples = Codec::calculateRequiredPcmSamples(m_telemetry.sample_rate, m_frame_duration_us);
        if (samples > 480) samples = 480;
        m_pcm_in_samples = samples;

        // 1. Pop audio from USB UAC1 or Tone Generator
        if (m_tone_test_mode) {
            if (m_tone_gen) {
                static int16_t tone_mono[480];
                m_tone_gen->generateFrame(tone_mono, samples);
                for (size_t i = 0; i < samples; ++i) {
                    m_pcm_stereo_in[i * 2]     = tone_mono[i];
                    m_pcm_stereo_in[i * 2 + 1] = tone_mono[i];
                }
            } else {
                memset(m_pcm_stereo_in, 0, samples * 2 * sizeof(int16_t));
            }
        } else {
            // Default Mode: Read live 48kHz 16-bit stereo PCM from PC via USB UAC1 directly into m_pcm_stereo_in
            size_t bytes_needed = samples * 2 * sizeof(int16_t);
            size_t bytes_read = usb_audio_read_pcm(m_pcm_stereo_in, bytes_needed);
            if (bytes_read < bytes_needed) {
                // Zero-fill remaining samples on underrun or idle host
                memset(reinterpret_cast<uint8_t*>(m_pcm_stereo_in) + bytes_read, 0, bytes_needed - bytes_read);
            }
        }

        // 2. Block-Vectorized DSP Filtering (Stereo LR4 HPF @ 100 Hz -> ch0/ch1, Polyphase Sub LPF @ 100 Hz -> ch5)
        for (size_t i = 0; i < samples; ++i) {
            float l_f = static_cast<float>(m_pcm_stereo_in[i * 2]);
            float r_f = static_cast<float>(m_pcm_stereo_in[i * 2 + 1]);

            s_stereo_in_f32[i * 2]     = l_f;
            s_stereo_in_f32[i * 2 + 1] = r_f;
            s_mono_in_f32[i]          = 0.5f * (l_f + r_f);
        }

        // 2.1 Stereo 4th-order Linkwitz-Riley High-Pass Filter (esp-dsp block SIMD on Left & Right)
        m_hpf_stereo.processStereo(s_stereo_in_f32, s_stereo_hp_f32, static_cast<int>(samples));

        for (size_t i = 0; i < samples; ++i) {
            m_pcm_left_hp[i]  = static_cast<int16_t>(std::clamp(s_stereo_hp_f32[i * 2], -32768.0f, 32767.0f));
            m_pcm_right_hp[i] = static_cast<int16_t>(std::clamp(s_stereo_hp_f32[i * 2 + 1], -32768.0f, 32767.0f));
        }

        // 2.2 Subwoofer: 2-Stage Polyphase Decimator & Subwoofer Crossover LPF (48 kHz mono -> 8 kHz sub)
        m_sub_decimator.process(s_mono_in_f32, s_sub_8k_f32);

        for (size_t i = 0; i < 80; ++i) {
            m_pcm_sub_8k[i] = static_cast<int16_t>(std::clamp(s_sub_8k_f32[i], -32768.0f, 32767.0f));
        }

        int64_t dsp_t1 = esp_timer_get_time();
        float dsp_ms = (dsp_t1 - frame_start_us) / 1000.0f;
        m_dsp_duration_buf.push(dsp_ms);

        // 3. LC3 Multi-Rate Encoding Passes (5 Encoders Total, All with LTPF Disabled)
        // -------------------------------------------------------------------------
        // Channel topology:
        // Ch 0: Left High-Pass (Satellite Left)
        // Ch 1: Right High-Pass (Satellite Right)
        // Ch 2: Center High-Pass (mapped from Left HP)
        // Ch 3: Left Surround (mapped from Left HP)
        // Ch 4: Right Surround (mapped from Right HP)
        // Ch 5: Subwoofer (8 kHz Linkwitz-Riley LP polyphase decimated, Packet Type 0x1338)

        static uint8_t encoded_sat_hq[2][LC3_FRAME_OCTETS_HQ];   // [0]: Left HQ, [1]: Right HQ (120B)
        static uint8_t encoded_sat_red[2][LC3_FRAME_OCTETS_RED]; // [0]: Left Red, [1]: Right Red (60B)
        static uint8_t encoded_sub_60[LC3_FRAME_OCTETS_SUB];      // Subwoofer (60B)

        // 3.1 Pass 1 (HQ / Main Codecs): Left HQ (120B, Enc 0), Right HQ (120B, Enc 1), Subwoofer (60B @ 8 kHz, Enc 4)
        int64_t main_enc_t0 = esp_timer_get_time();
        size_t actual_bytes = 0;
        m_lc3_codec.encodeFrame(m_pcm_left_hp, samples, encoded_sat_hq[0], LC3_FRAME_OCTETS_HQ, &actual_bytes, 0, 1);
        if (m_is_stereo || m_tone_test_mode) {
            m_lc3_codec.encodeFrame(m_pcm_right_hp, samples, encoded_sat_hq[1], LC3_FRAME_OCTETS_HQ, &actual_bytes, 1, 1);
        } else {
            memcpy(encoded_sat_hq[1], encoded_sat_hq[0], LC3_FRAME_OCTETS_HQ);
        }
        m_lc3_codec.encodeFrame(m_pcm_sub_8k, 80, encoded_sub_60, LC3_FRAME_OCTETS_SUB, &actual_bytes, 4, 1);
        int64_t main_enc_t1 = esp_timer_get_time();
        float enc_main_ms = (main_enc_t1 - main_enc_t0) / 1000.0f;
        m_enc_main_duration_buf.push(enc_main_ms);

        // 3.2 Pass 2 (Redundant Codecs): Left Red (60B, Enc 2) & Right Red (60B, Enc 3)
        int64_t red_enc_t0 = esp_timer_get_time();
        m_lc3_codec.encodeFrame(m_pcm_left_hp, samples, encoded_sat_red[0], LC3_FRAME_OCTETS_RED, &actual_bytes, 2, 1);
        if (m_is_stereo || m_tone_test_mode) {
            m_lc3_codec.encodeFrame(m_pcm_right_hp, samples, encoded_sat_red[1], LC3_FRAME_OCTETS_RED, &actual_bytes, 3, 1);
        } else {
            memcpy(encoded_sat_red[1], encoded_sat_red[0], LC3_FRAME_OCTETS_RED);
        }
        int64_t red_enc_t1 = esp_timer_get_time();
        float enc_red_ms = (red_enc_t1 - red_enc_t0) / 1000.0f;
        m_enc_red_duration_buf.push(enc_red_ms);

        // Map satellite channels (Ch 0..4):
        // Ch 0 (Left), Ch 2 (Center), Ch 3 (Left Surround) -> Left
        // Ch 1 (Right), Ch 4 (Right Surround) -> Right
        const uint8_t* sat_hq_ptr[NUM_SATELLITE_CHANNELS] = {
            encoded_sat_hq[0], // Ch 0: Left
            encoded_sat_hq[1], // Ch 1: Right
            encoded_sat_hq[0], // Ch 2: Center
            encoded_sat_hq[0], // Ch 3: Left Surround
            encoded_sat_hq[1]  // Ch 4: Right Surround
        };
        const uint8_t* sat_red_ptr[NUM_SATELLITE_CHANNELS] = {
            encoded_sat_red[0], // Ch 0: Left Red
            encoded_sat_red[1], // Ch 1: Right Red
            encoded_sat_red[0], // Ch 2: Center Red
            encoded_sat_red[0], // Ch 3: Left Surround Red
            encoded_sat_red[1]  // Ch 4: Right Surround Red
        };

        // 4. Prepare Broadcast Packets for all 6 Channels
        uint32_t t_now_tx_us = static_cast<uint32_t>(esp_timer_get_time());

        // 4.1 Prepare Satellite Packets (Ch 0..4)
        for (size_t ch = 0; ch < NUM_SATELLITE_CHANNELS; ++ch) {
            m_last_tx_pkt[ch].type_id = VSAF_TYPE_AUDIO_SATELLITE;
            m_last_tx_pkt[ch].packet_flags = make_packet_flags(static_cast<uint8_t>(ch), m_telemetry.sample_rate, 10000, false);
            m_last_tx_pkt[ch].seq = m_seq;
            m_last_tx_pkt[ch].t_tx1_us = t_now_tx_us;
            memcpy(m_last_tx_pkt[ch].data_t0, sat_hq_ptr[ch], LC3_FRAME_OCTETS_HQ);

            if (m_prev1_sat_valid[ch]) {
                memcpy(m_last_tx_pkt[ch].data_t_prev1, m_prev1_encoded_sat[ch], LC3_FRAME_OCTETS_RED);
            } else {
                memcpy(m_last_tx_pkt[ch].data_t_prev1, sat_red_ptr[ch], LC3_FRAME_OCTETS_RED);
            }

            if (m_prev2_sat_valid[ch]) {
                memcpy(m_last_tx_pkt[ch].data_t_prev2, m_prev2_encoded_sat[ch], LC3_FRAME_OCTETS_RED);
            } else {
                memcpy(m_last_tx_pkt[ch].data_t_prev2, sat_red_ptr[ch], LC3_FRAME_OCTETS_RED);
            }
        }

        // 4.2 Prepare Subwoofer Packet (Ch 5, Type 0x1338)
        m_last_tx_sub_pkt.type_id = VSAF_TYPE_AUDIO_SUBWOOFER;
        m_last_tx_sub_pkt.packet_flags = make_packet_flags(SUBWOOFER_CHANNEL_ID, 8000, 10000, false);
        m_last_tx_sub_pkt.seq = m_seq;
        m_last_tx_sub_pkt.t_tx1_us = t_now_tx_us;
        memcpy(m_last_tx_sub_pkt.data_t0, encoded_sub_60, LC3_FRAME_OCTETS_SUB);

        if (m_prev1_sub_valid) {
            memcpy(m_last_tx_sub_pkt.data_t_prev1, m_prev1_encoded_sub, LC3_FRAME_OCTETS_SUB);
        } else {
            memcpy(m_last_tx_sub_pkt.data_t_prev1, encoded_sub_60, LC3_FRAME_OCTETS_SUB);
        }

        if (m_prev2_sub_valid) {
            memcpy(m_last_tx_sub_pkt.data_t_prev2, m_prev2_encoded_sub, LC3_FRAME_OCTETS_SUB);
        } else {
            memcpy(m_last_tx_sub_pkt.data_t_prev2, encoded_sub_60, LC3_FRAME_OCTETS_SUB);
        }

        if (m_prev3_sub_valid) {
            memcpy(m_last_tx_sub_pkt.data_t_prev3, m_prev3_encoded_sub, LC3_FRAME_OCTETS_SUB);
        } else {
            memcpy(m_last_tx_sub_pkt.data_t_prev3, encoded_sub_60, LC3_FRAME_OCTETS_SUB);
        }

        // 4.3 Unconditionally update history buffers for next cycle
        for (size_t ch = 0; ch < NUM_SATELLITE_CHANNELS; ++ch) {
            memcpy(m_prev2_encoded_sat[ch], m_prev1_encoded_sat[ch], LC3_FRAME_OCTETS_RED);
            m_prev2_sat_valid[ch] = m_prev1_sat_valid[ch];
            memcpy(m_prev1_encoded_sat[ch], sat_red_ptr[ch], LC3_FRAME_OCTETS_RED);
            m_prev1_sat_valid[ch] = true;
        }

        memcpy(m_prev3_encoded_sub, m_prev2_encoded_sub, LC3_FRAME_OCTETS_SUB);
        m_prev3_sub_valid = m_prev2_sub_valid;
        memcpy(m_prev2_encoded_sub, m_prev1_encoded_sub, LC3_FRAME_OCTETS_SUB);
        m_prev2_sub_valid = m_prev1_sub_valid;
        memcpy(m_prev1_encoded_sub, encoded_sub_60, LC3_FRAME_OCTETS_SUB);
        m_prev1_sub_valid = true;

        // 4.4 Broadcast 6 Audio Channels over 802.11 ESP-NOW (Rotating sweep)
        int64_t tx_t0 = esp_timer_get_time();
        size_t start_offset = m_seq % MAX_SINK_NODES;
        for (size_t i = 0; i < MAX_SINK_NODES; ++i) {
            size_t ch = (start_offset + i) % MAX_SINK_NODES;

            bool request_ack = (i == (MAX_SINK_NODES - 1));

            const uint8_t* pkt_buf;
            size_t pkt_len;
            if (ch == SUBWOOFER_CHANNEL_ID) {
                if (request_ack) m_last_tx_sub_pkt.packet_flags |= 0x80;
                pkt_buf = reinterpret_cast<const uint8_t*>(&m_last_tx_sub_pkt);
                pkt_len = sizeof(vsaf_sub_packet_t);
            } else {
                if (request_ack) m_last_tx_pkt[ch].packet_flags |= 0x80;
                pkt_buf = reinterpret_cast<const uint8_t*>(&m_last_tx_pkt[ch]);
                pkt_len = sizeof(vsaf_audio_packet_t);
            }

            if (s_tx_done_sem) {
                xSemaphoreTake(s_tx_done_sem, 0);
            }

            esp_err_t send_err = esp_now_send(s_broadcast_mac, pkt_buf, pkt_len);

            if (send_err == ESP_OK) {
                if (s_tx_done_sem) {
                    // Microsecond-precision polling: max 1000 us (normal broadcast TX completes in ~500-700 us)
                    bool tx_completed = false;
                    int64_t tx_poll_deadline = esp_timer_get_time() + 1000;
                    while (esp_timer_get_time() < tx_poll_deadline) {
                        if (xSemaphoreTake(s_tx_done_sem, 0) == pdTRUE) {
                            tx_completed = true;
                            break;
                        }
                        esp_rom_delay_us(50);
                    }
                    if (tx_completed) {
                        m_consecutive_tx_timeouts = 0;
                        if (s_last_tx_status.load(std::memory_order_relaxed) != ESP_NOW_SEND_SUCCESS) {
                            m_tx_fail_count++;
                        }
                    } else {
                        m_tx_timeout_count++;
                        m_consecutive_tx_timeouts++;
                        if (m_consecutive_tx_timeouts >= 5) {
                            handleTxSubsystemHang();
                        }
                    }
                }
            } else {
                m_tx_mac_error_count++;
            }

            m_peers[ch].packets_sent++;
            m_tx_packets_this_sec++;
            m_tx_packets_total++;
            m_tx_packets_sec++;

            // Check deadline AFTER send: only abort remaining channels if time is exhausted
            int64_t elapsed_us = esp_timer_get_time() - frame_start_us;
            if (elapsed_us >= SWEEP_HARD_DEADLINE_US) {
                m_tx_deadline_drops += (MAX_SINK_NODES - 1 - i);
                break;
            }
        }

        int64_t tx_t1 = esp_timer_get_time();
        float tx_ms = (tx_t1 - tx_t0) / 1000.0f;
        m_tx_duration_buf.push(tx_ms);

        float total_core1_ms = (tx_t1 - frame_start_us) / 1000.0f;
        m_codec_duration_buf.push(total_core1_ms);
        m_audio_meter.pushFramePcm(m_pcm_left_hp, samples, 1);

        m_seq++;
    }
    vTaskDelete(nullptr);
}

void EspNowBroadcastEngine::handleTxSubsystemHang() {
    ESP_LOGE(TAG, "Wi-Fi Baseband TX hang detected (%lu consecutive frame timeouts)! Resetting MAC state...",
             (unsigned long)m_consecutive_tx_timeouts);
    // Deterministic placeholder recovery: Purge semaphore and replenish token
    if (s_tx_done_sem) {
        xSemaphoreTake(s_tx_done_sem, 0);
        xSemaphoreGive(s_tx_done_sem);
    }
    m_consecutive_tx_timeouts = 0;
}

// ---------------------------------------------------------------------------
// SINK Audio Loop: Jitter Buffer PREFILL -> I2S DMA Synced Output
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::sinkTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSinkLoop();
}

void EspNowBroadcastEngine::runSinkLoop() {
    ESP_LOGI(TAG, "SINK Audio Decoder & I2S Task started on Core 0");

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    static int32_t pcm_mono[480];
    static int32_t pcm_stereo[480 * 2];
    constexpr size_t SAMPLE_SIZE = sizeof(int32_t);
#else
    static int16_t pcm_mono[480];
    static int16_t pcm_stereo[480 * 2];
    constexpr size_t SAMPLE_SIZE = sizeof(int16_t);
#endif
    size_t samples_per_frame = (m_telemetry.sample_rate * 10) / 1000;
    uint32_t consecutive_underruns = 0;
    static constexpr size_t PREFILL_THRESHOLD = CONFIG_ESPNOW_PREFILL_THRESHOLD_FRAMES; // 2 packets = 20 ms cushion
    int64_t last_hop_time_us = esp_timer_get_time();

    while (m_running.load(std::memory_order_acquire)) {
        NetworkState current_state = m_state.load(std::memory_order_acquire);

        // -------------------------------------------------------------------
        // 0. IDLE: Audio completely paused, I2S DAC stopped, FIFO empty
        // -------------------------------------------------------------------
        if (current_state == NetworkState::IDLE) {
            if (m_i2s_dac) {
                m_i2s_dac->stop();
            }
            portENTER_CRITICAL(&m_sink_fifo_lock);
            m_sink_fifo_count = 0;
            m_sink_fifo_head = 0;
            m_sink_fifo_tail = 0;
            portEXIT_CRITICAL(&m_sink_fifo_lock);
            m_has_expected_seq = false;
            m_first_packet_received = false;

            // Sleep in low-power idle until button or command changes state
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
            continue;
        }

        // -------------------------------------------------------------------
        // 1. SCANNING: Dynamic channel hopping until audio stream is discovered
        // -------------------------------------------------------------------
        if (current_state == NetworkState::SCANNING) {
            if (!m_channel_locked.load(std::memory_order_acquire)) {
                int64_t now_us = esp_timer_get_time();
                if (now_us - last_hop_time_us >= 150000) { // Hop channel every 150 ms
                    last_hop_time_us = now_us;
                    uint8_t next_ch = (m_wifi_channel % 13) + 1;
                    setWifiChannel(next_ch);
                }
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
                continue;
            }

            portENTER_CRITICAL(&m_sink_fifo_lock);
            size_t buffered = m_sink_fifo_count;
            portEXIT_CRITICAL(&m_sink_fifo_lock);

            if (buffered >= PREFILL_THRESHOLD) {
                ESP_LOGI(TAG, "SINK: Jitter buffer prefill threshold reached (%zu pkts on Ch %d) -> PREFILL",
                         buffered, m_wifi_channel);
                transitionTo(NetworkState::PREFILL);
            } else {
                int64_t now_us = esp_timer_get_time();
                if (now_us - m_last_rx_audio_pkt_us.load(std::memory_order_acquire) >= 500000) {
                    m_channel_locked.store(false, std::memory_order_release);
                    last_hop_time_us = now_us;
                }
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
                continue;
            }
        }

        // -------------------------------------------------------------------
        // 2. PREFILL: Preload dual DMA descriptors while clocks are stopped
        // -------------------------------------------------------------------
        if (m_state.load(std::memory_order_acquire) == NetworkState::PREFILL) {
            if (m_i2s_dac) {
                m_i2s_dac->stop(); // Ensure stopped before preload
            }

            // Preload Frame 1 into DMA Descriptor 0
            SinkFifoItem item1 = {};
            bool has_item1 = false;
            portENTER_CRITICAL(&m_sink_fifo_lock);
            if (m_sink_fifo_count > 0) {
                item1 = m_sink_fifo[m_sink_fifo_tail];
                m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
                m_sink_fifo_count--;
                has_item1 = true;
            }
            portEXIT_CRITICAL(&m_sink_fifo_lock);

            if (has_item1) {
                size_t actual_samples = 0;
                uint32_t sr1 = (item1.flags != 0) ? get_flags_sample_rate(item1.flags) : m_telemetry.sample_rate;
                uint32_t dur1 = (item1.flags != 0) ? get_flags_frame_dur_us(item1.flags) : m_frame_duration_us;
                m_lc3_codec.decodeFrame(item1.data, item1.len, pcm_mono, 480, &actual_samples, sr1, dur1);
                if (actual_samples == 80 && samples_per_frame == 480) {
                    for (int i = 79; i >= 0; --i) {
                        auto s = pcm_mono[i];
                        for (int k = 0; k < 6; ++k) {
                            pcm_mono[i * 6 + k] = s;
                        }
                    }
                    actual_samples = 480;
                }
                float total_gain_db = m_current_gain_db + m_post_gain_db;
                float gain = (m_current_gain_db <= -90.0f) ? 0.0f : std::pow(10.0f, total_gain_db / 20.0f);
#if defined(CONFIG_IDF_TARGET_ESP32S3)
                for (size_t i = 0; i < samples_per_frame; ++i) {
                    int64_t val = static_cast<int64_t>(pcm_mono[i] * gain);
                    if (val > 8388607) val = 8388607;
                    if (val < -8388608) val = -8388608;
                    int32_t sample = static_cast<int32_t>(val << 8) & 0xffffff00;
                    pcm_stereo[i * 2]     = sample;
                    pcm_stereo[i * 2 + 1] = sample;
                }
#else
                for (size_t i = 0; i < samples_per_frame; ++i) {
                    int32_t val = static_cast<int32_t>(pcm_mono[i] * gain);
                    if (val > 32767) val = 32767;
                    if (val < -32768) val = -32768;
                    int16_t sample = static_cast<int16_t>(val);
                    pcm_stereo[i * 2]     = sample;
                    pcm_stereo[i * 2 + 1] = sample;
                }
#endif
                if (m_i2s_dac) {
                    size_t bytes_written = 0;
                    m_i2s_dac->preload(pcm_stereo, samples_per_frame * SAMPLE_SIZE * 2, &bytes_written);
                }
            }

            // Preload Frame 2 into DMA Descriptor 1
            SinkFifoItem item2 = {};
            bool has_item2 = false;
            portENTER_CRITICAL(&m_sink_fifo_lock);
            if (m_sink_fifo_count > 0) {
                item2 = m_sink_fifo[m_sink_fifo_tail];
                m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
                m_sink_fifo_count--;
                has_item2 = true;
            }
            portEXIT_CRITICAL(&m_sink_fifo_lock);

            if (has_item2) {
                size_t actual_samples = 0;
                uint32_t sr2 = (item2.flags != 0) ? get_flags_sample_rate(item2.flags) : m_telemetry.sample_rate;
                uint32_t dur2 = (item2.flags != 0) ? get_flags_frame_dur_us(item2.flags) : m_frame_duration_us;
                m_lc3_codec.decodeFrame(item2.data, item2.len, pcm_mono, 480, &actual_samples, sr2, dur2);
                if (actual_samples == 80 && samples_per_frame == 480) {
                    for (int i = 79; i >= 0; --i) {
                        auto s = pcm_mono[i];
                        for (int k = 0; k < 6; ++k) {
                            pcm_mono[i * 6 + k] = s;
                        }
                    }
                    actual_samples = 480;
                }
                float total_gain_db = m_current_gain_db + m_post_gain_db;
                float gain = (m_current_gain_db <= -90.0f) ? 0.0f : std::pow(10.0f, total_gain_db / 20.0f);
#if defined(CONFIG_IDF_TARGET_ESP32S3)
                for (size_t i = 0; i < samples_per_frame; ++i) {
                    int64_t val = static_cast<int64_t>(pcm_mono[i] * gain);
                    if (val > 8388607) val = 8388607;
                    if (val < -8388608) val = -8388608;
                    int32_t sample = static_cast<int32_t>(val << 8) & 0xffffff00;
                    pcm_stereo[i * 2]     = sample;
                    pcm_stereo[i * 2 + 1] = sample;
                }
#else
                for (size_t i = 0; i < samples_per_frame; ++i) {
                    int32_t val = static_cast<int32_t>(pcm_mono[i] * gain);
                    if (val > 32767) val = 32767;
                    if (val < -32768) val = -32768;
                    int16_t sample = static_cast<int16_t>(val);
                    pcm_stereo[i * 2]     = sample;
                    pcm_stereo[i * 2 + 1] = sample;
                }
#endif
                if (m_i2s_dac) {
                    size_t bytes_written = 0;
                    m_i2s_dac->preload(pcm_stereo, samples_per_frame * SAMPLE_SIZE * 2, &bytes_written);
                }
                m_expected_seq = item2.seq + 1;
                m_has_expected_seq = true;
            }

            if (m_i2s_dac) {
                m_i2s_dac->start(); // Start hardware DMA clocks with preloaded descriptors
            }
            consecutive_underruns = 0;
            transitionTo(NetworkState::STREAM);
            ESP_LOGI(TAG, "SINK: Preload complete. Transition to STREAM (FIFO cushion = %zu pkts).", m_sink_fifo_count);
            continue;
        }

        // -------------------------------------------------------------------
        // 3. STREAM: Hardware DMA Paced Audio Decode
        // -------------------------------------------------------------------
        // Buffer regulation: if buffer backlog exceeds target (e.g. > 18 frames = 180 ms), drop oldest frame cleanly
        portENTER_CRITICAL(&m_sink_fifo_lock);
        while (m_sink_fifo_count > 18) {
            m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count--;
            m_fifo_overflows++;
            if (m_sink_fifo_count > 0 && m_has_expected_seq) {
                // Keep expected sequence aligned with new tail to avoid synthesizing gap PLC for intentionally dropped frames
                m_expected_seq = m_sink_fifo[m_sink_fifo_tail].seq;
            }
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        // Pop next frame from Jitter FIFO
        SinkFifoItem item = {};
        bool has_item = false;
        bool is_gap_plc = false;
        portENTER_CRITICAL(&m_sink_fifo_lock);
        while (m_sink_fifo_count > 0) {
            item = m_sink_fifo[m_sink_fifo_tail];
            int8_t gap = m_has_expected_seq ? static_cast<int8_t>(static_cast<uint8_t>(item.seq - m_expected_seq)) : 0;
            if (m_has_expected_seq && (gap < -4 || gap > 10)) {
                // Large sequence jump or stream restart: resynchronize immediately
                m_expected_seq = item.seq;
                gap = 0;
            }
            if (m_has_expected_seq && gap < 0) {
                // Stale duplicate frame (slot was already rendered): discard it cleanly
                m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
                m_sink_fifo_count--;
                continue;
            }
            if (m_has_expected_seq && gap > 0 && gap <= 4) {
                // Gap in incoming sequence numbers: synthesize 1 PLC frame for missing sequence number
                is_gap_plc = true;
                has_item = false;
                m_expected_seq++;
                break;
            }
            // In-sequence frame (or resynchronized frame)
            m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count--;
            has_item = true;
            m_expected_seq = item.seq + 1;
            m_has_expected_seq = true;
            break;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        // JIT absorption: if FIFO was genuinely empty (and not a sequence gap), wait up to 6 ms for in-flight packet
        if (!has_item && !is_gap_plc) {
            int64_t wait_until_us = esp_timer_get_time() + 6000;
            while (esp_timer_get_time() < wait_until_us) {
                esp_rom_delay_us(50);
                portENTER_CRITICAL(&m_sink_fifo_lock);
                if (m_sink_fifo_count > 0) {
                    item = m_sink_fifo[m_sink_fifo_tail];
                    int8_t gap = m_has_expected_seq ? static_cast<int8_t>(static_cast<uint8_t>(item.seq - m_expected_seq)) : 0;
                    if (m_has_expected_seq && (gap < -4 || gap > 10)) {
                        m_expected_seq = item.seq;
                        gap = 0;
                    }
                    if (m_has_expected_seq && gap < 0) {
                        // Stale duplicate: drop and continue waiting
                        m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
                        m_sink_fifo_count--;
                        portEXIT_CRITICAL(&m_sink_fifo_lock);
                        continue;
                    }
                    if (m_has_expected_seq && gap > 0 && gap <= 4) {
                        // Sequence gap in arriving packet: trigger PLC for missing slot, do NOT pop item
                        is_gap_plc = true;
                        has_item = false;
                        m_expected_seq++;
                        portEXIT_CRITICAL(&m_sink_fifo_lock);
                        break;
                    }
                    m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
                    m_sink_fifo_count--;
                    has_item = true;
                    m_expected_seq = item.seq + 1;
                    m_has_expected_seq = true;
                    portEXIT_CRITICAL(&m_sink_fifo_lock);
                    break;
                }
                portEXIT_CRITICAL(&m_sink_fifo_lock);
            }
        }

        if (!has_item && !is_gap_plc) {
            if (m_has_expected_seq) {
                m_expected_seq++; // Advance sequence for this starvation slot rendered via PLC
            }
            consecutive_underruns++;
            if (consecutive_underruns >= CONFIG_ESPNOW_WATCHDOG_TIMEOUT_FRAMES) {
                // Extended outage (> 60 ms): pause I2S and transition to SCANNING to re-buffer 50 ms cushion
                ESP_LOGW(TAG, "SINK: %lu consecutive underruns -> returning to SCANNING to re-buffer cushion",
                         (unsigned long)consecutive_underruns);
                if (m_i2s_dac) {
                    m_i2s_dac->stop();
                }
                m_has_expected_seq = false;
                m_channel_locked.store(false, std::memory_order_release);
                transitionTo(NetworkState::SCANNING);
                consecutive_underruns = 0;
                last_hop_time_us = esp_timer_get_time();
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
        } else {
            consecutive_underruns = 0;
        }

        int64_t dec_t0 = esp_timer_get_time();
        size_t actual_samples = 0;
        if (has_item) {
            if (item.is_redundant) {
                m_redundancy_recovered_packets++;
            }
            uint32_t sr = (item.flags != 0) ? get_flags_sample_rate(item.flags) : m_telemetry.sample_rate;
            uint32_t dur = (item.flags != 0) ? get_flags_frame_dur_us(item.flags) : m_frame_duration_us;
            m_lc3_codec.decodeFrame(item.data, item.len, pcm_mono, 480, &actual_samples, sr, dur);
            if (actual_samples == 80 && samples_per_frame == 480) {
                for (int i = 79; i >= 0; --i) {
                    auto s = pcm_mono[i];
                    for (int k = 0; k < 6; ++k) {
                        pcm_mono[i * 6 + k] = s;
                    }
                }
                actual_samples = 480;
            }
        } else {
            // Packet Loss Concealment (PLC) during active STREAM
            m_lc3_codec.decodeFrame(nullptr, 0, pcm_mono, 480, &actual_samples);
            if (actual_samples == 80 && samples_per_frame == 480) {
                for (int i = 79; i >= 0; --i) {
                    auto s = pcm_mono[i];
                    for (int k = 0; k < 6; ++k) {
                        pcm_mono[i * 6 + k] = s;
                    }
                }
                actual_samples = 480;
            }
            m_plc_count++;
            if (!is_gap_plc) {
                m_fifo_underflows++; // Only genuine buffer starvations count as FIFO underflow
            }
        }
        float dec_ms = (esp_timer_get_time() - dec_t0) / 1000.0f;
        m_codec_duration_buf.push(dec_ms);
        m_audio_meter.pushFramePcm(pcm_mono, samples_per_frame);

        // Mono to Stereo duplication for I2S DAC
        float total_gain_db = m_current_gain_db + m_post_gain_db;
        float gain = (m_current_gain_db <= -90.0f) ? 0.0f : std::pow(10.0f, total_gain_db / 20.0f);
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        for (size_t i = 0; i < samples_per_frame; ++i) {
            int64_t val = static_cast<int64_t>(pcm_mono[i] * gain);
            if (val > 8388607) val = 8388607;
            if (val < -8388608) val = -8388608;
            int32_t sample = static_cast<int32_t>(val << 8) & 0xffffff00;
            pcm_stereo[i * 2]     = sample;
            pcm_stereo[i * 2 + 1] = sample;
        }

        if (m_i2s_dac) {
            size_t bytes_written = 0;
            m_i2s_dac->write(pcm_stereo, samples_per_frame * SAMPLE_SIZE * 2, &bytes_written, 50);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
#else
        for (size_t i = 0; i < samples_per_frame; ++i) {
            int32_t val = static_cast<int32_t>(pcm_mono[i] * gain);
            if (val > 32767) val = 32767;
            if (val < -32768) val = -32768;
            int16_t sample = static_cast<int16_t>(val);
            pcm_stereo[i * 2]     = sample;
            pcm_stereo[i * 2 + 1] = sample;
        }

        if (m_i2s_dac) {
            size_t bytes_written = 0;
            m_i2s_dac->write(pcm_stereo, samples_per_frame * SAMPLE_SIZE * 2, &bytes_written, 50);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
#endif
    }
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Telemetry & Rate Statistics
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::updatePerSecondStats() {
    m_stats_packets_per_sec = m_tx_packets_this_sec;
    m_stats_acks_per_sec = m_rx_acks_this_sec;

    if (m_stats_packets_per_sec > 0) {
        m_stats_ack_percentage = (static_cast<float>(m_stats_acks_per_sec) / static_cast<float>(m_stats_packets_per_sec)) * 100.0f;
        if (m_stats_ack_percentage > 100.0f) m_stats_ack_percentage = 100.0f;
    } else {
        m_stats_ack_percentage = 100.0f;
    }

    m_tx_packets_this_sec = 0;
    m_rx_acks_this_sec = 0;
}



void EspNowBroadcastEngine::update10HzTimeOffsetStats() {
    if (m_node_role == NODE_ROLE_SOURCE) {
        int64_t now_us = esp_timer_get_time();
        for (size_t i = 0; i < MAX_SINK_NODES; ++i) {
            if (m_peers[i].is_enabled && m_peers[i].status == PeerStatus::ONLINE) {
                // If no telemetry received for > 1.5 seconds, transition to OFFLINE
                if (m_peers[i].last_rx_telemetry_time_us > 0 && (now_us - m_peers[i].last_rx_telemetry_time_us) > 1500000) {
                    m_peers[i].status = PeerStatus::OFFLINE;
                    m_peers[i].last_net_rtt_us = 0;
                    m_peers[i].last_rtt_us = 0;
                }
            }
        }
    }
}

void EspNowBroadcastEngine::getTimeSyncStats(float& out_median_ms, float& out_range_ms, bool& out_has_data) const {
    m_time_offset_buf.computeStats(out_median_ms, out_range_ms, out_has_data);
}

size_t EspNowBroadcastEngine::getFifoCount() const {
    portENTER_CRITICAL((portMUX_TYPE*)&m_sink_fifo_lock);
    size_t count = m_sink_fifo_count;
    portEXIT_CRITICAL((portMUX_TYPE*)&m_sink_fifo_lock);
    return count;
}

size_t EspNowBroadcastEngine::getFifoCapacity() const {
    return SINK_FIFO_PACKETS;
}

bool EspNowBroadcastEngine::pushUsbAudioPcm(const int16_t* pcm_stereo, size_t samples_per_channel) {
    return true;
}

bool EspNowBroadcastEngine::processUsbVsafPacket(const uint8_t* data, size_t len) {
    return true;
}

} // namespace AudioNet
