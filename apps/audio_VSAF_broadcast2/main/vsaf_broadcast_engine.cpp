#include "vsaf_broadcast_engine.hpp"
#include "status_led.hpp"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_mac.h"
#include <cmath>

static const char* TAG = "VSAF_BCAST";

namespace AudioNet {

uint8_t EspNowBroadcastEngine::s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
SemaphoreHandle_t EspNowBroadcastEngine::s_tx_done_sem = nullptr;
static EspNowBroadcastEngine* s_engine_instance = nullptr;

static void IRAM_ATTR onEspNowSendCb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
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
      m_tx_phy_rate(WIFI_PHY_RATE_24M),
      m_peer_count(2),
      m_seq(0),
      m_octets_per_frame(CONFIG_ESPNOW_FRAME_LEN_OCTETS),
      m_frame_duration_us(10000),
      m_target_volume_u8(255),
      m_target_gain_db(0.0f),
      m_current_gain_db(0.0f),
      m_instant_vol_update(false),
      m_source_enc_task_handle(nullptr),
      m_source_tx_task_handle(nullptr),
      m_sink_task_handle(nullptr),
      m_ema_time_offset_ms(0.0f),
      m_last_master_time_us(0),
      m_last_local_time_us(0),
      m_last_rssi(-127),
      m_plc_count(0),
      m_fifo_underflows(0),
      m_fifo_overflows(0),
      m_last_rx_seq(0xFFFF),
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
    m_wifi_channel = wifi_channel;

    if (!s_tx_done_sem) {
        s_tx_done_sem = xSemaphoreCreateBinary();
    }

    // 1. Initialize Wi-Fi Station
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE));

    // Standard 2.4 GHz protocols (11b/g/n on S3, 11b/g/n/ax on C6)
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX));
#else
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
#endif

    esp_wifi_set_max_tx_power(78); // +19.5 dBm (maximum RF output power)

    // 2. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(onEspNowSendCb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(onEspNowRecvCb));

    // 3. Register Single Broadcast Peer (FF:FF:FF:FF:FF:FF) with Locked Rate 24 Mbps
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, s_broadcast_mac, 6);
    peer_info.channel = m_wifi_channel;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to register broadcast peer: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_now_rate_config_t rate_cfg = {
        .phymode = WIFI_PHY_MODE_11G,
        .rate = WIFI_PHY_RATE_24M,
        .ersu = false,
        .dcm = false
    };
    esp_now_set_peer_rate_config(s_broadcast_mac, &rate_cfg);

    // 4. Initialize Codec
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.initEncoder(m_telemetry.sample_rate, 2, m_frame_duration_us, m_octets_per_frame);
    } else {
        m_lc3_codec.initDecoder(m_telemetry.sample_rate, 1, m_frame_duration_us, m_octets_per_frame);
    }

    transitionTo(NetworkState::IDLE);
    ESP_LOGI(TAG, "EspNowBroadcastEngine initialized successfully (Role: %s, Node ID: %d, Ch: %d, Rate: 24M)",
             (m_node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK", m_node_id, m_wifi_channel);
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::start() {
    if (m_running.load(std::memory_order_acquire)) {
        return ESP_OK;
    }
    m_running.store(true, std::memory_order_release);

    if (m_node_role == NODE_ROLE_SOURCE) {
        transitionTo(NetworkState::IDLE);
        m_enc_ready.store(false);
        m_enc_ping_pong[0].valid = false;
        m_enc_ping_pong[1].valid = false;
        xTaskCreatePinnedToCore(sourceEncTaskTrampoline, "bcast_enc_task", 8192, this, 6, &m_source_enc_task_handle, 1);
        xTaskCreatePinnedToCore(sourceTxTaskTrampoline, "bcast_tx_task", 8192, this, 7, &m_source_tx_task_handle, 0);
    } else {
        transitionTo(NetworkState::SCANNING);
        xTaskCreatePinnedToCore(sinkTaskTrampoline, "bcast_snk_task", 16384, this, 6, &m_sink_task_handle, 0);
    }
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::stop() {
    m_running.store(false, std::memory_order_release);
    transitionTo(NetworkState::IDLE);
    if (m_source_enc_task_handle) {
        vTaskDelete(m_source_enc_task_handle);
        m_source_enc_task_handle = nullptr;
    }
    if (m_source_tx_task_handle) {
        vTaskDelete(m_source_tx_task_handle);
        m_source_tx_task_handle = nullptr;
    }
    if (m_sink_task_handle) {
        vTaskDelete(m_sink_task_handle);
        m_sink_task_handle = nullptr;
    }
    return ESP_OK;
}

void EspNowBroadcastEngine::transitionTo(NetworkState new_state) {
    m_state.store(new_state, std::memory_order_release);

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
    return ESP_OK;
}

esp_err_t EspNowBroadcastEngine::setWifiPhyRate(wifi_phy_mode_t phymode, wifi_phy_rate_t rate) {
    m_tx_phy_rate = rate;
    esp_now_rate_config_t rate_cfg = {
        .phymode = phymode,
        .rate = rate,
        .ersu = false,
        .dcm = false
    };
    esp_now_set_peer_rate_config(s_broadcast_mac, &rate_cfg);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Packet Callbacks
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::onPacketSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    // Handled via ISR semaphore give
}

void EspNowBroadcastEngine::onPacketReceived(const uint8_t* mac_addr, const uint8_t* data, int data_len, int8_t rssi, uint8_t rate) {
    if (!data || data_len < 2) return;
    int64_t t_now_us = esp_timer_get_time();

    uint16_t tag = *reinterpret_cast<const uint16_t*>(data);
    uint8_t rx_id = get_rx_id(tag);
    uint8_t pkt_type = get_pkt_type(tag);

    // =======================================================================
    // SINK NODE LOGIC: Receive Audio Broadcast
    // =======================================================================
    if (m_node_role == NODE_ROLE_SINK) {
        // If SINK is in IDLE, drop audio packet and do not send Soft-ACK
        if (m_state.load(std::memory_order_acquire) == NetworkState::IDLE) {
            return;
        }

        // Instant filter: Reject if not for our channel and not wildcard broadcast
        if (rx_id != m_target_channel && rx_id != NODE_ID_BROADCAST) {
            return;
        }

        if (pkt_type == PKT_TYPE_AUDIO || pkt_type == PKT_TYPE_ARQ_REPAIR) {
            if (data_len >= static_cast<int>(sizeof(vsaf_audio_packet_t) - 120)) {
                const auto* pkt = reinterpret_cast<const vsaf_audio_packet_t*>(data);
                handleAudioPacket(pkt, rssi, t_now_us);
            }
        }
        return;
    }

    // =======================================================================
    // SOURCE NODE LOGIC: Receive Soft-ACK Broadcast Feedback
    // =======================================================================
    if (m_node_role == NODE_ROLE_SOURCE) {
        if (pkt_type == PKT_TYPE_SOFT_ACK && data_len >= static_cast<int>(sizeof(vsaf_soft_ack_t))) {
            const auto* ack = reinterpret_cast<const vsaf_soft_ack_t*>(data);
            
            // Verify CRC-16
            uint16_t computed_crc = calc_crc16(data, sizeof(vsaf_soft_ack_t) - sizeof(uint16_t));
            if (computed_crc != ack->crc16) {
                return;
            }

            uint8_t sink_id = get_tx_id(tag);
            if (sink_id < MAX_SINK_NODES) {
                // Compute RTT metrics
                int64_t total_rtt = t_now_us - ack->t_tx1_echo;
                int64_t dwell_us = ack->t_dwell_us;
                int64_t net_rtt = total_rtt - dwell_us;
                if (total_rtt >= 0 && total_rtt < 15000) {
                    m_peers[sink_id].last_total_rtt_us = static_cast<uint32_t>(total_rtt);
                    m_peers[sink_id].last_dwell_us = static_cast<uint32_t>(dwell_us);
                    m_peers[sink_id].last_net_rtt_us = (net_rtt >= 0) ? static_cast<uint32_t>(net_rtt) : 0;
                    m_peers[sink_id].last_rtt_us = m_peers[sink_id].last_net_rtt_us;
                    m_peers[sink_id].avg_rtt_us = (m_peers[sink_id].avg_rtt_us == 0)
                        ? m_peers[sink_id].last_net_rtt_us
                        : ((m_peers[sink_id].avg_rtt_us * 7 + m_peers[sink_id].last_net_rtt_us) / 8);
                }

                // Dual-Way PTP Time Sync calculation
                int64_t t_rx1 = ack->t_sink_tx_us - ack->t_dwell_us;
                int64_t offset = ((t_rx1 - ack->t_tx1_echo) - (t_now_us - ack->t_sink_tx_us)) / 2;
                m_peers[sink_id].clock_offset_us = static_cast<int32_t>(offset);
                m_time_offset_buf.push(offset / 1000.0f);
                m_ema_time_offset_ms = (m_ema_time_offset_ms * 0.9f) + (offset / 1000.0f * 0.1f);

                // Update signal quality & status
                m_peers[sink_id].last_rssi = rssi; // Uplink RSSI
                m_peers[sink_id].downlink_rssi = ack->downlink_rssi; // Downlink RSSI
                m_peers[sink_id].fifo_fill = ack->fifo_fill;
                m_peers[sink_id].acks_received++;
                m_peers[sink_id].consecutive_ack_fails = 0;
                m_peers[sink_id].status = PeerStatus::ONLINE;

                // Only mark current frame acked if ack_seq matches current sequence
                if (ack->ack_seq == m_seq) {
                    m_sink_acked[sink_id] = true;
                }
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

void EspNowBroadcastEngine::handleAudioPacket(const vsaf_audio_packet_t* pkt, int8_t rssi, int64_t t_rx1_us) {
    m_last_rssi = rssi;
    m_last_master_time_us = pkt->t_tx1_us;
    m_last_local_time_us = static_cast<uint32_t>(t_rx1_us);

    // Compute instantaneous clock offset between master and local SINK clock
    int32_t instant_offset = static_cast<int32_t>(pkt->t_tx1_us - static_cast<uint32_t>(t_rx1_us));
    float offset_ms = instant_offset / 1000.0f;
    m_time_offset_buf.push(offset_ms);
    if (m_ema_time_offset_ms == 0.0f) {
        m_ema_time_offset_ms = offset_ms;
    } else {
        m_ema_time_offset_ms = (m_ema_time_offset_ms * 0.95f) + (offset_ms * 0.05f);
    }

    // Sequence tracking & deduplication: reject duplicates or stale packets
    int16_t seq_diff = static_cast<int16_t>(pkt->seq - m_last_rx_seq);
    bool is_duplicate = false;

    if (!m_first_packet_received || seq_diff < -1000 || seq_diff > 5000) {
        // Initial synchronization or master sequence discontinuity
        m_first_packet_received = true;
        m_last_rx_seq = pkt->seq;
        is_duplicate = false;
    } else if (seq_diff <= 0) {
        // Duplicate (e.g. ARQ retry for already received frame) or out-of-order stale packet
        is_duplicate = true;
    } else {
        // Fresh audio frame needed for streaming
        m_last_rx_seq = pkt->seq;
        is_duplicate = false;
    }

    if (!is_duplicate) {
        // Only count audio packets this node needs to stream
        m_rx_packets_total++;
        m_rx_packets_sec++;

        // Push LC3 payload to Jitter FIFO
        portENTER_CRITICAL(&m_sink_fifo_lock);
        if (m_sink_fifo_count < SINK_FIFO_PACKETS) {
            m_sink_fifo[m_sink_fifo_head].seq = pkt->seq;
            m_sink_fifo[m_sink_fifo_head].len = pkt->octets;
            memcpy(m_sink_fifo[m_sink_fifo_head].data, pkt->data, pkt->octets);
            m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
            m_sink_fifo_count++;
        } else {
            m_fifo_overflows++;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);

        // Wake up SINK audio task if waiting on packet
        if (m_sink_task_handle) {
            vTaskNotifyGiveFromISR(m_sink_task_handle, nullptr);
        }
    }

    // Immediately assemble and broadcast Soft-ACK (< 50 us turnaround)
    sendSoftAck(pkt->seq, pkt->t_tx1_us, t_rx1_us, rssi);
}

void EspNowBroadcastEngine::sendSoftAck(uint16_t ack_seq, uint32_t t_tx1_echo, int64_t t_rx1_us, int8_t rssi) {
    // Stagger Soft-ACK transmission across SINK nodes to prevent uplink RF collisions
    if (m_target_channel > 0 && m_target_channel < MAX_SINK_NODES) {
        esp_rom_delay_us(m_target_channel * 300);
    }
    int64_t t_tx2 = esp_timer_get_time();

    vsaf_soft_ack_t ack = {};
    ack.tag = make_tag(m_target_channel, NODE_ID_SOURCE, PKT_TYPE_SOFT_ACK, FLAG_TIME_SYNC_VALID);
    ack.ack_seq = ack_seq;
    ack.t_tx1_echo = t_tx1_echo;
    ack.t_dwell_us = static_cast<uint16_t>(t_tx2 - t_rx1_us);
    ack.t_sink_tx_us = static_cast<uint32_t>(t_tx2);
    ack.downlink_rssi = rssi;

    portENTER_CRITICAL(&m_sink_fifo_lock);
    ack.fifo_fill = static_cast<uint8_t>((m_sink_fifo_count * 100) / SINK_FIFO_PACKETS);
    portEXIT_CRITICAL(&m_sink_fifo_lock);

    ack.crc16 = calc_crc16(reinterpret_cast<const uint8_t*>(&ack), sizeof(vsaf_soft_ack_t) - sizeof(uint16_t));

    // Broadcast Soft-ACK frame (no hardware ACK expected)
    esp_now_send(s_broadcast_mac, reinterpret_cast<const uint8_t*>(&ack), sizeof(vsaf_soft_ack_t));
}

// ---------------------------------------------------------------------------
// SOURCE Audio Encoder Task (Core 0, Priority 3): Pre-encodes next frame
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::sourceEncTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSourceEncLoop();
}

void EspNowBroadcastEngine::runSourceEncLoop() {
    ESP_LOGI(TAG, "SOURCE Audio Encoder Task started on Core 1");

    static int16_t pcm_mono[480];

    while (m_running.load(std::memory_order_acquire)) {
        // Wait for notification from TX task (timeout 20 ms)
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
            if (m_state.load(std::memory_order_acquire) != NetworkState::CAST) {
                continue;
            }
        }

        if (m_state.load(std::memory_order_acquire) != NetworkState::CAST) {
            continue;
        }

        // 1. Audio Generation (Synth test tone or USB PCM)
        size_t samples = Codec::calculateRequiredPcmSamples(m_telemetry.sample_rate, m_frame_duration_us);
        if (m_tone_gen) {
            m_tone_gen->generateFrame(pcm_mono, samples);
        } else {
            memset(pcm_mono, 0, samples * sizeof(int16_t));
        }

        // Dual LC3 encode
        int64_t enc_t0 = esp_timer_get_time();
        size_t actual_bytes0 = 0;
        size_t actual_bytes1 = 0;
        uint8_t write_idx = m_enc_write_idx.load(std::memory_order_relaxed);

        // Feed both encoders with the same audio from the sine synth
        m_lc3_codec.encodeFrame(pcm_mono, samples, m_enc_ping_pong[write_idx].data[0], m_octets_per_frame, &actual_bytes0, 0);
        m_lc3_codec.encodeFrame(pcm_mono, samples, m_enc_ping_pong[write_idx].data[1], m_octets_per_frame, &actual_bytes1, 1);

        m_enc_ping_pong[write_idx].octets = static_cast<uint16_t>(m_octets_per_frame);
        m_enc_ping_pong[write_idx].valid = true;

        float enc_ms = (esp_timer_get_time() - enc_t0) / 1000.0f;
        m_codec_duration_buf.push(enc_ms);
        m_audio_meter.pushFramePcm(pcm_mono, samples);

        // Advance read pointer to this buffer and toggle write index
        m_enc_read_idx.store(write_idx, std::memory_order_release);
        m_enc_write_idx.store(1 - write_idx, std::memory_order_relaxed);
        m_enc_ready.store(true, std::memory_order_release);
    }
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// SOURCE 802.11 Primary Sweep & ARQ Task (Core 1, Priority 5): Exact 750us Slots
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::sourceTxTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSourceTxLoop();
}

void EspNowBroadcastEngine::runSourceTxLoop() {
    ESP_LOGI(TAG, "SOURCE 802.11 Primary Sweep & ARQ Task started on Core 0 (Priority 7)");

    static uint8_t encoded_channels[MAX_SINK_NODES][120];
    int64_t next_frame_deadline = esp_timer_get_time();

    while (m_running.load(std::memory_order_acquire)) {
        if (m_state.load(std::memory_order_acquire) != NetworkState::CAST) {
            vTaskDelay(pdMS_TO_TICKS(10));
            next_frame_deadline = esp_timer_get_time() + m_frame_duration_us;
            continue;
        }

        // Microsecond hardware timer pacing: wait until exact next_frame_deadline
        int64_t now_us = esp_timer_get_time();
        if (now_us > next_frame_deadline + 20000) {
            next_frame_deadline = now_us; // Resync if paused
        }
        int64_t wait_us = next_frame_deadline - esp_timer_get_time();
        if (wait_us > 1500) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Yield 1 ms to FreeRTOS on Core 1
        }
        while (esp_timer_get_time() < next_frame_deadline) {
            esp_rom_delay_us(20);
        }

        int64_t frame_start_us = next_frame_deadline;
        next_frame_deadline += m_frame_duration_us;

        // Immediately trigger background encoder on Core 0 to prepare NEXT frame
        if (m_source_enc_task_handle) {
            xTaskNotifyGive(m_source_enc_task_handle);
        }

        // Retrieve latest pre-encoded frame from background encoder (0 us execution delay)
        uint8_t read_idx = m_enc_read_idx.load(std::memory_order_acquire);
        if (m_enc_ready.load(std::memory_order_acquire) && m_enc_ping_pong[read_idx].valid) {
            for (size_t ch = 0; ch < MAX_SINK_NODES; ++ch) {
                // LC3-frame0 --> Packet 0, 2, 4 (Left, Center, Surround Left)
                // LC3-frame1 --> Packet 1, 3, 5 (Right, Surround Right, Subwoofer)
                uint8_t enc_idx = (ch % 2 == 0) ? 0 : 1;
                memcpy(encoded_channels[ch], m_enc_ping_pong[read_idx].data[enc_idx], m_octets_per_frame);
            }
        } else {
            // Encoder not yet ready on first frame: silence
            for (size_t ch = 0; ch < MAX_SINK_NODES; ++ch) {
                memset(encoded_channels[ch], 0, m_octets_per_frame);
            }
        }

        // Reset soft-ACK tracking for this frame
        for (size_t i = 0; i < MAX_SINK_NODES; ++i) {
            m_sink_acked[i] = false;
        }

        // 1. Primary Burst Sweep: Rapidly broadcast all 6 channels spaced by 200 microseconds
        // (All 6 channels complete in 1.2 ms; 2 channels in 0.4 ms, leaving 2.8 ms quiet listening window)
        for (size_t ch = 0; ch < MAX_SINK_NODES; ++ch) {
            int64_t slot_target = frame_start_us + ch * 200;
            while (esp_timer_get_time() < slot_target) {
                esp_rom_delay_us(5);
            }

            // Assemble Audio Broadcast Frame
            m_last_tx_pkt[ch].tag = make_tag(NODE_ID_SOURCE, ch, PKT_TYPE_AUDIO, FLAG_TIME_SYNC_VALID);
            m_last_tx_pkt[ch].seq = m_seq;
            m_last_tx_pkt[ch].t_tx1_us = static_cast<uint32_t>(esp_timer_get_time());
            m_last_tx_pkt[ch].sample_rate_khz = static_cast<uint8_t>(m_telemetry.sample_rate / 1000);
            m_last_tx_pkt[ch].frame_dur_us = static_cast<uint16_t>(m_frame_duration_us);
            m_last_tx_pkt[ch].octets = static_cast<uint8_t>(m_octets_per_frame);
            memcpy(m_last_tx_pkt[ch].data, encoded_channels[ch], m_octets_per_frame);

            size_t pkt_len = sizeof(vsaf_audio_packet_t) - 120 + m_octets_per_frame;
            esp_now_send(s_broadcast_mac, reinterpret_cast<const uint8_t*>(&m_last_tx_pkt[ch]), pkt_len);

            m_peers[ch].packets_sent++;
            m_tx_packets_this_sec++;
            m_tx_packets_total++;
            m_tx_packets_sec++;
        }

        // 2. Dedicated Quiet Listening Window (t = 1.2 ms to 4.0 ms) & ARQ Window (t = 4.0 ms)
        int64_t arq_slot_start = frame_start_us + 4000;
        while (esp_timer_get_time() < arq_slot_start) {
            int64_t remaining = arq_slot_start - esp_timer_get_time();
            if (remaining > 1500) {
                vTaskDelay(pdMS_TO_TICKS(1));
            } else if (remaining > 0) {
                esp_rom_delay_us(10);
            }
        }

        int64_t current_arq_slot = arq_slot_start;
        for (size_t ch = 0; ch < MAX_SINK_NODES; ++ch) {
            if (m_peers[ch].status == PeerStatus::ONLINE && !m_sink_acked[ch]) {
                while (esp_timer_get_time() < current_arq_slot) {
                    esp_rom_delay_us(5);
                }

                vsaf_audio_packet_t repair_pkt = m_last_tx_pkt[ch];
                repair_pkt.tag = make_tag(NODE_ID_SOURCE, ch, PKT_TYPE_ARQ_REPAIR, FLAG_RETRY | FLAG_TIME_SYNC_VALID);

                size_t pkt_len = sizeof(vsaf_audio_packet_t) - 120 + m_octets_per_frame;
                esp_now_send(s_broadcast_mac, reinterpret_cast<const uint8_t*>(&repair_pkt), pkt_len);

                m_peers[ch].arq_retries++;
                current_arq_slot += 200;
            }
        }

        // Evaluate circuit breaker after ARQ window: track consecutive misses
        for (size_t ch = 0; ch < MAX_SINK_NODES; ++ch) {
            if (m_peers[ch].status == PeerStatus::ONLINE) {
                if (!m_sink_acked[ch]) {
                    m_peers[ch].ack_failures++;
                    m_peers[ch].consecutive_ack_fails++;
                    m_tx_ack_fails_total++;
                    m_tx_ack_fails_sec++;
                    if (m_peers[ch].consecutive_ack_fails >= 10) {
                        m_peers[ch].status = PeerStatus::OFFLINE;
                    }
                } else {
                    if (m_peers[ch].consecutive_ack_fails > 0) {
                        m_peers[ch].arq_successes++;
                    }
                    m_peers[ch].consecutive_ack_fails = 0;
                }
            }
        }

        m_seq++;
    }
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// SINK Audio Loop: Jitter Buffer PREFILL -> I2S DMA Synced Output
// ---------------------------------------------------------------------------

void EspNowBroadcastEngine::sinkTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSinkLoop();
}

void EspNowBroadcastEngine::runSinkLoop() {
    ESP_LOGI(TAG, "SINK Audio Decoder & I2S Task started on Core 0");

    static int16_t pcm_mono[480];
    static int16_t pcm_stereo[480 * 2];
    size_t samples_per_frame = (m_telemetry.sample_rate * 10) / 1000;
    uint32_t consecutive_underruns = 0;
    static constexpr size_t PREFILL_THRESHOLD = 8; // 8 packets = 80 ms cushion

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
        // 1. SCANNING: Wait for Jitter Buffer to accumulate PREFILL threshold
        // -------------------------------------------------------------------
        if (current_state == NetworkState::SCANNING) {
            portENTER_CRITICAL(&m_sink_fifo_lock);
            size_t buffered = m_sink_fifo_count;
            portEXIT_CRITICAL(&m_sink_fifo_lock);

            if (buffered >= PREFILL_THRESHOLD) {
                ESP_LOGI(TAG, "SINK: Jitter buffer prefill threshold reached (%zu pkts) -> PREFILL", buffered);
                transitionTo(NetworkState::PREFILL);
            } else {
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
                m_lc3_codec.decodeFrame(item1.data, item1.len, pcm_mono, 480, &actual_samples);
                float gain = std::pow(10.0f, m_current_gain_db / 20.0f);
                for (size_t i = 0; i < samples_per_frame; ++i) {
                    int16_t sample = static_cast<int16_t>(pcm_mono[i] * gain);
                    pcm_stereo[i * 2]     = sample;
                    pcm_stereo[i * 2 + 1] = sample;
                }
                if (m_i2s_dac) {
                    size_t bytes_written = 0;
                    m_i2s_dac->preload(pcm_stereo, samples_per_frame * sizeof(int16_t) * 2, &bytes_written);
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
                m_lc3_codec.decodeFrame(item2.data, item2.len, pcm_mono, 480, &actual_samples);
                float gain = std::pow(10.0f, m_current_gain_db / 20.0f);
                for (size_t i = 0; i < samples_per_frame; ++i) {
                    int16_t sample = static_cast<int16_t>(pcm_mono[i] * gain);
                    pcm_stereo[i * 2]     = sample;
                    pcm_stereo[i * 2 + 1] = sample;
                }
                if (m_i2s_dac) {
                    size_t bytes_written = 0;
                    m_i2s_dac->preload(pcm_stereo, samples_per_frame * sizeof(int16_t) * 2, &bytes_written);
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
        // Buffer regulation: if buffer backlog exceeds target (e.g. > 14 frames), drop oldest frame cleanly
        portENTER_CRITICAL(&m_sink_fifo_lock);
        while (m_sink_fifo_count > 14) {
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
            int16_t gap = m_has_expected_seq ? static_cast<int16_t>(item.seq - m_expected_seq) : 0;
            if (m_has_expected_seq && (gap < -4 || gap > 10)) {
                // Large sequence jump or stream restart: resynchronize immediately
                m_expected_seq = item.seq;
                gap = 0;
            }
            if (m_has_expected_seq && gap < 0) {
                // Stale frame that arrived late after PLC: drop it and check next in FIFO
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
            // In-sequence frame
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
            m_has_expected_seq = false; // Re-sync sequence counter when genuinely starved
            consecutive_underruns++;
            if (consecutive_underruns >= 10) {
                // 100 ms of missing audio: transition back to SCANNING
                ESP_LOGW(TAG, "SINK: 10 consecutive underruns -> returning to SCANNING");
                if (m_i2s_dac) {
                    m_i2s_dac->stop();
                }
                m_has_expected_seq = false;
                transitionTo(NetworkState::SCANNING);
                consecutive_underruns = 0;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
        } else {
            consecutive_underruns = 0;
        }

        int64_t dec_t0 = esp_timer_get_time();
        size_t actual_samples = 0;
        if (has_item) {
            m_lc3_codec.decodeFrame(item.data, item.len, pcm_mono, 480, &actual_samples);
        } else {
            // Packet Loss Concealment (PLC) during active STREAM
            m_lc3_codec.decodeFrame(nullptr, 0, pcm_mono, 480, &actual_samples);
            m_plc_count++;
            if (!is_gap_plc) {
                m_fifo_underflows++; // Only genuine buffer starvations count as FIFO underflow
            }
        }
        float dec_ms = (esp_timer_get_time() - dec_t0) / 1000.0f;
        m_codec_duration_buf.push(dec_ms);
        m_audio_meter.pushFramePcm(pcm_mono, samples_per_frame);

        // Mono to Stereo duplication for I2S DAC
        float gain = std::pow(10.0f, m_current_gain_db / 20.0f);
        for (size_t i = 0; i < samples_per_frame; ++i) {
            int16_t sample = static_cast<int16_t>(pcm_mono[i] * gain);
            pcm_stereo[i * 2]     = sample;
            pcm_stereo[i * 2 + 1] = sample;
        }

        if (m_i2s_dac) {
            size_t bytes_written = 0;
            m_i2s_dac->write(pcm_stereo, samples_per_frame * sizeof(int16_t) * 2, &bytes_written, 50);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
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
