#include "espnow_unicast_engine.hpp"
#include "console.hpp"
#include "config.h"
#include "status_led.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_idf_version.h"
#include "usb_audio.hpp"
#include <cstring>
#include <cmath>

static const char* TAG = "ESPNOW_UNICAST";

namespace AudioNet {

static inline float volume_u8_to_db(uint8_t vol) {
    if (vol == 0) return -120.0f; // MUTE
    return CONFIG_VOLUME_MIN_DB + static_cast<float>(vol - 1) * (CONFIG_VOLUME_MAX_DB - CONFIG_VOLUME_MIN_DB) / 254.0f;
}

static inline float db_to_linear(float gain_db) {
    if (gain_db <= -100.0f) return 0.0f;
    return std::pow(10.0f, gain_db / 20.0f);
}

static inline uint8_t delay_ms_to_code(uint16_t delay_ms) {
    switch (delay_ms) {
        case 20:  return 0;
        case 30:  return 1;
        case 40:  return 2;
        case 50:  return 3;
        case 60:  return 4;
        case 80:  return 5;
        case 100: return 6;
        case 200: return 7;
        default:  return 3; // 50 ms default
    }
}

static inline uint16_t code_to_delay_ms(uint8_t code) {
    switch (code & 0x07) {
        case 0: return 20;
        case 1: return 30;
        case 2: return 40;
        case 3: return 50;
        case 4: return 60;
        case 5: return 80;
        case 6: return 100;
        case 7: return 200;
        default: return 50;
    }
}

static inline uint16_t encode_vsaf_flags(uint32_t sample_rate_hz, uint32_t frame_duration_us, uint16_t delay_ms = 50) {
    uint16_t sr_code = 4; // 48 kHz default
    switch (sample_rate_hz) {
        case 8000:  sr_code = 0; break;
        case 16000: sr_code = 1; break;
        case 24000: sr_code = 2; break;
        case 32000: sr_code = 3; break;
        case 48000: sr_code = 4; break;
        case 96000: sr_code = 5; break;
        default:    sr_code = 4; break;
    }
    uint16_t dur_code = (frame_duration_us == 7500) ? 1 : 0;
    uint16_t delay_code = delay_ms_to_code(delay_ms);
    return (sr_code & 0x07) | ((dur_code & 0x03) << 3) | ((delay_code & 0x07) << 5);
}

static inline void decode_vsaf_flags(uint16_t flags, uint32_t* out_sr_hz, uint32_t* out_dur_us, uint16_t* out_delay_ms = nullptr) {
    uint16_t sr_code = flags & 0x07;
    uint16_t dur_code = (flags >> 3) & 0x03;
    uint16_t delay_code = (flags >> 5) & 0x07;
    if (out_sr_hz) {
        switch (sr_code) {
            case 0:  *out_sr_hz = 8000; break;
            case 1:  *out_sr_hz = 16000; break;
            case 2:  *out_sr_hz = 24000; break;
            case 3:  *out_sr_hz = 32000; break;
            case 4:  *out_sr_hz = 48000; break;
            case 5:  *out_sr_hz = 96000; break;
            default: *out_sr_hz = 48000; break;
        }
    }
    if (out_dur_us) {
        *out_dur_us = (dur_code == 1) ? 7500 : 10000;
    }
    if (out_delay_ms) {
        *out_delay_ms = code_to_delay_ms(delay_code);
    }
}

static EspNowUnicastEngine* s_instance = nullptr;
static TaskHandle_t s_audio_task_handle = nullptr;

// Core 0 LC3 Secondary Encoder Worker (Dual-Core parallel encoding for ESP32-S3)
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
struct Lc3EncodeJob {
    const int16_t* pcm;
    size_t samples;
    uint8_t* out_lc3;
    size_t out_max_len;
    size_t* out_actual_len;
    TaskHandle_t caller_task;
};

static TaskHandle_t s_lc3_worker_task_handle = nullptr;
static volatile Lc3EncodeJob s_lc3_job = {};
static Codec::Lc3CodecEngine* s_worker_codec_r = nullptr;

static void lc3_encoder_worker_core0(void* pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_worker_codec_r && s_lc3_job.pcm && s_lc3_job.out_lc3 && s_lc3_job.out_actual_len) {
            s_worker_codec_r->encodeFrame(s_lc3_job.pcm, s_lc3_job.samples,
                                          s_lc3_job.out_lc3, s_lc3_job.out_max_len,
                                          s_lc3_job.out_actual_len);
        }
        if (s_lc3_job.caller_task) {
            xTaskNotifyGive(s_lc3_job.caller_task);
        }
    }
}
#endif

// Wi-Fi TX Completion Synchronization Semaphore
static SemaphoreHandle_t s_tx_done_sem = nullptr;
static StaticSemaphore_t s_tx_done_sem_buf;

// Thread-safe SPSC FIFO for SINK LC3 frames
struct Lc3RxFrame {
    uint8_t  data[MAX_LC3_FRAME_OCTETS];
    uint8_t  len;
    uint8_t  seq;
    uint16_t sample_rate_hz;
    uint16_t frame_duration_us;
    uint16_t presentation_delay_ms;
    uint32_t master_time_us;
    int64_t  rx_local_time_us;
};

static constexpr size_t LC3_RX_FIFO_CAPACITY = 24;
static Lc3RxFrame s_rx_fifo[LC3_RX_FIFO_CAPACITY];
static size_t s_rx_fifo_head = 0;
static size_t s_rx_fifo_tail = 0;
static size_t s_rx_fifo_count = 0;
static portMUX_TYPE s_fifo_mux = portMUX_INITIALIZER_UNLOCKED;

static inline bool push_rx_lc3_frame(const uint8_t* data, size_t len, uint8_t seq,
                                     uint16_t sample_rate_hz, uint16_t frame_duration_us,
                                     uint16_t presentation_delay_ms,
                                     uint32_t master_time_us, int64_t rx_local_time_us) {
    if (!data || len == 0 || len > MAX_LC3_FRAME_OCTETS) return false;
    bool overflow = false;
    taskENTER_CRITICAL(&s_fifo_mux);
    if (s_rx_fifo_count >= LC3_RX_FIFO_CAPACITY) {
        s_rx_fifo_tail = (s_rx_fifo_tail + 1) % LC3_RX_FIFO_CAPACITY;
        s_rx_fifo_count--;
        overflow = true;
    }
    s_rx_fifo[s_rx_fifo_head].len = static_cast<uint8_t>(len);
    s_rx_fifo[s_rx_fifo_head].seq = seq;
    s_rx_fifo[s_rx_fifo_head].sample_rate_hz = sample_rate_hz;
    s_rx_fifo[s_rx_fifo_head].frame_duration_us = frame_duration_us;
    s_rx_fifo[s_rx_fifo_head].presentation_delay_ms = presentation_delay_ms;
    s_rx_fifo[s_rx_fifo_head].master_time_us = master_time_us;
    s_rx_fifo[s_rx_fifo_head].rx_local_time_us = rx_local_time_us;
    memcpy(s_rx_fifo[s_rx_fifo_head].data, data, len);
    s_rx_fifo_head = (s_rx_fifo_head + 1) % LC3_RX_FIFO_CAPACITY;
    s_rx_fifo_count++;
    taskEXIT_CRITICAL(&s_fifo_mux);
    return !overflow;
}

static inline bool pop_rx_lc3_frame(uint8_t* out_data, size_t* out_len, uint8_t* out_seq = nullptr,
                                     uint16_t* out_sample_rate = nullptr, uint16_t* out_frame_duration = nullptr,
                                     uint16_t* out_presentation_delay_ms = nullptr,
                                     uint32_t* out_master_time_us = nullptr, int64_t* out_rx_time = nullptr) {
    if (!out_data || !out_len) return false;
    taskENTER_CRITICAL(&s_fifo_mux);
    if (s_rx_fifo_count == 0) {
        taskEXIT_CRITICAL(&s_fifo_mux);
        *out_len = 0;
        return false;
    }
    *out_len = s_rx_fifo[s_rx_fifo_tail].len;
    if (out_seq) *out_seq = s_rx_fifo[s_rx_fifo_tail].seq;
    if (out_sample_rate) *out_sample_rate = s_rx_fifo[s_rx_fifo_tail].sample_rate_hz;
    if (out_frame_duration) *out_frame_duration = s_rx_fifo[s_rx_fifo_tail].frame_duration_us;
    if (out_presentation_delay_ms) *out_presentation_delay_ms = s_rx_fifo[s_rx_fifo_tail].presentation_delay_ms;
    if (out_master_time_us) *out_master_time_us = s_rx_fifo[s_rx_fifo_tail].master_time_us;
    if (out_rx_time) *out_rx_time = s_rx_fifo[s_rx_fifo_tail].rx_local_time_us;
    memcpy(out_data, s_rx_fifo[s_rx_fifo_tail].data, *out_len);
    s_rx_fifo_tail = (s_rx_fifo_tail + 1) % LC3_RX_FIFO_CAPACITY;
    s_rx_fifo_count--;
    taskEXIT_CRITICAL(&s_fifo_mux);
    return true;
}

static inline void clear_rx_fifo() {
    taskENTER_CRITICAL(&s_fifo_mux);
    s_rx_fifo_head = 0;
    s_rx_fifo_tail = 0;
    s_rx_fifo_count = 0;
    taskEXIT_CRITICAL(&s_fifo_mux);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onEspNowSendCb(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    if (s_instance && tx_info) s_instance->onPacketSent(tx_info->des_addr, status);
}
static void onEspNowRecvCb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int data_len) {
    if (!s_instance || !recv_info || !data) return;
    int8_t rssi = -127;
    uint8_t rate = 0;
    if (recv_info->rx_ctrl) {
        rssi = recv_info->rx_ctrl->rssi;
        rate = recv_info->rx_ctrl->rate;
    }
    s_instance->onPacketReceived(recv_info->src_addr, data, data_len, rssi, rate);
}
#else
static void onEspNowSendCb(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (s_instance) s_instance->onPacketSent(mac_addr, status);
}
static void onEspNowRecvCb(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
    if (!s_instance || !data) return;
    s_instance->onPacketReceived(mac_addr, data, data_len, -127, 0);
}
#endif

EspNowUnicastEngine::EspNowUnicastEngine(Codec::Lc3CodecEngine& lc3_codec,
                                         Audio::ToneGenerator* tone_gen,
                                         Hardware::I2sAudioDriver* i2s_dac)
    : m_lc3_codec(lc3_codec),
      m_tone_gen(tone_gen),
      m_i2s_dac(i2s_dac),
      m_node_role(NODE_ROLE_SINK),
      m_node_id(CONFIG_ACTIVE_NODE_ID) {
    s_instance = this;
    memset(m_peers, 0, sizeof(m_peers));
}

EspNowUnicastEngine::~EspNowUnicastEngine() {
    stop();
    if (m_i2s_start_timer) {
        esp_timer_stop(m_i2s_start_timer);
        esp_timer_delete(m_i2s_start_timer);
        m_i2s_start_timer = nullptr;
    }
    if (s_instance == this) s_instance = nullptr;
}

esp_err_t EspNowUnicastEngine::init(uint8_t role, uint8_t node_id, uint8_t wifi_channel) {
    m_node_role = role;
    m_node_id = node_id;

    // Default tone frequencies (100% full scale amplitude)
    if (m_tone_gen) m_tone_gen->init(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 440.0f, 110.0f, 440.0f, 100.0f);
    m_tone_gen_r.init(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 880.0f, 220.0f, 880.0f, 100.0f);

    float init_vol_db = volume_u8_to_db(m_target_volume_u8.load());
    m_current_gain_db = init_vol_db;
    m_current_linear_gain = db_to_linear(init_vol_db);

    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.initEncoder(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 1, 10000, m_octets_per_frame);
        m_lc3_codec_r.initEncoder(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 1, 10000, m_octets_per_frame);

        // Initialize Subwoofer LC3 Encoder (8000 Hz, 1-ch, 10ms, 80 octets = 64 kbps)
        m_lc3_codec_sub.initEncoder(CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ, 1, 10000, CONFIG_ESPNOW_SUB_FRAME_LEN_OCTETS);

        // Initialize 4th-order Linkwitz-Riley Low-Pass Filter @ 100 Hz
        m_sub_lr4_filter.init(CONFIG_ESPNOW_SAMPLE_RATE_HZ, CONFIG_ESPNOW_SUB_LP_HZ);
    } else {
        m_lc3_codec.initDecoder(CONFIG_ESPNOW_SAMPLE_RATE_HZ, 1, 10000, m_octets_per_frame);

        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &i2sStartTimerCallback;
        timer_args.arg = this;
        timer_args.name = "i2s_start";
        esp_timer_create(&timer_args, &m_i2s_start_timer);
    }

    // Default known peers for SOURCE (initially OFFLINE until SINK_HELLO is received)
    if (m_node_role == NODE_ROLE_SOURCE) {
        const uint8_t left_mac[6]  = {0xB0, 0xA6, 0x04, 0x99, 0x38, 0x44}; // Node 23
        const uint8_t right_mac[6] = {0xB0, 0xA6, 0x04, 0x99, 0x18, 0xE4}; // Node 24

        addPeer(left_mac, 0, "Sink-Left");
        addPeer(right_mac, 1, "Sink-Right");
    }

    // Initialize Wi-Fi
    if (!m_wifi_initialized) {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());

        // Lock to specified Wi-Fi channel
        ESP_ERROR_CHECK(esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));

        // Standard 2.4 GHz protocols (11b/g/n on S3, 11b/g/n/ax on C6).
        // Rate locking to 24 Mbps OFDM is enforced via esp_wifi_config_espnow_rate and peer_info.rate
#if defined(CONFIG_IDF_TARGET_ESP32C6)
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX));
#else
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
#endif

        // Set transmit power (+9.0 dBm = 36 * 0.25 dBm)
        esp_wifi_set_max_tx_power(12); // +3.00 dBm (12 * 0.25 dBm)

        // Initialize ESP-NOW
        ESP_ERROR_CHECK(esp_now_init());
        ESP_ERROR_CHECK(esp_now_register_send_cb(onEspNowSendCb));
        ESP_ERROR_CHECK(esp_now_register_recv_cb(onEspNowRecvCb));

        // Register Broadcast Peer (FF:FF:FF:FF:FF:FF) for SINK handshake announcements
        const uint8_t bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_peer_info_t bcast_peer = {};
        memcpy(bcast_peer.peer_addr, bcast_mac, 6);
        bcast_peer.channel = wifi_channel;
        bcast_peer.ifidx = WIFI_IF_STA;
        bcast_peer.encrypt = false;
        esp_now_add_peer(&bcast_peer);

        // Configure default locked PHY rate (24.0 Mbps OFDM: ~100 us airtime, high RF robustness)
        setWifiPhyRate(WIFI_PHY_MODE_11G, WIFI_PHY_RATE_24M);

        m_wifi_initialized = true;
    }

    // Register active unicast peers in ESP-NOW hardware table
    for (int i = 0; i < m_peer_count; i++) {
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, m_peers[i].mac, 6);
        peer_info.channel = wifi_channel;
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;
        esp_now_add_peer(&peer_info);

        esp_now_rate_config_t rate_cfg = {
            .phymode = m_tx_phy_mode,
            .rate = m_tx_phy_rate,
            .ersu = false,
            .dcm = false
        };
        esp_now_set_peer_rate_config(m_peers[i].mac, &rate_cfg);
    }

    transitionTo(NetworkState::IDLE);
    return ESP_OK;
}

esp_err_t EspNowUnicastEngine::start() {
    if (m_audio_task_running) return ESP_OK;

    m_audio_task_running = true;
    if (!s_tx_done_sem) {
        s_tx_done_sem = xSemaphoreCreateBinaryStatic(&s_tx_done_sem_buf);
    }
    uint32_t stack_size = 16384;
    UBaseType_t priority = 5;
    BaseType_t core_id = (m_node_role == NODE_ROLE_SOURCE) ? 1 : 0;

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
    if (m_node_role == NODE_ROLE_SOURCE && !s_lc3_worker_task_handle) {
        s_worker_codec_r = &m_lc3_codec_r;
        xTaskCreatePinnedToCore(lc3_encoder_worker_core0, "lc3_worker_c0", 16384, nullptr, 6, &s_lc3_worker_task_handle, 0);
    }
#endif

    xTaskCreatePinnedToCore(audioTaskRoutine, "unicast_audio_tsk", stack_size, this, priority, &s_audio_task_handle, core_id);

    if (m_node_role == NODE_ROLE_SOURCE) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (m_tone_test_mode || usb_audio_is_streaming()) {
            transitionTo(NetworkState::CAST);
        } else {
            transitionTo(NetworkState::IDLE);
        }
#else
        transitionTo(NetworkState::CAST);
#endif
    } else {
        transitionTo(NetworkState::SCANNING);
    }


    return ESP_OK;
}

esp_err_t EspNowUnicastEngine::stop() {
    m_audio_task_running = false;
    transitionTo(NetworkState::IDLE);
    return ESP_OK;
}

void EspNowUnicastEngine::setVolume(uint8_t vol_u8, bool instant) {
    m_target_volume_u8.store(vol_u8, std::memory_order_relaxed);
    if (instant) {
        m_instant_volume_requested.store(true, std::memory_order_relaxed);
    }
    float db = getTargetVolumeDb();
    if (vol_u8 == 0) {
        ESP_LOGI(TAG, "Volume Set: MUTE (0)");
    } else {
        ESP_LOGI(TAG, "Volume Set: %u / 255 (%+5.1f dB)%s", vol_u8, db, instant ? " [INSTANT]" : " [SLEW 96dB/s]");
    }
}

float EspNowUnicastEngine::getTargetVolumeDb() const {
    uint8_t vol = m_target_volume_u8.load(std::memory_order_relaxed);
    return volume_u8_to_db(vol);
}

void EspNowUnicastEngine::sendVolumeCommand(uint8_t channel_id, uint8_t vol_u8, bool instant) {
    vsaf_unicast_header_t vol_pkt = {};
    vol_pkt.seq = 0;
    vol_pkt.octets = 0; // Control frame
    vol_pkt.ctrl.opcode = static_cast<uint8_t>(ControlOpcode::VOLUME_SET);
    vol_pkt.ctrl.channel_id = channel_id;
    vol_pkt.ctrl.volume_u8 = vol_u8;
    vol_pkt.ctrl.flags = instant ? 0x01 : 0x00;

    uint8_t target_macs[MAX_UNICAST_SINKS][6];
    int target_count = 0;

    taskENTER_CRITICAL(&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        if (m_peers[i].is_enabled && m_peers[i].status == PeerStatus::ONLINE) {
            if (channel_id == 0xFF || m_peers[i].channel_id == channel_id) {
                if (target_count < MAX_UNICAST_SINKS) {
                    memcpy(target_macs[target_count++], m_peers[i].mac, 6);
                }
            }
        }
    }
    taskEXIT_CRITICAL(&m_peer_mux);

    for (int i = 0; i < target_count; i++) {
        if (s_tx_done_sem) xSemaphoreTake(s_tx_done_sem, 0);
        if (esp_now_send(target_macs[i], reinterpret_cast<const uint8_t*>(&vol_pkt), sizeof(vol_pkt)) == ESP_OK) {
            if (s_tx_done_sem) xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(3));
        }
    }

    float db = volume_u8_to_db(vol_u8);
    ESP_LOGI(TAG, "SOURCE dispatched VOLUME_SET -> Channel %u : %u/255 (%+5.1f dB)%s",
             channel_id, vol_u8, db, instant ? " [INSTANT]" : " [SLEW]");
}

bool EspNowUnicastEngine::addPeer(const uint8_t* mac, uint8_t channel_id, const char* name) {
    if (!mac) return false;
    taskENTER_CRITICAL(&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        if (memcmp(m_peers[i].mac, mac, 6) == 0) {
            m_peers[i].channel_id = channel_id;
            if (name) strncpy(m_peers[i].name, name, sizeof(m_peers[i].name) - 1);
            m_peers[i].is_enabled = true;
            m_peers[i].status = PeerStatus::OFFLINE; // Initial state until SINK_HELLO is received
            m_peers[i].consecutive_ack_fails = 0;
            taskEXIT_CRITICAL(&m_peer_mux);
            return true;
        }
    }
    if (m_peer_count >= MAX_UNICAST_SINKS) {
        taskEXIT_CRITICAL(&m_peer_mux);
        return false;
    }
    int idx = m_peer_count++;
    memcpy(m_peers[idx].mac, mac, 6);
    m_peers[idx].channel_id = channel_id;
    if (name) strncpy(m_peers[idx].name, name, sizeof(m_peers[idx].name) - 1);
    else snprintf(m_peers[idx].name, sizeof(m_peers[idx].name), "Sink-%u", (unsigned int)(channel_id & 0x07));
    m_peers[idx].is_enabled = true;
    m_peers[idx].status = PeerStatus::OFFLINE;
    m_peers[idx].session_start_time_us = 0;
    m_peers[idx].packets_sent = 0;
    m_peers[idx].acks_received = 0;
    m_peers[idx].ack_failures = 0;
    m_peers[idx].consecutive_ack_fails = 0;
    m_peers[idx].last_rssi = -127;
    m_peers[idx].last_tx_duration_us = 200;
    m_peers[idx].ema_tx_duration_us = 200;
    taskEXIT_CRITICAL(&m_peer_mux);

    if (m_wifi_initialized) {
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, mac, 6);
        peer_info.channel = 1;
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;
        esp_now_add_peer(&peer_info);

        esp_now_rate_config_t rate_cfg = {
            .phymode = m_tx_phy_mode,
            .rate = m_tx_phy_rate,
            .ersu = false,
            .dcm = false
        };
        esp_now_set_peer_rate_config(mac, &rate_cfg);
    }
    return true;
}

bool EspNowUnicastEngine::addOrUpdatePeerFromHello(const uint8_t* mac, uint8_t channel_id, const char* name) {
    if (!mac) return false;
    taskENTER_CRITICAL(&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        if (memcmp(m_peers[i].mac, mac, 6) == 0) {
            m_peers[i].channel_id = channel_id;
            m_peers[i].status = PeerStatus::ONLINE;
            m_peers[i].is_enabled = true;
            if (m_peers[i].session_start_time_us == 0) {
                m_peers[i].session_start_time_us = esp_timer_get_time();
            }
            taskEXIT_CRITICAL(&m_peer_mux);
            return true;
        }
    }
    if (m_peer_count >= MAX_UNICAST_SINKS) {
        taskEXIT_CRITICAL(&m_peer_mux);
        return false;
    }
    int idx = m_peer_count++;
    memcpy(m_peers[idx].mac, mac, 6);
    m_peers[idx].channel_id = channel_id;
    if (name) strncpy(m_peers[idx].name, name, sizeof(m_peers[idx].name) - 1);
    else snprintf(m_peers[idx].name, sizeof(m_peers[idx].name), "Sink-%u", (unsigned int)(channel_id & 0x07));
    m_peers[idx].is_enabled = true;
    m_peers[idx].status = PeerStatus::ONLINE;
    m_peers[idx].session_start_time_us = esp_timer_get_time();
    m_peers[idx].packets_sent = 0;
    m_peers[idx].acks_received = 0;
    m_peers[idx].ack_failures = 0;
    m_peers[idx].consecutive_ack_fails = 0;
    m_peers[idx].last_rssi = -127;
    m_peers[idx].last_tx_duration_us = 200;
    m_peers[idx].ema_tx_duration_us = 200;
    taskEXIT_CRITICAL(&m_peer_mux);

    if (m_wifi_initialized) {
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, mac, 6);
        peer_info.channel = 1;
        peer_info.ifidx = WIFI_IF_STA;
        peer_info.encrypt = false;
        esp_now_add_peer(&peer_info);

        esp_now_rate_config_t rate_cfg = {
            .phymode = m_tx_phy_mode,
            .rate = m_tx_phy_rate,
            .ersu = false,
            .dcm = false
        };
        esp_now_set_peer_rate_config(mac, &rate_cfg);
    }
    
    return true;
}

bool EspNowUnicastEngine::removePeer(const uint8_t* mac) {
    if (!mac) return false;
    taskENTER_CRITICAL(&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        if (memcmp(m_peers[i].mac, mac, 6) == 0) {
            for (int j = i; j < m_peer_count - 1; j++) {
                m_peers[j] = m_peers[j + 1];
            }
            m_peer_count--;
            taskEXIT_CRITICAL(&m_peer_mux);
            if (m_wifi_initialized) {
                esp_now_del_peer(mac);
            }
            return true;
        }
    }
    taskEXIT_CRITICAL(&m_peer_mux);
    return false;
}

bool EspNowUnicastEngine::setPeerEnabled(const uint8_t* mac, bool enabled) {
    if (!mac) return false;
    taskENTER_CRITICAL(&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        if (memcmp(m_peers[i].mac, mac, 6) == 0) {
            m_peers[i].is_enabled = enabled;
            m_peers[i].status = enabled ? PeerStatus::ONLINE : PeerStatus::DISABLED;
            if (enabled) {
                m_peers[i].consecutive_ack_fails = 0;
                if (m_state == NetworkState::CAST) {
                    m_peers[i].session_start_time_us = esp_timer_get_time();
                }
            }
            taskEXIT_CRITICAL(&m_peer_mux);
            return true;
        }
    }
    taskEXIT_CRITICAL(&m_peer_mux);
    return false;
}

const SinkPeerConfig* EspNowUnicastEngine::getPeer(int index) const {
    if (index < 0 || index >= m_peer_count) return nullptr;
    return &m_peers[index];
}

const SinkPeerConfig* EspNowUnicastEngine::getPeerByMac(const uint8_t* mac) const {
    if (!mac) return nullptr;
    for (int i = 0; i < m_peer_count; i++) {
        if (memcmp(m_peers[i].mac, mac, 6) == 0) return &m_peers[i];
    }
    return nullptr;
}

void EspNowUnicastEngine::getNodeStatusString(char* out_buf, size_t max_len) const {
    if (!out_buf || max_len < 7) return;

    // Channels: 0: Left, 1: Right, 2: Center, 3: Left Surround, 4: Right Surround, 5: Subwoofer
    char statuses[7] = "OOOOOO"; // Default all 6 channels to Offline

    taskENTER_CRITICAL((portMUX_TYPE*)&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        uint8_t ch = m_peers[i].channel_id;
        if (ch < 6) {
            if (!m_peers[i].is_enabled || m_peers[i].status == PeerStatus::DISABLED) {
                statuses[ch] = 'D';
            } else if (m_peers[i].status == PeerStatus::ONLINE) {
                statuses[ch] = '1';
            } else {
                statuses[ch] = 'O';
            }
        }
    }
    taskEXIT_CRITICAL((portMUX_TYPE*)&m_peer_mux);

    snprintf(out_buf, max_len, "%s", statuses);
}

void EspNowUnicastEngine::resetPeerStats() {
    taskENTER_CRITICAL(&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        m_peers[i].packets_sent = 0;
        m_peers[i].acks_received = 0;
        m_peers[i].ack_failures = 0;
        m_peers[i].consecutive_ack_fails = 0;
    }
    taskEXIT_CRITICAL(&m_peer_mux);
    m_tx_packets_total.store(0, std::memory_order_relaxed);
    m_tx_acks_total.store(0, std::memory_order_relaxed);
    m_tx_ack_fails_total.store(0, std::memory_order_relaxed);
    m_tx_acks_sec.store(0, std::memory_order_relaxed);
    m_tx_ack_fails_sec.store(0, std::memory_order_relaxed);
}

esp_err_t EspNowUnicastEngine::setSampleRate(uint32_t sample_rate_hz) {
    m_telemetry.sample_rate = sample_rate_hz;
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.reconfigureEncoder(sample_rate_hz, m_octets_per_frame, m_frame_duration_us);
        m_lc3_codec_r.reconfigureEncoder(sample_rate_hz, m_octets_per_frame, m_frame_duration_us);
        m_sub_lr4_filter.init(sample_rate_hz, m_sub_cutoff_hz);
    } else {
        m_lc3_codec.reconfigureDecoder(sample_rate_hz, m_octets_per_frame, m_frame_duration_us);
    }
    if (m_i2s_dac) m_i2s_dac->reconfigureSampleRate(sample_rate_hz, m_frame_duration_us);
    if (m_tone_gen) m_tone_gen->setSampleRate(sample_rate_hz);
    m_tone_gen_r.setSampleRate(sample_rate_hz);
    return ESP_OK;
}

esp_err_t EspNowUnicastEngine::setBitDepth(uint8_t bit_depth) {
    m_telemetry.bit_depth = bit_depth;
    if (m_i2s_dac) m_i2s_dac->setBitDepth(bit_depth);
    return ESP_OK;
}

esp_err_t EspNowUnicastEngine::setFrameLen(uint16_t frame_len_octets) {
    if (frame_len_octets > MAX_LC3_FRAME_OCTETS) return ESP_ERR_INVALID_ARG;
    m_octets_per_frame = frame_len_octets;
    m_telemetry.frame_len = frame_len_octets;
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.reconfigureEncoder(m_telemetry.sample_rate, m_octets_per_frame, m_frame_duration_us);
        m_lc3_codec_r.reconfigureEncoder(m_telemetry.sample_rate, m_octets_per_frame, m_frame_duration_us);
    } else {
        m_lc3_codec.reconfigureDecoder(m_telemetry.sample_rate, m_octets_per_frame, m_frame_duration_us);
    }
    return ESP_OK;
}

esp_err_t EspNowUnicastEngine::setFrameDuration(uint32_t frame_duration_us) {
    m_frame_duration_us = frame_duration_us;
    m_telemetry.frame_duration_us = frame_duration_us;
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.reconfigureEncoder(m_telemetry.sample_rate, m_octets_per_frame, m_frame_duration_us);
        m_lc3_codec_r.reconfigureEncoder(m_telemetry.sample_rate, m_octets_per_frame, m_frame_duration_us);
    } else {
        m_lc3_codec.reconfigureDecoder(m_telemetry.sample_rate, m_octets_per_frame, m_frame_duration_us);
    }
    return ESP_OK;
}

esp_err_t EspNowUnicastEngine::setWifiPhyRate(wifi_phy_mode_t phymode, wifi_phy_rate_t rate) {
    m_tx_phy_mode = phymode;
    m_tx_phy_rate = rate;
    if (m_wifi_initialized) {
        esp_now_rate_config_t rate_cfg = {
            .phymode = phymode,
            .rate = rate,
            .ersu = false,
            .dcm = false
        };
        // Lock broadcast peer rate
        const uint8_t bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_set_peer_rate_config(bcast_mac, &rate_cfg);

        taskENTER_CRITICAL(&m_peer_mux);
        for (int i = 0; i < m_peer_count; i++) {
            if (m_peers[i].is_enabled) {
                esp_now_set_peer_rate_config(m_peers[i].mac, &rate_cfg);
            }
        }
        taskEXIT_CRITICAL(&m_peer_mux);
        esp_wifi_config_80211_tx_rate(WIFI_IF_STA, rate);
    }
    return ESP_OK;
}

void EspNowUnicastEngine::onPacketSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (s_tx_done_sem) {
        xSemaphoreGive(s_tx_done_sem);
    }

    if (!mac_addr) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            m_tx_acks_total.fetch_add(1, std::memory_order_relaxed);
            m_tx_acks_sec.fetch_add(1, std::memory_order_relaxed);
        } else {
            m_tx_ack_fails_total.fetch_add(1, std::memory_order_relaxed);
            m_tx_ack_fails_sec.fetch_add(1, std::memory_order_relaxed);
        }
        return;
    }
    taskENTER_CRITICAL(&m_peer_mux);
    for (int i = 0; i < m_peer_count; i++) {
        if (memcmp(m_peers[i].mac, mac_addr, 6) == 0) {
            m_peers[i].packets_sent++;
            if (status == ESP_NOW_SEND_SUCCESS) {
                m_peers[i].acks_received++;
                m_peers[i].consecutive_ack_fails = 0;
                if (m_peers[i].is_enabled) {
                    m_peers[i].status = PeerStatus::ONLINE;
                }
                m_tx_acks_total.fetch_add(1, std::memory_order_relaxed);
                m_tx_acks_sec.fetch_add(1, std::memory_order_relaxed);
            } else {
                m_peers[i].ack_failures++;
                m_peers[i].consecutive_ack_fails++;
                // Immediate circuit breaker: Trip to OFFLINE on the very first missing ACK!
                // This guarantees that a dead or deeply faded peer is immediately removed
                // from the next 10 ms streaming cycle so it cannot steal radio airtime.
                if (m_peers[i].consecutive_ack_fails >= 1) {
                    m_peers[i].status = PeerStatus::OFFLINE;
                    m_peers[i].ema_tx_duration_us = 20000; // Heavily demote priority sort
                }
                m_tx_ack_fails_total.fetch_add(1, std::memory_order_relaxed);
                m_tx_ack_fails_sec.fetch_add(1, std::memory_order_relaxed);
            }
            break;
        }
    }
    taskEXIT_CRITICAL(&m_peer_mux);
}

void EspNowUnicastEngine::onPacketReceived(const uint8_t* mac_addr, const uint8_t* data, int data_len, int8_t rssi, uint8_t rate) {
    if (!data || data_len < sizeof(vsaf_unicast_header_t)) return;

    const vsaf_unicast_header_t* hdr = reinterpret_cast<const vsaf_unicast_header_t*>(data);

    m_last_rx_rssi.store(rssi, std::memory_order_relaxed);
    m_last_rx_rate.store(rate, std::memory_order_relaxed);

    if (hdr->octets == 0) {
        // Control & Handshake Packet Dispatcher
        if (hdr->ctrl.opcode == static_cast<uint8_t>(ControlOpcode::SINK_HELLO)) {
            uint8_t req_channel = hdr->ctrl.channel_id;
            addOrUpdatePeerFromHello(mac_addr, req_channel);
        } else if (hdr->ctrl.opcode == static_cast<uint8_t>(ControlOpcode::SINK_BYE)) {
            taskENTER_CRITICAL(&m_peer_mux);
            for (int i = 0; i < m_peer_count; i++) {
                if (memcmp(m_peers[i].mac, mac_addr, 6) == 0) {
                    m_peers[i].status = PeerStatus::OFFLINE;
                    break;
                }
            }
            taskEXIT_CRITICAL(&m_peer_mux);
            
        } else if (hdr->ctrl.opcode == static_cast<uint8_t>(ControlOpcode::VOLUME_SET)) {
            // Volume Command from SOURCE
            if (hdr->ctrl.channel_id == 0xFF || hdr->ctrl.channel_id == m_target_channel) {
                bool instant = (hdr->ctrl.flags & 0x01) != 0;
                setVolume(hdr->ctrl.volume_u8, instant);
            }
        }
        return;
    }

    // Audio Packet Ingestion (for SINK node)
    m_rx_packets_total.fetch_add(1, std::memory_order_relaxed);
    m_rx_packets_sec.fetch_add(1, std::memory_order_relaxed);

    size_t payload_len = data_len - sizeof(vsaf_unicast_header_t);
    if (payload_len == 0 || payload_len > MAX_LC3_FRAME_OCTETS) return;

    int64_t rx_local_time_us = esp_timer_get_time();

    uint32_t packet_sr = 48000;
    uint32_t packet_dur = 10000;
    uint16_t packet_delay_ms = 50;
    decode_vsaf_flags(hdr->audio.flags, &packet_sr, &packet_dur, &packet_delay_ms);

    // Calculate delta between master_time_us and local reception time
    int32_t instant_offset_32 = static_cast<int32_t>(static_cast<int64_t>(hdr->audio.master_time_us) - rx_local_time_us);
    int64_t instant_offset = instant_offset_32;

    if (m_master_time_offset_us == 0) {
        m_master_time_offset_us = instant_offset;
    } else {
        m_master_time_offset_us = (m_master_time_offset_us * 95 + instant_offset * 5) / 100;
    }
    m_time_offset_ring_buffer.push(static_cast<float>(instant_offset) / 1000.0f);
    m_last_sync_time_us.store(rx_local_time_us, std::memory_order_relaxed);

    if (!push_rx_lc3_frame(data + sizeof(vsaf_unicast_header_t), payload_len, hdr->seq,
                           packet_sr, packet_dur, packet_delay_ms, hdr->audio.master_time_us, rx_local_time_us)) {
        m_fifo_overflow.fetch_add(1, std::memory_order_relaxed);
    } else {
        if (s_audio_task_handle) {
            xTaskNotifyGive(s_audio_task_handle);
        }
    }
}


void EspNowUnicastEngine::transitionTo(NetworkState new_state) {
    if (m_state == new_state) return;
    const char* old_str = getStateString();
    m_state = new_state;
    const char* new_str = getStateString();

    print_console("\n[STATE CHANGE] %s ---> %s (Node %u)\n", old_str, new_str, m_node_id);

    // Reset error and PLC counters on stream activation
    if (new_state == NetworkState::CAST || new_state == NetworkState::STREAM || new_state == NetworkState::PREFILL) {
        m_lc3_codec.resetPlcCount();
        m_fifo_underrun.store(0, std::memory_order_relaxed);
        m_fifo_overflow.store(0, std::memory_order_relaxed);
        if (m_i2s_dac) m_i2s_dac->resetUnderrunCount();

        int64_t now_us = esp_timer_get_time();
        m_session_start_time_us.store(now_us, std::memory_order_relaxed);
        taskENTER_CRITICAL(&m_peer_mux);
        for (int i = 0; i < m_peer_count; i++) {
            if (m_peers[i].is_enabled && m_peers[i].status == PeerStatus::ONLINE) {
                m_peers[i].session_start_time_us = now_us;
            }
        }
        taskEXIT_CRITICAL(&m_peer_mux);
    }

    // Update Status LED state
    switch (new_state) {
        case NetworkState::CAST:
            Hardware::getStatusLed().setSystemState(Hardware::SystemState::BROADCASTING_TONE);
            break;
        case NetworkState::STREAM:
        case NetworkState::PREFILL:
            Hardware::getStatusLed().setSystemState(Hardware::SystemState::STREAM);
            break;
        case NetworkState::SCANNING:
            Hardware::getStatusLed().setSystemState(Hardware::SystemState::SCANNING);
            break;
        case NetworkState::IDLE:
        case NetworkState::OFF:
        default:
            Hardware::getStatusLed().setSystemState(Hardware::SystemState::IDLE);
            break;
    }

    ESP_LOGI(TAG, "State Transition: -> %s",
             (new_state == NetworkState::CAST) ? "CAST" :
             (new_state == NetworkState::STREAM) ? "STREAM" :
             (new_state == NetworkState::PREFILL) ? "PREFILL" :
             (new_state == NetworkState::SCANNING) ? "SCANNING" : "IDLE");
}

const char* EspNowUnicastEngine::getStateString() const {
    switch (m_state) {
        case NetworkState::OFF:         return "OFF";
        case NetworkState::IDLE:        return "IDLE";
        case NetworkState::SCANNING:    return "SCANNING";
        case NetworkState::PREFILL:     return "PREFILL";
        case NetworkState::STREAM:      return "STREAM";
        case NetworkState::CAST:        return "CAST";
        default:                        return "UNKNOWN";
    }
}

const char* EspNowUnicastEngine::getActiveCodecName() const {
    return "LC3";
}

const char* EspNowUnicastEngine::getWifiPhyRateString() const {
    switch (m_tx_phy_rate) {
        case WIFI_PHY_RATE_MCS0_LGI: return "HT20 MCS0 (6.5M)";
        case WIFI_PHY_RATE_MCS1_LGI: return "HT20 MCS1 (13.0M)";
        case WIFI_PHY_RATE_MCS2_LGI: return "HT20 MCS2 (19.5M)";
        case WIFI_PHY_RATE_MCS3_LGI: return "HT20 MCS3 (26.0M)";
        case WIFI_PHY_RATE_6M:       return "OFDM 6M";
        case WIFI_PHY_RATE_9M:       return "OFDM 9M";
        case WIFI_PHY_RATE_12M:      return "OFDM 12M";
        case WIFI_PHY_RATE_18M:      return "OFDM 18M";
        case WIFI_PHY_RATE_24M:      return "OFDM 24M";
        case WIFI_PHY_RATE_36M:      return "OFDM 36M";
        case WIFI_PHY_RATE_48M:      return "OFDM 48M";
        case WIFI_PHY_RATE_54M:      return "OFDM 54M";
        default:                     return "CUSTOM";
    }
}

void EspNowUnicastEngine::resetStreamingCounters() {
    m_tx_packets_total.store(0, std::memory_order_relaxed);
    m_tx_packets_sec.store(0, std::memory_order_relaxed);
    m_tx_acks_total.store(0, std::memory_order_relaxed);
    m_tx_ack_fails_total.store(0, std::memory_order_relaxed);
    m_rx_packets_total.store(0, std::memory_order_relaxed);
    m_rx_packets_sec.store(0, std::memory_order_relaxed);
    resetPeerStats();
}

void EspNowUnicastEngine::resetErrorCounters() {
    m_fifo_overflow.store(0, std::memory_order_relaxed);
    m_fifo_underrun.store(0, std::memory_order_relaxed);
    m_lc3_codec.resetPlcCount();
    if (m_i2s_dac) m_i2s_dac->resetUnderrunCount();
}

void EspNowUnicastEngine::update10HzTimeOffsetStats() {
    float med, rng;
    bool valid;
    m_time_offset_ring_buffer.computeStats(med, rng, valid);
    m_cached_rb_median_ms.store(med, std::memory_order_relaxed);
    m_cached_rb_range_ms.store(rng, std::memory_order_relaxed);
    m_has_cached_offset_stats.store(valid, std::memory_order_relaxed);
}

void EspNowUnicastEngine::getTimeOffsetStats(float& out_ema_ms, float& out_rb_med_ms, float& out_rb_rng_ms, bool& out_has_stats) const {
    out_ema_ms = static_cast<float>(m_master_time_offset_us) / 1000.0f;
    out_rb_med_ms = m_cached_rb_median_ms.load(std::memory_order_relaxed);
    out_rb_rng_ms = m_cached_rb_range_ms.load(std::memory_order_relaxed);
    out_has_stats = m_has_cached_offset_stats.load(std::memory_order_relaxed);
}

uint64_t EspNowUnicastEngine::getMasterTimeMs() const {
    int64_t now = esp_timer_get_time();
    return static_cast<uint64_t>((now + m_master_time_offset_us) / 1000);
}

bool EspNowUnicastEngine::isMasterTimeValid() const {
    return (m_last_sync_time_us.load(std::memory_order_relaxed) > 0);
}

void EspNowUnicastEngine::audioTaskRoutine(void* pvParameters) {
    EspNowUnicastEngine* self = reinterpret_cast<EspNowUnicastEngine*>(pvParameters);
    if (self->m_node_role == NODE_ROLE_SOURCE) {
        self->runSourceLoop();
    } else {
        self->runSinkLoop();
    }
    vTaskDelete(nullptr);
}

void EspNowUnicastEngine::i2sStartTimerCallback(void* arg) {
    auto* self = reinterpret_cast<EspNowUnicastEngine*>(arg);
    if (self && self->m_i2s_dac) {
        self->m_i2s_dac->start();
    }
}

void EspNowUnicastEngine::runSourceLoop() {
    int16_t pcm_ch0[MAX_PCM_FRAME_SAMPLES];
    int16_t pcm_ch1[MAX_PCM_FRAME_SAMPLES];
    int16_t sub_mono_48k[MAX_PCM_FRAME_SAMPLES];
    int16_t sub_filtered_48k[MAX_PCM_FRAME_SAMPLES];
    int16_t sub_pcm_8k[80];

    uint8_t lc3_ch0[MAX_LC3_FRAME_OCTETS];
    uint8_t lc3_ch1[MAX_LC3_FRAME_OCTETS];
    uint8_t lc3_sub[MAX_LC3_FRAME_OCTETS];
    size_t len_sub = 0;

    uint8_t tx_packet[sizeof(vsaf_unicast_header_t) + MAX_LC3_FRAME_OCTETS];
    uint8_t seq = 0;

    int64_t next_frame_time_us = esp_timer_get_time();

    while (m_audio_task_running) {
        int64_t now_us = esp_timer_get_time();
        if (now_us < next_frame_time_us) {
            int64_t wait_us = next_frame_time_us - now_us;
            if (wait_us > 2000) {
                vTaskDelay(pdMS_TO_TICKS((wait_us - 1500) / 1000));
            }
            while (esp_timer_get_time() < next_frame_time_us) {
                esp_rom_delay_us(10);
            }
        } else if (now_us > next_frame_time_us + 20000) {
            next_frame_time_us = now_us;
        }
        next_frame_time_us += m_frame_duration_us;

        // 1. Check if node is IDLE or OFF: sleep and do not transmit
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (m_state == NetworkState::IDLE) {
            if (m_tone_test_mode || usb_audio_is_streaming()) {
                ESP_LOGI(TAG, "SOURCE: Active audio stream detected -> Transition to CAST");
                transitionTo(NetworkState::CAST);
                next_frame_time_us = esp_timer_get_time() + m_frame_duration_us;
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
        }
#endif
        if (m_state != NetworkState::CAST) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Snapshot registered peers
        SinkPeerConfig peers_snap[MAX_UNICAST_SINKS];
        int peer_count_snap = 0;
        bool has_sub_peer = false;

        taskENTER_CRITICAL(&m_peer_mux);
        peer_count_snap = m_peer_count;
        for (int i = 0; i < peer_count_snap; i++) {
            peers_snap[i] = m_peers[i];
            if (peers_snap[i].is_enabled && peers_snap[i].status == PeerStatus::ONLINE && peers_snap[i].channel_id == SUB_CHANNEL_ID) {
                has_sub_peer = true;
            }
        }
        taskEXIT_CRITICAL(&m_peer_mux);

        // Priority Sorting by Airtime Quality:
        // 1. ONLINE peers always precede OFFLINE/DISABLED peers.
        // 2. Among ONLINE peers, sort by lowest ema_tx_duration_us first.
        // Rock-solid peers (acknowledged in ~150 us) transmit first;
        // failing/sluggish peers (retrying for several ms) transmit last, preventing them from blocking healthy peers!
        for (int i = 0; i < peer_count_snap - 1; i++) {
            for (int j = i + 1; j < peer_count_snap; j++) {
                bool a_online = (peers_snap[i].is_enabled && peers_snap[i].status == PeerStatus::ONLINE);
                bool b_online = (peers_snap[j].is_enabled && peers_snap[j].status == PeerStatus::ONLINE);
                bool swap_needed = false;
                if (!a_online && b_online) {
                    swap_needed = true;
                } else if (a_online && b_online) {
                    if (peers_snap[j].ema_tx_duration_us < peers_snap[i].ema_tx_duration_us) {
                        swap_needed = true;
                    }
                }
                if (swap_needed) {
                    SinkPeerConfig tmp = peers_snap[i];
                    peers_snap[i] = peers_snap[j];
                    peers_snap[j] = tmp;
                }
            }
        }

        // ======================= MODE 2: CAST (Internal Test Tone or UAC) =======================
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (!m_tone_test_mode && !usb_audio_is_streaming()) {
            ESP_LOGI(TAG, "SOURCE: USB audio stream stopped/inactive -> Transition to IDLE");
            usb_audio_clear_buffer();
            transitionTo(NetworkState::IDLE);
            continue;
        }
#endif
        size_t samples_per_frame = (m_telemetry.sample_rate * m_frame_duration_us) / 1000000;
        if (samples_per_frame > MAX_PCM_FRAME_SAMPLES) samples_per_frame = MAX_PCM_FRAME_SAMPLES;

        if (m_tone_test_mode) {
            if (m_tone_gen) m_tone_gen->generateFrame(pcm_ch0, samples_per_frame);
            m_tone_gen_r.generateFrame(pcm_ch1, samples_per_frame);
        } else {
            size_t bytes_to_read = samples_per_frame * 2 * 2; // Stereo, 16-bit
            uint8_t usb_pcm_buf[MAX_PCM_FRAME_SAMPLES * 4];
            size_t bytes_read = 0;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
            bytes_read = usb_audio_read_pcm(usb_pcm_buf, bytes_to_read);
#endif
            if (bytes_read == bytes_to_read) {
                int16_t* interleaved = reinterpret_cast<int16_t*>(usb_pcm_buf);
                for (size_t i = 0; i < samples_per_frame; i++) {
                    pcm_ch0[i] = interleaved[i * 2];
                    pcm_ch1[i] = interleaved[i * 2 + 1];
                }
            } else {
                // Brief underrun during active stream: zero-pad rather than blasting tone
                memset(pcm_ch0, 0, samples_per_frame * sizeof(int16_t));
                memset(pcm_ch1, 0, samples_per_frame * sizeof(int16_t));
            }
        }


        int64_t enc_start = esp_timer_get_time();
        size_t len_ch0 = 0, len_ch1 = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
        if (s_lc3_worker_task_handle) {
            s_lc3_job.pcm = pcm_ch1;
            s_lc3_job.samples = samples_per_frame;
            s_lc3_job.out_lc3 = lc3_ch1;
            s_lc3_job.out_max_len = sizeof(lc3_ch1);
            s_lc3_job.out_actual_len = &len_ch1;
            s_lc3_job.caller_task = xTaskGetCurrentTaskHandle();
            xTaskNotifyGive(s_lc3_worker_task_handle);

            m_lc3_codec.encodeFrame(pcm_ch0, samples_per_frame, lc3_ch0, sizeof(lc3_ch0), &len_ch0);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        } else {
            m_lc3_codec.encodeFrame(pcm_ch0, samples_per_frame, lc3_ch0, sizeof(lc3_ch0), &len_ch0);
            m_lc3_codec_r.encodeFrame(pcm_ch1, samples_per_frame, lc3_ch1, sizeof(lc3_ch1), &len_ch1);
        }
#else
        m_lc3_codec.encodeFrame(pcm_ch0, samples_per_frame, lc3_ch0, sizeof(lc3_ch0), &len_ch0);
        m_lc3_codec_r.encodeFrame(pcm_ch1, samples_per_frame, lc3_ch1, sizeof(lc3_ch1), &len_ch1);
#endif

        if (has_sub_peer) {
            for (size_t i = 0; i < samples_per_frame; i++) {
                int32_t sum = static_cast<int32_t>(pcm_ch0[i]) + static_cast<int32_t>(pcm_ch1[i]);
                sub_mono_48k[i] = static_cast<int16_t>(sum / 2);
            }
            m_sub_lr4_filter.process(sub_mono_48k, sub_filtered_48k, samples_per_frame);
            Dsp::SubwooferResampler::downsample48kTo8k(sub_filtered_48k, sub_pcm_8k, samples_per_frame);
            m_lc3_codec_sub.encodeFrame(sub_pcm_8k, 80, lc3_sub, sizeof(lc3_sub), &len_sub);
        }

        int64_t enc_end = esp_timer_get_time();
        m_codec_duration_ring_buffer.push(static_cast<uint32_t>(enc_end - enc_start));
        m_audio_meter.pushFramePcm(pcm_ch0, samples_per_frame);

        for (int i = 0; i < peer_count_snap; i++) {
            if (!peers_snap[i].is_enabled || peers_snap[i].status != PeerStatus::ONLINE) {
                continue;
            }

            vsaf_unicast_header_t* hdr = reinterpret_cast<vsaf_unicast_header_t*>(tx_packet);
            hdr->seq = seq;
            hdr->audio.master_time_us = now_us;

            const uint8_t* payload = nullptr;
            size_t payload_len = 0;

            if (peers_snap[i].channel_id == SUB_CHANNEL_ID) {
                hdr->octets = static_cast<uint8_t>(CONFIG_ESPNOW_SUB_FRAME_LEN_OCTETS);
                hdr->audio.flags = encode_vsaf_flags(CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ, m_frame_duration_us);
                payload = lc3_sub;
                payload_len = len_sub;
            } else if (peers_snap[i].channel_id == 1) {
                hdr->octets = static_cast<uint8_t>(m_octets_per_frame);
                hdr->audio.flags = encode_vsaf_flags(m_telemetry.sample_rate, m_frame_duration_us);
                payload = lc3_ch1;
                payload_len = len_ch1;
            } else {
                hdr->octets = static_cast<uint8_t>(m_octets_per_frame);
                hdr->audio.flags = encode_vsaf_flags(m_telemetry.sample_rate, m_frame_duration_us);
                payload = lc3_ch0;
                payload_len = len_ch0;
            }

            if (!payload || payload_len == 0) continue;
            if (payload_len > MAX_LC3_FRAME_OCTETS) payload_len = MAX_LC3_FRAME_OCTETS;

            memcpy(tx_packet + sizeof(vsaf_unicast_header_t), payload, payload_len);
            size_t packet_size = sizeof(vsaf_unicast_header_t) + payload_len;

            // Measure actual hardware airtime + ACK turnaround duration with microsecond timer
            int64_t t_tx_start = esp_timer_get_time();

            // Strategy D: Strict Send-Done Synchronization
            // 1. Drain any residual token from previous frames before transmitting
            if (s_tx_done_sem) {
                xSemaphoreTake(s_tx_done_sem, 0);
            }

            esp_err_t send_err = esp_now_send(peers_snap[i].mac, tx_packet, packet_size);
            if (send_err == ESP_OK) {
                // 2. Wait strictly for Wi-Fi MAC to finish transmitting before proceeding to the next peer.
                // With 24 Mbps locked rate, healthy ACK returns in < 250 us.
                // On missing ACK, the hardware executes retries for up to ~12 ms.
                // We use a 20 ms safety watchdog timeout so we never push another packet while the radio is busy!
                BaseType_t tx_done = pdFALSE;
                if (s_tx_done_sem) {
                    tx_done = xSemaphoreTake(s_tx_done_sem, pdMS_TO_TICKS(20));
                }
                int64_t t_tx_end = esp_timer_get_time();
                uint32_t measured_airtime_us = (t_tx_end > t_tx_start) ? static_cast<uint32_t>(t_tx_end - t_tx_start) : 200;

                m_tx_packets_total.fetch_add(1, std::memory_order_relaxed);
                m_tx_packets_sec.fetch_add(1, std::memory_order_relaxed);

                taskENTER_CRITICAL(&m_peer_mux);
                for (int p = 0; p < m_peer_count; p++) {
                    if (memcmp(m_peers[p].mac, peers_snap[i].mac, 6) == 0) {
                        m_peers[p].packets_sent++;
                        m_peers[p].last_tx_duration_us = measured_airtime_us;

                        // Smooth airtime quality with EMA (75% history, 25% new sample)
                        if (m_peers[p].ema_tx_duration_us == 0) {
                            m_peers[p].ema_tx_duration_us = measured_airtime_us;
                        } else {
                            m_peers[p].ema_tx_duration_us = (m_peers[p].ema_tx_duration_us * 3 + measured_airtime_us) / 4;
                        }

                        // If safety watchdog timed out (>20 ms), immediately trip circuit breaker
                        if (tx_done != pdTRUE) {
                            m_peers[p].consecutive_ack_fails++;
                            m_peers[p].status = PeerStatus::OFFLINE;
                            m_peers[p].ema_tx_duration_us = 20000; // Heavily penalize priority sort
                        }
                        break;
                    }
                }
                taskEXIT_CRITICAL(&m_peer_mux);
            } else if (send_err == ESP_ERR_ESPNOW_NO_MEM) {
                // Hardware Wi-Fi queue was blocked by previous un-ACKed transmission
                m_tx_ack_fails_total.fetch_add(1, std::memory_order_relaxed);
                m_tx_ack_fails_sec.fetch_add(1, std::memory_order_relaxed);

                taskENTER_CRITICAL(&m_peer_mux);
                for (int p = 0; p < m_peer_count; p++) {
                    if (memcmp(m_peers[p].mac, peers_snap[i].mac, 6) == 0) {
                        m_peers[p].ack_failures++;
                        m_peers[p].consecutive_ack_fails++;
                        m_peers[p].status = PeerStatus::OFFLINE;
                        m_peers[p].ema_tx_duration_us = 20000;
                        break;
                    }
                }
                taskEXIT_CRITICAL(&m_peer_mux);
            }
        }
        seq++;
    }
}

void EspNowUnicastEngine::runSinkLoop() {
    ESP_LOGI(TAG, "SINK Unicast Audio Task started on Core %d", xPortGetCoreID());

    int16_t decoded_pcm_raw[MAX_PCM_FRAME_SAMPLES] = {0};
    int16_t stereo_pcm[MAX_PCM_FRAME_SAMPLES * 2] = {0};
    uint8_t current_lc3_buf[MAX_LC3_FRAME_OCTETS] = {0};
    size_t actual_samples = 0;
    size_t bytes_written = 0;
    uint32_t consecutive_empty_frames = 0;
    int64_t last_hello_send_time_us = 0;
    uint8_t hello_seq = 0;

    auto apply_volume_and_format_stereo = [&](int16_t* src, size_t count, bool is_first_frame = false) -> size_t {
        uint8_t target_u8 = m_target_volume_u8.load(std::memory_order_relaxed);
        float target_db = volume_u8_to_db(target_u8);
        float target_linear = db_to_linear(target_db);

        float start_linear;
        float end_linear;

        if (is_first_frame) {
            // Smooth 10 ms soft-fade-in on the very first frame to eliminate any DAC startup click/pop/blast
            start_linear = 0.0f;
            end_linear = target_linear;
            m_current_gain_db = target_db;
            m_current_linear_gain = target_linear;
        } else if (m_instant_volume_requested.exchange(false, std::memory_order_relaxed)) {
            m_current_gain_db = target_db;
            m_current_linear_gain = target_linear;
            start_linear = target_linear;
            end_linear = target_linear;
        } else {
            float frame_sec = static_cast<float>(m_frame_duration_us) / 1000000.0f;
            float max_step_db = CONFIG_VOLUME_SLEW_RATE_DB_PER_SEC * frame_sec; // 0.96 dB per 10ms
            if (m_current_gain_db < target_db) {
                m_current_gain_db += max_step_db;
                if (m_current_gain_db > target_db) m_current_gain_db = target_db;
            } else if (m_current_gain_db > target_db) {
                m_current_gain_db -= max_step_db;
                if (m_current_gain_db < target_db) m_current_gain_db = target_db;
            }
            start_linear = m_current_linear_gain;
            end_linear = db_to_linear(m_current_gain_db);
            m_current_linear_gain = end_linear;
        }

        // Per-sample linear interpolation to eliminate zipper noise and pop
        float gain_step = (count > 1) ? ((end_linear - start_linear) / static_cast<float>(count)) : 0.0f;
        float g = start_linear;
        for (size_t i = 0; i < count; ++i) {
            float s = static_cast<float>(src[i]) * g;
            if (s > 32767.0f) s = 32767.0f;
            else if (s < -32768.0f) s = -32768.0f;
            int16_t sample_val = static_cast<int16_t>(s);
            stereo_pcm[2 * i]     = sample_val;
            stereo_pcm[2 * i + 1] = sample_val;
            g += gain_step;
        }
        return count * 2 * sizeof(int16_t);
    };

    while (m_audio_task_running) {
        switch (m_state) {
            case NetworkState::OFF:
            case NetworkState::IDLE: {
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
            }

            case NetworkState::SCANNING: {
                size_t buffered = s_rx_fifo_count;
                if (buffered >= m_prefill_threshold_frames) {
                    ESP_LOGI(TAG, "SINK: Prefill threshold reached (FIFO = %zu pkts). Transitioning to PREFILL...", buffered);
                    transitionTo(NetworkState::PREFILL);
                } else {
                    // SINK-Initiated Handshake Announcement: Emit 2 Hz SINK_HELLO broadcast while scanning
                    int64_t now_scan_us = esp_timer_get_time();
                    if (now_scan_us - last_hello_send_time_us >= 500000) { // 500 ms = 2 Hz
                        last_hello_send_time_us = now_scan_us;
                        vsaf_unicast_header_t hello_pkt = {};
                        hello_pkt.seq = hello_seq++;
                        hello_pkt.octets = 0; // Control / Handshake packet
                        hello_pkt.ctrl.opcode = static_cast<uint8_t>(ControlOpcode::SINK_HELLO);
                        hello_pkt.ctrl.channel_id = m_target_channel; // 0=Left, 1=Right, 5=Sub
                        hello_pkt.ctrl.volume_u8 = m_target_volume_u8.load(std::memory_order_relaxed);
                        hello_pkt.ctrl.flags = 0x00;
                        const uint8_t bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                        esp_now_send(bcast_mac, reinterpret_cast<const uint8_t*>(&hello_pkt), sizeof(hello_pkt));
                    }
                    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
                }
                break;
            }

            case NetworkState::PREFILL: {
                size_t lc3_len = 0;
                uint8_t seq = 0;
                uint16_t frame_sr = 0;
                uint16_t frame_dur = 0;
                uint16_t presentation_delay_ms = 50;
                uint32_t master_time_us = 0;
                int64_t rx_time_us = 0;

                bool found_future_packet = false;

                while (!found_future_packet) {
                    if (s_rx_fifo_count == 0) {
                        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
                        if (s_rx_fifo_count == 0) {
                            break;
                        }
                    }

                    if (!pop_rx_lc3_frame(current_lc3_buf, &lc3_len, &seq, &frame_sr, &frame_dur,
                                          &presentation_delay_ms, &master_time_us, &rx_time_us) || lc3_len == 0) {
                        break;
                    }

                    int64_t now_local_us = esp_timer_get_time();
                    int64_t master_time_estimate_us = now_local_us + m_master_time_offset_us;
                    int64_t presentation_time_us = static_cast<int64_t>(master_time_us) + (static_cast<int64_t>(presentation_delay_ms) * 1000LL);

                    if (presentation_time_us <= master_time_estimate_us) {
                        // Presentation time is in the past -> discard stale packet and pop next
                        continue;
                    }

                    // Packet found with presentation time in the future!
                    found_future_packet = true;

                    if (frame_sr != m_telemetry.sample_rate || frame_dur != m_telemetry.frame_duration_us) {
                        m_telemetry.sample_rate = frame_sr;
                        m_telemetry.frame_duration_us = frame_dur;
                        uint16_t expected_octets = (frame_sr == CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ) ? CONFIG_ESPNOW_SUB_FRAME_LEN_OCTETS : m_octets_per_frame;
                        m_lc3_codec.reconfigureDecoder(frame_sr, expected_octets, frame_dur);
                        if (m_i2s_dac) {
                            m_i2s_dac->reconfigureSampleRate(frame_sr, frame_dur);
                        }
                    }

                    // Decode frame 0 and load in first DMA descriptor
                    m_lc3_codec.decodeFrame(current_lc3_buf, lc3_len, decoded_pcm_raw, MAX_PCM_FRAME_SAMPLES, &actual_samples,
                                            frame_sr, frame_dur);
                    m_audio_meter.pushFramePcm(decoded_pcm_raw, actual_samples);
                    if (m_i2s_dac && actual_samples > 0) {
                        size_t stereo_bytes = apply_volume_and_format_stereo(decoded_pcm_raw, actual_samples, true);
                        m_i2s_dac->preload(stereo_pcm, stereo_bytes, &bytes_written);
                    }
                    m_last_rx_seq = seq;
                    m_has_last_rx_seq = true;

                    // Pop the next LC3 packet (if available, wait up to 5 ms) and load second DMA descriptor
                    size_t lc3_len_2 = 0;
                    uint8_t seq_2 = 0;
                    uint16_t frame_sr_2 = 0, frame_dur_2 = 0, delay_ms_2 = 0;
                    uint32_t master_time_2 = 0;
                    int64_t rx_time_2 = 0;

                    if (!pop_rx_lc3_frame(current_lc3_buf, &lc3_len_2, &seq_2, &frame_sr_2, &frame_dur_2,
                                          &delay_ms_2, &master_time_2, &rx_time_2)) {
                        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
                        pop_rx_lc3_frame(current_lc3_buf, &lc3_len_2, &seq_2, &frame_sr_2, &frame_dur_2,
                                         &delay_ms_2, &master_time_2, &rx_time_2);
                    }

                    if (lc3_len_2 > 0) {
                        size_t actual_samples_2 = 0;
                        m_lc3_codec.decodeFrame(current_lc3_buf, lc3_len_2, decoded_pcm_raw, MAX_PCM_FRAME_SAMPLES, &actual_samples_2,
                                                frame_sr_2, frame_dur_2);
                        m_audio_meter.pushFramePcm(decoded_pcm_raw, actual_samples_2);
                        if (m_i2s_dac && actual_samples_2 > 0) {
                            size_t stereo_bytes_2 = apply_volume_and_format_stereo(decoded_pcm_raw, actual_samples_2, false);
                            m_i2s_dac->preload(stereo_pcm, stereo_bytes_2, &bytes_written);
                        }
                        m_last_rx_seq = seq_2;
                    }

                    // Set up high precision timer to start clocks exactly at presentation time
                    int64_t target_start_local_us = presentation_time_us - m_master_time_offset_us;
                    int64_t wait_us = target_start_local_us - esp_timer_get_time();
                    if (wait_us > 0) {
                        esp_timer_stop(m_i2s_start_timer); // Cancel any pending timer
                        esp_timer_start_once(m_i2s_start_timer, wait_us);
                    } else if (m_i2s_dac) {
                        m_i2s_dac->start();
                    }

                    consecutive_empty_frames = 0;
                    transitionTo(NetworkState::STREAM);
                    break;
                }
                break;
            }

            case NetworkState::STREAM: {
                if (m_i2s_dac && !m_i2s_dac->isRunning()) {
                    // Wait for the high-precision timer to start the I2S clocks.
                    // Do not process PLC or watchdog until the stream actually starts.
                    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
                    break;
                }

                uint32_t wait_timeout = (m_telemetry.frame_duration_us / 1000) * 2 + 5;
                if (m_i2s_dac) {
                    m_i2s_dac->waitForDmaSlot(wait_timeout);
                }

                // Buffer depth regulation: allow 50 ms presentation cushion (5 frames) with headroom
                // Discard excess stale backlog (> 120 ms) to prevent runaway latency drift
                taskENTER_CRITICAL(&s_fifo_mux);
                while (s_rx_fifo_count > 12) {
                    s_rx_fifo_tail = (s_rx_fifo_tail + 1) % LC3_RX_FIFO_CAPACITY;
                    s_rx_fifo_count--;
                    m_fifo_overflow.fetch_add(1, std::memory_order_relaxed);
                }
                taskEXIT_CRITICAL(&s_fifo_mux);

                size_t lc3_len = 0;
                uint8_t seq = 0;
                uint16_t frame_sr = 0;
                uint16_t frame_dur = 0;
                uint16_t presentation_delay_ms = 50;
                uint32_t master_time_us = 0;
                int64_t rx_time_us = 0;

                bool has_packet = pop_rx_lc3_frame(current_lc3_buf, &lc3_len, &seq, &frame_sr, &frame_dur,
                                                   &presentation_delay_ms, &master_time_us, &rx_time_us);

                // JIT absorption: wait up to 5 ms if FIFO is momentarily empty
                if (!has_packet) {
                    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5)) > 0) {
                        has_packet = pop_rx_lc3_frame(current_lc3_buf, &lc3_len, &seq, &frame_sr, &frame_dur,
                                                       &presentation_delay_ms, &master_time_us, &rx_time_us);
                    }
                }

                if (has_packet && lc3_len > 0) {
                    consecutive_empty_frames = 0;

                    if (frame_sr != m_telemetry.sample_rate || frame_dur != m_telemetry.frame_duration_us) {
                        m_telemetry.sample_rate = frame_sr;
                        m_telemetry.frame_duration_us = frame_dur;
                        uint16_t expected_octets = (frame_sr == CONFIG_ESPNOW_SUB_SAMPLE_RATE_HZ) ? CONFIG_ESPNOW_SUB_FRAME_LEN_OCTETS : m_octets_per_frame;
                        m_lc3_codec.reconfigureDecoder(frame_sr, expected_octets, frame_dur);
                        if (m_i2s_dac) {
                            m_i2s_dac->reconfigureSampleRate(frame_sr, frame_dur);
                        }
                        ESP_LOGW(TAG, "Sample rate changed to %u Hz. Transitioning to PREFILL.", frame_sr);
                        clear_rx_fifo();
                        m_state = NetworkState::PREFILL;
                        ESP_LOGI(TAG, "State Transition: STRM -> PREFILL");
                        break;
                    }

                    // Sequence gap / Packet loss check
                    if (m_has_last_rx_seq) {
                        uint8_t expected_seq = (m_last_rx_seq + 1) & 0xFF;
                        if (seq != expected_seq) {
                            uint8_t gap = (seq - expected_seq) & 0xFF;
                            if (gap > 0 && gap < 10) {
                                // Synthesize 1 PLC frame for missing expected_seq and push current packet back for next DMA slot
                                size_t plc_samples = 0;
                                m_lc3_codec.decodeFrame(nullptr, 0, decoded_pcm_raw, MAX_PCM_FRAME_SAMPLES, &plc_samples,
                                                        frame_sr, frame_dur);
                                m_last_rx_seq = expected_seq;

                                taskENTER_CRITICAL(&s_fifo_mux);
                                s_rx_fifo_tail = (s_rx_fifo_tail + LC3_RX_FIFO_CAPACITY - 1) % LC3_RX_FIFO_CAPACITY;
                                s_rx_fifo[s_rx_fifo_tail].len = static_cast<uint8_t>(lc3_len);
                                s_rx_fifo[s_rx_fifo_tail].seq = seq;
                                s_rx_fifo[s_rx_fifo_tail].sample_rate_hz = frame_sr;
                                s_rx_fifo[s_rx_fifo_tail].frame_duration_us = frame_dur;
                                s_rx_fifo[s_rx_fifo_tail].presentation_delay_ms = presentation_delay_ms;
                                s_rx_fifo[s_rx_fifo_tail].master_time_us = master_time_us;
                                s_rx_fifo[s_rx_fifo_tail].rx_local_time_us = rx_time_us;
                                memcpy(s_rx_fifo[s_rx_fifo_tail].data, current_lc3_buf, lc3_len);
                                s_rx_fifo_count++;
                                taskEXIT_CRITICAL(&s_fifo_mux);

                                if (m_i2s_dac && plc_samples > 0) {
                                    size_t plc_bytes = apply_volume_and_format_stereo(decoded_pcm_raw, plc_samples);
                                    m_i2s_dac->write(stereo_pcm, plc_bytes, &bytes_written, 15);
                                }
                                break;
                            }
                        }
                    }
                    m_last_rx_seq = seq;
                    m_has_last_rx_seq = true;

                    // Decode frame directly at native stream sample rate (80 samples for 8 kHz, 480 samples for 48 kHz)
                    int64_t dec_start = esp_timer_get_time();
                    m_lc3_codec.decodeFrame(current_lc3_buf, lc3_len, decoded_pcm_raw, MAX_PCM_FRAME_SAMPLES, &actual_samples,
                                            frame_sr, frame_dur);
                    int64_t dec_end = esp_timer_get_time();
                    m_codec_duration_ring_buffer.push(static_cast<uint32_t>(dec_end - dec_start));

                    m_audio_meter.pushFramePcm(decoded_pcm_raw, actual_samples);

                    if (m_i2s_dac && actual_samples > 0) {
                        size_t stereo_bytes = apply_volume_and_format_stereo(decoded_pcm_raw, actual_samples);
                        m_i2s_dac->write(stereo_pcm, stereo_bytes, &bytes_written, 15);
                    }
                } else {
                    // Packet Loss Concealment (PLC)
                    consecutive_empty_frames++;
                    m_fifo_underrun.fetch_add(1, std::memory_order_relaxed);
                    if (consecutive_empty_frames >= m_watchdog_timeout_frames) {
                        ESP_LOGW(TAG, "Watchdog timeout (%lu missing frames) -> SCANNING", (unsigned long)consecutive_empty_frames);
                        transitionTo(NetworkState::SCANNING);
                        clear_rx_fifo();
                        if (m_i2s_dac) m_i2s_dac->stop();
                    } else {
                        size_t plc_samples = 0;
                        m_lc3_codec.decodeFrame(nullptr, 0, decoded_pcm_raw, MAX_PCM_FRAME_SAMPLES, &plc_samples,
                                                m_telemetry.sample_rate, m_telemetry.frame_duration_us);
                        if (m_i2s_dac && plc_samples > 0) {
                            size_t plc_bytes = apply_volume_and_format_stereo(decoded_pcm_raw, plc_samples);
                            m_i2s_dac->write(stereo_pcm, plc_bytes, &bytes_written, 15);
                        }
                    }
                }
                break;
            }

            default:
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
        }
    }
}

} // namespace AudioNet
