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
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "esp_system.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

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
    // Runs in the esp_timer task (core 0, prio 22): stamp the tick, wake core 0 (DSP) and the RF task
    auto* engine = static_cast<EspNowBroadcastEngine*>(arg);
    if (!engine) return;
    engine->m_src_tick_us.store(esp_timer_get_time(), std::memory_order_release);
    if (engine->m_audio_dsp_task_handle) {
        xTaskNotifyGive(engine->m_audio_dsp_task_handle);
    }
    if (engine->m_src_rf_task) {
        xTaskNotify(engine->m_src_rf_task, SRC_BIT_TICK, eSetBits);
    }
}

static void IRAM_ATTR onEspNowSendCb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
    EspNowBroadcastEngine::s_last_tx_status.store(status, std::memory_order_relaxed);
    if (s_engine_instance) {
        s_engine_instance->onSrcTxDone(status); // SOURCE RF task hand-off (no-op on SINKs)
        s_engine_instance->onSinkTxDone();      // SINK reply TX-done for the time-sync two-step (no-op on SOURCE)
    }
    if (EspNowBroadcastEngine::s_tx_done_sem) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(EspNowBroadcastEngine::s_tx_done_sem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void IRAM_ATTR onEspNowRecvCb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len) {
    // Receive timestamp first (time sync uses it): esp_timer at callback entry, plus the hardware timestamp for
    // the 'ptp' probe that compares the two clocks
    const int64_t t_rx_us = esp_timer_get_time();
    if (!s_engine_instance || !data || data_len < 2) return;
    int8_t rssi = (esp_now_info->rx_ctrl) ? esp_now_info->rx_ctrl->rssi : -127;
    uint8_t rate = (esp_now_info->rx_ctrl) ? esp_now_info->rx_ctrl->rate : 0;
    uint32_t hw_ts = (esp_now_info->rx_ctrl) ? esp_now_info->rx_ctrl->timestamp : 0;
    s_engine_instance->onPacketReceived(esp_now_info->src_addr, data, data_len, rssi, rate, t_rx_us, hw_ts);
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
    // Static RX buffers are reserved at init (~1.6 KB each). A SINK receives the whole 600 pkt/s broadcast;
    // the SOURCE only receives one 16 B telemetry reply per 10 ms. The SOURCE has no PSRAM and its internal
    // heap must also hold 11 LC3 encoders and the pipeline task stacks, so it keeps far fewer.
    cfg.static_rx_buf_num = (role == NODE_ROLE_SOURCE) ? 6 : 32;
    cfg.dynamic_rx_buf_num = (role == NODE_ROLE_SOURCE) ? 32 : 64;
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

    // Must be set after esp_wifi_start() (see config.h for the quantization of this setting)
    ESP_ERROR_CHECK(setWifiTxPower(CONFIG_VSAF_TX_POWER_QDBM));

    // 2. SOURCE boot channel: the fixed preferred channel (default), or optionally a boot RF survey that may
    //    move away from it if it looks busy (CONFIG_VSAF_BOOT_CHANNEL_SCAN, see config.h for why it is off)
    if (m_node_role == NODE_ROLE_SOURCE) {
#if CONFIG_VSAF_BOOT_CHANNEL_SCAN
        m_wifi_channel = scanAndSelectBestChannel(CONFIG_VSAF_BOOT_SCAN_DWELL_MS, true, true,
                                                  CONFIG_VSAF_PREFERRED_WIFI_CHANNEL);
#else
        setWifiChannel(CONFIG_VSAF_PREFERRED_WIFI_CHANNEL);
#endif
    } else {
        // SINK: start the channel search where the SOURCE boots (then hop upward, see runSinkLoop)
        setWifiChannel(CONFIG_VSAF_PREFERRED_WIFI_CHANNEL);
    }

    // 3. Initialize ESP-NOW + 4. register the single broadcast peer (FF:FF:FF:FF:FF:FF)
    esp_err_t ret = initEspNowLayer();
    if (ret != ESP_OK) {
        return ret;
    }

    if (m_node_role == NODE_ROLE_SOURCE) {
        setWifiPhyProfile(WifiPhyProfile::PRIMARY_HT20_MCS3);
    } else {
        setWifiPhyProfile(WifiPhyProfile::TERTIARY_HT20_MCS0); // Robust MCS0 for SINK Soft-ACK telemetry replies
    }

    // 5. Initialize Codec (SOURCE: 5 encoders - Left HQ 48k, Right HQ 48k, Left Red 48k, Right Red 48k, Sub 8k)
    if (m_node_role == NODE_ROLE_SOURCE) {
        m_lc3_codec.initEncoder(m_telemetry.sample_rate, Codec::Lc3CodecEngine::NUM_ENCODERS, m_frame_duration_us,
                                m_octets_per_frame);
    } else {
        m_lc3_codec.initDecoder(m_telemetry.sample_rate, 1, m_frame_duration_us, m_octets_per_frame);
    }

    transitionTo(NetworkState::IDLE);
    ESP_LOGI(TAG, "EspNowBroadcastEngine initialized successfully (Role: %s, Node ID: %d, Ch: %d, Rate: %s)",
             (m_node_role == NODE_ROLE_SOURCE) ? "SOURCE" : "SINK", m_node_id, m_wifi_channel, getWifiPhyRateString());
    return ESP_OK;
}

/**
 * @brief Initialize ESP-NOW, register send/recv callbacks and the broadcast peer.
 *
 * Used at boot and by recoverEspNowTx(). The peer follows the current STA channel (channel = 0).
 *
 * @return ESP_OK, or the first failing esp_now_* error.
 */
esp_err_t EspNowBroadcastEngine::initEspNowLayer() {
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_ERROR_CHECK(esp_now_register_send_cb(onEspNowSendCb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(onEspNowRecvCb));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, s_broadcast_mac, 6);
    peer_info.channel = 0; // 0 = follow current STA interface channel
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to register broadcast peer: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/**
 * @brief Rebuild the ESP-NOW TX path after a detected wedge (SOURCE TX task context).
 *
 * Wedge = CAST but no successful TX completion for a whole check window, or a run of esp_now_send()
 * errors. Seen 2026-10-24 after runtime channel/PHY changes: the SOURCE kept "sending" (TX stage ~0.1 ms,
 * counters rising) while nothing went on air until a reboot. Deinit + init ESP-NOW, re-add the peer,
 * re-apply PHY rate and channel. Costs one overrun frame; SINKs recover via redundancy/PLC.
 *
 * @param why Reason string for the log (input).
 */
// Survives esp_restart() (not power-on): counts reboots forced by an unrecoverable TX wedge
static RTC_NOINIT_ATTR uint32_t s_wedge_reboot_magic;
static RTC_NOINIT_ATTR uint32_t s_wedge_reboots;
static constexpr uint32_t WEDGE_MAGIC = 0x7EDCE001;

uint32_t EspNowBroadcastEngine::getTxWedgeReboots() {
    return s_wedge_reboot_magic == WEDGE_MAGIC ? s_wedge_reboots : 0;
}

/**
 * @brief Restart the Wi-Fi driver (stop + start) and re-apply all radio settings and ESP-NOW.
 *
 * Frees the driver's TX buffers, which an ESP-NOW deinit / init does not: seen 2026-10-25 after 30 min of
 * streaming, the driver stopped delivering TX-done callbacks, every esp_now_send() then returned
 * ESP_ERR_ESPNOW_NO_MEM and 59,000 ESP-NOW re-inits in 50 min did not help.
 *
 * @return ESP_OK, or the first failing esp_wifi / esp_now error.
 */
esp_err_t EspNowBroadcastEngine::restartWifiRadio() {
    m_wifi_restarts++;
    if (m_sim_wedge.load(std::memory_order_relaxed) == 1) {
        m_sim_wedge.store(0, std::memory_order_relaxed);  // test hook: this level heals on a Wi-Fi restart
    }
    esp_now_deinit();
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi restart failed: %s", esp_err_to_name(err));
        return err;
    }
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE);
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX);
#else
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
#endif
    setWifiTxPower(m_tx_power_req_qdbm.load(std::memory_order_relaxed));
    err = initEspNowLayer();
    if (err == ESP_OK) {
        setWifiPhyRate(m_tx_phy_mode, m_tx_phy_rate);
    }
    return err;
}

void EspNowBroadcastEngine::recoverEspNowTx(const char* why) {
    m_tx_recoveries++;
    // Escalate while recoveries do not bring back a single successful TX:
    //   strikes 1-2: ESP-NOW re-init, 3-5: Wi-Fi driver restart, 6: reboot (SINKs hold the channel for
    //   CONFIG_VSAF_SCAN_HOLD_MS = 3 s and the SOURCE is back on air well within that)
    m_tx_strikes = m_tx_ok_since_recovery ? 1 : m_tx_strikes + 1;
    m_tx_ok_since_recovery = false;
    if (m_tx_strikes >= 6) {
        ESP_LOGE(TAG, "ESP-NOW TX path wedged (%s): Wi-Fi restarts did not help, rebooting the SOURCE", why);
        if (s_wedge_reboot_magic != WEDGE_MAGIC) {
            s_wedge_reboot_magic = WEDGE_MAGIC;
            s_wedge_reboots = 0;
        }
        s_wedge_reboots++;
        vTaskDelay(pdMS_TO_TICKS(20));  // let the log line out
        esp_restart();
    } else if (m_tx_strikes >= 3) {
        ESP_LOGE(TAG, "ESP-NOW TX path wedged (%s): restarting the Wi-Fi driver (recovery #%lu, strike %lu)",
                 why, (unsigned long)m_tx_recoveries.load(std::memory_order_relaxed), (unsigned long)m_tx_strikes);
        restartWifiRadio();
    } else {
        ESP_LOGE(TAG, "ESP-NOW TX path wedged (%s): re-initializing ESP-NOW (recovery #%lu, strike %lu)",
                 why, (unsigned long)m_tx_recoveries.load(std::memory_order_relaxed), (unsigned long)m_tx_strikes);
        esp_now_deinit();
        if (initEspNowLayer() == ESP_OK) {
            setWifiPhyRate(m_tx_phy_mode, m_tx_phy_rate);
            esp_wifi_set_channel(m_wifi_channel, WIFI_SECOND_CHAN_NONE);
        }
    }
    if (s_tx_done_sem) {
        xSemaphoreTake(s_tx_done_sem, 0);
        xSemaphoreGive(s_tx_done_sem);
    }
    m_consecutive_tx_timeouts = 0;
    m_consecutive_send_errors = 0;
    m_tx_ok_in_window = 0;
    m_tx_window_frames = 0;
    m_src_tx_pkts_in_window = 0;
    m_src_tx_fast_pkts = 0;
    m_src_tx_attempts_in_window = 0;
    m_src_tx_done_flag.store(false, std::memory_order_relaxed);
}

void EspNowBroadcastEngine::onSrcTxDone(esp_now_send_status_t status) {
    // Wi-Fi task context: timestamp the completion and wake the SOURCE RF task (if this is a SOURCE)
    if (!m_src_rf_task) {
        return;
    }
    m_src_tx_done_us.store(esp_timer_get_time(), std::memory_order_relaxed);
    m_src_tx_status.store(static_cast<int>(status), std::memory_order_relaxed);
    m_src_tx_done_flag.store(true, std::memory_order_release);
    xTaskNotify(m_src_rf_task, SRC_BIT_TXDONE, eSetBits);
}

/**
 * @brief Request a Wi-Fi channel change. On a streaming SOURCE it is applied by the TX task at the next
 * frame boundary (never while a broadcast burst is in flight); otherwise immediately.
 *
 * @param channel 1..13
 * @return ESP_OK, or ESP_ERR_INVALID_ARG.
 */
esp_err_t EspNowBroadcastEngine::requestWifiChannel(uint8_t channel) {
    if (channel < 1 || channel > 13) return ESP_ERR_INVALID_ARG;
    if (m_node_role == NODE_ROLE_SOURCE && m_running.load(std::memory_order_acquire)) {
        m_pending_channel.store(channel, std::memory_order_release);
        return ESP_OK;
    }
    esp_err_t err = setWifiChannel(channel);
    if (err == ESP_OK) {
        setWifiPhyRate(m_tx_phy_mode, m_tx_phy_rate); // keep the broadcast rate across channel changes
    }
    return err;
}

/**
 * @brief Request a broadcast PHY rate change (same deferral rules as requestWifiChannel()).
 */
esp_err_t EspNowBroadcastEngine::requestWifiPhyRate(wifi_phy_mode_t phymode, wifi_phy_rate_t rate) {
    if (m_node_role == NODE_ROLE_SOURCE && m_running.load(std::memory_order_acquire)) {
        m_pending_phy_mode = phymode;
        m_pending_phy_rate = rate;
        m_pending_phy.store(true, std::memory_order_release);
        return ESP_OK;
    }
    return setWifiPhyRate(phymode, rate);
}

/**
 * @brief Apply deferred channel / PHY requests (SOURCE RF task only, called on a frame tick while no
 * packet is in flight). A channel change always re-applies the current PHY rate.
 */
void EspNowBroadcastEngine::applyPendingRadioChanges() {
    if (m_debug_break_tx.exchange(false, std::memory_order_acq_rel)) {
        // Test hook (console 'txfault'): break the TX path to exercise wedge detection + recovery
        ESP_LOGW(TAG, "txfault: esp_now_deinit() to simulate a wedged TX path");
        esp_now_deinit();
    }
    uint8_t ch = m_pending_channel.exchange(0, std::memory_order_acq_rel);
    bool phy = m_pending_phy.exchange(false, std::memory_order_acq_rel);
    if (ch == 0 && !phy) {
        return;
    }
    // Caller (SOURCE RF task) guarantees the radio is idle: no packet in flight
    if (ch != 0) {
        setWifiChannel(ch);
    }
    if (phy) {
        setWifiPhyRate(m_pending_phy_mode, m_pending_phy_rate);
    } else {
        setWifiPhyRate(m_tx_phy_mode, m_tx_phy_rate);
    }
    ESP_LOGI(TAG, "Radio change applied at frame boundary: ch %u, %s", m_wifi_channel, getWifiPhyRateString());
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

        // 1. SOURCE pipeline tasks (see vsaf_source_pipeline.cpp):
        //    core 0: USB + DSP + encoding (prio 5, below TinyUSB 6 / esp_timer 22 / Wi-Fi 23)
        //    core 1: encoding (prio 5) and the event-driven RF owner (prio 7, preempts the encoder)
        m_boot_heap_before_tasks = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        m_boot_task_ok[0] = xTaskCreatePinnedToCore(srcRfTaskTrampoline, "src_rf", 4096, this, 7, &m_src_rf_task, 1) == pdPASS;
        m_boot_task_ok[1] = xTaskCreatePinnedToCore(srcEncTaskTrampoline, "src_enc1", 5120, this, 5, &m_src_enc_task, 1) == pdPASS;
        m_boot_task_ok[2] = xTaskCreatePinnedToCore(srcDspTaskTrampoline, "src_dsp0", 6144, this, 5, &m_audio_dsp_task_handle, 0) == pdPASS;
        m_boot_heap_after_tasks = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        if (!m_boot_task_ok[0] || !m_boot_task_ok[1] || !m_boot_task_ok[2]) {
            ESP_LOGE(TAG, "SOURCE pipeline task creation FAILED (rf=%d enc1=%d dsp0=%d), free internal heap %lu B",
                     m_boot_task_ok[0], m_boot_task_ok[1], m_boot_task_ok[2], (unsigned long)m_boot_heap_after_tasks);
        }

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
    if (m_src_enc_task) {
        vTaskDelete(m_src_enc_task);
        m_src_enc_task = nullptr;
    }
    if (m_src_rf_task) {
        vTaskDelete(m_src_rf_task);
        m_src_rf_task = nullptr;
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
    if (new_state == NetworkState::SCANNING) {
        // Stream lost: the time sync is held over for 30 s while streaming, but after an outage the SOURCE may
        // have rebooted (new time base), so wait for a fresh control frame
        portENTER_CRITICAL(&m_sync_mux);
        m_sync.valid = false;
        portEXIT_CRITICAL(&m_sync_mux);
    }

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

/**
 * @brief Set the Wi-Fi TX power and cache ESP-IDF's readback (read once here, never polled).
 *
 * The radio quantizes the request to fixed levels, but no public ESP-IDF API reports the applied level:
 * esp_wifi_get_max_tx_power() echoes the request and wifi_country_t.max_tx_power stays at the regulatory
 * maximum (20 dBm). Verified on S3 and C6 (2026-10-25); the real output is only measurable through RSSI.
 *
 * @param power_0_25dbm Requested power in 0.25 dBm units, clamped to [8, 84] (2..20 dBm).
 * @return Result of esp_wifi_set_max_tx_power().
 */
esp_err_t EspNowBroadcastEngine::setWifiTxPower(int8_t power_0_25dbm) {
    if (power_0_25dbm < 8) power_0_25dbm = 8;
    if (power_0_25dbm > 84) power_0_25dbm = 84;
    esp_err_t err = esp_wifi_set_max_tx_power(power_0_25dbm);

    int8_t readback = 0;
    if (esp_wifi_get_max_tx_power(&readback) == ESP_OK) {
        m_tx_power_req_qdbm.store(readback, std::memory_order_relaxed);
    }
    ESP_LOGI(TAG, "Wi-Fi TX power %.2f dBm requested (readback %d x 0.25 dBm; the radio quantizes it)%s",
             power_0_25dbm / 4.0f, m_tx_power_req_qdbm.load(), err == ESP_OK ? "" : " [set failed]");
    return err;
}

int8_t EspNowBroadcastEngine::getWifiTxPower() const {
    return m_tx_power_req_qdbm.load(std::memory_order_relaxed);
}

/**
 * Passive 802.11 survey of channels 1..13 and channel selection.
 *
 * Per channel: `score` from frames decoded on that channel (count, bytes, strength, 40 MHz use).
 * Then `adj_score` = score + overlap-weighted scores of channels within +/-4, because a 20 MHz
 * channel also suffers from neighbours whose frames are not decodable on it (e.g. APs on ch 6
 * still raise the noise floor on ch 9-10 while ch 10 itself decodes nothing).
 *
 * @param dwell_ms_per_ch  Listen time per channel (ms); >= 205 ms covers two beacon intervals.
 * @param auto_apply       Switch the radio to the selected channel (else restore m_wifi_channel).
 * @param print_results    Print the survey table to the console.
 * @param preferred_ch     0: select the lowest adj_score. 1..13: keep this channel unless its
 *                         adj_score > CONFIG_VSAF_CHANNEL_BUSY_SCORE, then fall back to the lowest.
 * @return Selected channel (1..13).
 */
uint8_t EspNowBroadcastEngine::scanAndSelectBestChannel(uint32_t dwell_ms_per_ch, bool auto_apply, bool print_results,
                                                        uint8_t preferred_ch) {
    WifiChannelScanResult results[14] = {}; // Index 1..13

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
        results[ch].score = score;
    }

    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);

    // Overlap weighting: channels are 5 MHz apart, 20 MHz wide -> neighbours up to +/-4 overlap.
    static constexpr uint32_t kOverlapPct[5] = {100, 60, 35, 15, 5};
    uint8_t best_ch = 1;
    uint32_t lowest_score = 0xFFFFFFFF;
    for (int ch = 1; ch <= 13; ++ch) {
        uint32_t adj = 0;
        for (int d = -4; d <= 4; ++d) {
            int n = ch + d;
            if (n >= 1 && n <= 13) {
                adj += (results[n].score * kOverlapPct[d < 0 ? -d : d]) / 100;
            }
        }
        results[ch].adj_score = adj;
        if (adj < lowest_score) {
            lowest_score = adj;
            best_ch = static_cast<uint8_t>(ch);
        }
    }

    const char* reason = "cleanest channel";
    if (preferred_ch >= 1 && preferred_ch <= 13) {
        if (results[preferred_ch].adj_score <= CONFIG_VSAF_CHANNEL_BUSY_SCORE) {
            best_ch = preferred_ch;
            lowest_score = results[preferred_ch].adj_score;
            reason = "preferred channel, not busy";
        } else {
            reason = "preferred channel busy -> cleanest channel";
        }
    }

    if (print_results) {
        print_console("\n================================= 802.11 RF SPECTRUM SURVEY =================================\n");
        print_console(" Ch | Packets |   Bytes | Max RSSI | Avg RSSI | 40MHz | Score |   Adj | Rating (by Adj)\n");
        print_console("----+---------+---------+----------+----------+-------+-------+-------+-----------------------\n");
        for (uint8_t ch = 1; ch <= 13; ++ch) {
            const char* rating = "CLEAN";
            const uint32_t adj = results[ch].adj_score;
            if (adj == 0) rating = "PERFECT (Silent)";
            else if (adj < 50) rating = "EXCELLENT";
            else if (adj < 150) rating = "GOOD";
            else if (adj < 350) rating = "MODERATE NOISE";
            else rating = "CONGESTED";

            char sel_mark[20] = "";
            if (ch == best_ch) {
                snprintf(sel_mark, sizeof(sel_mark), " <-- SELECTED");
            }
            print_console(" %2u | %7lu | %6luB |  %4d dBm |  %4d dBm |  %3s  | %5lu | %5lu | %-16s%s\n",
                          ch,
                          (unsigned long)results[ch].packet_count,
                          (unsigned long)results[ch].byte_count,
                          results[ch].max_rssi,
                          results[ch].avg_rssi,
                          results[ch].has_40mhz ? "YES" : "No",
                          (unsigned long)results[ch].score,
                          (unsigned long)adj,
                          rating,
                          sel_mark);
        }
        print_console("==============================================================================================\n");
        print_console("[RF SNIFFER] Selected Wi-Fi channel: %u (Adj score %lu, %s; busy threshold %d)\n\n",
                      best_ch, (unsigned long)lowest_score, reason, CONFIG_VSAF_CHANNEL_BUSY_SCORE);
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

void EspNowBroadcastEngine::onPacketReceived(const uint8_t* mac_addr, const uint8_t* data, int data_len, int8_t rssi, uint8_t rate,
                                             int64_t t_rx_us, uint32_t rx_hw_ts) {
    if (!data || data_len < 2) return;
    m_raw_espnow_rx_count++;
    int64_t t_now_us = (t_rx_us != 0) ? t_rx_us : esp_timer_get_time();

    // Probe: callback time minus hardware RX timestamp (same clock -> small, steady value)
    if (rx_hw_ts != 0) {
        const int32_t d = static_cast<int32_t>(static_cast<uint32_t>(t_now_us) - rx_hw_ts);
        if (d < m_rxts_min.load(std::memory_order_relaxed)) m_rxts_min.store(d, std::memory_order_relaxed);
        if (d > m_rxts_max.load(std::memory_order_relaxed)) m_rxts_max.store(d, std::memory_order_relaxed);
        m_rxts_n.fetch_add(1, std::memory_order_relaxed);
    }

    uint16_t type_id = *reinterpret_cast<const uint16_t*>(data);

    // =======================================================================
    // SINK NODE LOGIC: Receive Audio Broadcast
    // =======================================================================
    if (m_node_role == NODE_ROLE_SINK) {
        if (m_state.load(std::memory_order_acquire) == NetworkState::IDLE) {
            return;
        }
        if (type_id == VSAF_TYPE_CONTROL) {
            sinkOnControl(data, data_len);
            return;
        }

        if ((type_id == VSAF_TYPE_AUDIO_SATELLITE && data_len >= static_cast<int>(sizeof(vsaf_audio_packet_t))) ||
            (type_id == VSAF_TYPE_AUDIO_SUBWOOFER && data_len >= static_cast<int>(sizeof(vsaf_sub_packet_t)))) {
            m_raw_audio_pkt_count++;
            m_channel_locked.store(true, std::memory_order_release);
            m_last_rx_audio_pkt_us.store(t_now_us, std::memory_order_release);

            // Sync self-test: every SINK receives the subwoofer packet; once per second (frame time % 1 s) record
            // its arrival converted to SOURCE time, to compare the SINKs' time bases from the host
            if (type_id == VSAF_TYPE_AUDIO_SUBWOOFER && m_syncprobe.load(std::memory_order_relaxed)) {
                uint32_t frame_t;
                memcpy(&frame_t, data + 4, sizeof(frame_t));
                if ((frame_t / 10000U) % 100U == 0) {
                    const uint32_t rx_master = static_cast<uint32_t>(t_now_us) -
                                               static_cast<uint32_t>(m_sync_offset_used.load(std::memory_order_relaxed));
                    portENTER_CRITICAL(&m_probe_mux);
                    m_probe.rx_new = true;
                    m_probe.rx_frame_t = frame_t;
                    m_probe.rx_master = rx_master;
                    portEXIT_CRITICAL(&m_probe_mux);
                }
            }

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

            // Verify CRC-16 (over the first 18 bytes of the telemetry structure)
            uint16_t computed_crc = calc_crc16(data, sizeof(vsaf_sink_telemetry_t) - 2);
            if (computed_crc != ack->crc16) {
                return;
            }

            uint8_t sink_id = ack->sink_id;
            if (sink_id < MAX_SINK_NODES) {
                const uint32_t t4 = static_cast<uint32_t>(t_now_us);
                // Round trip (single step, display): t4 - t1 - dwell, t1 = TX-done of the REQ_ACK packet
                const uint8_t s = ack->ack_seq;
                const uint32_t f_tag = m_ptp_t1_frame[s].load(std::memory_order_acquire);
                const uint32_t t1 = m_ptp_t1[s].load(std::memory_order_relaxed);
                const int32_t total_rtt = static_cast<int32_t>(t4 - t1);
                const int32_t dwell_us = ack->t_dwell_us;
                const int32_t net_rtt = total_rtt - dwell_us;
                if (f_tag != 0 && static_cast<uint8_t>(f_tag) == s && total_rtt >= 0 && total_rtt < 30000) {
                    m_peers[sink_id].last_total_rtt_us = static_cast<uint32_t>(total_rtt);
                    m_peers[sink_id].last_dwell_us = static_cast<uint32_t>(dwell_us);
                    m_peers[sink_id].last_net_rtt_us = (net_rtt >= 0) ? static_cast<uint32_t>(net_rtt) : 0;
                    m_peers[sink_id].last_rtt_us = m_peers[sink_id].last_net_rtt_us;
                    m_peers[sink_id].avg_rtt_us = (m_peers[sink_id].avg_rtt_us == 0)
                        ? m_peers[sink_id].last_net_rtt_us
                        : ((m_peers[sink_id].avg_rtt_us * 7 + m_peers[sink_id].last_net_rtt_us) / 8);
                }

                // Time sync: two-step PTP-like exchange (vsaf_time_sync.cpp)
                ptpOnTelemetry(ack, t4);

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

    // Sequence tracking & multi-tier redundancy recovery. Every FIFO item carries the SOURCE frame time of its
    // frame: t_tx1 is t0 of this packet's t0 frame, the redundant frame t-k belongs to t0 - k * 10 ms.
    const uint8_t* t0_data = is_sub ? sub_pkt->data_t0 : sat_pkt->data_t0;
    const uint8_t  t0_len = static_cast<uint8_t>(is_sub ? LC3_FRAME_OCTETS_RED : LC3_FRAME_OCTETS_HQ);
    const uint8_t* hist[3] = {is_sub ? sub_pkt->data_t_prev1 : sat_pkt->data_t_prev1,
                              is_sub ? sub_pkt->data_t_prev2 : sat_pkt->data_t_prev2,
                              is_sub ? sub_pkt->data_t_prev3 : nullptr};
    const int max_hist = is_sub ? 3 : 2;
    const uint32_t frame_us = get_flags_frame_dur_us(flags);
    int8_t seq_diff = static_cast<int8_t>(static_cast<uint8_t>(seq - m_last_rx_seq));

    int recover = -1;  // history frames to push before t0; -1 = stale / duplicate packet
    if (!m_first_packet_received || seq_diff < -64 || seq_diff > 64) {
        // Initial synchronization or major sequence discontinuity
        m_first_packet_received = true;
        recover = 0;
    } else if (seq_diff >= 1) {
        // seq_diff - 1 packets were lost: recover as many as the redundancy carries (t-1, t-2, sub also t-3)
        recover = std::min<int>(seq_diff - 1, max_hist);
    }
    if (recover >= 0) {
        m_last_rx_seq = seq;
        portENTER_CRITICAL(&m_sink_fifo_lock);
        for (int k = recover; k >= 1; --k) {
            sinkFifoPush(static_cast<uint8_t>(seq - k), LC3_FRAME_OCTETS_RED, flags, true, hist[k - 1],
                         t_tx1_us - static_cast<uint32_t>(k) * frame_us);
        }
        if (!sinkFifoPush(seq, t0_len, flags, false, t0_data, t_tx1_us)) {
            m_fifo_overflows++;
        }
        portEXIT_CRITICAL(&m_sink_fifo_lock);
        m_rx_packets_total += static_cast<uint32_t>(recover + 1);
        m_rx_packets_sec += static_cast<uint32_t>(recover + 1);
    }
    // (If seq_diff <= 0, duplicate/stale frame: ignore)

    // Wake up SINK audio task if waiting
    if (m_sink_task_handle) {
        vTaskNotifyGiveFromISR(m_sink_task_handle, nullptr);
    }

    // SINK Reply on Command Only:
    // Only transmit telemetry reply if SOURCE explicitly requested it in packet_flags (bit 7)
    if (get_flags_req_ack(flags)) {
        sendSinkTelemetry(seq, t_rx1_us, rssi);
    }
}

/**
 * @brief Push one frame into the SINK FIFO (m_sink_fifo_lock held).
 * @return false if the FIFO is full.
 */
bool EspNowBroadcastEngine::sinkFifoPush(uint8_t seq, uint8_t len, uint8_t flags, bool redundant, const uint8_t* src,
                                        uint32_t t_frame) {
    if (m_sink_fifo_count >= SINK_FIFO_PACKETS || !src) {
        return false;
    }
    SinkFifoItem& it = m_sink_fifo[m_sink_fifo_head];
    it.t_frame = t_frame;
    it.seq = seq;
    it.len = len;
    it.flags = flags;
    it.is_redundant = redundant;
    memcpy(it.data, src, len);
    m_sink_fifo_head = (m_sink_fifo_head + 1) % SINK_FIFO_PACKETS;
    m_sink_fifo_count++;
    return true;
}

/**
 * @brief Telemetry reply = time-sync delay request (see vsaf_time_sync.cpp and vsaf_protocol.hpp).
 *
 * @param ack_seq  seq of the REQ_ACK packet being answered.
 * @param t_rx1_us t2: receive time of that packet (SINK esp_timer).
 * @param rssi     its RSSI.
 */
void EspNowBroadcastEngine::sendSinkTelemetry(uint8_t ack_seq, int64_t t_rx1_us, int8_t rssi) {
    vsaf_sink_telemetry_t telem = {};
    telem.type_id = VSAF_TYPE_SINK_TELEMETRY;
    telem.sink_id = m_target_channel;
    telem.ack_seq = ack_seq;
    telem.t_rx_us = static_cast<uint32_t>(t_rx1_us);
    telem.downlink_rssi = rssi;

    portENTER_CRITICAL(&m_sink_fifo_lock);
    telem.fifo_fill = static_cast<uint8_t>((m_sink_fifo_count * 100) / SINK_FIFO_PACKETS);
    portEXIT_CRITICAL(&m_sink_fifo_lock);

    // Two-step: TX-done delay of our previous reply (measured in its send callback)
    if (m_prev_reply_valid.exchange(false, std::memory_order_acq_rel)) {
        telem.flags |= VSAF_TELEM_FLAG_PREV_DELAY_VALID;
        telem.prev_ack_seq = m_prev_reply_seq.load(std::memory_order_relaxed);
        telem.prev_tx_delay_us = m_prev_reply_delay_us.load(std::memory_order_relaxed);
    }
    telem.playout_err_us = m_playout_err_report.load(std::memory_order_relaxed);

    // t3 (submit time) as late as possible, right before the send
    const int64_t t_tx2 = esp_timer_get_time();
    telem.t_dwell_us = static_cast<uint16_t>(std::min<int64_t>(t_tx2 - t_rx1_us, 65535));
    telem.crc16 = calc_crc16(reinterpret_cast<const uint8_t*>(&telem), sizeof(vsaf_sink_telemetry_t) - 2);

    m_reply_seq.store(ack_seq, std::memory_order_relaxed);
    m_reply_submit_us.store(static_cast<uint32_t>(t_tx2), std::memory_order_relaxed);
    m_reply_inflight.store(true, std::memory_order_release);
    if (esp_now_send(s_broadcast_mac, reinterpret_cast<const uint8_t*>(&telem), sizeof(vsaf_sink_telemetry_t)) != ESP_OK) {
        m_reply_inflight.store(false, std::memory_order_relaxed);
    }
}

// SOURCE audio pipeline (DSP, parallel LC3 encoding on both cores, event-driven RF TX) lives in
// vsaf_source_pipeline.cpp.

void EspNowBroadcastEngine::handleTxSubsystemHang() {
    // Runs inside the real-time TX loop and can trigger many times per second on a busy channel:
    // log at most once per second (formatting + CDC output per event used to cost frame budget).
    static int64_t s_last_log_us = 0;
    static uint32_t s_suppressed = 0;
    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_log_us >= 1000000) {
        ESP_LOGW(TAG, "TX-done not received for %lu consecutive packets (%lu more such events in the last second)",
                 (unsigned long)m_consecutive_tx_timeouts, (unsigned long)s_suppressed);
        s_last_log_us = now_us;
        s_suppressed = 0;
    } else {
        s_suppressed++;
    }
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

// ---------------------------------------------------------------------------
// SINK clock-drift compensation helpers
// ---------------------------------------------------------------------------
namespace {
/**
 * @brief Index k in [8, n-9) where the waveform is smoothest (smallest |x[k+1] - x[k]|): least audible spot.
 */
template <typename T>
size_t driftSmoothestIndex(const T* x, size_t n) {
    size_t best = n / 2;
    int64_t best_d = INT64_MAX;
    for (size_t k = 8; k + 9 < n; ++k) {
        int64_t d = static_cast<int64_t>(x[k + 1]) - static_cast<int64_t>(x[k]);
        if (d < 0) d = -d;
        if (d < best_d) {
            best_d = d;
            best = k;
        }
    }
    return best;
}
} // namespace

/**
 * @brief Insert one interpolated sample into a frame (buffer must hold n + 1 samples).
 * @param x Mono PCM frame, n samples.
 * @return n + 1.
 */
template <typename T>
size_t EspNowBroadcastEngine::driftInsertSample(T* x, size_t n) {
    const size_t k = driftSmoothestIndex(x, n);
    for (size_t i = n; i > k + 1; --i) {
        x[i] = x[i - 1];
    }
    x[k + 1] = static_cast<T>((static_cast<int64_t>(x[k]) + static_cast<int64_t>(x[k + 2])) / 2);
    return n + 1;
}

/**
 * @brief Drop one sample from a frame by merging two neighbours into their average.
 * @param x Mono PCM frame, n samples.
 * @return n - 1.
 */
template <typename T>
size_t EspNowBroadcastEngine::driftDropSample(T* x, size_t n) {
    const size_t k = driftSmoothestIndex(x, n);
    x[k] = static_cast<T>((static_cast<int64_t>(x[k]) + static_cast<int64_t>(x[k + 1])) / 2);
    for (size_t i = k + 1; i + 1 < n; ++i) {
        x[i] = x[i + 1];
    }
    return n - 1;
}

void EspNowBroadcastEngine::sinkTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSinkLoop();
}

void EspNowBroadcastEngine::runSinkLoop() {
    ESP_LOGI(TAG, "SINK Audio Decoder & I2S Task started on Core 0");

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // One extra sample: drift compensation may play 481 samples from a 480-sample frame
    static int32_t pcm_mono[480 + 1];
    static int32_t pcm_stereo[(480 + 1) * 2];
    constexpr size_t SAMPLE_SIZE = sizeof(int32_t);
#else
    static int16_t pcm_mono[480 + 1];
    static int16_t pcm_stereo[(480 + 1) * 2];
    constexpr size_t SAMPLE_SIZE = sizeof(int16_t);
#endif
    // Clock-drift compensation (SINK crystal vs SOURCE): keep the FIFO depth at its level after start-up by
    // playing 481 (insert) or 479 (drop) samples from one frame. Depth filter in Q16 frames (16 fractional bits,
    // so the EMA can settle on any level; with Q8 and a >> 8 step it could not rise within the last frame).
    static constexpr int32_t  DRIFT_EMA_SHIFT = 8;        // EMA time constant 256 frames (~2.6 s)
    static constexpr uint32_t DRIFT_LEARN_FRAMES = 300;   // learn the target 3 s after entering STREAM
    static constexpr int32_t  DRIFT_DEADBAND_Q16 = 1 << 15; // +/- 0.5 frame around the target
    static constexpr uint32_t DRIFT_MIN_SPACING = 16;     // >= 16 frames between corrections (<= ~130 ppm)
    static constexpr int32_t  DRIFT_MIN_TARGET_Q16 = 2 << 16;
    int32_t  drift_ema_q16 = -1;
    int64_t  i2s_written = 0;   // samples written to the I2S DMA since start() (preload included)
    int32_t  drift_target_q16 = -1;
    uint32_t drift_frames = 0;
    uint32_t drift_last_adj_frame = 0;
    size_t samples_per_frame = (m_telemetry.sample_rate * 10) / 1000;
    // Scheduled playout (time sync): every frame's first sample is placed on its presentation time
    // t_frame + latency + trim (SOURCE clock) converted with the offset from the control frames
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    static int32_t zero_stereo[480 * 2] = {};
#else
    static int16_t zero_stereo[480 * 2] = {};
#endif
    // DMA staging: the I2S driver with 2 descriptors takes a fresh descriptor on every i2s_channel_write()
    // (its free-buffer queue holds one entry), so a short write leaves zeros at the end of a descriptor and a
    // long one spills into a nearly empty descriptor. All output therefore goes through this buffer and only
    // whole descriptors (getDmaFrameSamples(), 5 ms by default) are written; a dropped / inserted sample then really shifts the
    // timeline by one sample. Queued audio = i2s_written + stage_n - played.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    static int32_t dac_stage[480 * 2];
#else
    static int16_t dac_stage[480 * 2];
#endif
    size_t   stage_n = 0;        // stereo frames waiting in dac_stage
    uint32_t q_ovf_seen = m_i2s_dac ? m_i2s_dac->getQueueOverflows() : 0;
    auto dac_push = [&](const auto* src, size_t n) {
        const size_t desc = std::min<size_t>(m_i2s_dac ? m_i2s_dac->getDmaFrameSamples() : samples_per_frame, 480);
        while (n > 0 && m_i2s_dac) {
            const size_t take = std::min(n, desc - stage_n);
            memcpy(&dac_stage[stage_n * 2], src, take * 2 * SAMPLE_SIZE);
            stage_n += take;
            src += take * 2;
            n -= take;
            if (stage_n >= desc) {
                size_t bytes_written = 0;
                m_i2s_dac->write(dac_stage, desc * SAMPLE_SIZE * 2, &bytes_written, 50);
                i2s_written += static_cast<int64_t>(bytes_written / (SAMPLE_SIZE * 2));
                stage_n = 0;
            }
        }
    };
    static constexpr int32_t SYNC_COARSE_US = 3000;     // larger errors: skip frames / drop or pad samples
    static constexpr int64_t SYNC_MAX_PAD_SAMPLES = 4800; // at most 100 ms of silence per frame when early
    static constexpr int32_t SYNC_THETA_SLEW_US = 2;      // offset in use follows the estimate <= 2 us / frame
    static constexpr int32_t SYNC_THETA_JUMP_US = 1000;   // larger estimate changes: jump and realign
    bool     sync_aligned = false;
    bool     theta_valid = false;
    int32_t  theta_used = 0;
    int32_t  err_filt_us = 0;     // filtered playout error (us), from err_filt_q8
    int32_t  err_filt_q8 = 0;     // same in Q8 (us * 256), so the slow EMA has no integer dead zone
    uint32_t next_frame_t = 0;
    bool     next_frame_t_valid = false;
    uint32_t last_fine_frame = 0;
    int64_t  sync_wait_start_us = 0;
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
        //    Starts on CONFIG_VSAF_PREFERRED_WIFI_CHANNEL (set in init), then hops upward with wrap (13 -> 1),
        //    CONFIG_VSAF_SCAN_DWELL_MS per channel. Any VSAF audio packet locks the channel (handleRx).
        // -------------------------------------------------------------------
        if (current_state == NetworkState::SCANNING) {
            if (!m_channel_locked.load(std::memory_order_acquire)) {
                int64_t now_us = esp_timer_get_time();
                if (now_us - last_hop_time_us >= CONFIG_VSAF_SCAN_DWELL_MS * 1000LL) {
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
                // Start scheduled (time-synced) playback if possible: wait up to CONFIG_VSAF_SYNC_WAIT_MS for the
                // first control frame with an estimate for our channel, keeping the FIFO short meanwhile
                const int64_t now_us = esp_timer_get_time();
                int32_t th = 0;
                uint32_t lat = 0;
                int16_t tr = 0;
                const bool synced = sinkCurrentTheta(now_us, th, lat, tr);
                if (sync_wait_start_us == 0) {
                    sync_wait_start_us = now_us;
                }
                if (!synced && now_us - sync_wait_start_us < CONFIG_VSAF_SYNC_WAIT_MS * 1000LL) {
                    portENTER_CRITICAL(&m_sink_fifo_lock);
                    while (m_sink_fifo_count > PREFILL_THRESHOLD + 2) {
                        m_sink_fifo_tail = (m_sink_fifo_tail + 1) % SINK_FIFO_PACKETS;
                        m_sink_fifo_count--;
                    }
                    portEXIT_CRITICAL(&m_sink_fifo_lock);
                    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
                    continue;
                }
                sync_wait_start_us = 0;
                ESP_LOGI(TAG, "SINK: Jitter buffer prefill threshold reached (%zu pkts on Ch %d), time sync %s -> PREFILL",
                         buffered, m_wifi_channel, synced ? "valid" : "NOT valid (unsynchronized start)");
                transitionTo(NetworkState::PREFILL);
            } else {
                int64_t now_us = esp_timer_get_time();
                // Stream gone (or a stray packet locked a quiet channel): hold, then resume the sweep
                if (now_us - m_last_rx_audio_pkt_us.load(std::memory_order_acquire) >= CONFIG_VSAF_SCAN_HOLD_MS * 1000LL) {
                    ESP_LOGI(TAG, "SINK: no audio on Ch %d for %d ms -> channel search", m_wifi_channel,
                             CONFIG_VSAF_SCAN_HOLD_MS);
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
                q_ovf_seen = m_i2s_dac->getQueueOverflows();
            }
            i2s_written = 0;
            stage_n = 0;

            int32_t pf_theta = 0;
            uint32_t pf_lat = 0;
            int16_t pf_trim = 0;
            const bool pf_synced = sinkCurrentTheta(esp_timer_get_time(), pf_theta, pf_lat, pf_trim);
            if (pf_synced) {
                // Scheduled playout: start the DAC on silence, the first frame is placed on its presentation time
                for (int d = 0; d < 2 && m_i2s_dac; ++d) {
                    size_t bytes_written = 0;
                    m_i2s_dac->preload(zero_stereo, samples_per_frame * SAMPLE_SIZE * 2, &bytes_written);
                    i2s_written += static_cast<int64_t>(bytes_written / (SAMPLE_SIZE * 2));
                }
                m_has_expected_seq = false;
            } else {
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
                    i2s_written += static_cast<int64_t>(bytes_written / (SAMPLE_SIZE * 2));
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
                    i2s_written += static_cast<int64_t>(bytes_written / (SAMPLE_SIZE * 2));
                }
                m_expected_seq = item2.seq + 1;
                m_has_expected_seq = true;
            }
            }  // !pf_synced

            if (m_i2s_dac) {
                m_i2s_dac->start(); // Start hardware DMA clocks with preloaded descriptors
            }
            sync_aligned = false;
            theta_valid = false;
            err_filt_us = 0;
            err_filt_q8 = 0;
            next_frame_t_valid = false;
            consecutive_underruns = 0;
            drift_ema_q16 = -1;
            drift_target_q16 = -1;
            drift_frames = 0;
            drift_last_adj_frame = 0;
            m_drift_adj_total.store(0, std::memory_order_relaxed);
            m_drift_adj_reported = 0;
            m_frames_played.store(0, std::memory_order_relaxed);
            m_drift_target_x10.store(-1, std::memory_order_relaxed);
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
            // Wait at most 6 ms, and only while the DMA queue still has audio (minus decode + write time)
            int64_t jit_us = 6000;
            if (m_i2s_dac) {
                const int64_t queued = i2s_written + static_cast<int64_t>(stage_n) - m_i2s_dac->getPlayedSamples();
                const int64_t queued_us = (queued * 1000000) / static_cast<int64_t>(samples_per_frame * 100);
                jit_us = std::clamp<int64_t>(queued_us - CONFIG_VSAF_JIT_MARGIN_US, 0, 6000);
            }
            int64_t wait_until_us = esp_timer_get_time() + jit_us;
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
                // Keep the channel: the SCANNING state waits CONFIG_VSAF_SCAN_HOLD_MS for the stream to return
                // (e.g. SOURCE reboot) before it sweeps the other channels
                m_channel_locked.store(true, std::memory_order_release);
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
        // THD+N self-test, tap 1: decoder output of HQ frames (before sample edits and gain)
        const bool thdn_frame = m_thdn_state.load(std::memory_order_acquire) == 1 && has_item && !item.is_redundant &&
                                actual_samples == 480 && samples_per_frame == 480;
        if (thdn_frame) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
            m_thdn_dec.addFrame(pcm_mono, 1, 0);  // 24-bit in int32
#else
            m_thdn_dec.addFrame(pcm_mono, 1, 0);  // 16-bit
#endif
        }

        // Frame time of this frame (SOURCE clock): from the FIFO item, or the expected one for a concealed frame
        const uint32_t frame_us = 10000;
        uint32_t frame_t = 0;
        bool frame_t_valid = false;
        if (has_item) {
            frame_t = item.t_frame;
            frame_t_valid = true;
            if (next_frame_t_valid) {
                const int32_t jump = static_cast<int32_t>(frame_t - next_frame_t);
                if (jump > 2000 || jump < -2000) {
                    sync_aligned = false;  // discontinuity in the SOURCE timeline (restart): realign
                    m_sync_why[0].fetch_add(1, std::memory_order_relaxed);
                }
            }
        } else if (next_frame_t_valid) {
            frame_t = next_frame_t;
            frame_t_valid = true;
        }

        // Buffered audio after this pop: FIFO frames + samples still queued in the I2S DMA (telemetry, and the
        // fallback drift control). The DMA queue also gives the play time of the next sample written.
        size_t out_samples = samples_per_frame;
        size_t skip_head = 0;      // samples dropped from the start of this frame (coarse alignment, late)
        int64_t pad_zeros = 0;     // silence written before this frame (coarse alignment, early)
        bool skip_frame = false;   // late by >= 1 frame: do not play it
        const uint32_t sr_hz = static_cast<uint32_t>(samples_per_frame * 100);
        int64_t dma_queued = 0;
        int64_t now_us = esp_timer_get_time();
        {
            portENTER_CRITICAL(&m_sink_fifo_lock);
            const uint32_t depth = static_cast<uint32_t>(m_sink_fifo_count);
            portEXIT_CRITICAL(&m_sink_fifo_lock);
            if (m_i2s_dac) {
                now_us = esp_timer_get_time();
                // DMA underruns: each one played a descriptor of zeros that was never written
                const uint32_t q_ovf = m_i2s_dac->getQueueOverflows();
                if (q_ovf != q_ovf_seen) {
                    i2s_written += static_cast<int64_t>(q_ovf - q_ovf_seen) * m_i2s_dac->getDmaFrameSamples();
                    q_ovf_seen = q_ovf;
                    sync_aligned = false;
                }
                const int64_t dbg_played = m_i2s_dac->getPlayedSamples();
                dma_queued = i2s_written + static_cast<int64_t>(stage_n) - dbg_played;
                if (dma_queued < m_dbg_q_min.load(std::memory_order_relaxed)) m_dbg_q_min.store((int32_t)dma_queued, std::memory_order_relaxed);
                if (dma_queued > m_dbg_q_max.load(std::memory_order_relaxed)) m_dbg_q_max.store((int32_t)dma_queued, std::memory_order_relaxed);
                m_dbg_played.store(dbg_played, std::memory_order_relaxed);
                m_dbg_written.store(i2s_written, std::memory_order_relaxed);
                m_dbg_t.store(now_us, std::memory_order_relaxed);
                if (dma_queued < -static_cast<int64_t>(samples_per_frame)) {
                    // Unexplained (missed overflow event): re-base the counter and realign
                    i2s_written -= dma_queued;
                    dma_queued = 0;
                    sync_aligned = false;
                    m_dma_rebase.fetch_add(1, std::memory_order_relaxed);
                }
            }
            const int64_t occ_samples = static_cast<int64_t>(depth) * samples_per_frame + dma_queued;
            const int32_t depth_q16 = static_cast<int32_t>((occ_samples << 16) / static_cast<int64_t>(samples_per_frame));
            m_fifo_depth_sum.fetch_add(static_cast<uint32_t>((occ_samples * 100) / static_cast<int64_t>(samples_per_frame)),
                                       std::memory_order_relaxed);
            m_fifo_depth_n.fetch_add(1, std::memory_order_relaxed);
            if (depth < m_fifo_depth_min.load(std::memory_order_relaxed)) {
                m_fifo_depth_min.store(depth, std::memory_order_relaxed);
            }
            drift_ema_q16 = (drift_ema_q16 < 0) ? depth_q16 : drift_ema_q16 + ((depth_q16 - drift_ema_q16) >> DRIFT_EMA_SHIFT);
            drift_frames++;
            m_frames_played.store(drift_frames, std::memory_order_relaxed);
        }

        int32_t s_theta = 0;
        uint32_t s_lat = 0;
        int16_t s_trim = 0;
        const bool synced = frame_t_valid && m_i2s_dac && !m_sync_disable.load(std::memory_order_relaxed) && sinkCurrentTheta(now_us, s_theta, s_lat, s_trim);
        if (synced) {
            // Offset in use: follows the estimate slowly, jumps (and realigns) only on a large change
            if (!theta_valid || s_theta - theta_used > SYNC_THETA_JUMP_US || s_theta - theta_used < -SYNC_THETA_JUMP_US) {
                if (theta_valid) {
                    m_sync_why[1].fetch_add(1, std::memory_order_relaxed);
                }
                theta_used = s_theta;
                theta_valid = true;
                sync_aligned = false;
            } else {
                theta_used += std::clamp<int32_t>(s_theta - theta_used, -SYNC_THETA_SLEW_US, SYNC_THETA_SLEW_US);
            }
            m_sync_offset_used.store(theta_used, std::memory_order_relaxed);

            // Error of this frame's first sample: predicted DAC time vs. presentation time (SINK clock)
            const uint32_t pt_local = frame_t + s_lat + static_cast<uint32_t>(static_cast<int32_t>(s_trim)) +
                                      static_cast<uint32_t>(theta_used);
            const uint32_t play_local = static_cast<uint32_t>(now_us) +
                                        static_cast<uint32_t>((dma_queued * 1000000) / sr_hz) + CONFIG_VSAF_DAC_DELAY_US;
            const int32_t err_us = static_cast<int32_t>(play_local - pt_local);  // + = would play late
            const int32_t abs_err = err_us < 0 ? -err_us : err_us;
            if (abs_err > m_sync_err_maxabs.load(std::memory_order_relaxed)) {
                m_sync_err_maxabs.store(abs_err, std::memory_order_relaxed);
            }

            if (!sync_aligned || err_us > SYNC_COARSE_US || err_us < -SYNC_COARSE_US) {
                // Coarse: start, recovery or a timeline change
                const int64_t err_samples = (static_cast<int64_t>(err_us) * sr_hz) / 1000000;
                {
                    static uint32_t s_last_coarse_frame = 0;
                    portENTER_CRITICAL(&m_sink_fifo_lock);
                    const uint32_t d_fifo = static_cast<uint32_t>(m_sink_fifo_count);
                    portEXIT_CRITICAL(&m_sink_fifo_lock);
                    CoarseEvent& ev = m_coarse_ring[m_coarse_head % 8];
                    ev.err_us = err_us;
                    ev.fifo = static_cast<uint16_t>(d_fifo);
                    ev.dma_queued = static_cast<int16_t>(std::clamp<int64_t>(dma_queued, -32768, 32767));
                    ev.has_item = has_item ? 1 : 0;
                    ev.was_aligned = sync_aligned ? 1 : 0;
                    ev.frames_since = static_cast<uint16_t>(std::min<uint32_t>(drift_frames - s_last_coarse_frame, 65535));
                    s_last_coarse_frame = drift_frames;
                    m_coarse_head++;
                }
                if (err_samples >= static_cast<int64_t>(samples_per_frame)) {
                    skip_frame = true;  // late by a frame or more: drop it, try the next one
                    m_sync_why[2].fetch_add(1, std::memory_order_relaxed);
                } else if (err_samples > 0) {
                    skip_head = static_cast<size_t>(err_samples);
                    sync_aligned = true;
                    m_sync_why[3].fetch_add(1, std::memory_order_relaxed);
                } else {
                    m_sync_why[4].fetch_add(1, std::memory_order_relaxed);
                    pad_zeros = std::min<int64_t>(-err_samples, SYNC_MAX_PAD_SAMPLES);
                    sync_aligned = (-err_samples <= SYNC_MAX_PAD_SAMPLES);
                }
                err_filt_us = 0;
                err_filt_q8 = 0;
                m_sync_coarse.fetch_add(1, std::memory_order_relaxed);
            } else {
                // Fine: filtered error (EMA 1/32, ~320 ms: the raw per-frame error has ~+/-100 us of ISR and
                // task-wake jitter), at most one inserted / dropped sample every 2 frames, deadband +/-3/4 sample
                err_filt_q8 += ((err_us << 8) - err_filt_q8) / 32;
                err_filt_us = err_filt_q8 >> 8;
                const int32_t sample_us = static_cast<int32_t>(1000000 / sr_hz);
                if (drift_frames - last_fine_frame >= 2 && samples_per_frame >= 32) {
                    if (err_filt_us > (sample_us * 3) / 4) {
                        out_samples = driftDropSample(pcm_mono, samples_per_frame);
                        err_filt_q8 -= sample_us << 8;
                        err_filt_us = err_filt_q8 >> 8;
                        last_fine_frame = drift_frames;
                        m_sync_fine.fetch_add(1, std::memory_order_relaxed);
                        m_drift_adj_total.fetch_sub(1, std::memory_order_relaxed);
                    } else if (err_filt_us < -(sample_us * 3) / 4) {
                        out_samples = driftInsertSample(pcm_mono, samples_per_frame);
                        err_filt_q8 += sample_us << 8;
                        err_filt_us = err_filt_q8 >> 8;
                        last_fine_frame = drift_frames;
                        m_sync_fine.fetch_add(1, std::memory_order_relaxed);
                        m_drift_adj_total.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
            m_sync_err_filt.store(err_filt_us, std::memory_order_relaxed);
            m_playout_err_report.store(static_cast<int16_t>(std::clamp<int32_t>(sync_aligned ? err_filt_us : err_us, -32767, 32767)),
                                       std::memory_order_relaxed);

            // Self-test probe: predicted play time of the first sample of the frame at each full second
            if (m_syncprobe.load(std::memory_order_relaxed) && (frame_t / 10000U) % 100U == 0 && !skip_frame) {
                const int32_t applied_us = static_cast<int32_t>((skip_head * 1000000ULL) / sr_hz) -
                                           static_cast<int32_t>((pad_zeros * 1000000LL) / sr_hz);
                portENTER_CRITICAL(&m_probe_mux);
                m_probe.play_new = true;
                m_probe.play_frame_t = frame_t;
                m_probe.play_master = play_local - static_cast<uint32_t>(theta_used) - static_cast<uint32_t>(applied_us);
                m_probe.play_err_us = err_us - applied_us;
                portEXIT_CRITICAL(&m_probe_mux);
            }
        } else {
            // No time sync: unsynchronized playout, FIFO-level clock-drift compensation (section 5.3 of the README)
            sync_aligned = false;
            theta_valid = false;
            m_sync_unsynced_frames.fetch_add(1, std::memory_order_relaxed);
            m_playout_err_report.store(VSAF_PLAYOUT_ERR_UNKNOWN, std::memory_order_relaxed);
            if (drift_target_q16 < 0 && drift_frames >= DRIFT_LEARN_FRAMES) {
                drift_target_q16 = std::max(drift_ema_q16, DRIFT_MIN_TARGET_Q16);
                m_drift_target_x10.store(static_cast<int32_t>((static_cast<int64_t>(drift_target_q16) * 10 + 32768) >> 16),
                                         std::memory_order_relaxed);
            }
            if (drift_target_q16 >= 0 && m_drift_corr_enabled.load(std::memory_order_relaxed) &&
                drift_frames - drift_last_adj_frame >= DRIFT_MIN_SPACING && samples_per_frame >= 32) {
                const int32_t err = drift_ema_q16 - drift_target_q16;
                if (err < -DRIFT_DEADBAND_Q16) {
                    // FIFO draining: SINK clock fast -> play one sample more from this frame
                    out_samples = driftInsertSample(pcm_mono, samples_per_frame);
                    m_drift_adj_total.fetch_add(1, std::memory_order_relaxed);
                    drift_last_adj_frame = drift_frames;
                } else if (err > DRIFT_DEADBAND_Q16) {
                    // FIFO filling: SINK clock slow -> play one sample less
                    out_samples = driftDropSample(pcm_mono, samples_per_frame);
                    m_drift_adj_total.fetch_sub(1, std::memory_order_relaxed);
                    drift_last_adj_frame = drift_frames;
                }
            }
        }
        m_sync_aligned.store(synced && sync_aligned, std::memory_order_relaxed);
        if (frame_t_valid) {
            next_frame_t = frame_t + frame_us;
            next_frame_t_valid = true;
        }
        if (skip_frame) {
            continue;  // late: not played, the next frame is checked right away
        }

        // Silence before the frame (coarse alignment, early)
        while (pad_zeros > 0 && m_i2s_dac) {
            const size_t n = static_cast<size_t>(std::min<int64_t>(pad_zeros, static_cast<int64_t>(samples_per_frame)));
            dac_push(zero_stereo, n);
            pad_zeros -= static_cast<int64_t>(n);
        }
        if (skip_head >= out_samples) {
            skip_head = 0;
        }

        // Mono to Stereo duplication for I2S DAC (from skip_head on)
        float total_gain_db = m_current_gain_db + m_post_gain_db;
        float gain = (m_current_gain_db <= -90.0f) ? 0.0f : std::pow(10.0f, total_gain_db / 20.0f);
        const size_t n_out = out_samples - skip_head;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        for (size_t i = 0; i < n_out; ++i) {
            int64_t val = static_cast<int64_t>(pcm_mono[i + skip_head] * gain);
            if (val > 8388607) val = 8388607;
            if (val < -8388608) val = -8388608;
            int32_t sample = static_cast<int32_t>(val << 8) & 0xffffff00;
            pcm_stereo[i * 2]     = sample;
            pcm_stereo[i * 2 + 1] = sample;
        }
#else
        for (size_t i = 0; i < n_out; ++i) {
            int32_t val = static_cast<int32_t>(pcm_mono[i + skip_head] * gain);
            if (val > 32767) val = 32767;
            if (val < -32768) val = -32768;
            int16_t sample = static_cast<int16_t>(val);
            pcm_stereo[i * 2]     = sample;
            pcm_stereo[i * 2 + 1] = sample;
        }
#endif
        if (m_i2s_dac) {
            // THD+N self-test, tap 2: the samples handed to I2S (after gain and clamping), unedited frames only
            if (thdn_frame) {
                if (n_out == 480 && skip_head == 0) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
                    m_thdn_out.addFrame(pcm_stereo, 2, 8);  // 24-bit << 8 in int32 -> 24-bit
#else
                    m_thdn_out.addFrame(pcm_stereo, 2, 0);
#endif
                } else {
                    m_thdn_skipped++;
                }
                if (m_thdn_dec.frames() >= m_thdn_target_frames) {
                    m_thdn_state.store(2, std::memory_order_release);
                }
            }
            dac_push(pcm_stereo, n_out);
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
