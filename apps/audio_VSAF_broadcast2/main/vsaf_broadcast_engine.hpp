#pragma once

#include "config.h"
#include "vsaf_protocol.hpp"
#include "lc3_codec.hpp"
#include "tone_generator.hpp"
#include "i2s_audio.hpp"
#include "audio_metering.hpp"
#include "biquad_filter.hpp"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <atomic>
#include <vector>
#include <algorithm>
#include <cstring>

namespace AudioNet {

enum class NetworkState : uint8_t {
    OFF = 0,
    IDLE,
    SCANNING,
    PREFILL,
    STREAM,
    CAST
};

enum class PeerStatus : uint8_t {
    DISABLED = 0,
    OFFLINE  = 1,
    ONLINE   = 2
};

enum class WifiPhyProfile : uint8_t {
    PRIMARY_HT20_MCS3 = 0,   // 802.11n HT20, MCS3 (16-QAM 1/2, 26.0 Mbps) - Primary Default
    SECONDARY_OFDM_12M = 1,  // 802.11g OFDM, 12 Mbps (QPSK 1/2)           - Secondary Fallback
    TERTIARY_HT20_MCS0 = 2   // 802.11n HT20, MCS0 (BPSK 1/2, 6.5 Mbps)    - Tertiary Long-Range
};

struct SinkPeerConfig {
    uint8_t    channel_id;             // 0: Left, 1: Right, 2: Center, 3: L-Surround, 4: R-Surround, 5: Sub
    char       name[24];
    bool       is_enabled;
    PeerStatus status;                 // ONLINE, OFFLINE, DISABLED
    int64_t    session_start_time_us;
    int64_t    last_rx_telemetry_time_us; // Last received telemetry timestamp (us)
    uint32_t   packets_sent;
    uint32_t   acks_received;
    uint32_t   ack_failures;
    uint32_t   consecutive_ack_fails;  // 5 consecutive misses -> OFFLINE
    uint32_t   arq_retries;            // Number of ARQ retransmissions sent
    uint32_t   arq_successes;          // Number of times ARQ recovered the packet
    int8_t     last_rssi;              // Uplink RSSI from SINK Soft-ACK (dBm)
    int8_t     downlink_rssi;          // Downlink RSSI measured by SINK (dBm)
    uint32_t   last_rtt_us;            // Most recent Net RTT (us)
    uint32_t   avg_rtt_us;             // Smoothed average Net RTT (us)
    uint32_t   last_total_rtt_us;      // Total RTT: t_rx2 - t_tx1_echo (us)
    uint32_t   last_dwell_us;          // SINK turnaround dwell time: t_tx2 - t_rx1 (us)
    uint32_t   last_net_rtt_us;        // Net RF propagation RTT: Total - Dwell (us)
    int32_t    clock_offset_us;        // PTP dual-way clock offset (us)
    uint8_t    fifo_fill;              // SINK buffer occupancy (0-100%)
};

struct StreamTelemetry {
    uint32_t sample_rate = CONFIG_ESPNOW_SAMPLE_RATE_HZ;
    uint16_t frame_len = CONFIG_ESPNOW_FRAME_LEN_OCTETS;
    uint32_t frame_duration_us = 10000;
    uint8_t  bit_depth = 16;
    uint8_t  active_peers_count = 0;
};

// SPSC Ring Buffer for Codec & Timing Duration Tracking
template <typename T, size_t N>
class SpscDurationRingBuffer {
public:
    SpscDurationRingBuffer() : m_head(0), m_count(0) {}

    void push(T val) {
        m_buffer[m_head] = val;
        m_head = (m_head + 1) % N;
        if (m_count < N) m_count++;
    }

    void getStats(float& out_avg, float& out_peak, bool& out_has_data) const {
        if (m_count == 0) {
            out_avg = 0.0f;
            out_peak = 0.0f;
            out_has_data = false;
            return;
        }
        out_has_data = true;
        T sum = 0;
        T max_val = 0;
        for (size_t i = 0; i < m_count; i++) {
            sum += m_buffer[i];
            if (m_buffer[i] > max_val) max_val = m_buffer[i];
        }
        out_avg = static_cast<float>(sum) / static_cast<float>(m_count);
        out_peak = static_cast<float>(max_val);
    }

private:
    T m_buffer[N] = {};
    size_t m_head;
    size_t m_count;
};

class TimeOffsetRingBuffer {
public:
    static constexpr size_t CAPACITY = 50;

    TimeOffsetRingBuffer() : m_head(0), m_count(0) {}

    void push(float val_ms) {
        m_buffer[m_head] = val_ms;
        m_head = (m_head + 1) % CAPACITY;
        if (m_count < CAPACITY) m_count++;
    }

    void computeStats(float& out_median, float& out_range, bool& out_has_data) const {
        if (m_count == 0) {
            out_median = 0.0f;
            out_range = 0.0f;
            out_has_data = false;
            return;
        }
        out_has_data = true;
        float sorted[CAPACITY];
        memcpy(sorted, m_buffer, m_count * sizeof(float));
        std::sort(sorted, sorted + m_count);

        size_t n = m_count;
        if (n % 2 == 1) {
            out_median = sorted[n / 2];
        } else {
            out_median = (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5f;
        }
        out_range = sorted[n - 1] - sorted[0];
    }

    void clear() {
        m_head = 0;
        m_count = 0;
    }

private:
    float m_buffer[CAPACITY] = {};
    size_t m_head;
    size_t m_count;
};

class EspNowBroadcastEngine {
public:
    EspNowBroadcastEngine(Codec::Lc3CodecEngine& primary_codec, Audio::ToneGenerator* primary_tone_gen = nullptr, Hardware::I2sAudioDriver* i2s_dac = nullptr);
    ~EspNowBroadcastEngine();

    esp_err_t init(uint8_t role, uint8_t node_id, uint8_t wifi_channel = 1);
    esp_err_t start();
    esp_err_t stop();

    // SINK Peer Management (SOURCE node: Channels 0 to 5)
    bool addPeer(uint8_t channel_id, const char* name = nullptr);
    bool addPeer(const uint8_t* mac, uint8_t channel_id, const char* name = nullptr) { return addPeer(channel_id, name); }
    bool removePeer(uint8_t channel_id);
    bool setPeerEnabled(uint8_t channel_id, bool enabled);
    const SinkPeerConfig* getPeer(int index) const;
    void resetPeerStats();
    int  getPeerCount() const { return m_peer_count; }
    void getNodeStatusString(char* out_buf, size_t max_len) const;

    // Volume Control & Slew Limiter
    void setVolume(uint8_t vol_u8, bool instant = false);
    uint8_t getVolume() const { return m_target_volume_u8.load(std::memory_order_relaxed); }
    float getTargetVolumeDb() const;
    float getCurrentSlewDb() const { return m_current_gain_db; }
    void sendVolumeCommand(uint8_t channel_id, uint8_t vol_u8, bool instant = false);
    void setPostGainDb(float db) { m_post_gain_db = db; }
    float getPostGainDb() const { return m_post_gain_db; }

    // Dynamic Stream Configuration
    esp_err_t setSampleRate(uint32_t sample_rate_hz);
    esp_err_t setBitDepth(uint8_t bit_depth);
    esp_err_t setFrameLen(uint16_t frame_len_octets);
    esp_err_t setFrameDuration(uint32_t frame_duration_us);
    esp_err_t setWifiPhyProfile(WifiPhyProfile profile);
    esp_err_t setWifiPhyRate(wifi_phy_mode_t phymode, wifi_phy_rate_t rate);

    wifi_phy_mode_t getWifiPhyMode() const { return m_tx_phy_mode; }
    wifi_phy_rate_t getWifiPhyRate() const { return m_tx_phy_rate; }
    uint32_t getSampleRate() const { return m_telemetry.sample_rate; }
    uint32_t getFrameDurationUs() const { return m_frame_duration_us; }
    uint16_t getFrameLen() const { return m_octets_per_frame; }
    uint8_t  getBitDepth() const { return m_telemetry.bit_depth; }

    // Wi-Fi RF Sniffer & Channel Management
    struct WifiChannelScanResult {
        uint8_t  channel;
        uint32_t packet_count;
        uint32_t byte_count;
        int8_t   max_rssi;
        int8_t   avg_rssi;
        bool     has_40mhz;
        uint32_t score; // Lower is cleaner
    };

    uint8_t scanAndSelectBestChannel(uint32_t dwell_ms_per_ch = 120, bool auto_apply = true, bool print_results = true);
    esp_err_t setWifiChannel(uint8_t channel);
    uint8_t getWifiChannel() const { return m_wifi_channel; }
    bool isChannelLocked() const { return m_channel_locked.load(std::memory_order_acquire); }
    void lockChannel(bool locked) { m_channel_locked.store(locked, std::memory_order_release); }
    esp_err_t setWifiTxPower(int8_t power_0_25dbm);
    int8_t getWifiTxPower() const;

    // Multi-Channel Target Selection (SINK node: 0: Left, 1: Right, 5: Subwoofer)
    void setTargetChannel(uint8_t channel_id);
    uint8_t getTargetChannel() const { return m_target_channel; }

    // Packet Callbacks
    void onPacketSent(const uint8_t* mac_addr, esp_now_send_status_t status);
    void onPacketReceived(const uint8_t* mac_addr, const uint8_t* data, int data_len, int8_t rssi = -127, uint8_t rate = 0);

    // Audio Input Pipeline
    bool pushUsbAudioPcm(const int16_t* pcm_stereo, size_t samples_per_channel);
    bool processUsbVsafPacket(const uint8_t* data, size_t len);

    // State Machine
    NetworkState getState() const { return m_state.load(std::memory_order_acquire); }
    const char* getStateString() const;
    void transitionTo(NetworkState new_state);

    // Telemetry & Diagnostics Compatibility Interface
    const StreamTelemetry& getStreamTelemetry() const { return m_telemetry; }
    const StreamTelemetry& getTelemetry() const { return m_telemetry; }
    const char* getActiveCodecName() const { return "LC3"; }
    const char* getWifiPhyRateString() const;
    bool hasLocalAudioOutput() const { return (m_i2s_dac != nullptr && m_node_role == NODE_ROLE_SINK); }
    void setToneTestMode(bool enable) { m_tone_test_mode = enable; }
    bool isToneTestMode() const { return m_tone_test_mode; }

    // Audio Metering
    int16_t getAudioFrameRMS_int16() { return m_audio_meter.getAudioFrameRMS_int16(); }
    int16_t getAudioPeak_int16() { return m_audio_meter.getAudioFramePeak_int16(); }
    float getAudioFrameRMS_dBFS() { return m_audio_meter.getAudioFrameRMS_dBFS(); }
    float getAudioPeak_dBFS() { return m_audio_meter.getAudioFramePeak_dBFS(); }

    // Statistics Getters
    uint32_t getTxPacketsTotal() const { return m_tx_packets_total.load(std::memory_order_relaxed); }
    uint32_t getAndResetTxPacketsSec() { return m_tx_packets_sec.exchange(0, std::memory_order_relaxed); }
    uint32_t getTxAcksTotal() const { return m_tx_acks_total.load(std::memory_order_relaxed); }
    uint32_t getTxAckFailsTotal() const { return m_tx_ack_fails_total.load(std::memory_order_relaxed); }
    uint32_t getTxAcksSec() const { return m_tx_acks_sec.load(std::memory_order_relaxed); }
    uint32_t getTxAckFailsSec() const { return m_tx_ack_fails_sec.load(std::memory_order_relaxed); }
    uint32_t getAndResetTxAcksSec() { return m_tx_acks_sec.exchange(0, std::memory_order_relaxed); }
    uint32_t getAndResetTxAckFailsSec() { return m_tx_ack_fails_sec.exchange(0, std::memory_order_relaxed); }

    uint32_t getRxPacketsTotal() const { return m_rx_packets_total.load(std::memory_order_relaxed); }
    uint32_t getAndResetRxPacketsSec() { return m_rx_packets_sec.exchange(0, std::memory_order_relaxed); }
    uint32_t getRawEspNowRxCount() const { return m_raw_espnow_rx_count.load(std::memory_order_relaxed); }
    uint32_t getRawAudioPktCount() const { return m_raw_audio_pkt_count.load(std::memory_order_relaxed); }
    uint8_t getLastSeenRxId() const { return m_last_seen_rx_id.load(std::memory_order_relaxed); }

    uint32_t getFifoOverflowCount() const { return m_fifo_overflows.load(std::memory_order_relaxed); }
    uint32_t getFifoUnderrunCount() const { return m_fifo_underflows.load(std::memory_order_relaxed); }
    uint32_t getAndResetFifoOverflowCount() { return m_fifo_overflows.exchange(0, std::memory_order_relaxed); }
    uint32_t getAndResetFifoUnderrunCount() { return m_fifo_underflows.exchange(0, std::memory_order_relaxed); }

    uint32_t getAndResetDmaUnderrunCount() { return m_i2s_dac ? m_i2s_dac->getAndResetUnderrunCount() : 0; }
    uint32_t getDmaUnderrunCount() const { return m_i2s_dac ? m_i2s_dac->getUnderrunCount() : 0; }
    uint8_t  getHardwareGainDb() const { return m_i2s_dac ? m_i2s_dac->getHardwareGainDb() : 0; }
    void     setHardwareGain(Hardware::Max98357Gain gain) { if (m_i2s_dac) m_i2s_dac->setHardwareGain(gain); }
    uint32_t getPlcCount() const { return m_plc_count.load(std::memory_order_relaxed); }
    uint32_t getAndResetPlcCount() { return m_plc_count.exchange(0, std::memory_order_relaxed); }

    void resetStreamingCounters() {
        m_plc_count.store(0, std::memory_order_relaxed);
        m_fifo_underflows.store(0, std::memory_order_relaxed);
        m_fifo_overflows.store(0, std::memory_order_relaxed);
        m_rx_packets_sec.store(0, std::memory_order_relaxed);
        m_rx_packets_total.store(0, std::memory_order_relaxed);
        if (m_i2s_dac) {
            m_i2s_dac->resetUnderrunCount();
        }
    }
    void resetErrorCounters() {
        m_plc_count.store(0, std::memory_order_relaxed);
        m_fifo_underflows.store(0, std::memory_order_relaxed);
        m_fifo_overflows.store(0, std::memory_order_relaxed);
        if (m_i2s_dac) {
            m_i2s_dac->resetUnderrunCount();
        }
    }

    uint64_t getMasterTimeMs() const { return m_last_master_time_us / 1000ULL; }
    bool     isMasterTimeValid() const { return m_first_packet_received; }
    int8_t   getLastRssi() const { return m_last_rssi; }

    void updatePerSecondStats();
    void update10HzTimeOffsetStats();
    void getTimeSyncStats(float& out_median_ms, float& out_range_ms, bool& out_has_data) const;

    void getTimeOffsetStats(float& out_ema_ms, float& out_rb_med_ms, float& out_rb_rng_ms, bool& out_has_stats) const {
        out_ema_ms = m_ema_time_offset_ms;
        m_time_offset_buf.computeStats(out_rb_med_ms, out_rb_rng_ms, out_has_stats);
    }

    void getCodecDurationStats(float& out_avg_ms, float& out_peak_ms, bool& out_has_data) const {
        m_codec_duration_buf.getStats(out_avg_ms, out_peak_ms, out_has_data);
    }

    void getStageDurationStats(float& out_dsp, float& out_enc1, float& out_enc2, float& out_enc3, float& out_tx) const {
        float peak = 0.0f;
        bool has_data = false;
        m_dsp_duration_buf.getStats(out_dsp, peak, has_data);
        m_enc1_duration_buf.getStats(out_enc1, peak, has_data);
        m_enc2_duration_buf.getStats(out_enc2, peak, has_data);
        m_enc3_duration_buf.getStats(out_enc3, peak, has_data);
        m_tx_duration_buf.getStats(out_tx, peak, has_data);
    }

    void setStereo(bool stereo) { m_is_stereo = stereo; }
    bool isStereo() const { return m_is_stereo; }

    size_t   getFifoCount() const;
    size_t   getFifoCapacity() const;

    uint32_t getRedundancyRecoveredCount() const { return m_redundancy_recovered_packets.load(std::memory_order_relaxed); }
    uint32_t getAndResetRedundancyRecoveredCount() { return m_redundancy_recovered_packets.exchange(0, std::memory_order_relaxed); }

    uint32_t getTxTimeoutCount() const { return m_tx_timeout_count.load(std::memory_order_relaxed); }
    uint32_t getTxFailCount() const { return m_tx_fail_count.load(std::memory_order_relaxed); }
    uint32_t getTxMacErrorCount() const { return m_tx_mac_error_count.load(std::memory_order_relaxed); }
    uint32_t getTxDeadlineDropsCount() const { return m_tx_deadline_drops.load(std::memory_order_relaxed); }
    uint32_t getAndResetTxDeadlineDropsCount() { return m_tx_deadline_drops.exchange(0, std::memory_order_relaxed); }

    static SemaphoreHandle_t s_tx_done_sem;
    static std::atomic<esp_now_send_status_t> s_last_tx_status;

private:
    static void IRAM_ATTR frameTimerCb(void* arg);
    static void audioDspTaskTrampoline(void* arg);
    static void sinkTaskTrampoline(void* arg);
    void runAudioDspLoop();
    void runSinkLoop();

    void handleAudioPacket(const vsaf_audio_packet_t* pkt, int8_t rssi, int64_t t_rx1_us);
    void sendSinkTelemetry(uint8_t ack_seq, uint32_t t_tx1_echo, int64_t t_rx1_us, int8_t rssi);

    Codec::Lc3CodecEngine&          m_lc3_codec;
    Audio::ToneGenerator*           m_tone_gen;
    Hardware::I2sAudioDriver*       m_i2s_dac;
    AudioMetering::AudioSignalMeter m_audio_meter;

    uint8_t                    m_node_role;
    uint8_t                    m_node_id;
    uint8_t                    m_wifi_channel;
    uint8_t                    m_target_channel;
    std::atomic<bool>          m_channel_locked{false};
    std::atomic<NetworkState>  m_state;
    std::atomic<bool>          m_running{false};
    StreamTelemetry            m_telemetry;
    bool                       m_tone_test_mode;
    bool                       m_is_stereo;

    wifi_phy_mode_t            m_tx_phy_mode;
    wifi_phy_rate_t            m_tx_phy_rate;
    static uint8_t             s_broadcast_mac[6];

    SinkPeerConfig             m_peers[MAX_SINK_NODES];
    int                        m_peer_count;
    portMUX_TYPE               m_peer_lock;
    bool                       m_sink_acked[MAX_SINK_NODES];

    uint8_t                    m_seq;
    uint16_t                   m_octets_per_frame;
    uint32_t                   m_frame_duration_us;
    vsaf_audio_packet_t        m_last_tx_pkt[MAX_SINK_NODES];
    uint8_t                    m_prev_encoded_channels[MAX_SINK_NODES][LC3_FRAME_OCTETS];
    bool                       m_prev_encoded_valid[MAX_SINK_NODES];

    std::atomic<uint8_t>       m_target_volume_u8;
    float                      m_target_gain_db;
    float                      m_current_gain_db;
    float                      m_post_gain_db{0.0f};
    bool                       m_instant_vol_update;

    esp_timer_handle_t         m_frame_timer{nullptr};
    TaskHandle_t               m_audio_dsp_task_handle{nullptr};
    TaskHandle_t               m_sink_task_handle{nullptr};

    std::atomic<uint32_t>      m_tx_timeout_count{0};
    std::atomic<uint32_t>      m_tx_fail_count{0};
    std::atomic<uint32_t>      m_tx_mac_error_count{0};
    std::atomic<uint32_t>      m_tx_deadline_drops{0};
    uint32_t                   m_consecutive_tx_timeouts{0};

    void handleTxSubsystemHang();

    // Single interleaved stereo PCM input capture buffer (480 stereo samples = 960 int16_t = 10 ms @ 48 kHz stereo)
    int16_t                    m_pcm_stereo_in[480 * 2];
    size_t                     m_pcm_in_samples{480};

    // Filtered PCM Channel Buffers
    int16_t                    m_pcm_left_hp[480];
    int16_t                    m_pcm_right_hp[480];

    // Subwoofer 8 kHz decimation buffer (80 samples = 10 ms @ 8 kHz)
    int16_t                    m_pcm_sub_8k[80];

    // Crossover DSP Filters (esp-dsp SIMD block accelerated)
    DSP::LinkwitzRiley4Stereo          m_hpf_stereo;
    DSP::SubwooferPolyphaseDecimator   m_sub_decimator;

    mutable SpscDurationRingBuffer<float, 64> m_codec_duration_buf;
    mutable SpscDurationRingBuffer<float, 64> m_dsp_duration_buf;
    mutable SpscDurationRingBuffer<float, 64> m_enc1_duration_buf;
    mutable SpscDurationRingBuffer<float, 64> m_enc2_duration_buf;
    mutable SpscDurationRingBuffer<float, 64> m_enc3_duration_buf;
    mutable SpscDurationRingBuffer<float, 64> m_tx_duration_buf;
    mutable TimeOffsetRingBuffer              m_time_offset_buf;
    float                                     m_ema_time_offset_ms;
    uint32_t                                  m_last_master_time_us;
    uint32_t                                  m_last_local_time_us;
    int8_t                                    m_last_rssi;

    std::atomic<uint32_t>      m_plc_count;
    std::atomic<uint32_t>      m_fifo_underflows;
    std::atomic<uint32_t>      m_fifo_overflows;
    std::atomic<uint32_t>      m_redundancy_recovered_packets{0};
    uint8_t                    m_last_rx_seq;
    bool                       m_first_packet_received;
    uint8_t                    m_expected_seq;
    bool                       m_has_expected_seq;
    std::atomic<int64_t>       m_last_rx_audio_pkt_us{0};
    std::atomic<uint32_t>      m_raw_espnow_rx_count{0};
    std::atomic<uint32_t>      m_raw_audio_pkt_count{0};
    std::atomic<uint8_t>       m_last_seen_rx_id{255};

    static constexpr size_t    SINK_FIFO_PACKETS = 24;
    struct SinkFifoItem {
        uint8_t  seq;
        uint8_t  len;
        uint8_t  flags;
        uint8_t  data[LC3_FRAME_OCTETS];
    };
    SinkFifoItem               m_sink_fifo[SINK_FIFO_PACKETS];
    size_t                     m_sink_fifo_head;
    size_t                     m_sink_fifo_tail;
    size_t                     m_sink_fifo_count;
    portMUX_TYPE               m_sink_fifo_lock;

    std::atomic<uint32_t>      m_tx_packets_total{0};
    std::atomic<uint32_t>      m_tx_packets_sec{0};
    std::atomic<uint32_t>      m_tx_acks_total{0};
    std::atomic<uint32_t>      m_tx_acks_sec{0};
    std::atomic<uint32_t>      m_tx_ack_fails_total{0};
    std::atomic<uint32_t>      m_tx_ack_fails_sec{0};
    std::atomic<uint32_t>      m_rx_packets_total{0};
    std::atomic<uint32_t>      m_rx_packets_sec{0};

    uint32_t                   m_tx_packets_this_sec{0};
    uint32_t                   m_rx_acks_this_sec{0};
    uint32_t                   m_stats_packets_per_sec{0};
    uint32_t                   m_stats_acks_per_sec{0};
    float                      m_stats_ack_percentage{100.0f};
};

using EspNowUnicastEngine = EspNowBroadcastEngine;

} // namespace AudioNet
