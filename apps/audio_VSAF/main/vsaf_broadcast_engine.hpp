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

/**
 * @brief Lock-free timing accumulator (microseconds). Any task may add(); one owner takes() a window.
 */
struct TimingAcc {
    std::atomic<uint32_t> sum{0};
    std::atomic<uint32_t> max{0};
    std::atomic<uint32_t> n{0};

    void add(uint32_t v) {
        sum.fetch_add(v, std::memory_order_relaxed);
        n.fetch_add(1, std::memory_order_relaxed);
        uint32_t m = max.load(std::memory_order_relaxed);
        while (v > m && !max.compare_exchange_weak(m, v, std::memory_order_relaxed)) {
        }
    }
    struct Snap {
        uint32_t sum = 0, max = 0, n = 0;
        uint32_t avg() const { return n ? sum / n : 0; }
    };
    Snap take() {
        Snap s;
        s.sum = sum.exchange(0, std::memory_order_relaxed);
        s.max = max.exchange(0, std::memory_order_relaxed);
        s.n = n.exchange(0, std::memory_order_relaxed);
        return s;
    }
};

/**
 * @brief SOURCE pipeline timing over one completed window (~1 s = 100 frames), all values in microseconds.
 *
 * Offsets ("_at") are measured from the frame-timer tick that started the frame.
 */
struct SourceTimingSnapshot {
    uint32_t frames = 0;              // frames processed in the window
    TimingAcc::Snap dsp;              // DSP stage (core 0) per frame
    TimingAcc::Snap enc_hq;           // HQ job: shared analysis + 48 kHz HQ frame (120 B)
    TimingAcc::Snap enc_mq;           // MQ job: 60 B quantize + write on the HQ job's analysis
    TimingAcc::Snap enc_sub;          // one 8 kHz subwoofer encode (60 B)
    uint32_t enc_busy_core_us[2] = {};// total encode CPU per core in the window
    uint32_t enc_jobs_core[2] = {};   // encode jobs executed per core
    TimingAcc::Snap tx_lat;           // packet submit -> TX-done callback (queue + airtime)
    TimingAcc::Snap tx_call;          // CPU time inside esp_now_send() (src_rf, core 1)
    TimingAcc::Snap tx_wake;          // TX-done callback -> src_rf running (task wake latency)
    TimingAcc::Snap tx_gap;           // radio idle between a TX-done and the next submit (waiting for encodes)
    TimingAcc::Snap tx_first_at;      // first packet submitted
    TimingAcc::Snap tx_last_done_at;  // last packet of the frame completed (critical path)
    TimingAcc::Snap jobs_done_at;     // all encode jobs of the frame finished
    uint32_t overruns = 0;            // frame ticks merged because core 0 was still busy (frame lost)
    uint32_t cancelled_jobs = 0;      // MQ/HQ jobs of the previous frame cancelled by a new frame
    uint32_t mq_missing = 0;          // packets built without a fresh t-1/t-2 MQ frame
    uint32_t deadline_skips = 0;      // packets not sent because the 9.5 ms TX deadline passed
    uint32_t tx_timeouts = 0;         // TX-done not received within the timeout
    uint32_t rf_wakes = 0;            // src_rf wake-ups in the window
    uint32_t mq_waits = 0;            // MQ jobs that had to wait for their HQ phase on the other core
    uint32_t rf_active_us = 0;        // src_rf time from wake to next sleep (code incl. esp_now_send)
    uint32_t free_internal_heap = 0;  // bytes, at window end
    uint32_t min_free_internal_heap = 0;
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
        uint32_t score;     // Activity decoded on this channel only. Lower is cleaner
        uint32_t adj_score; // score + overlap-weighted scores of channels within +/-4 (20 MHz overlap)
    };

    // preferred_ch: 0 = pure "cleanest wins"; 1..13 = keep this channel unless its adj_score > CONFIG_VSAF_CHANNEL_BUSY_SCORE
    uint8_t scanAndSelectBestChannel(uint32_t dwell_ms_per_ch = 120, bool auto_apply = true, bool print_results = true,
                                     uint8_t preferred_ch = 0);
    esp_err_t setWifiChannel(uint8_t channel);
    // Console-facing radio changes: deferred to a frame boundary on a running SOURCE (see .cpp)
    esp_err_t requestWifiChannel(uint8_t channel);
    esp_err_t requestWifiPhyRate(wifi_phy_mode_t phymode, wifi_phy_rate_t rate);
    uint32_t  getTxRecoveryCount() const { return m_tx_recoveries.load(std::memory_order_relaxed); }
    void      requestTxFaultInjection() { m_debug_break_tx.store(true, std::memory_order_release); } // test only
    uint8_t getWifiChannel() const { return m_wifi_channel; }
    bool isChannelLocked() const { return m_channel_locked.load(std::memory_order_acquire); }
    void lockChannel(bool locked) { m_channel_locked.store(locked, std::memory_order_release); }
    esp_err_t setWifiTxPower(int8_t power_0_25dbm);
    int8_t getWifiTxPower() const;  // TX power readback (0.25 dBm units, = requested), cached at the last change

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

    // SINK clock-drift compensation (single-sample insert / drop, see runSinkLoop)
    struct SinkDriftStats {
        uint32_t depth_avg_x10;   // buffered audio (FIFO + I2S DMA queue) after pop, window average (frames x 10)
        uint32_t depth_min;       // minimum FIFO depth (FIFO only) in the window (frames)
        uint32_t depth_n;         // frames in the window (0 = not streaming)
        int32_t  adj_window;      // net corrections in the window (+ inserted, - dropped samples)
        int32_t  adj_total;       // net corrections since STREAM start
        uint32_t frames_played;   // frames played since STREAM start
        int32_t  target_x10;      // learned target depth (frames x 10), -1 until learned
        bool     enabled;
    };
    /**
     * @brief Snapshot and reset the per-window SINK FIFO / drift statistics (called by the 1 s heartbeat).
     */
    void takeSinkDriftStats(SinkDriftStats& out) {
        uint32_t sum = m_fifo_depth_sum.exchange(0, std::memory_order_relaxed);
        out.depth_n = m_fifo_depth_n.exchange(0, std::memory_order_relaxed);
        out.depth_min = m_fifo_depth_min.exchange(UINT32_MAX, std::memory_order_relaxed);
        out.depth_avg_x10 = out.depth_n ? (sum + out.depth_n * 5) / (out.depth_n * 10) : 0;  // sum in frames x 100
        int32_t total = m_drift_adj_total.load(std::memory_order_relaxed);
        out.adj_window = total - m_drift_adj_reported;
        m_drift_adj_reported = total;
        out.adj_total = total;
        out.frames_played = m_frames_played.load(std::memory_order_relaxed);
        out.target_x10 = m_drift_target_x10.load(std::memory_order_relaxed);
        out.enabled = m_drift_corr_enabled.load(std::memory_order_relaxed);
    }
    template <typename T> static size_t driftInsertSample(T* x, size_t n);
    template <typename T> static size_t driftDropSample(T* x, size_t n);
    void setDriftCorrection(bool on) { m_drift_corr_enabled.store(on, std::memory_order_relaxed); }
    bool getDriftCorrection() const { return m_drift_corr_enabled.load(std::memory_order_relaxed); }
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

    /**
     * @brief SOURCE heartbeat stage timings from the last completed ~1 s window (ms, per frame averages).
     * @param out_dsp      DSP stage on core 0.
     * @param out_enc_main CPU spent on the 5 HQ (analysis + 120 B) + 1 sub encodes (sum over both cores).
     * @param out_enc_red  CPU spent on the 5 MQ phases (60 B quantize, shared analysis).
     * @param out_tx       Offset of the frame's last TX completion from the frame tick (critical path, not CPU).
     */
    void getStageDurationStats(float& out_dsp, float& out_enc_main, float& out_enc_red, float& out_tx) const {
        SourceTimingSnapshot s;
        getSourceTiming(s);
        float frames = s.frames ? static_cast<float>(s.frames) : 1.0f;
        out_dsp = s.dsp.avg() / 1000.0f;
        out_enc_main = (s.enc_hq.sum + s.enc_sub.sum) / 1000.0f / frames;
        out_enc_red = s.enc_mq.sum / 1000.0f / frames;
        out_tx = s.tx_last_done_at.avg() / 1000.0f;
    }

    // SOURCE pipeline diagnostics / test knobs
    void getSourceTiming(SourceTimingSnapshot& out) const {
        portENTER_CRITICAL(&m_src_timing_mux);
        out = m_src_timing_snap;
        portEXIT_CRITICAL(&m_src_timing_mux);
    }
    // Boot-time memory / task report (console 'mem')
    uint32_t m_boot_heap_before_tasks = 0;
    uint32_t m_boot_heap_after_tasks = 0;
    bool     m_boot_task_ok[3] = {false, false, false}; // src_rf, src_enc1, src_dsp0
    TaskHandle_t getSrcTaskHandle(int i) const {
        return i == 0 ? m_src_rf_task : (i == 1 ? m_src_enc_task : m_audio_dsp_task_handle);
    }
    void setDspExtraLoadUs(uint32_t us) { m_src_dsp_extra_us.store(us, std::memory_order_relaxed); }
    uint32_t getDspExtraLoadUs() const { return m_src_dsp_extra_us.load(std::memory_order_relaxed); }
    // Debug 'encbench [core] [irq]': LC3 encode cost per test signal on a scratch encoder, in a top-priority task on
    // `core`; irq_masked = pure CPU cost, otherwise ISR load included. Prints through `print` (printf-style).
    void runEncoderBench(void (*print)(const char*, ...), int core, bool irq_masked);
    void srcWakeRfIfWaiting(uint32_t bit);
    // Debug: 1 = only core 0 encodes (src_enc1 idles), 2 = both cores (default). Isolates dual-core contention.
    void setEncoderCores(uint32_t n) { m_src_enc_cores.store(n == 1 ? 1 : 2, std::memory_order_relaxed); }
    uint32_t getEncoderCores() const { return m_src_enc_cores.load(std::memory_order_relaxed); }
    // Called from the ESP-NOW send callback (Wi-Fi task context)
    void onSrcTxDone(esp_now_send_status_t status);

    void setStereo(bool stereo) { m_is_stereo = stereo; }
    bool isStereo() const { return m_is_stereo; }

    size_t   getFifoCount() const;
    size_t   getFifoCapacity() const;

    uint32_t getRedundancyRecoveredCount() const { return m_redundancy_recovered_packets.load(std::memory_order_relaxed); }
    uint32_t getAndResetRedundancyRecoveredCount() { return m_redundancy_recovered_packets.exchange(0, std::memory_order_relaxed); }

    uint32_t getTxTimeoutCount() const { return m_tx_timeout_count.load(std::memory_order_relaxed); }
    uint32_t getTxFailCount() const { return m_tx_fail_count.load(std::memory_order_relaxed); }
    uint32_t getTxMacErrorCount() const { return m_tx_mac_error_count.load(std::memory_order_relaxed); }
    int32_t  getLastTxError() const { return m_last_tx_err.load(std::memory_order_relaxed); }
    uint32_t getTxNoMemCount() const { return m_tx_err_no_mem.load(std::memory_order_relaxed); }
    uint32_t getTxDeadlineDropsCount() const { return m_tx_deadline_drops.load(std::memory_order_relaxed); }
    uint32_t getAndResetTxDeadlineDropsCount() { return m_tx_deadline_drops.exchange(0, std::memory_order_relaxed); }

    static SemaphoreHandle_t s_tx_done_sem;
    static std::atomic<esp_now_send_status_t> s_last_tx_status;

private:
    static void IRAM_ATTR frameTimerCb(void* arg);
    static void sinkTaskTrampoline(void* arg);
    void runSinkLoop();

    // ---------------- SOURCE pipeline (vsaf_source_pipeline.cpp) ----------------
    // core 0 "src_dsp0" (prio 5): USB read + DSP, publishes the frame, then encodes jobs
    // core 1 "src_enc1" (prio 5): encodes jobs of the published frame
    // core 1 "src_rf"   (prio 7): event-driven RF owner: radio health, one packet in flight, timestamp at submit
    // HQ = shared analysis + HQ frame (lc3_encode_dual_begin), MQ = MQ frame of that analysis (_end)
    enum class SrcJobType : uint8_t { SUB, HQ, MQ };
    struct SrcJob {
        SrcJobType type;
        uint8_t ch; // satellite 0..4, or SUBWOOFER_CHANNEL_ID
    };
    static constexpr uint32_t SRC_NUM_JOBS = 1 + 2 * NUM_SATELLITE_CHANNELS; // sub + 5 HQ + 5 MQ
    static constexpr uint32_t SRC_NUM_PKTS = MAX_SINK_NODES;                 // 5 satellites + sub
    struct SrcFrame {
        uint32_t frame_no = 0;  // monotonic frame counter (seq = frame_no & 0xFF)
        int64_t  t0_us = 0;     // frame-timer tick
        size_t   samples = 480;
        uint8_t  ack_ch = 0;    // channel whose packet requests the telemetry reply (sent last)
        SrcJob   jobs[SRC_NUM_JOBS] = {};
        uint8_t  tx_order[SRC_NUM_PKTS] = {};
        std::atomic<uint32_t> next_job{SRC_NUM_JOBS}; // claim cursor; >= SRC_NUM_JOBS = nothing to claim
        std::atomic<uint32_t> jobs_done{0};
        std::atomic<uint32_t> in_flight{0};
        std::atomic<uint8_t>  pkt_ready[SRC_NUM_PKTS] = {};
        std::atomic<uint8_t>  hq_done[NUM_SATELLITE_CHANNELS] = {}; // HQ phase finished (MQ may run)
    };
    static constexpr uint32_t SRC_BIT_TICK   = 1u << 0; // frame timer fired (radio health / pending changes)
    static constexpr uint32_t SRC_BIT_FRAME  = 1u << 1; // core 0 published a new frame
    static constexpr uint32_t SRC_BIT_PKT    = 1u << 2; // a packet became ready
    static constexpr uint32_t SRC_BIT_TXDONE = 1u << 3; // ESP-NOW send callback
    static constexpr int64_t  SRC_TX_DEADLINE_US     = 9500; // no new submit after this offset in the frame
    static constexpr int64_t  SRC_ACK_HOLD_US        = 8000; // send the reply-request packet even if others are late
    static constexpr int64_t  SRC_TX_DONE_TIMEOUT_US = 2500; // give up waiting for a TX-done callback
    static constexpr uint32_t SRC_TX_MIN_PLAUSIBLE_US = 60;  // a 248 B broadcast cannot complete faster on air
    static constexpr uint32_t SRC_TIMING_WINDOW_FRAMES = 100;

    static void srcDspTaskTrampoline(void* arg);
    static void srcEncTaskTrampoline(void* arg);
    static void srcRfTaskTrampoline(void* arg);
    void runSrcDspLoop();
    void runSrcEncLoop();
    void runSrcRfLoop();
    void srcPrepareFrame(SrcFrame& f, uint32_t frame_no, int64_t t0_us, size_t samples);
    void srcRunDsp(SrcFrame& f);
    void srcWorkJobs(SrcFrame& f);
    void srcDoJob(SrcFrame& f, const SrcJob& job);
    void srcCopyMqHistory(uint32_t frame_no, uint8_t ch, uint8_t* t_prev1, uint8_t* t_prev2);
    void srcRotateTimingWindow();

    TaskHandle_t               m_src_enc_task{nullptr};
    TaskHandle_t               m_src_rf_task{nullptr};
    SrcFrame                   m_src_frames[2];                          // by frame_no & 1
    std::atomic<uint32_t>      m_src_pub_frame_no{0};                    // last frame published by core 0
    std::atomic<int64_t>       m_src_tick_us{0};                         // time of the last frame-timer tick
    int16_t                    m_src_pcm_sat[2][NUM_SATELLITE_CHANNELS][480];
    int16_t                    m_src_pcm_sub[2][80];
    vsaf_audio_packet_t        m_src_pkt_sat[2][NUM_SATELLITE_CHANNELS];
    vsaf_sub_packet_t          m_src_pkt_sub[2];
    uint8_t                    m_src_mq[3][NUM_SATELLITE_CHANNELS][LC3_FRAME_OCTETS_RED]; // MQ(frame) by frame_no % 3
    std::atomic<uint32_t>      m_src_mq_tag[3][NUM_SATELLITE_CHANNELS] = {};               // frame_no that wrote it
    uint8_t                    m_src_sub_hist[4][LC3_FRAME_OCTETS_SUB];                     // sub t0 by frame_no % 4
    std::atomic<uint32_t>      m_src_sub_tag[4] = {};
    std::atomic<uint32_t>      m_src_enc_cores{2};                       // debug: encoder cores (1 or 2)
    std::atomic<uint32_t>      m_src_dsp_extra_us{0};                    // synthetic DSP load (headroom test)
    // TX-done hand-off (Wi-Fi task -> RF task)
    std::atomic<bool>          m_src_tx_done_flag{false};
    std::atomic<bool>          m_src_rf_wants_pkt{false};               // src_rf idle, waiting for a packet / frame
    std::atomic<uint32_t>      m_acc_rf_wakes{0}, m_acc_rf_active_us{0}, m_acc_mq_waits{0};
    std::atomic<int64_t>       m_src_tx_done_us{0};
    std::atomic<int>           m_src_tx_status{0};
    // Timing accumulators (current window) and the last completed window
    TimingAcc                  m_acc_dsp, m_acc_enc_hq, m_acc_enc_mq, m_acc_enc_sub;
    TimingAcc                  m_acc_tx_lat, m_acc_tx_call, m_acc_tx_wake, m_acc_tx_gap, m_acc_tx_first, m_acc_tx_last, m_acc_jobs_done;
    std::atomic<uint32_t>      m_acc_enc_busy_us[2] = {};
    std::atomic<uint32_t>      m_acc_enc_jobs[2] = {};
    std::atomic<uint32_t>      m_acc_frames{0}, m_acc_overruns{0}, m_acc_cancelled{0}, m_acc_mq_missing{0};
    std::atomic<uint32_t>      m_acc_deadline_skips{0}, m_acc_tx_timeouts{0};
    uint32_t                   m_src_tx_pkts_in_window{0}, m_src_tx_fast_pkts{0}, m_src_tx_attempts_in_window{0};
    mutable portMUX_TYPE       m_src_timing_mux = portMUX_INITIALIZER_UNLOCKED;
    SourceTimingSnapshot       m_src_timing_snap;

    void handleAudioPacket(const uint8_t* data, size_t data_len, int8_t rssi, int64_t t_rx1_us);
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
    std::atomic<int32_t>       m_last_tx_err{0};      // last esp_now_send() error (esp_err_t), 0 = none yet
    std::atomic<uint32_t>      m_tx_err_no_mem{0};    // esp_now_send() == ESP_ERR_ESPNOW_NO_MEM (driver TX queue full)
    std::atomic<uint32_t>      m_tx_deadline_drops{0};
    uint32_t                   m_consecutive_tx_timeouts{0};

    // TX path health / recovery (SOURCE TX task only, except the atomics)
    static constexpr uint32_t  TX_HEALTH_WINDOW_FRAMES   = 50; // 0.5 s: zero successful TX-done in a window -> wedged
    static constexpr uint32_t  TX_MAX_CONSECUTIVE_ERRORS = 30; // 5 frames of esp_now_send() errors -> wedged
    static constexpr float     TX_MIN_PLAUSIBLE_SWEEP_MS = 0.5f; // 6 broadcasts cannot finish faster on air
    uint32_t                   m_consecutive_send_errors{0};
    uint32_t                   m_tx_ok_in_window{0};
    uint32_t                   m_tx_window_frames{0};
    std::atomic<uint32_t>      m_tx_recoveries{0};
    esp_err_t initEspNowLayer();
    void recoverEspNowTx(const char* why);
    void applyPendingRadioChanges();

    // Deferred radio change requests (console task -> SOURCE TX task)
    std::atomic<uint8_t>       m_pending_channel{0};   // 0 = none
    std::atomic<bool>          m_debug_break_tx{false};
    std::atomic<bool>          m_pending_phy{false};
    wifi_phy_mode_t            m_pending_phy_mode{WIFI_PHY_MODE_HT20};
    wifi_phy_rate_t            m_pending_phy_rate{WIFI_PHY_RATE_MCS3_LGI};

    void handleTxSubsystemHang();

    // Single interleaved stereo PCM input capture buffer (480 stereo samples = 960 int16_t = 10 ms @ 48 kHz stereo)
    int16_t                    m_pcm_stereo_in[480 * 2];
    size_t                     m_pcm_in_samples{480};

    // Filtered PCM Channel Buffers

    // Subwoofer 8 kHz decimation buffer (80 samples = 10 ms @ 8 kHz)

    // Crossover DSP Filters (esp-dsp SIMD block accelerated)
    DSP::LinkwitzRiley4Stereo          m_hpf_stereo;
    DSP::SubwooferPolyphaseDecimator   m_sub_decimator;

    mutable SpscDurationRingBuffer<float, 64> m_codec_duration_buf;
    mutable TimeOffsetRingBuffer              m_time_offset_buf;
    float                                     m_ema_time_offset_ms;
    uint32_t                                  m_last_master_time_us;
    uint32_t                                  m_last_local_time_us;
    int8_t                                    m_last_rssi;

    std::atomic<uint32_t>      m_plc_count;
    std::atomic<uint32_t>      m_fifo_underflows;
    // SINK drift compensation state / telemetry (written by the SINK task, read by the heartbeat)
    std::atomic<uint32_t>      m_fifo_depth_sum{0}, m_fifo_depth_n{0}, m_fifo_depth_min{UINT32_MAX};
    std::atomic<int32_t>       m_drift_adj_total{0};      // + inserted / - dropped samples since STREAM start
    int32_t                    m_drift_adj_reported = 0;  // heartbeat-side copy for the per-window delta
    std::atomic<uint32_t>      m_frames_played{0};
    std::atomic<int32_t>       m_drift_target_x10{-1};
    std::atomic<bool>          m_drift_corr_enabled{true};
    // Wi-Fi TX power, read back from ESP-IDF once per change (setWifiTxPower), never polled
    std::atomic<int8_t>        m_tx_power_req_qdbm{0};     // 0.25 dBm units
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
        bool     is_redundant;
        uint8_t  data[LC3_FRAME_OCTETS_HQ];
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
