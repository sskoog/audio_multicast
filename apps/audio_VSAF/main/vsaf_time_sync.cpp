// ---------------------------------------------------------------------------
// VSAF time synchronization (SOURCE master clock -> every SINK)
//
// Exchange (PTP-like delay request / response, two-step), one per REQ_ACK packet, i.e. every 6th frame per SINK:
//   t1  SOURCE TX-done time of the REQ_ACK audio packet          (recorded by src_rf, indexed by seq)
//   t2  SINK receive time of that packet                         (reply field t_rx_us)
//   t3  SINK TX-done time of its reply = submit (t2 + dwell) + submit->TX-done delay, which is only known after
//       the reply has left; the SINK reports it in its NEXT reply (prev_tx_delay_us), so each exchange is
//       completed one reply later
//   t4  SOURCE receive time of the reply
//   offset (SINK - SOURCE) = (t2 - t1) - rtt / 2,  rtt = (t2 - t1) + (t4 - t3)
// All four timestamps are "end of frame" events taken in the ESP-NOW callbacks, so the remaining bias is the
// difference of callback latencies (constant per board type). Queueing / channel access only ever adds delay,
// so the SOURCE keeps the last 16 samples per channel and uses the one with the smallest rtt within 1 s,
// projected to now with a drift estimate (slope of the best offset over >= 5 s). Lost replies only cost samples.
//
// Every 100 ms the SOURCE broadcasts a control frame with offset / drift / rtt / trim per channel and the playout
// latency. A SINK converts the presentation time of frame N (t0(N) + latency + trim, SOURCE clock) into its own
// clock with offset + drift * (t - t_ref) and schedules the frame's first sample on it (runSinkLoop).
// All timestamp arithmetic is modulo 2^32.
// ---------------------------------------------------------------------------
#include "vsaf_broadcast_engine.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cstring>

static const char* TAG = "VSAF_SYNC";

namespace AudioNet {

namespace {
constexpr uint32_t PTP_WINDOW_US = 1000000;        // samples older than this are not used
constexpr int32_t  PTP_MAX_RTT_US = 20000;         // implausible samples are dropped
constexpr int32_t  PTP_DRIFT_BASELINE_US = 5000000; // drift is measured over at least 5 s
constexpr int64_t  PTP_MAX_DRIFT_PPB = 200000;     // 200 ppm: anything larger is a glitch
constexpr uint32_t PTP_HOLDOVER_US = 30000000;     // no fresh sample: project the newest one with the drift
constexpr int64_t  SINK_SYNC_MAX_AGE_US = 30000000; // holdover on the last control frame (drift-projected)
// Holdover accuracy: a drift estimate good to ~0.1 ppm keeps the projection within ~3 us over 30 s. Seen
// 2026-10-25: 5 s of heavy air congestion cost ch 1 / ch 5 all replies for > 1 s; with a 1 s limit the SOURCE
// marked them invalid and Node 24 dropped to unsynced playout for 1 s.
}  // namespace

// ---------------------------------------------------------------------------
// SOURCE side
// ---------------------------------------------------------------------------

/**
 * @brief Process one telemetry reply: complete the previous exchange (two-step) and store this one as pending.
 *
 * Called from the ESP-NOW receive callback (Wi-Fi task).
 *
 * @param t  Reply (CRC already checked).
 * @param t4 SOURCE receive time of the reply (low 32 bits of esp_timer).
 */
void EspNowBroadcastEngine::ptpOnTelemetry(const vsaf_sink_telemetry_t* t, uint32_t t4) {
    const uint8_t ch = t->sink_id;
    if (ch >= MAX_SINK_NODES) {
        return;
    }
    const uint8_t s = t->ack_seq;
    const uint32_t f_tag = m_ptp_t1_frame[s].load(std::memory_order_acquire);
    const uint32_t t1 = m_ptp_t1[s].load(std::memory_order_relaxed);
    const int32_t age = static_cast<int32_t>(t4 - t1);
    const bool t1_ok = f_tag != 0 && static_cast<uint8_t>(f_tag) == s && age > 0 && age < 50000;

    portENTER_CRITICAL(&m_ptp_mux);
    PtpChannel& c = m_ptp[ch];
    c.playout_err_us = t->playout_err_us;
    if (c.pend_valid) {
        if ((t->flags & VSAF_TELEM_FLAG_PREV_DELAY_VALID) && t->prev_ack_seq == c.pend_seq &&
            static_cast<int32_t>(t4 - c.pend_t4) < 500000) {
            const uint32_t t3 = c.pend_t3s + t->prev_tx_delay_us;
            const uint32_t a = c.pend_t2 - c.pend_t1;  // offset + downlink delay (modulo 2^32)
            const uint32_t b = c.pend_t4 - t3;         // -offset + uplink delay
            const int32_t rtt = static_cast<int32_t>(a + b);
            if (rtt > 0 && rtt < PTP_MAX_RTT_US) {
                PtpSample& smp = c.s[c.head];
                smp.t_ref = c.pend_t1;
                smp.rtt = rtt;
                smp.theta = static_cast<int32_t>(a - static_cast<uint32_t>(rtt / 2));
                c.head = static_cast<uint8_t>((c.head + 1) % PTP_SAMPLES);
                if (c.count < PTP_SAMPLES) c.count++;
                c.samples_total++;
            }
        } else {
            c.twostep_miss++;
        }
        c.pend_valid = false;
    }
    if (t1_ok) {
        c.pend_valid = true;
        c.pend_seq = s;
        c.pend_t1 = t1;
        c.pend_t2 = t->t_rx_us;
        c.pend_t3s = t->t_rx_us + t->t_dwell_us;
        c.pend_t4 = t4;
    } else {
        c.t1_miss++;
    }
    portEXIT_CRITICAL(&m_ptp_mux);
}

/**
 * @brief Best offset estimate of one channel (m_ptp_mux held): smallest-rtt sample within the window, projected
 *        to now32 with the drift estimate. Also updates the drift estimate.
 *
 * @param theta_now [out] SINK - SOURCE at now32 (us, modulo 2^32).
 * @param rtt       [out] round trip of the sample used (us).
 * @param n         [out] samples within the window.
 * @return false if there is no sample within the window.
 */
bool EspNowBroadcastEngine::ptpBest(PtpChannel& c, uint32_t now32, int32_t& theta_now, uint16_t& rtt, uint8_t& n) {
    int best = -1;
    n = 0;
    for (int i = 0; i < c.count; ++i) {
        const PtpSample& smp = c.s[i];
        const int32_t age = static_cast<int32_t>(now32 - smp.t_ref);
        if (age < 0 || age > static_cast<int32_t>(PTP_WINDOW_US)) {
            continue;
        }
        n++;
        if (best < 0 || smp.rtt < c.s[best].rtt) {
            best = i;
        }
    }
    bool holdover = false;
    if (best < 0) {
        // Holdover: newest sample within PTP_HOLDOVER_US, only with a drift estimate to project it
        for (int i = 0; i < c.count; ++i) {
            const int32_t age = static_cast<int32_t>(now32 - c.s[i].t_ref);
            if (age < 0 || age > static_cast<int32_t>(PTP_HOLDOVER_US)) {
                continue;
            }
            if (best < 0 || static_cast<int32_t>(c.s[i].t_ref - c.s[best].t_ref) > 0) {
                best = i;
            }
        }
        if (best < 0 || !c.drift_valid) {
            return false;
        }
        holdover = true;
        c.holdover_uses++;
    }
    const PtpSample& b = c.s[best];

    // Drift: slope of the best offset over >= 5 s, smoothed (not from a held-over sample)
    if (holdover) {
        // keep the drift estimate as it is
    } else if (!c.drift_ref_valid) {
        c.drift_ref_valid = true;
        c.drift_ref_t = b.t_ref;
        c.drift_ref_theta = b.theta;
    } else {
        const int32_t dt = static_cast<int32_t>(b.t_ref - c.drift_ref_t);
        if (dt >= PTP_DRIFT_BASELINE_US) {
            const int32_t dth = static_cast<int32_t>(static_cast<uint32_t>(b.theta) - static_cast<uint32_t>(c.drift_ref_theta));
            const int64_t ppb = static_cast<int64_t>(dth) * 1000000000LL / dt;
            if (ppb > -PTP_MAX_DRIFT_PPB && ppb < PTP_MAX_DRIFT_PPB) {
                if (!c.drift_valid) {
                    c.drift_ppb = static_cast<int32_t>(ppb);
                    c.drift_valid = true;
                } else {
                    c.drift_ppb += static_cast<int32_t>((ppb - c.drift_ppb) / 4);
                }
            }
            c.drift_ref_t = b.t_ref;
            c.drift_ref_theta = b.theta;
        } else if (dt < 0) {
            c.drift_ref_t = b.t_ref;  // stale reference (time wrap or restart)
            c.drift_ref_theta = b.theta;
        }
    }
    const int32_t age = static_cast<int32_t>(now32 - b.t_ref);
    const int32_t proj = c.drift_valid ? static_cast<int32_t>(static_cast<int64_t>(c.drift_ppb) * age / 1000000000LL) : 0;
    theta_now = static_cast<int32_t>(static_cast<uint32_t>(b.theta) + static_cast<uint32_t>(proj));
    rtt = static_cast<uint16_t>(std::clamp<int32_t>(b.rtt, 1, 65535));
    return true;
}

/**
 * @brief Fill the 100 ms control frame (called by src_rf just before sending it).
 */
void EspNowBroadcastEngine::srcBuildControl(vsaf_control_t& out) {
    const uint32_t now32 = static_cast<uint32_t>(esp_timer_get_time());
    out = {};
    out.type_id = VSAF_TYPE_CONTROL;
    out.version = VSAF_CONTROL_VERSION;
    out.seq = m_ctrl_seq++;
    out.t_ref_us = now32;
    out.latency_us = m_latency_us.load(std::memory_order_relaxed);
    for (size_t ch = 0; ch < MAX_SINK_NODES; ++ch) {
        int32_t theta = 0;
        uint16_t rtt = 0;
        uint8_t n = 0;
        portENTER_CRITICAL(&m_ptp_mux);
        const bool ok = ptpBest(m_ptp[ch], now32, theta, rtt, n);
        const int32_t drift = m_ptp[ch].drift_valid ? m_ptp[ch].drift_ppb : 0;
        portEXIT_CRITICAL(&m_ptp_mux);
        out.offset_us[ch] = ok ? theta : 0;
        out.drift_ppb[ch] = ok ? drift : 0;
        out.rtt_us[ch] = ok ? rtt : 0;
        out.trim_us[ch] = m_trim_us[ch].load(std::memory_order_relaxed);
    }
    out.crc16 = calc_crc16(reinterpret_cast<const uint8_t*>(&out), sizeof(out) - 4);
}

void EspNowBroadcastEngine::getPtpStatus(uint8_t ch, PtpChannelStatus& out) {
    out = {};
    if (ch >= MAX_SINK_NODES) {
        return;
    }
    const uint32_t now32 = static_cast<uint32_t>(esp_timer_get_time());
    portENTER_CRITICAL(&m_ptp_mux);
    PtpChannel& c = m_ptp[ch];
    out.valid = ptpBest(c, now32, out.offset_us, out.rtt_us, out.samples);
    out.drift_ppb = c.drift_valid ? c.drift_ppb : 0;
    out.samples_total = c.samples_total;
    out.twostep_miss = c.twostep_miss;
    out.t1_miss = c.t1_miss;
    out.holdover_uses = c.holdover_uses;
    out.playout_err_us = c.playout_err_us;
    portEXIT_CRITICAL(&m_ptp_mux);
    out.trim_us = m_trim_us[ch].load(std::memory_order_relaxed);
}

void EspNowBroadcastEngine::setChannelTrimUs(uint8_t ch, int16_t us) {
    if (ch < MAX_SINK_NODES) {
        m_trim_us[ch].store(us, std::memory_order_relaxed);
    }
}

// ---------------------------------------------------------------------------
// SINK side
// ---------------------------------------------------------------------------

/**
 * @brief Control frame from the SOURCE (Wi-Fi task): take over the estimate for our channel.
 */
void EspNowBroadcastEngine::sinkOnControl(const uint8_t* data, int len) {
    if (len < static_cast<int>(sizeof(vsaf_control_t))) {
        return;
    }
    vsaf_control_t c;
    memcpy(&c, data, sizeof(c));
    if (c.version != VSAF_CONTROL_VERSION ||
        calc_crc16(reinterpret_cast<const uint8_t*>(&c), sizeof(c) - 4) != c.crc16) {
        return;
    }
    const uint8_t ch = m_target_channel;
    if (ch >= MAX_SINK_NODES) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&m_sync_mux);
    m_sync.ctrl_frames++;
    m_sync.latency = c.latency_us;
    m_sync.trim = c.trim_us[ch];
    if (c.rtt_us[ch] != 0) {
        m_sync.valid = true;
        m_sync.offset = c.offset_us[ch];
        m_sync.t_ref = c.t_ref_us;
        m_sync.drift = c.drift_ppb[ch];
        m_sync.rtt = c.rtt_us[ch];
        m_sync.last_ctrl_us = now;
    }
    portEXIT_CRITICAL(&m_sync_mux);
}

/**
 * @brief Current clock offset (SINK - SOURCE) and playout parameters, if the sync is valid.
 *
 * @param now_local SINK esp_timer (us).
 * @return false if no estimate for our channel arrived within the last second.
 */
bool EspNowBroadcastEngine::sinkCurrentTheta(int64_t now_local, int32_t& theta, uint32_t& latency, int16_t& trim) {
    portENTER_CRITICAL(&m_sync_mux);
    const SinkSync s = m_sync;
    portEXIT_CRITICAL(&m_sync_mux);
    latency = s.latency;
    trim = s.trim;
    if (!s.valid || now_local - s.last_ctrl_us > SINK_SYNC_MAX_AGE_US) {
        return false;
    }
    const uint32_t master_now = static_cast<uint32_t>(now_local) - static_cast<uint32_t>(s.offset);
    const int32_t dt = static_cast<int32_t>(master_now - s.t_ref);
    const int32_t proj = static_cast<int32_t>(static_cast<int64_t>(s.drift) * dt / 1000000000LL);
    theta = static_cast<int32_t>(static_cast<uint32_t>(s.offset) + static_cast<uint32_t>(proj));
    return true;
}

void EspNowBroadcastEngine::getSinkSyncStatus(SinkSyncStatus& out, bool reset_window) {
    out = {};
    portENTER_CRITICAL(&m_sync_mux);
    const SinkSync s = m_sync;
    portEXIT_CRITICAL(&m_sync_mux);
    const int64_t now = esp_timer_get_time();
    out.sync_valid = s.valid && now - s.last_ctrl_us <= SINK_SYNC_MAX_AGE_US;
    out.aligned = m_sync_aligned.load(std::memory_order_relaxed);
    out.offset_us = m_sync_offset_used.load(std::memory_order_relaxed);
    out.drift_ppb = s.drift;
    out.ctrl_age_ms = s.valid ? static_cast<uint32_t>((now - s.last_ctrl_us) / 1000) : 0;
    out.latency_us = s.latency;
    out.trim_us = s.trim;
    out.rtt_us = s.rtt;
    out.err_filt_us = m_sync_err_filt.load(std::memory_order_relaxed);
    out.err_max_abs_us = reset_window ? m_sync_err_maxabs.exchange(0, std::memory_order_relaxed)
                                      : m_sync_err_maxabs.load(std::memory_order_relaxed);
    out.coarse_aligns = m_sync_coarse.load(std::memory_order_relaxed);
    out.fine_corrections = m_sync_fine.load(std::memory_order_relaxed);
    out.ctrl_frames = s.ctrl_frames;
    out.why_jump = m_sync_why[0].load(std::memory_order_relaxed);
    out.why_theta = m_sync_why[1].load(std::memory_order_relaxed);
    out.why_skip = m_sync_why[2].load(std::memory_order_relaxed);
    out.why_head = m_sync_why[3].load(std::memory_order_relaxed);
    out.why_pad = m_sync_why[4].load(std::memory_order_relaxed);
}

void EspNowBroadcastEngine::getRxTsProbe(int32_t& min_d, int32_t& max_d, uint32_t& n, bool reset) {
    if (reset) {
        min_d = m_rxts_min.exchange(INT32_MAX, std::memory_order_relaxed);
        max_d = m_rxts_max.exchange(INT32_MIN, std::memory_order_relaxed);
        n = m_rxts_n.exchange(0, std::memory_order_relaxed);
    } else {
        min_d = m_rxts_min.load(std::memory_order_relaxed);
        max_d = m_rxts_max.load(std::memory_order_relaxed);
        n = m_rxts_n.load(std::memory_order_relaxed);
    }
}

bool EspNowBroadcastEngine::takeSyncProbe(SyncProbe& out) {
    portENTER_CRITICAL(&m_probe_mux);
    out = m_probe;
    m_probe.rx_new = false;
    m_probe.play_new = false;
    portEXIT_CRITICAL(&m_probe_mux);
    return out.rx_new || out.play_new;
}

/**
 * @brief ESP-NOW send callback on a SINK (Wi-Fi task): TX-done of our telemetry reply -> two-step correction.
 */
void EspNowBroadcastEngine::onSinkTxDone() {
    if (!m_reply_inflight.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t d = now - m_reply_submit_us.load(std::memory_order_relaxed);
    m_prev_reply_delay_us.store(static_cast<uint16_t>(std::min<uint32_t>(d, 65535)), std::memory_order_relaxed);
    m_prev_reply_seq.store(m_reply_seq.load(std::memory_order_relaxed), std::memory_order_relaxed);
    m_prev_reply_valid.store(d < 65535, std::memory_order_release);
}

}  // namespace AudioNet
