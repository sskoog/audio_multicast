// ---------------------------------------------------------------------------
// VSAF2 SOURCE audio pipeline: one 10 ms frame across both ESP32-S3 cores.
//
//   esp_timer tick (core 0, prio 22) --notify--> src_dsp0 + src_rf
//
//   core 0  src_dsp0 (prio 5): USB read -> DSP -> publish frame -> encode jobs
//   core 1  src_enc1 (prio 5): encode jobs of the published frame
//   core 1  src_rf   (prio 7): RF owner, event driven (frame tick, packet ready, TX-done)
//
// Jobs per frame, claimed in this order by whichever core is free (lock-free cursor):
//   SUB, HQ x5 (rotating channel order), MQ x5 (same rotation)
// Satellites use two-phase dual-rate encoding (liblc3_dual): the HQ job runs the shared LC3 analysis and
// writes the 120 B frame (lc3_encode_dual_begin); the MQ job later quantizes the same analysis at 60 B
// (lc3_encode_dual_end), the redundancy that packets N+1 (t-1) and N+2 (t-2) carry. So HQ packets are ready
// as early as with plain encoders, and MQ costs only its quantization. HQ and SUB jobs build their VSAF
// packet completely except the TX timestamp. Jobs still unclaimed when core 0 publishes the next frame are
// cancelled (counted).
//
// The RF task keeps exactly one packet in flight: it submits the next ready packet the moment the previous
// TX-done arrives, writes t_tx1_us immediately before esp_now_send(), and never submits after the 9.5 ms
// deadline. The packet carrying the telemetry reply request (round robin) is sent last in the burst.
// Because nothing piles up in the driver queue, ESP_ERR_ESPNOW_NO_MEM cannot build up.
// ---------------------------------------------------------------------------
#include "vsaf_broadcast_engine.hpp"
#include "usb_audio.hpp"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "esp_cpu.h"
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "lc3.h"
#endif
#include <cmath>
#include <algorithm>
#include <cstring>

static const char* TAG = "VSAF_SRC";

namespace AudioNet {

void EspNowBroadcastEngine::srcDspTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSrcDspLoop();
}
void EspNowBroadcastEngine::srcEncTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSrcEncLoop();
}
void EspNowBroadcastEngine::srcRfTaskTrampoline(void* arg) {
    static_cast<EspNowBroadcastEngine*>(arg)->runSrcRfLoop();
}

// ---------------------------------------------------------------------------
// core 0: frame start, DSP, publish, encode
// ---------------------------------------------------------------------------
void EspNowBroadcastEngine::runSrcDspLoop() {
    ESP_LOGI(TAG, "src_dsp0 started on core %d (USB + DSP + LC3 jobs)", xPortGetCoreID());
    uint32_t frame_no = m_src_pub_frame_no.load(std::memory_order_acquire);

    while (true) {
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (ticks == 0) {
            continue;
        }
        if (ticks > 1) {
            // Ticks merged while core 0 was still busy: those frames were never produced
            m_acc_overruns.fetch_add(ticks - 1, std::memory_order_relaxed);
        }
        if (m_state.load(std::memory_order_acquire) != NetworkState::CAST) {
            continue;
        }

        int64_t t0_us = m_src_tick_us.load(std::memory_order_acquire);
        size_t samples = Codec::calculateRequiredPcmSamples(m_telemetry.sample_rate, m_frame_duration_us);
        samples = std::min<size_t>(samples, 480);

        if (++frame_no == 0) {
            frame_no = 1; // 0 means "never written" in the history tags
        }
        SrcFrame& f = m_src_frames[frame_no & 1];
        // The slot was last used by frame_no - 2. An encode of that frame still running here would mean it
        // took > 10 ms; give it a moment rather than overwrite its PCM input.
        for (int spin = 0; spin < 200 && f.in_flight.load(std::memory_order_acquire) != 0; ++spin) {
            esp_rom_delay_us(10);
        }

        srcPrepareFrame(f, frame_no, t0_us, samples);
        srcRunDsp(f);

        // MQ (and any HQ) jobs of the previous frame nobody has claimed yet are dropped now
        SrcFrame& prev = m_src_frames[(frame_no - 1) & 1];
        uint32_t old_cursor = prev.next_job.exchange(SRC_NUM_JOBS, std::memory_order_acq_rel);
        if (old_cursor < SRC_NUM_JOBS) {
            m_acc_cancelled.fetch_add(SRC_NUM_JOBS - old_cursor, std::memory_order_relaxed);
        }

        // Publish: jobs become claimable, core 1 and the RF task learn about the new frame
        f.next_job.store(0, std::memory_order_release);
        m_src_pub_frame_no.store(frame_no, std::memory_order_release);
        m_seq = static_cast<uint8_t>(frame_no & 0xFF);
        if (m_src_enc_task) {
            xTaskNotifyGive(m_src_enc_task);
        }
        srcWakeRfIfWaiting(SRC_BIT_FRAME);

        srcWorkJobs(f);
        m_acc_frames.fetch_add(1, std::memory_order_relaxed);
    }
}

/**
 * @brief Reset a frame slot and build its job list and TX order (core 0, before publishing).
 *
 * @param f        Slot (m_src_frames[frame_no & 1]).
 * @param frame_no Monotonic frame number (>= 1).
 * @param t0_us    Frame-timer tick time (us).
 * @param samples  PCM samples per channel for this frame (480 at 48 kHz / 10 ms).
 */
void EspNowBroadcastEngine::srcPrepareFrame(SrcFrame& f, uint32_t frame_no, int64_t t0_us, size_t samples) {
    f.next_job.store(SRC_NUM_JOBS, std::memory_order_relaxed); // closed until published
    f.jobs_done.store(0, std::memory_order_relaxed);
    for (auto& r : f.pkt_ready) {
        r.store(0, std::memory_order_relaxed);
    }
    for (auto& d : f.hq_done) {
        d.store(0, std::memory_order_relaxed);
    }
    f.frame_no = frame_no;
    f.t0_us = t0_us;
    // Frame time for the SINKs: the periodic esp_timer fires on an exact 10 ms grid, but the callback (and
    // so t0_us) jitters with the load on core 0. Round to the grid; re-anchor if the timer phase changed.
    if (m_src_grid_epoch_us < 0) {
        m_src_grid_epoch_us = t0_us;
    }
    {
        const int64_t period = 10000;
        const int64_t k = (t0_us - m_src_grid_epoch_us + period / 2) / period;
        int64_t grid = m_src_grid_epoch_us + k * period;
        if (t0_us - grid > 2000 || grid - t0_us > 2000) {
            m_src_grid_epoch_us = t0_us;  // timer restarted with a new phase
            grid = t0_us;
        }
        f.t_grid_us = grid;
        const int32_t dev = static_cast<int32_t>(t0_us > grid ? t0_us - grid : grid - t0_us);
        if (dev > m_src_grid_dev_max.load(std::memory_order_relaxed)) {
            m_src_grid_dev_max.store(dev, std::memory_order_relaxed);
        }
    }
    f.samples = samples;
    f.ack_ch = static_cast<uint8_t>(frame_no % SRC_NUM_PKTS);

    // Work order: SUB, then HQ, then MQ; satellite order rotates each frame so no channel is always last
    uint32_t k = 0;
    f.jobs[k++] = {SrcJobType::SUB, SUBWOOFER_CHANNEL_ID};
    for (uint32_t i = 0; i < NUM_SATELLITE_CHANNELS; ++i) {
        f.jobs[k++] = {SrcJobType::HQ, static_cast<uint8_t>((i + frame_no) % NUM_SATELLITE_CHANNELS)};
    }
    for (uint32_t i = 0; i < NUM_SATELLITE_CHANNELS; ++i) {
        f.jobs[k++] = {SrcJobType::MQ, static_cast<uint8_t>((i + frame_no) % NUM_SATELLITE_CHANNELS)};
    }

    // TX order follows readiness (sub, then HQ rotation), with the reply-request channel moved to the end
    uint8_t ready_order[SRC_NUM_PKTS];
    ready_order[0] = SUBWOOFER_CHANNEL_ID;
    for (uint32_t i = 0; i < NUM_SATELLITE_CHANNELS; ++i) {
        ready_order[1 + i] = static_cast<uint8_t>((i + frame_no) % NUM_SATELLITE_CHANNELS);
    }
    uint32_t w = 0;
    for (uint8_t ch : ready_order) {
        if (ch != f.ack_ch) {
            f.tx_order[w++] = ch;
        }
    }
    f.tx_order[w] = f.ack_ch;
}

/**
 * @brief USB/tone input and DSP for one frame (core 0). Produces the 5 satellite PCM streams and the
 * 8 kHz sub stream for this frame's parity buffer.
 *
 * Current DSP: stereo LR4 high-pass 100 Hz for the satellites, 2-stage polyphase decimation + LR4 low-pass
 * for the sub. Satellite mapping is a placeholder until the spatial upmix exists: ch0 L, ch1 R, ch2 L,
 * ch3 L, ch4 R (encoded as 5 independent streams, so CPU cost is already realistic).
 * `dspload <us>` adds a synthetic busy load to test headroom for future spatial DSP.
 */
void EspNowBroadcastEngine::srcRunDsp(SrcFrame& f) {
    static float s_stereo_in_f32[480 * 2];
    static float s_stereo_hp_f32[480 * 2];
    static float s_mono_in_f32[480];
    static float s_sub_8k_f32[80];

    const int64_t t_start = esp_timer_get_time();
    const size_t samples = f.samples;
    const uint32_t par = f.frame_no & 1;

    if (m_tone_test_mode) {
        static int16_t tone_mono[480];
        if (m_tone_gen) {
            m_tone_gen->generateFrame(tone_mono, samples);
        } else {
            memset(tone_mono, 0, sizeof(tone_mono));
        }
        for (size_t i = 0; i < samples; ++i) {
            m_pcm_stereo_in[i * 2] = tone_mono[i];
            m_pcm_stereo_in[i * 2 + 1] = tone_mono[i];
        }
    } else {
        size_t bytes_needed = samples * 2 * sizeof(int16_t);
        size_t bytes_read = usb_audio_read_pcm(m_pcm_stereo_in, bytes_needed);
        if (bytes_read < bytes_needed) {
            memset(reinterpret_cast<uint8_t*>(m_pcm_stereo_in) + bytes_read, 0, bytes_needed - bytes_read);
        }
    }
    const bool stereo = m_is_stereo || m_tone_test_mode;

    for (size_t i = 0; i < samples; ++i) {
        float l = static_cast<float>(m_pcm_stereo_in[i * 2]);
        float r = stereo ? static_cast<float>(m_pcm_stereo_in[i * 2 + 1]) : l;
        s_stereo_in_f32[i * 2] = l;
        s_stereo_in_f32[i * 2 + 1] = r;
        s_mono_in_f32[i] = 0.5f * (l + r);
    }
    m_hpf_stereo.processStereo(s_stereo_in_f32, s_stereo_hp_f32, static_cast<int>(samples));
    m_sub_decimator.process(s_mono_in_f32, s_sub_8k_f32);

    int16_t* left = m_src_pcm_sat[par][0];
    int16_t* right = m_src_pcm_sat[par][1];
    for (size_t i = 0; i < samples; ++i) {
        left[i] = static_cast<int16_t>(std::clamp(s_stereo_hp_f32[i * 2], -32768.0f, 32767.0f));
        right[i] = static_cast<int16_t>(std::clamp(s_stereo_hp_f32[i * 2 + 1], -32768.0f, 32767.0f));
    }
    memcpy(m_src_pcm_sat[par][2], left, samples * sizeof(int16_t));  // Center (placeholder)
    memcpy(m_src_pcm_sat[par][3], left, samples * sizeof(int16_t));  // Surround L (placeholder)
    memcpy(m_src_pcm_sat[par][4], right, samples * sizeof(int16_t)); // Surround R (placeholder)
    for (size_t i = 0; i < 80; ++i) {
        m_src_pcm_sub[par][i] = static_cast<int16_t>(std::clamp(s_sub_8k_f32[i], -32768.0f, 32767.0f));
    }

    uint32_t extra = m_src_dsp_extra_us.load(std::memory_order_relaxed);
    if (extra > 0) {
        esp_rom_delay_us(extra); // synthetic DSP load (headroom test)
    }

    m_audio_meter.pushFramePcm(left, samples, 1);
    m_acc_dsp.add(static_cast<uint32_t>(esp_timer_get_time() - t_start));
}

// ---------------------------------------------------------------------------
// core 1: encoder
// ---------------------------------------------------------------------------
void EspNowBroadcastEngine::runSrcEncLoop() {
    ESP_LOGI(TAG, "src_enc1 started on core %d (LC3 jobs)", xPortGetCoreID());
    uint32_t worked_no = 0;
    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        // Work on the newest published frame; if another frame was published meanwhile, continue with it
        while (true) {
            uint32_t pub = m_src_pub_frame_no.load(std::memory_order_acquire);
            if (pub == 0 || pub == worked_no) {
                break;
            }
            worked_no = pub;
            if (m_src_enc_cores.load(std::memory_order_relaxed) == 1) {
                continue; // debug: core 0 encodes alone
            }
            srcWorkJobs(m_src_frames[pub & 1]);
        }
    }
}

/**
 * @brief Wake src_rf only if it is idle and waiting for a packet or a new frame.
 *
 * src_rf sets m_src_rf_wants_pkt before it sleeps and then re-checks for work; the producer publishes its
 * data (seq_cst) before testing the flag. One of the two always sees the other, so no wake-up is lost, and
 * src_rf is not woken for packets it would only find while already awake.
 *
 * @param bit notification bit to set (SRC_BIT_PKT / SRC_BIT_FRAME).
 */
void EspNowBroadcastEngine::srcWakeRfIfWaiting(uint32_t bit) {
    if (m_src_rf_task && m_src_rf_wants_pkt.exchange(false, std::memory_order_seq_cst)) {
        xTaskNotify(m_src_rf_task, bit, eSetBits);
    }
}

/**
 * @brief Claim and run jobs of frame `f` until none are left (both cores; lock-free cursor).
 */
void EspNowBroadcastEngine::srcWorkJobs(SrcFrame& f) {
    const int core = (xPortGetCoreID() == 0) ? 0 : 1;
    while (true) {
        uint32_t idx = f.next_job.fetch_add(1, std::memory_order_acq_rel);
        if (idx >= SRC_NUM_JOBS) {
            break;
        }
        int64_t t = esp_timer_get_time();
        if (m_src_enc_cores.load(std::memory_order_relaxed) == 1 && (t - f.t0_us) > 8000) {
            // Debug single-core mode: drop the rest of this frame so the idle task (task WDT) still runs
            f.next_job.store(SRC_NUM_JOBS, std::memory_order_release);
            break;
        }
        f.in_flight.fetch_add(1, std::memory_order_acq_rel);
        srcDoJob(f, f.jobs[idx]);
        int64_t t_end = esp_timer_get_time();
        m_acc_enc_busy_us[core].fetch_add(static_cast<uint32_t>(t_end - t), std::memory_order_relaxed);
        m_acc_enc_jobs[core].fetch_add(1, std::memory_order_relaxed);
        uint32_t done = f.jobs_done.fetch_add(1, std::memory_order_acq_rel) + 1;
        f.in_flight.fetch_sub(1, std::memory_order_release);
        if (done == SRC_NUM_JOBS) {
            m_acc_jobs_done.add(static_cast<uint32_t>(t_end - f.t0_us));
        }
    }
}

/**
 * @brief Fill the t-1 / t-2 redundancy slots of a satellite packet from the MQ history.
 *
 * MQ(frame) is written by that frame's MQ job and tagged with its frame number. A missing entry (MQ job
 * cancelled or still running) is replaced by the other available one, so the bitstream always stays a valid
 * LC3 frame; only the very first frames after start fall back to zeros. Missing entries are counted.
 *
 * @param frame_no Frame the packet belongs to.
 * @param ch       Satellite channel 0..4.
 * @param t_prev1  Destination for MQ(frame_no - 1), LC3_FRAME_OCTETS_RED bytes.
 * @param t_prev2  Destination for MQ(frame_no - 2), LC3_FRAME_OCTETS_RED bytes.
 */
void EspNowBroadcastEngine::srcCopyMqHistory(uint32_t frame_no, uint8_t ch, uint8_t* t_prev1, uint8_t* t_prev2) {
    const uint32_t want1 = frame_no - 1;
    const uint32_t want2 = frame_no - 2;
    const uint32_t s1 = want1 % 3;
    const uint32_t s2 = want2 % 3;
    const bool ok1 = want1 > 0 && m_src_mq_tag[s1][ch].load(std::memory_order_acquire) == want1;
    const bool ok2 = want2 > 0 && m_src_mq_tag[s2][ch].load(std::memory_order_acquire) == want2;

    if (ok1) {
        memcpy(t_prev1, m_src_mq[s1][ch], LC3_FRAME_OCTETS_RED);
    } else if (ok2) {
        memcpy(t_prev1, m_src_mq[s2][ch], LC3_FRAME_OCTETS_RED);
    } else {
        memset(t_prev1, 0, LC3_FRAME_OCTETS_RED);
    }
    if (ok2) {
        memcpy(t_prev2, m_src_mq[s2][ch], LC3_FRAME_OCTETS_RED);
    } else {
        memcpy(t_prev2, t_prev1, LC3_FRAME_OCTETS_RED);
    }
    if ((!ok1 || !ok2) && frame_no > 3) {
        m_acc_mq_missing.fetch_add(1, std::memory_order_relaxed);
    }
}

/**
 * @brief Run one encode job and, for HQ/SUB, finish its VSAF packet (all fields but t_tx1_us).
 */
void EspNowBroadcastEngine::srcDoJob(SrcFrame& f, const SrcJob& job) {
    const uint32_t par = f.frame_no & 1;
    const uint8_t seq = static_cast<uint8_t>(f.frame_no & 0xFF);
    size_t actual = 0;

    switch (job.type) {
    case SrcJobType::SUB: {
        vsaf_sub_packet_t& p = m_src_pkt_sub[par];
        p.type_id = VSAF_TYPE_AUDIO_SUBWOOFER;
        p.packet_flags = make_packet_flags(SUBWOOFER_CHANNEL_ID, 8000, 10000, false);
        p.seq = seq;
        p.t_tx1_us = static_cast<uint32_t>(f.t_grid_us);  // frame time t0(N) on the 10 ms grid: same in all 6 packets
        int64_t t = esp_timer_get_time();
        m_lc3_codec.encodeFrame(m_src_pcm_sub[par], 80, p.data_t0, LC3_FRAME_OCTETS_SUB, &actual,
                                Codec::Lc3CodecEngine::SUB_ENCODER_INDEX, 1);
        m_acc_enc_sub.add(static_cast<uint32_t>(esp_timer_get_time() - t));

        uint8_t* prev[3] = {p.data_t_prev1, p.data_t_prev2, p.data_t_prev3};
        for (uint32_t k = 1; k <= 3; ++k) {
            uint32_t want = f.frame_no - k;
            uint32_t slot = want % 4;
            if (want > 0 && m_src_sub_tag[slot].load(std::memory_order_acquire) == want) {
                memcpy(prev[k - 1], m_src_sub_hist[slot], LC3_FRAME_OCTETS_SUB);
            } else {
                memcpy(prev[k - 1], p.data_t0, LC3_FRAME_OCTETS_SUB);
            }
        }
        const uint32_t hslot = f.frame_no % 4;
        memcpy(m_src_sub_hist[hslot], p.data_t0, LC3_FRAME_OCTETS_SUB);
        m_src_sub_tag[hslot].store(f.frame_no, std::memory_order_release);

        f.pkt_ready[SUBWOOFER_CHANNEL_ID].store(1, std::memory_order_seq_cst);
        srcWakeRfIfWaiting(SRC_BIT_PKT);
        break;
    }
    case SrcJobType::HQ: {
        vsaf_audio_packet_t& p = m_src_pkt_sat[par][job.ch];
        p.type_id = VSAF_TYPE_AUDIO_SATELLITE;
        p.packet_flags = make_packet_flags(job.ch, m_telemetry.sample_rate, 10000, false);
        p.seq = seq;
        p.t_tx1_us = static_cast<uint32_t>(f.t_grid_us);  // frame time t0(N) on the 10 ms grid: same in all 6 packets
        // Phase 1: shared analysis + HQ frame; the MQ frame of this analysis stays pending for the MQ job
        int64_t t = esp_timer_get_time();
        m_lc3_codec.encodeFrameDualBegin(m_src_pcm_sat[par][job.ch], p.data_t0, LC3_FRAME_OCTETS_HQ,
                                         LC3_FRAME_OCTETS_RED, job.ch);
        m_acc_enc_hq.add(static_cast<uint32_t>(esp_timer_get_time() - t));
        f.hq_done[job.ch].store(1, std::memory_order_release);
        srcCopyMqHistory(f.frame_no, job.ch, p.data_t_prev1, p.data_t_prev2);

        f.pkt_ready[job.ch].store(1, std::memory_order_seq_cst);
        srcWakeRfIfWaiting(SRC_BIT_PKT);
        break;
    }
    case SrcJobType::MQ: {
        // Phase 2: MQ frame of this channel's analysis. Its HQ job was claimed earlier and normally finished;
        // it can still run on the other core only if that core was preempted, so wait for it (rare, counted).
        if (!f.hq_done[job.ch].load(std::memory_order_acquire)) {
            m_acc_mq_waits.fetch_add(1, std::memory_order_relaxed);
            while (!f.hq_done[job.ch].load(std::memory_order_acquire)) {
                esp_rom_delay_us(5);
            }
        }
        const uint32_t slot = f.frame_no % 3;
        int64_t t = esp_timer_get_time();
        m_lc3_codec.encodeFrameDualEnd(m_src_mq[slot][job.ch], job.ch);
        m_acc_enc_mq.add(static_cast<uint32_t>(esp_timer_get_time() - t));
        m_src_mq_tag[slot][job.ch].store(f.frame_no, std::memory_order_release);
        break;
    }
    }
}

// ---------------------------------------------------------------------------
// core 1: RF owner (highest application priority on core 1)
// ---------------------------------------------------------------------------
void EspNowBroadcastEngine::runSrcRfLoop() {
    ESP_LOGI(TAG, "src_rf started on core %d (event-driven TX, 1 packet in flight)", xPortGetCoreID());
    uint32_t cur_no = 0;
    SrcFrame* f = nullptr;
    uint8_t sent_mask = 0;
    uint32_t n_sent = 0;
    bool frame_open = false;
    bool busy = false;
    int64_t t_submit = 0;
    int64_t t_last_done = 0;
    uint32_t tick_count = 0;
    // Packet in flight: 0 = audio, 1 = audio with REQ_ACK (time-sync t1 is recorded at its TX-done), 2 = control
    int inflight_kind = 0;
    uint32_t inflight_frame = 0;
    bool ctrl_sent = false;

    // Next packet to submit for the open frame, or -1. Sets ack_held when only the held reply-request
    // packet is ready (it goes last, unless the other packets are very late).
    bool ack_held = false;
    auto pick_next = [&](int64_t elapsed) -> int {
        ack_held = false;
        for (uint32_t k = 0; k < SRC_NUM_PKTS; ++k) {
            uint8_t ch = f->tx_order[k];
            if ((sent_mask & (1u << ch)) || !f->pkt_ready[ch].load(std::memory_order_seq_cst)) {
                continue;
            }
            if (ch == f->ack_ch && n_sent < SRC_NUM_PKTS - 1 && elapsed < SRC_ACK_HOLD_US) {
                ack_held = true;
                continue;
            }
            return ch;
        }
        return -1;
    };

    while (true) {
        // Sleep until the next event. No polling: TX-done, packet-ready and frame events wake this task;
        // the only timed events are the TX-done timeout and the release of a held reply-request packet.
        TickType_t wait = portMAX_DELAY;
        if (busy) {
            wait = pdMS_TO_TICKS(3);  // TX-done timeout check
        } else {
            m_src_rf_wants_pkt.store(true, std::memory_order_seq_cst);
            int64_t t = esp_timer_get_time();
            const bool new_frame = m_src_pub_frame_no.load(std::memory_order_seq_cst) != cur_no;
            int ready = (!new_frame && f && frame_open) ? pick_next(t - f->t0_us) : -1;
            if (new_frame || ready >= 0) {
                wait = 0;  // work already there
            } else if (ack_held) {
                int64_t until = f->t0_us + SRC_ACK_HOLD_US - t;
                wait = static_cast<TickType_t>(until / 1000 + 1);
            }
        }
        uint32_t bits = 0;
        if (wait != 0) {
            xTaskNotifyWait(0, UINT32_MAX, &bits, wait);
        } else {
            xTaskNotifyWait(0, UINT32_MAX, &bits, 0);  // collect pending bits without sleeping
        }
        m_src_rf_wants_pkt.store(false, std::memory_order_relaxed);
        m_acc_rf_wakes.fetch_add(1, std::memory_order_relaxed);
        int64_t now = esp_timer_get_time();

        // 1. Completion or timeout of the packet in flight
        if (busy) {
            if (m_src_tx_done_flag.exchange(false, std::memory_order_acq_rel)) {
                int64_t t_done = m_src_tx_done_us.load(std::memory_order_relaxed);
                if (t_done < t_submit) {
                    t_done = now;
                }
                uint32_t lat = static_cast<uint32_t>(t_done - t_submit);
                m_acc_tx_wake.add(now > t_done ? static_cast<uint32_t>(now - t_done) : 0U);
                m_acc_tx_lat.add(lat);
                m_src_tx_pkts_in_window++;
                if (lat < SRC_TX_MIN_PLAUSIBLE_US) {
                    m_src_tx_fast_pkts++;
                }
                if (m_src_tx_status.load(std::memory_order_relaxed) == ESP_NOW_SEND_SUCCESS) {
                    m_tx_ok_in_window++;
                } else {
                    m_tx_fail_count++;
                }
                m_consecutive_tx_timeouts = 0;
                busy = false;
                t_last_done = t_done;
                if (inflight_kind == 1) {
                    // Time sync t1: TX-done of the REQ_ACK packet, indexed by its seq (tag = frame number)
                    const uint8_t seq8 = static_cast<uint8_t>(inflight_frame);
                    m_ptp_t1[seq8].store(static_cast<uint32_t>(t_done), std::memory_order_relaxed);
                    m_ptp_t1_frame[seq8].store(inflight_frame, std::memory_order_release);
                }
            } else if (now - t_submit > SRC_TX_DONE_TIMEOUT_US) {
                m_tx_timeout_count++;
                m_acc_tx_timeouts.fetch_add(1, std::memory_order_relaxed);
                m_consecutive_tx_timeouts++;
                if (m_consecutive_tx_timeouts >= 5) {
                    handleTxSubsystemHang();
                }
                busy = false;
                t_last_done = now;
            }
        }

        // 2. Frame tick: radio maintenance while idle, TX health, timing window
        if (bits & SRC_BIT_TICK) {
            if (!busy) {
                applyPendingRadioChanges();
            }
            const bool casting = m_state.load(std::memory_order_acquire) == NetworkState::CAST;
            if (m_consecutive_send_errors >= TX_MAX_CONSECUTIVE_ERRORS) {
                recoverEspNowTx("consecutive esp_now_send() errors");
            } else if (++m_tx_window_frames >= TX_HEALTH_WINDOW_FRAMES) {
                if (casting && m_src_tx_attempts_in_window > 0 && m_tx_ok_in_window == 0) {
                    recoverEspNowTx("no successful TX completion in 0.5 s");
                } else if (casting && m_src_tx_pkts_in_window > 0 && m_src_tx_fast_pkts == m_src_tx_pkts_in_window) {
                    recoverEspNowTx("implausibly fast TX completions (< 60 us) for 0.5 s");
                }
                m_tx_ok_in_window = 0;
                m_tx_window_frames = 0;
                m_src_tx_pkts_in_window = 0;
                m_src_tx_fast_pkts = 0;
                m_src_tx_attempts_in_window = 0;
            }
            if (++tick_count % SRC_TIMING_WINDOW_FRAMES == 0) {
                srcRotateTimingWindow();
            }
        }

        // 3. New frame published by core 0?
        uint32_t pub = m_src_pub_frame_no.load(std::memory_order_acquire);
        if (pub != cur_no && pub != 0) {
            if (frame_open && n_sent < SRC_NUM_PKTS) {
                uint32_t unsent = SRC_NUM_PKTS - n_sent;
                m_acc_deadline_skips.fetch_add(unsent, std::memory_order_relaxed);
                m_tx_deadline_drops += unsent;
            }
            cur_no = pub;
            f = &m_src_frames[pub & 1];
            sent_mask = 0;
            n_sent = 0;
            frame_open = true;
            ctrl_sent = false;
        }

        // 4. Submit the next ready packet (radio idle)
        if (f && frame_open && !busy) {
            int64_t elapsed = now - f->t0_us;
            if (elapsed > SRC_TX_DEADLINE_US) {
                uint32_t unsent = SRC_NUM_PKTS - n_sent;
                m_acc_deadline_skips.fetch_add(unsent, std::memory_order_relaxed);
                m_tx_deadline_drops += unsent;
                frame_open = false;
            } else {
                // Reply-request packet goes last (its SINK answers immediately), unless others are very late
                int pick = pick_next(elapsed);
                if (pick >= 0) {
                    const uint32_t par = cur_no & 1;
                    uint8_t* buf;
                    size_t len;
                    if (pick == SUBWOOFER_CHANNEL_ID) {
                        vsaf_sub_packet_t& p = m_src_pkt_sub[par];
                        if (pick == f->ack_ch) p.packet_flags |= 0x80;
                        buf = reinterpret_cast<uint8_t*>(&p);
                        len = sizeof(p);
                    } else {
                        vsaf_audio_packet_t& p = m_src_pkt_sat[par][pick];
                        if (pick == f->ack_ch) p.packet_flags |= 0x80;
                        buf = reinterpret_cast<uint8_t*>(&p);
                        len = sizeof(p);
                    }
                    m_src_tx_done_flag.store(false, std::memory_order_release);
                    int64_t ts = esp_timer_get_time();
                    // t_tx1_us already holds the frame time t0(N) (set in srcDoJob); the precise TX time the time
                    // sync needs is the TX-done of the REQ_ACK packet, recorded when it completes
                    inflight_kind = (pick == f->ack_ch) ? 1 : 0;
                    inflight_frame = cur_no;
                    esp_err_t err = esp_now_send(s_broadcast_mac, buf, len);
                    m_acc_tx_call.add(static_cast<uint32_t>(esp_timer_get_time() - ts));

                    sent_mask |= static_cast<uint8_t>(1u << pick);
                    n_sent++;
                    if (err == ESP_OK) {
                        busy = true;
                        t_submit = ts;
                        m_src_tx_attempts_in_window++;
                        m_consecutive_send_errors = 0;
                        if (n_sent == 1) {
                            m_acc_tx_first.add(static_cast<uint32_t>(ts - f->t0_us));
                        } else {
                            m_acc_tx_gap.add(static_cast<uint32_t>(ts - t_last_done));
                        }
                        m_peers[pick].packets_sent++;
                        m_tx_packets_this_sec++;
                        m_tx_packets_total++;
                        m_tx_packets_sec++;
                    } else {
                        m_tx_mac_error_count++;
                        m_consecutive_send_errors++;
                        m_last_tx_err.store(static_cast<int32_t>(err), std::memory_order_relaxed);
                        if (err == ESP_ERR_ESPNOW_NO_MEM) {
                            m_tx_err_no_mem++;
                        }
                        t_last_done = esp_timer_get_time();
                    }
                }
            }
        }

        // 4b. Control frame (time sync estimates + playout parameters) every 10th frame, after the audio packets
        const bool ctrl_due = (cur_no % SRC_CONTROL_EVERY_FRAMES) == 0;
        if (f && frame_open && n_sent == SRC_NUM_PKTS && !busy && ctrl_due && !ctrl_sent &&
            (now - f->t0_us) < SRC_TX_DEADLINE_US) {
            srcBuildControl(m_ctrl_pkt);
            m_src_tx_done_flag.store(false, std::memory_order_release);
            const int64_t ts = esp_timer_get_time();
            ctrl_sent = true;
            if (esp_now_send(s_broadcast_mac, reinterpret_cast<const uint8_t*>(&m_ctrl_pkt), sizeof(m_ctrl_pkt)) == ESP_OK) {
                busy = true;
                t_submit = ts;
                inflight_kind = 2;
                m_src_tx_attempts_in_window++;
            }
        }

        // 5. Frame complete: every packet submitted (and the control frame, if due) and the last one finished
        if (f && frame_open && n_sent == SRC_NUM_PKTS && !busy && (!ctrl_due || ctrl_sent)) {
            m_acc_tx_last.add(static_cast<uint32_t>(t_last_done - f->t0_us));
            frame_open = false;
        }
        m_acc_rf_active_us.fetch_add(static_cast<uint32_t>(esp_timer_get_time() - now), std::memory_order_relaxed);
    }
}

/**
 * @brief Close the current timing window into m_src_timing_snap (RF task, every 100 frame ticks).
 */
void EspNowBroadcastEngine::srcRotateTimingWindow() {
    SourceTimingSnapshot s;
    s.frames = m_acc_frames.exchange(0, std::memory_order_relaxed);
    s.dsp = m_acc_dsp.take();
    s.enc_hq = m_acc_enc_hq.take();
    s.enc_mq = m_acc_enc_mq.take();
    s.enc_sub = m_acc_enc_sub.take();
    for (int c = 0; c < 2; ++c) {
        s.enc_busy_core_us[c] = m_acc_enc_busy_us[c].exchange(0, std::memory_order_relaxed);
        s.enc_jobs_core[c] = m_acc_enc_jobs[c].exchange(0, std::memory_order_relaxed);
    }
    s.tx_lat = m_acc_tx_lat.take();
    s.tx_call = m_acc_tx_call.take();
    s.tx_wake = m_acc_tx_wake.take();
    s.tx_gap = m_acc_tx_gap.take();
    s.tx_first_at = m_acc_tx_first.take();
    s.tx_last_done_at = m_acc_tx_last.take();
    s.jobs_done_at = m_acc_jobs_done.take();
    s.overruns = m_acc_overruns.exchange(0, std::memory_order_relaxed);
    s.rf_wakes = m_acc_rf_wakes.exchange(0, std::memory_order_relaxed);
    s.mq_waits = m_acc_mq_waits.exchange(0, std::memory_order_relaxed);
    s.rf_active_us = m_acc_rf_active_us.exchange(0, std::memory_order_relaxed);
    s.cancelled_jobs = m_acc_cancelled.exchange(0, std::memory_order_relaxed);
    s.mq_missing = m_acc_mq_missing.exchange(0, std::memory_order_relaxed);
    s.deadline_skips = m_acc_deadline_skips.exchange(0, std::memory_order_relaxed);
    s.tx_timeouts = m_acc_tx_timeouts.exchange(0, std::memory_order_relaxed);
    s.free_internal_heap = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    s.min_free_internal_heap = static_cast<uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    portENTER_CRITICAL(&m_src_timing_mux);
    m_src_timing_snap = s;
    portEXIT_CRITICAL(&m_src_timing_mux);
}

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ---------------------------------------------------------------------------
// Debug: LC3 encoder micro-benchmark (console 'encbench [core] [irq]')
// ---------------------------------------------------------------------------
namespace {
constexpr int BENCH_FRAMES = 24;  // consecutive frames per signal (encoder state evolves)
constexpr int BENCH_WARMUP = 2;   // first frames not timed (all frames are compared)
constexpr int BENCH_NCFG = 4;
constexpr int BENCH_NSIG = 5;
const char* const BENCH_ENC_NAMES[BENCH_NCFG] = {"HQ  48k 120B", "MQ  48k  60B", "SUB  8k  60B", "DUAL HQ+MQ"};
const char* const BENCH_SIG_NAMES[BENCH_NSIG] = {"silence", "sine 1k -6dB", "noise -6dB", "attacks", "live music"};

struct EncBenchCtx {
    const int16_t* live_sat;      // copy of the last published satellite frame (480 samples)
    const int16_t* live_sub;      // copy of the last published sub frame (80 samples)
    bool           irq_masked;    // true: pure CPU cost; false: interrupts allowed (only ISRs preempt)
    uint32_t       avg_us[BENCH_NCFG][BENCH_NSIG];
    uint32_t       max_us[BENCH_NCFG][BENCH_NSIG];
    uint32_t       mismatch[BENCH_NSIG];  // dual-rate frames that differ from separate HQ / MQ encoders
    bool           ok;
    TaskHandle_t   caller;
};

/**
 * @brief Fill one test-signal frame.
 * @param sig  Signal index (BENCH_SIG_NAMES). "attacks" = silence with a loud noise burst in the second
 *             half of every 3rd frame, so the HQ attack detector fires (exercises the non-shared SNS path).
 * @param fr   Frame index; ns samples per frame at sr Hz; seed = LCG state (in/out).
 */
void benchFillPcm(int16_t* pcm, int sig, int fr, int ns, int sr, uint32_t& seed, const EncBenchCtx* ctx) {
    for (int i = 0; i < ns; ++i) {
        int n = fr * ns + i;
        switch (sig) {
            case 0: pcm[i] = 0; break;
            case 1: pcm[i] = static_cast<int16_t>(16384.0f * sinf(6.2831853f * 1000.0f * n / sr)); break;
            case 2: seed = seed * 1664525u + 1013904223u;
                    pcm[i] = static_cast<int16_t>((static_cast<int32_t>(seed >> 16) - 32768) / 2); break;
            case 3: seed = seed * 1664525u + 1013904223u;
                    pcm[i] = (fr % 3 == 2 && i >= ns / 2)
                                 ? static_cast<int16_t>(static_cast<int32_t>(seed >> 16) - 32768) : 0;
                    break;
            default: pcm[i] = (sr == 8000) ? ctx->live_sub[i] : ctx->live_sat[i]; break;
        }
    }
}

/**
 * @brief Worker task: time single-rate encodes (HQ, MQ, SUB) and the dual-rate HQ+MQ encode per test signal,
 *        and check the dual-rate output bit for bit against two separate encoders.
 * @param arg EncBenchCtx*; results in avg_us / max_us (us per 10 ms frame, from 240 MHz cycles), mismatch.
 */
void encBenchTask(void* arg) {
    auto* ctx = static_cast<EncBenchCtx*>(arg);
    struct Cfg { int sr; int ns; int octets; };
    const Cfg cfgs[3] = {{48000, 480, 120}, {48000, 480, 60}, {8000, 80, 60}};
    const unsigned enc_size = lc3_encoder_size(10000, 48000);
    const unsigned rate_size = lc3_encoder_rate_size(10000, 48000);
    // Scratch (separate blocks, the heap is fragmented): 3 encoders (reference HQ, reference MQ, dual primary),
    // the rate state and the PCM frame
    constexpr uint32_t CAPS = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    void* mem_a = heap_caps_malloc(enc_size, CAPS);
    void* mem_b = heap_caps_malloc(enc_size, CAPS);
    void* mem_d = heap_caps_malloc(enc_size, CAPS);
    void* mem_r = heap_caps_malloc(rate_size, CAPS);
    int16_t* pcm = static_cast<int16_t*>(heap_caps_malloc(480 * sizeof(int16_t), CAPS));
    ctx->ok = mem_a && mem_b && mem_d && mem_r && pcm;
    if (ctx->ok) {
        uint8_t out_a[120], out_b[60], dual_a[120], dual_b[60];
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

        for (int ci = 0; ci < BENCH_NCFG; ++ci) {
            const bool dual = ci == 3;
            const Cfg& c = dual ? cfgs[0] : cfgs[ci];
            for (int sig = 0; sig < BENCH_NSIG; ++sig) {
                lc3_encoder_t enc = lc3_setup_encoder(10000, c.sr, c.sr, mem_a);
                lc3_encoder_disable_ltpf(enc);
                lc3_encoder_t ref_b = nullptr, enc_d = nullptr;
                lc3_encoder_rate_t rate = nullptr;
                if (dual) {
                    ref_b = lc3_setup_encoder(10000, c.sr, c.sr, mem_b);
                    lc3_encoder_disable_ltpf(ref_b);
                    enc_d = lc3_setup_encoder(10000, c.sr, c.sr, mem_d);
                    lc3_encoder_disable_ltpf(enc_d);
                    rate = lc3_setup_encoder_rate(enc_d, mem_r);
                }
                uint32_t seed = 12345;
                uint64_t sum = 0;
                uint32_t mx = 0;
                uint32_t bad = 0;
                for (int fr = 0; fr < BENCH_FRAMES; ++fr) {
                    benchFillPcm(pcm, sig, fr, c.ns, c.sr, seed, ctx);
                    uint32_t cyc;
                    if (!dual) {
                        if (ctx->irq_masked) portENTER_CRITICAL(&mux);
                        uint32_t c0 = esp_cpu_get_cycle_count();
                        lc3_encode(enc, LC3_PCM_FORMAT_S16, pcm, 1, c.octets, out_a);
                        cyc = esp_cpu_get_cycle_count() - c0;
                        if (ctx->irq_masked) portEXIT_CRITICAL(&mux);
                    } else {
                        if (ctx->irq_masked) portENTER_CRITICAL(&mux);
                        uint32_t c0 = esp_cpu_get_cycle_count();
                        lc3_encode_dual(enc_d, rate, LC3_PCM_FORMAT_S16, pcm, 1, 120, dual_a, 60, dual_b);
                        cyc = esp_cpu_get_cycle_count() - c0;
                        if (ctx->irq_masked) portEXIT_CRITICAL(&mux);
                        // Reference: two independent encoders on the same PCM (not timed)
                        lc3_encode(enc, LC3_PCM_FORMAT_S16, pcm, 1, 120, out_a);
                        lc3_encode(ref_b, LC3_PCM_FORMAT_S16, pcm, 1, 60, out_b);
                        if (memcmp(out_a, dual_a, 120) != 0 || memcmp(out_b, dual_b, 60) != 0) {
                            bad++;
                        }
                    }
                    if (fr >= BENCH_WARMUP) {
                        sum += cyc;
                        mx = std::max(mx, cyc);
                    }
                }
                ctx->avg_us[ci][sig] = static_cast<uint32_t>(sum / (BENCH_FRAMES - BENCH_WARMUP) / 240);
                ctx->max_us[ci][sig] = mx / 240;
                if (dual) {
                    ctx->mismatch[sig] = bad;
                }
            }
        }
    }
    free(mem_a);
    free(mem_b);
    free(mem_d);
    free(mem_r);
    free(pcm);
    xTaskNotifyGive(ctx->caller);
    vTaskDelete(nullptr);
}
} // namespace

/**
 * @brief Encode fixed test signals on scratch liblc3 encoders and print the cost per 10 ms frame.
 *
 * Runs in a temporary task at the highest priority (above Wi-Fi) pinned to `core`, so no task preempts it.
 * With irq_masked the result is the pure CPU cost; without it, the difference to the masked run is the
 * interrupt load on that core. Compare with the wall-clock encode times of the live pipeline ('timing').
 * Same settings as the pipeline: 10 ms, LTPF off, 48 kHz 120 B (HQ) / 60 B (MQ), 8 kHz 60 B (sub).
 * The DUAL row times lc3_encode_dual (HQ + MQ in one call) and counts frames whose output differs from two
 * separate encoders (expected 0: bit-exact).
 *
 * @param print      printf-style output function (console).
 * @param core       0 or 1.
 * @param irq_masked true = interrupts masked during each encode.
 */
void EspNowBroadcastEngine::runEncoderBench(void (*print)(const char*, ...), int core, bool irq_masked) {
    auto* ctx = static_cast<EncBenchCtx*>(heap_caps_calloc(1, sizeof(EncBenchCtx) + (480 + 80) * sizeof(int16_t),
                                                           MALLOC_CAP_INTERNAL));
    if (!ctx) {
        print("[ENCBENCH] no memory\n");
        return;
    }
    auto* live = reinterpret_cast<int16_t*>(ctx + 1);
    uint32_t pub = m_src_pub_frame_no.load(std::memory_order_acquire);
    memcpy(live, m_src_pcm_sat[pub & 1][0], 480 * sizeof(int16_t));
    memcpy(live + 480, m_src_pcm_sub[pub & 1], 80 * sizeof(int16_t));
    ctx->live_sat = live;
    ctx->live_sub = live + 480;
    ctx->irq_masked = irq_masked;
    ctx->caller = xTaskGetCurrentTaskHandle();
    core = (core == 0) ? 0 : 1;
    if (xTaskCreatePinnedToCore(encBenchTask, "encbench", 6144, ctx, configMAX_PRIORITIES - 1, nullptr, core) != pdPASS) {
        print("[ENCBENCH] task creation failed\n");
        free(ctx);
        return;
    }
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000)) == 0) {
        print("[ENCBENCH] timeout\n");
        return; // ctx intentionally leaked: the worker may still use it
    }
    if (!ctx->ok) {
        print("[ENCBENCH] no memory for scratch encoders\n");
    } else {
        print("\n[ENCBENCH] core %d, prio %d, interrupts %s, %d frames/signal (first %d not timed), LTPF off\n", core,
              configMAX_PRIORITIES - 1, irq_masked ? "masked" : "ENABLED", BENCH_FRAMES, BENCH_WARMUP);
        print("  %-13s %-13s %8s %8s %s\n", "encoder", "signal", "avg us", "max us", "dual != separate");
        for (int ci = 0; ci < BENCH_NCFG; ++ci) {
            for (int sig = 0; sig < BENCH_NSIG; ++sig) {
                if (ci == 3) {
                    print("  %-13s %-13s %8lu %8lu %lu/%d frames\n", BENCH_ENC_NAMES[ci], BENCH_SIG_NAMES[sig],
                          (unsigned long)ctx->avg_us[ci][sig], (unsigned long)ctx->max_us[ci][sig],
                          (unsigned long)ctx->mismatch[sig], BENCH_FRAMES);
                } else {
                    print("  %-13s %-13s %8lu %8lu\n", BENCH_ENC_NAMES[ci], BENCH_SIG_NAMES[sig],
                          (unsigned long)ctx->avg_us[ci][sig], (unsigned long)ctx->max_us[ci][sig]);
                }
            }
        }
    }
    free(ctx);
}

#else  // !CONFIG_IDF_TARGET_ESP32S3
// The SOURCE (and thus the LC3 encoder bank) exists only on the ESP32-S3. Keeping liblc3 out of other
// targets matters: with CONFIG_LC3_CODE_IN_IRAM it would cost ~100 KB of internal RAM on the C6 SINKs.
void EspNowBroadcastEngine::runEncoderBench(void (*print)(const char*, ...), int core, bool irq_masked) {
    (void)core;
    (void)irq_masked;
    print("[ENCBENCH] only available on the ESP32-S3 SOURCE\n");
}
#endif // CONFIG_IDF_TARGET_ESP32S3

} // namespace AudioNet
