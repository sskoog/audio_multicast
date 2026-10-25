#pragma once

#include <cstdint>
#include <cstddef>
#include <climits>

namespace AudioNet {

static constexpr uint16_t VSAF_TYPE_AUDIO_SATELLITE = 0x1337; // High-Passed Satellite Audio (Ch 0, 1, 2, 3, 4)
static constexpr uint16_t VSAF_TYPE_AUDIO           = 0x1337; // Backwards-compatible alias
static constexpr uint16_t VSAF_TYPE_AUDIO_SUBWOOFER = 0x1338; // Subwoofer Audio (Ch 5, 4x60B frames)
static constexpr uint16_t VSAF_TYPE_CONTROL         = 0x1350; // Control packet (from SOURCE)
static constexpr uint16_t VSAF_TYPE_SINK_TELEMETRY  = 0x1360; // Telemetry reply (from SINK)

static constexpr uint8_t NODE_ID_SOURCE          = 0x0;
static constexpr uint8_t NODE_ID_BROADCAST       = 0x7;
static constexpr size_t  MAX_SINK_NODES          = 6;
static constexpr size_t  NUM_SATELLITE_CHANNELS  = 5; // Ch 0..4 (Left, Right, Center, Surround Left, Surround Right)
static constexpr uint8_t SUBWOOFER_CHANNEL_ID    = 5; // Ch 5 (Subwoofer)
static constexpr size_t  LC3_FRAME_OCTETS_HQ     = 120; // 96 kbps @ 10ms 48k
static constexpr size_t  LC3_FRAME_OCTETS_RED    = 60;  // 48 kbps @ 10ms 48k
static constexpr size_t  LC3_FRAME_OCTETS_SUB    = 60;  // 48 kbps @ 10ms 8k Subwoofer
static constexpr size_t  LC3_FRAME_OCTETS        = 120; // Backwards-compatible alias

// packet_flags Bitfield Layout:
// Bit 0    : Frame Duration (0 = 7.5 ms [deprecated], 1 = 10.0 ms [standard])
// Bits 1-3 : Sample Rate Code (0=8k, 1=16k, 2=24k, 3=32k, 4=48k)
// Bits 4-6 : Receiver Channel ID (0..5 = Sinks 0..5, 7 = Broadcast)
// Bit 7    : Request for ACK / Reply Flag (1 = SINK must reply with telemetry frame, 0 = no reply)

inline constexpr uint8_t make_packet_flags(uint8_t rx_id, uint32_t sample_rate_hz, uint32_t frame_dur_us, bool request_ack) {
    uint8_t dur_bit = (frame_dur_us >= 10000) ? 1 : 0;
    uint8_t sr_code = 4; // Default 48k
    switch (sample_rate_hz) {
        case 8000:  sr_code = 0; break;
        case 16000: sr_code = 1; break;
        case 24000: sr_code = 2; break;
        case 32000: sr_code = 3; break;
        case 48000: default: sr_code = 4; break;
    }
    return static_cast<uint8_t>(
        (dur_bit & 0x01) |
        ((sr_code & 0x07) << 1) |
        ((rx_id & 0x07) << 4) |
        (request_ack ? 0x80 : 0x00)
    );
}

inline constexpr uint8_t get_flags_rx_id(uint8_t flags) { return static_cast<uint8_t>((flags >> 4) & 0x07); }
inline constexpr uint32_t get_flags_sample_rate(uint8_t flags) {
    constexpr uint32_t sr_lut[8] = {8000, 16000, 24000, 32000, 48000, 48000, 48000, 48000};
    return sr_lut[(flags >> 1) & 0x07];
}
inline constexpr uint32_t get_flags_frame_dur_us(uint8_t flags) {
    return (flags & 0x01) ? 10000 : 7500;
}
inline constexpr bool get_flags_req_ack(uint8_t flags) {
    return (flags & 0x80) != 0;
}

// Time base (VSAF 3.1): t_tx1_us of an audio packet is the SOURCE frame time t0(N) of its t0 frame: the low 32 bits
// of the SOURCE esp_timer at that frame's 10 ms tick, identical in all 6 packets of the frame. The presentation time
// of frame N is t0(N) + latency (control frame); the redundant frames t-k of a packet have t0 - k * 10 ms.
// All timestamp arithmetic is modulo 2^32 (wraps every 71.6 min).

// VSAF 3.0 Satellite Audio Broadcast Packet (Strictly 248 bytes, 32-bit word aligned)
struct __attribute__((packed)) vsaf_audio_packet_t {
    uint16_t type_id;                             // 0x1337 (offset 0)
    uint8_t  packet_flags;                        // Packed config, target receiver, and req-ack (offset 2)
    uint8_t  seq;                                 // Monotonic 8-bit sequence number (offset 3)
    uint32_t t_tx1_us;                            // SOURCE frame time t0(N), see above (offset 4)
    uint8_t  data_t0[LC3_FRAME_OCTETS_HQ];        // Current frame t0: 120 bytes (offset 8)
    uint8_t  data_t_prev1[LC3_FRAME_OCTETS_RED];  // Redundant frame t-1: 60 bytes (offset 128)
    uint8_t  data_t_prev2[LC3_FRAME_OCTETS_RED];  // Redundant frame t-2: 60 bytes (offset 188)
};
static_assert(sizeof(vsaf_audio_packet_t) == 248, "Satellite packet must be exactly 248 bytes");

// VSAF 3.0 Subwoofer Audio Broadcast Packet (Strictly 248 bytes, 32-bit word aligned)
struct __attribute__((packed)) vsaf_sub_packet_t {
    uint16_t type_id;                             // 0x1338 (offset 0)
    uint8_t  packet_flags;                        // Packed config, target receiver, and req-ack (offset 2)
    uint8_t  seq;                                 // Monotonic 8-bit sequence number (offset 3)
    uint32_t t_tx1_us;                            // SOURCE frame time t0(N), see above (offset 4)
    uint8_t  data_t0[LC3_FRAME_OCTETS_RED];       // Current Sub frame t0: 60 bytes (offset 8)
    uint8_t  data_t_prev1[LC3_FRAME_OCTETS_RED];  // Redundant Sub frame t-1: 60 bytes (offset 68)
    uint8_t  data_t_prev2[LC3_FRAME_OCTETS_RED];  // Redundant Sub frame t-2: 60 bytes (offset 128)
    uint8_t  data_t_prev3[LC3_FRAME_OCTETS_RED];  // Redundant Sub frame t-3: 60 bytes (offset 188)
};
static_assert(sizeof(vsaf_sub_packet_t) == 248, "Subwoofer packet must be exactly 248 bytes");

// VSAF 3.1 Round-Robin SINK Telemetry Reply = PTP delay request (Strictly 20 bytes)
// Time-sync exchange (two-step):
//   t1 = SOURCE TX-done time of the REQ_ACK audio packet (kept in the SOURCE, indexed by seq)
//   t2 = SINK receive time of that packet                      -> t_rx_us
//   t3 = SINK submit time of this reply = t2 + t_dwell_us; the reply's real TX-done time is reported in the NEXT
//        reply as prev_tx_delay_us (submit -> TX-done), which removes the random channel-access backoff
//   t4 = SOURCE receive time of this reply
//   offset (SINK - SOURCE) = ((t2 - t1) - (t4 - t3)) / 2,  round trip = (t2 - t1) + (t4 - t3)
static constexpr uint8_t VSAF_TELEM_FLAG_PREV_DELAY_VALID = 0x01;
static constexpr int16_t VSAF_PLAYOUT_ERR_UNKNOWN = INT16_MIN;
struct __attribute__((packed)) vsaf_sink_telemetry_t {
    uint16_t type_id;                      // 0x1360
    uint8_t  sink_id;                      // Channel / SINK ID (0..5)
    uint8_t  ack_seq;                      // seq of the REQ_ACK packet this reply answers
    uint32_t t_rx_us;                      // t2: SINK clock (low 32 bits) when that packet was received
    uint16_t t_dwell_us;                   // t3 - t2: submit time of this reply relative to t2 (us)
    int8_t   downlink_rssi;                // Downlink RSSI measured by SINK (dBm)
    uint8_t  fifo_fill;                    // SINK audio FIFO fill level (0-100%)
    uint8_t  prev_ack_seq;                 // ack_seq of the previous reply of this SINK
    uint8_t  flags;                        // VSAF_TELEM_FLAG_*
    uint16_t prev_tx_delay_us;             // previous reply: submit -> TX-done (us)
    int16_t  playout_err_us;               // SINK playout error vs presentation time (+ = late), or UNKNOWN
    uint16_t crc16;                        // Checksum over the preceding 18 bytes
};
static_assert(sizeof(vsaf_sink_telemetry_t) == 20, "Telemetry packet must be exactly 20 bytes");

// VSAF 3.1 Control frame (SOURCE -> all SINKs, broadcast every 100 ms): time sync + playout parameters.
// For channel c with rtt_us[c] != 0: SINK clock - SOURCE clock at SOURCE time t is
//   offset_us[c] + drift_ppb[c] * (t - t_ref_us) / 1e9
// Presentation time of frame N on SINK c (SOURCE clock): t0(N) + latency_us + trim_us[c].
static constexpr uint8_t VSAF_CONTROL_VERSION = 1;
struct __attribute__((packed)) vsaf_control_t {
    uint16_t type_id;                      // 0x1350
    uint8_t  version;                      // VSAF_CONTROL_VERSION
    uint8_t  seq;                          // control frame counter
    uint32_t t_ref_us;                     // SOURCE time the offsets refer to
    uint32_t latency_us;                   // playout latency added to the frame time
    int32_t  offset_us[MAX_SINK_NODES];    // per channel: SINK clock - SOURCE clock at t_ref (modulo 2^32)
    int32_t  drift_ppb[MAX_SINK_NODES];    // per channel: d(offset)/dt in parts per billion
    uint16_t rtt_us[MAX_SINK_NODES];       // per channel: round trip of the best sample (0 = no estimate)
    int16_t  trim_us[MAX_SINK_NODES];      // per channel: acoustic / pipeline trim added to the presentation time
    uint16_t crc16;                        // Checksum over the preceding bytes
    uint16_t reserved;
};
static_assert(sizeof(vsaf_control_t) == 88, "Control packet must be exactly 88 bytes");

// Fast CCITT-16 CRC
inline uint16_t calc_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

} // namespace AudioNet
