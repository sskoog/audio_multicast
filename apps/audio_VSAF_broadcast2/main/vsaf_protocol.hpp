#pragma once

#include <cstdint>
#include <cstddef>

namespace AudioNet {

// 16-Bit Tag Nibble Layout (0xTTRS)
// Bits 0-3  : TX_ID (Sender Node ID: 0x0 = SOURCE, 0x1-0x6 = SINK 1-6)
// Bits 4-7  : RX_ID (Target Receiver: 0x0-0x6 = Specific Node, 0xF = Broadcast/All)
// Bits 8-11 : TYPE  (Packet Type: 0=Audio, 1=Soft-ACK, 2=ARQ Repair, 3=Beacon)
// Bits 12-15: FLAGS (Bit 12: Retransmit, Bit 13: TimeSyncValid, Bits 14-15: Version 0)

enum PacketType : uint8_t {
    PKT_TYPE_AUDIO        = 0x0,
    PKT_TYPE_SOFT_ACK     = 0x1,
    PKT_TYPE_ARQ_REPAIR   = 0x2,
    PKT_TYPE_SYNC_BEACON  = 0x3,
};

enum PacketFlags : uint8_t {
    FLAG_NONE             = 0x00,
    FLAG_RETRY            = 0x01, // Bit 12
    FLAG_TIME_SYNC_VALID  = 0x02, // Bit 13
};

static constexpr uint8_t NODE_ID_SOURCE    = 0x0;
static constexpr uint8_t NODE_ID_BROADCAST = 0xF;
static constexpr size_t  MAX_SINK_NODES    = 6;

// Helper to construct 16-bit tag
inline constexpr uint16_t make_tag(uint8_t tx_id, uint8_t rx_id, uint8_t type, uint8_t flags = 0) {
    return static_cast<uint16_t>(
        (tx_id & 0x0F) |
        ((rx_id & 0x0F) << 4) |
        ((type & 0x0F) << 8) |
        ((flags & 0x0F) << 12)
    );
}

// Helpers to extract tag fields
inline constexpr uint8_t get_tx_id(uint16_t tag) { return static_cast<uint8_t>(tag & 0x0F); }
inline constexpr uint8_t get_rx_id(uint16_t tag) { return static_cast<uint8_t>((tag >> 4) & 0x0F); }
inline constexpr uint8_t get_pkt_type(uint16_t tag) { return static_cast<uint8_t>((tag >> 8) & 0x0F); }
inline constexpr uint8_t get_flags(uint16_t tag) { return static_cast<uint8_t>((tag >> 12) & 0x0F); }

// Primary & ARQ Audio Broadcast Frame Format
struct __attribute__((packed)) vsaf_audio_packet_t {
    uint16_t tag;             // 0xTTRS
    uint16_t seq;             // Sequence number (0-65535)
    uint32_t t_tx1_us;        // SOURCE microsecond presentation timestamp (esp_timer_get_time())
    uint8_t  sample_rate_khz; // 8, 16, 24, 32, 48
    uint16_t frame_dur_us;    // 10000 (10.0 ms) or 7500 (7.5 ms)
    uint8_t  octets;          // LC3 payload length (e.g. 120 bytes)
    uint8_t  data[120];       // LC3 compressed audio frame
};

// Soft-ACK Broadcast Feedback Frame Format (18 bytes)
struct __attribute__((packed)) vsaf_soft_ack_t {
    uint16_t tag;             // 0xTTRS (TYPE = 0x1)
    uint16_t ack_seq;         // Sequence number being acknowledged
    uint32_t t_tx1_echo;      // Echoed t_tx1_us from audio packet
    uint16_t t_dwell_us;      // Processing & turnaround time inside SINK (t_tx2 - t_rx1) in microseconds
    uint32_t t_sink_tx_us;    // Local SINK timestamp when soft-ACK was transmitted (t_tx2)
    int8_t   downlink_rssi;   // RSSI of received audio packet measured by SINK (-dBm)
    uint8_t  fifo_fill;       // SINK audio FIFO fill level (0-100%)
    uint16_t crc16;           // Simple checksum
};

// Simple fast 16-bit CRC for telemetry integrity
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
