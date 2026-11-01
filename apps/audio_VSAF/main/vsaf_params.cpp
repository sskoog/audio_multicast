/**
 * @file vsaf_params.cpp
 * @brief VSAF parameter table (see vsaf_params.hpp). Covers the SOURCE console commands that set or report a
 * value; text-only diagnostics (help, log, tasks, timing / ptp / peer / txstat / mem dumps, encbench, scan)
 * stay console-only. SINK-only commands (postgain, ch, driftcorr, syncoff, I2S tests...) are not in this table.
 */
#include "vsaf_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "usb_audio.hpp"
#include "vsaf_broadcast_engine.hpp"

namespace AudioNet {
namespace Params {

namespace {

EspNowBroadcastEngine* s_engine = nullptr;
portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
StatusSnapshot s_status = {};

const char* const kModeNames[] = {"pause", "cast", "noise"};  // noise = test tones (cal.mode picks which)
enum Mode : uint8_t { MODE_PAUSE = 0, MODE_CAST = 1, MODE_NOISE = 2 };
const char* const kOffOn[] = {"off", "on"};
const char* const kNoYes[] = {"no", "yes"};
const char* const kPhyNames[] = {"ht0", "ht1", "ht2", "ht3", "ofdm6", "ofdm9", "ofdm12", "ofdm18",
                                 "ofdm24", "ofdm36", "ofdm48", "ofdm54"};
const wifi_phy_rate_t kPhyRates[] = {WIFI_PHY_RATE_MCS0_LGI, WIFI_PHY_RATE_MCS1_LGI, WIFI_PHY_RATE_MCS2_LGI,
                                     WIFI_PHY_RATE_MCS3_LGI, WIFI_PHY_RATE_6M, WIFI_PHY_RATE_9M, WIFI_PHY_RATE_12M,
                                     WIFI_PHY_RATE_18M, WIFI_PHY_RATE_24M, WIFI_PHY_RATE_36M, WIFI_PHY_RATE_48M,
                                     WIFI_PHY_RATE_54M};
// esp_wifi_set_max_tx_power(): the radio only has these levels (esp_wifi.h mapping table, 0.25 dBm units);
// a setting between two levels transmits at the lower one
const int8_t kTxPowerQdbm[] = {8, 20, 28, 34, 44, 52, 56, 60, 66, 72, 80};
const char* const kTxPowerNames[] = {"2", "5", "7", "8.5", "11", "13", "14", "15", "16.5", "18", "20"};
const char* const kTxFaultNames[] = {"once", "wifi", "reboot"};
const char* const kCalNames[] = {"synth", "pink", "sine"};  // = EspNowBroadcastEngine::ToneKind
const char* const kHpOrderNames[] = {"0", "1", "2", "3", "4"};  // 0 = no high-pass
const char* const kLpOrderNames[] = {"2", "3", "4", "6"};
const int kLpOrders[] = {2, 3, 4, 6};
const char* const kHilbertModeNames[] = {"bypass", "on"};
const char* const kSubModeNames[] = {"bypass", "zc", "nccf"};

// IDs: high byte = group (0x01 system / network, 0x02 SOURCE pipeline, 0x03 radio status, 0x04 audio status,
// 0x10 audio / source / radio controls, 0x11 time sync, 0x12 calibration, 0x1F tests). Never reuse an ID for a
// different meaning; add new parameters with new IDs.
const Desc kTable[] = {
    {0x0100, "sys.cpu_pct", "%", Type::INT, Access::RO, 0, 100, 0, nullptr, 0,
     "CPU load: SOURCE (DSP + encode) / 2 cores, SINK decode / frame", 0},
    {0x0101, "sys.temp_c", "\xC2\xB0" "C", Type::FLOAT, Access::RO, -40, 125, 0, nullptr, 0,
     "On-chip temperature", 0},
    {0x0102, "sys.uptime_s", "s", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Time since boot", 0},
    {0x0103, "sys.heap_free_kb", "KB", Type::FLOAT, Access::RO, 0, 512, 0, nullptr, 0,
     "Free internal heap", 1},
    {0x0104, "sys.heap_min_kb", "KB", Type::FLOAT, Access::RO, 0, 512, 0, nullptr, 0,
     "Lowest free internal heap since boot", 1},
    {0x0105, "sys.heap_block_kb", "KB", Type::FLOAT, Access::RO, 0, 512, 0, nullptr, 0,
     "Largest free internal block", 1},
    {0x01F0, "sys.clear_counters", "", Type::INT, Access::ACTION, 0, 1, 0, nullptr, 0,
     "Reset frame, PLC and error counters (console clear)", 0},
    {0x01F1, "sys.reboot", "", Type::INT, Access::ACTION, 0, 1, 0, nullptr, 0,
     "Restart the SOURCE (console reboot)", 0},
    {0x01F2, "sys.bootloader", "", Type::INT, Access::ACTION, 0, 1, 0, nullptr, 0,
     "Reboot into the ROM download bootloader (COM16)", 0},
    {0x0110, "net.peers_online", "", Type::INT, Access::RO, 0, 6, 0, nullptr, 0,
     "SINKs online", 0},
    {0x0111, "net.peers_mask", "", Type::INT, Access::RO, 0, 63, 0, nullptr, 0,
     "Online channels, bit c = channel c (0 L, 1 R, 2 C, 3 LS, 4 RS, 5 Sub)", 0},
    {0x0130, "net.ch0_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 0 (L) uplink RSSI of its last reply", 0},
    {0x0131, "net.ch1_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 1 (R) uplink RSSI of its last reply", 0},
    {0x0132, "net.ch2_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 2 (C) uplink RSSI of its last reply", 0},
    {0x0133, "net.ch3_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 3 (LS) uplink RSSI of its last reply", 0},
    {0x0134, "net.ch4_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 4 (RS) uplink RSSI of its last reply", 0},
    {0x0135, "net.ch5_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 5 (Sub) uplink RSSI of its last reply", 0},
    {0x0140, "net.ch0_dl_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 0 (L) downlink RSSI the SINK measured on its REQ_ACK packet (SOURCE TX power shows here)", 0},
    {0x0141, "net.ch1_dl_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 1 (R) downlink RSSI the SINK measured on its REQ_ACK packet (SOURCE TX power shows here)", 0},
    {0x0142, "net.ch2_dl_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 2 (C) downlink RSSI the SINK measured on its REQ_ACK packet (SOURCE TX power shows here)", 0},
    {0x0143, "net.ch3_dl_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 3 (LS) downlink RSSI the SINK measured on its REQ_ACK packet (SOURCE TX power shows here)", 0},
    {0x0144, "net.ch4_dl_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 4 (RS) downlink RSSI the SINK measured on its REQ_ACK packet (SOURCE TX power shows here)", 0},
    {0x0145, "net.ch5_dl_rssi_dbm", "dBm", Type::INT, Access::RO, -127, 0, -127, nullptr, 0,
     "Channel 5 (Sub) downlink RSSI the SINK measured on its REQ_ACK packet (SOURCE TX power shows here)", 0},
    {0x0150, "net.ch0_reply_loss_pct", "%", Type::FLOAT, Access::RO, 0, 100, 0, nullptr, 0,
     "Channel 0 (L) telemetry replies missing in the last second (REQ_ACK packet or reply lost)", 1},
    {0x0151, "net.ch1_reply_loss_pct", "%", Type::FLOAT, Access::RO, 0, 100, 0, nullptr, 0,
     "Channel 1 (R) telemetry replies missing in the last second (REQ_ACK packet or reply lost)", 1},
    {0x0152, "net.ch2_reply_loss_pct", "%", Type::FLOAT, Access::RO, 0, 100, 0, nullptr, 0,
     "Channel 2 (C) telemetry replies missing in the last second (REQ_ACK packet or reply lost)", 1},
    {0x0153, "net.ch3_reply_loss_pct", "%", Type::FLOAT, Access::RO, 0, 100, 0, nullptr, 0,
     "Channel 3 (LS) telemetry replies missing in the last second (REQ_ACK packet or reply lost)", 1},
    {0x0154, "net.ch4_reply_loss_pct", "%", Type::FLOAT, Access::RO, 0, 100, 0, nullptr, 0,
     "Channel 4 (RS) telemetry replies missing in the last second (REQ_ACK packet or reply lost)", 1},
    {0x0155, "net.ch5_reply_loss_pct", "%", Type::FLOAT, Access::RO, 0, 100, 0, nullptr, 0,
     "Channel 5 (Sub) telemetry replies missing in the last second (REQ_ACK packet or reply lost)", 1},
    {0x0160, "net.ch0_replies_missed", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Channel 0 (L) telemetry replies missing since boot / clear", 0},
    {0x0161, "net.ch1_replies_missed", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Channel 1 (R) telemetry replies missing since boot / clear", 0},
    {0x0162, "net.ch2_replies_missed", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Channel 2 (C) telemetry replies missing since boot / clear", 0},
    {0x0163, "net.ch3_replies_missed", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Channel 3 (LS) telemetry replies missing since boot / clear", 0},
    {0x0164, "net.ch4_replies_missed", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Channel 4 (RS) telemetry replies missing since boot / clear", 0},
    {0x0165, "net.ch5_replies_missed", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Channel 5 (Sub) telemetry replies missing since boot / clear", 0},
    {0x0200, "dsp.time_ms", "ms", Type::FLOAT, Access::RO, 0, 10, 0, nullptr, 0,
     "DSP stage per frame", 2},
    {0x0201, "enc.hq_ms", "ms", Type::FLOAT, Access::RO, 0, 20, 0, nullptr, 0,
     "HQ + sub LC3 encode per frame, both cores", 2},
    {0x0202, "enc.mq_ms", "ms", Type::FLOAT, Access::RO, 0, 20, 0, nullptr, 0,
     "MQ (redundancy) LC3 encode per frame, both cores", 2},
    {0x0203, "src.overruns", "", Type::INT, Access::RO, 0, 1000, 0, nullptr, 0,
     "Frames lost to core-0 overruns in the last timing window (~1 s)", 0},
    {0x0204, "src.deadline_skips", "", Type::INT, Access::RO, 0, 1000, 0, nullptr, 0,
     "Packets not sent (9.5 ms deadline) in the last timing window", 0},
    {0x0210, "debug.dsp_load_us", "us", Type::INT, Access::RW, 0, 9000, 0, nullptr, 0,
     "Synthetic DSP load per frame (console dspload)", 0},
    {0x0211, "debug.enc_cores", "", Type::INT, Access::RW, 1, 2, 2, nullptr, 0,
     "LC3 encode on core 0 only (1) or both cores (2) (console enccores)", 0},
    {0x0220, "dsp.sub_voiced", "", Type::ENUM, Access::RO, 0, 1, 0, kNoYes, static_cast<uint8_t>(sizeof(kNoYes) / sizeof(kNoYes[0])),
     "Subharmonic NCCF voiced pitch lock active", 0},
    {0x0221, "dsp.sub_f0_hz", "Hz", Type::FLOAT, Access::RO, 0, 200, 0, nullptr, 0,
     "Subharmonic NCCF tracked fundamental f0", 1},
    {0x0222, "dsp.sub_fsub_hz", "Hz", Type::FLOAT, Access::RO, 0, 100, 0, nullptr, 0,
     "Subharmonic synthesized octave-down frequency (f0 / 2)", 1},
    {0x0223, "dsp.sub_nccf_peak", "", Type::FLOAT, Access::RO, -1, 1, 0, nullptr, 0,
     "Subharmonic NCCF normalized cross-correlation peak", 2},
    {0x0224, "dsp.sub_duck_gain", "", Type::FLOAT, Access::RO, 0, 1, 1, nullptr, 0,
     "Subharmonic direct-bass ducking gain", 2},
    {0x0300, "rf.tx_end_ms", "ms", Type::FLOAT, Access::RO, 0, 10, 0, nullptr, 0,
     "Last packet of the frame on air, after the frame tick (deadline 9.5 ms)", 2},
    {0x0301, "rf.tx_pkts_s", "1/s", Type::INT, Access::RO, 0, 1000, 0, nullptr, 0,
     "Packets sent per second", 0},
    {0x0302, "rf.tx_end_max_ms", "ms", Type::FLOAT, Access::RO, 0, 10, 0, nullptr, 0,
     "Worst TX end in the last timing window", 2},
    {0x0303, "rf.recoveries", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "TX-wedge recoveries since boot (console txstat)", 0},
    {0x0304, "rf.wifi_restarts", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Wi-Fi driver restarts by the recovery escalation", 0},
    {0x0305, "rf.wedge_reboots", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Reboots forced by an unrecoverable TX wedge, since power-on", 0},
    {0x0306, "rf.send_errors", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "esp_now_send() errors since boot", 0},
    {0x0307, "rf.no_mem_errors", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "esp_now_send() ESP_ERR_ESPNOW_NO_MEM since boot", 0},
    {0x0308, "rf.txdone_timeouts", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "TX-done callbacks not received in time, since boot", 0},
    {0x0309, "rf.status_fails", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "TX-done with a failure status, since boot", 0},
    {0x030A, "rf.deadline_drops", "", Type::INT, Access::RO, 0, 1e9, 0, nullptr, 0,
     "Packets dropped at the TX deadline, since boot", 0},
    {0x1002, "rf.channel", "", Type::INT, Access::RW, 1, 13, 13, nullptr, 0,
     "Wi-Fi channel (applied at the next frame boundary)", 0},
    {0x1003, "rf.phy", "", Type::ENUM, Access::RW, 0, 11, 3, kPhyNames, static_cast<uint8_t>(sizeof(kPhyNames) / sizeof(kPhyNames[0])),
     "Broadcast PHY rate (applied at the next frame boundary)", 0},
    {0x1004, "rf.tx_power_dbm", "dBm", Type::ENUM, Access::RW, 0, 10, 3, kTxPowerNames, static_cast<uint8_t>(sizeof(kTxPowerNames) / sizeof(kTxPowerNames[0])),
     "TX power: one of the radio's 11 levels (esp_wifi.h table); readback = level in effect for the setting", 0},
    {0x1F00, "test.txfault", "", Type::ENUM, Access::ACTION, 0, 2, 0, kTxFaultNames, static_cast<uint8_t>(sizeof(kTxFaultNames) / sizeof(kTxFaultNames[0])),
     "TX fault test: once (ESP-NOW re-init), wifi (Wi-Fi restart), reboot", 0},
    {0x0400, "audio.rms_dbfs", "dBFS", Type::FLOAT, Access::RO, -100, 0, -100, nullptr, 0,
     "SOURCE input RMS, -100 = silent or off", 1},
    {0x0410, "usb.streaming", "", Type::ENUM, Access::RO, 0, 1, 0, kNoYes, static_cast<uint8_t>(sizeof(kNoYes) / sizeof(kNoYes[0])),
     "Host is streaming USB audio", 0},
    {0x0411, "usb.host_vol_db", "dB", Type::FLOAT, Access::RO, -90, 0, 0, nullptr, 0,
     "USB host (Windows) volume as applied: lower of master and channels", 1},
    {0x0412, "usb.host_mute", "", Type::ENUM, Access::RO, 0, 1, 0, kOffOn, static_cast<uint8_t>(sizeof(kOffOn) / sizeof(kOffOn[0])),
     "USB host mute", 0},
    {0x1001, "audio.source", "", Type::ENUM, Access::RW, 0, 2, 1, kModeNames, static_cast<uint8_t>(sizeof(kModeNames) / sizeof(kModeNames[0])),
     "SOURCE input / state: pause (IDLE), cast (USB audio), noise (test tone of cal.mode)", 0},
    {0x1000, "audio.master_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Master volume (all channels), -96 = mute; set = target, read = in effect (100 dB/s ramp)", 1},
    {0x1005, "audio.mute", "", Type::ENUM, Access::RW, 0, 1, 0, kOffOn, static_cast<uint8_t>(sizeof(kOffOn) / sizeof(kOffOn[0])),
     "Master mute, ramped (console mute / unmute)", 0},
    {0x1010, "audio.ch0_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Channel 0 (L) gain on the SOURCE, ramped 100 dB/s (console volch); not applied to caltone", 1},
    {0x1011, "audio.ch1_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Channel 1 (R) gain on the SOURCE, ramped 100 dB/s (console volch); not applied to caltone", 1},
    {0x1012, "audio.ch2_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Channel 2 (C) gain on the SOURCE, ramped 100 dB/s (console volch); not applied to caltone", 1},
    {0x1013, "audio.ch3_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Channel 3 (LS) gain on the SOURCE, ramped 100 dB/s (console volch); not applied to caltone", 1},
    {0x1014, "audio.ch4_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Channel 4 (RS) gain on the SOURCE, ramped 100 dB/s (console volch); not applied to caltone", 1},
    {0x1015, "audio.ch5_db", "dB", Type::FLOAT, Access::RW, -96, 0, 0, nullptr, 0,
     "Channel 5 (Sub) gain on the SOURCE, ramped 100 dB/s (console volch); not applied to caltone", 1},
    {0x1100, "sync.latency_ms", "ms", Type::INT, Access::RW, 20, 200, 50, nullptr, 0,
     "Playout latency (console latency)", 0},
    {0x1110, "sync.ch0_trim_us", "us", Type::INT, Access::RW, -30000, 30000, 0, nullptr, 0,
     "Channel 0 (L) presentation-time trim, + = later (console trim)", 0},
    {0x1111, "sync.ch1_trim_us", "us", Type::INT, Access::RW, -30000, 30000, 0, nullptr, 0,
     "Channel 1 (R) presentation-time trim, + = later (console trim)", 0},
    {0x1112, "sync.ch2_trim_us", "us", Type::INT, Access::RW, -30000, 30000, 0, nullptr, 0,
     "Channel 2 (C) presentation-time trim, + = later (console trim)", 0},
    {0x1113, "sync.ch3_trim_us", "us", Type::INT, Access::RW, -30000, 30000, 0, nullptr, 0,
     "Channel 3 (LS) presentation-time trim, + = later (console trim)", 0},
    {0x1114, "sync.ch4_trim_us", "us", Type::INT, Access::RW, -30000, 30000, 0, nullptr, 0,
     "Channel 4 (RS) presentation-time trim, + = later (console trim)", 0},
    {0x1115, "sync.ch5_trim_us", "us", Type::INT, Access::RW, -30000, 30000, 0, nullptr, 0,
     "Channel 5 (Sub) presentation-time trim, + = later (console trim)", 0},
    {0x1120, "sync.ch0_rtt_us", "us", Type::INT, Access::RO, 0, 20000, 0, nullptr, 0,
     "Channel 0 (L) best two-step round trip, 0 = no estimate", 0},
    {0x1121, "sync.ch1_rtt_us", "us", Type::INT, Access::RO, 0, 20000, 0, nullptr, 0,
     "Channel 1 (R) best two-step round trip, 0 = no estimate", 0},
    {0x1122, "sync.ch2_rtt_us", "us", Type::INT, Access::RO, 0, 20000, 0, nullptr, 0,
     "Channel 2 (C) best two-step round trip, 0 = no estimate", 0},
    {0x1123, "sync.ch3_rtt_us", "us", Type::INT, Access::RO, 0, 20000, 0, nullptr, 0,
     "Channel 3 (LS) best two-step round trip, 0 = no estimate", 0},
    {0x1124, "sync.ch4_rtt_us", "us", Type::INT, Access::RO, 0, 20000, 0, nullptr, 0,
     "Channel 4 (RS) best two-step round trip, 0 = no estimate", 0},
    {0x1125, "sync.ch5_rtt_us", "us", Type::INT, Access::RO, 0, 20000, 0, nullptr, 0,
     "Channel 5 (Sub) best two-step round trip, 0 = no estimate", 0},
    {0x1130, "sync.ch0_drift_ppm", "ppm", Type::FLOAT, Access::RO, -200, 200, 0, nullptr, 0,
     "Channel 0 (L) SINK clock vs SOURCE", 2},
    {0x1131, "sync.ch1_drift_ppm", "ppm", Type::FLOAT, Access::RO, -200, 200, 0, nullptr, 0,
     "Channel 1 (R) SINK clock vs SOURCE", 2},
    {0x1132, "sync.ch2_drift_ppm", "ppm", Type::FLOAT, Access::RO, -200, 200, 0, nullptr, 0,
     "Channel 2 (C) SINK clock vs SOURCE", 2},
    {0x1133, "sync.ch3_drift_ppm", "ppm", Type::FLOAT, Access::RO, -200, 200, 0, nullptr, 0,
     "Channel 3 (LS) SINK clock vs SOURCE", 2},
    {0x1134, "sync.ch4_drift_ppm", "ppm", Type::FLOAT, Access::RO, -200, 200, 0, nullptr, 0,
     "Channel 4 (RS) SINK clock vs SOURCE", 2},
    {0x1135, "sync.ch5_drift_ppm", "ppm", Type::FLOAT, Access::RO, -200, 200, 0, nullptr, 0,
     "Channel 5 (Sub) SINK clock vs SOURCE", 2},
    {0x1140, "sync.ch0_err_us", "us", Type::INT, Access::RO, -32767, 32767, 0, nullptr, 0,
     "Channel 0 (L) playout error reported by the SINK (+ = late), - = unknown", 0},
    {0x1141, "sync.ch1_err_us", "us", Type::INT, Access::RO, -32767, 32767, 0, nullptr, 0,
     "Channel 1 (R) playout error reported by the SINK (+ = late), - = unknown", 0},
    {0x1142, "sync.ch2_err_us", "us", Type::INT, Access::RO, -32767, 32767, 0, nullptr, 0,
     "Channel 2 (C) playout error reported by the SINK (+ = late), - = unknown", 0},
    {0x1143, "sync.ch3_err_us", "us", Type::INT, Access::RO, -32767, 32767, 0, nullptr, 0,
     "Channel 3 (LS) playout error reported by the SINK (+ = late), - = unknown", 0},
    {0x1144, "sync.ch4_err_us", "us", Type::INT, Access::RO, -32767, 32767, 0, nullptr, 0,
     "Channel 4 (RS) playout error reported by the SINK (+ = late), - = unknown", 0},
    {0x1145, "sync.ch5_err_us", "us", Type::INT, Access::RO, -32767, 32767, 0, nullptr, 0,
     "Channel 5 (Sub) playout error reported by the SINK (+ = late), - = unknown", 0},
    {0x1200, "cal.mode", "", Type::ENUM, Access::RW, 0, 2, 0, kCalNames, static_cast<uint8_t>(sizeof(kCalNames) / sizeof(kCalNames[0])),
     "Test tone played while audio.source = noise: synth, pink (500-2000 Hz + 40-100 Hz), sine (cal.sat_hz + cal.sub_hz); level = audio.master_db", 0},
    {0x1202, "cal.sat_hz", "Hz", Type::INT, Access::RW, 100, 10000, 1000, nullptr, 0,
     "Sine test tone: satellite-band frequency", 0},
    {0x1203, "cal.sub_hz", "Hz", Type::INT, Access::RW, 20, 100, 63, nullptr, 0,
     "Sine test tone: sub-band frequency", 0},
    {0x1300, "xo.hp_hilbert_hz", "Hz", Type::FLOAT, Access::RW, 70, 200, 100, nullptr, 0,
     "Satellite high-pass cutoff before the Hilbert transform", 1},
    {0x1306, "xo.hp_hilbert_order", "", Type::ENUM, Access::RW, 0, 4, 2, kHpOrderNames, static_cast<uint8_t>(sizeof(kHpOrderNames) / sizeof(kHpOrderNames[0])),
     "Satellite high-pass order before the Hilbert transform (0 = no filter)", 0},
    {0x1307, "xo.lp_sub_hz", "Hz", Type::FLOAT, Access::RW, 70, 200, 100, nullptr, 0,
     "Subwoofer low-pass cutoff in the 48 -> 8 kHz downsampling stage", 1},
    {0x1308, "xo.lp_sub_order", "", Type::ENUM, Access::RW, 0, 3, 2, kLpOrderNames, static_cast<uint8_t>(sizeof(kLpOrderNames) / sizeof(kLpOrderNames[0])),
     "Subwoofer low-pass Butterworth order (2, 3, 4, 6)", 0},
    {0x1301, "dsp.hilbert.animate", "", Type::ENUM, Access::RW, 0, 1, 1, kHilbertModeNames, static_cast<uint8_t>(sizeof(kHilbertModeNames) / sizeof(kHilbertModeNames[0])),
     "5-satellite Hilbert quadrature spatial upmixer: bypass or on", 0},
    {0x1312, "dsp.lfo_hz", "Hz", Type::FLOAT, Access::RW, -5, 5, 0, nullptr, 0,
     "Satellite spatial azimuth LFO modulation rate (0 = off)", 2},
    {0x1313, "dsp.lfo_span_deg", "deg", Type::FLOAT, Access::RW, -360, 360, 180, nullptr, 0,
     "Satellite spatial azimuth LFO modulation span", 1},
    {0x1310, "dsp.mix_wet", "", Type::FLOAT, Access::RW, 0, 1, 1, nullptr, 0,
     "Satellite spatial wet/dry blend (0 = dry HPF, 1 = full Hilbert upmix)", 2},
    {0x1311, "dsp.side_widening", "", Type::FLOAT, Access::RW, 0, 2, 1, nullptr, 0,
     "Satellite Side (L-R) width gain before quadrature steering", 2},
    {0x1320, "dsp.ch0_azimuth_deg", "deg", Type::FLOAT, Access::RW, -360, 360, -90, nullptr, 0,
     "Channel 0 (L) Hilbert spatial steering angle", 1},
    {0x1321, "dsp.ch1_azimuth_deg", "deg", Type::FLOAT, Access::RW, -360, 360, 90, nullptr, 0,
     "Channel 1 (R) Hilbert spatial steering angle", 1},
    {0x1322, "dsp.ch2_azimuth_deg", "deg", Type::FLOAT, Access::RW, -360, 360, 0, nullptr, 0,
     "Channel 2 (C) Hilbert spatial steering angle", 1},
    {0x1323, "dsp.ch3_azimuth_deg", "deg", Type::FLOAT, Access::RW, -360, 360, -135, nullptr, 0,
     "Channel 3 (LS) Hilbert spatial steering angle", 1},
    {0x1324, "dsp.ch4_azimuth_deg", "deg", Type::FLOAT, Access::RW, -360, 360, 135, nullptr, 0,
     "Channel 4 (RS) Hilbert spatial steering angle", 1},
    {0x1330, "dsp.ch0_dfs_hz", "Hz", Type::FLOAT, Access::RW, -20, 20, 0, nullptr, 0,
     "Channel 0 (L) Barberpole single-sideband frequency shift", 2},
    {0x1331, "dsp.ch1_dfs_hz", "Hz", Type::FLOAT, Access::RW, -20, 20, 0, nullptr, 0,
     "Channel 1 (R) Barberpole single-sideband frequency shift", 2},
    {0x1332, "dsp.ch2_dfs_hz", "Hz", Type::FLOAT, Access::RW, -20, 20, 0, nullptr, 0,
     "Channel 2 (C) Barberpole single-sideband frequency shift", 2},
    {0x1333, "dsp.ch3_dfs_hz", "Hz", Type::FLOAT, Access::RW, -20, 20, 0, nullptr, 0,
     "Channel 3 (LS) Barberpole single-sideband frequency shift", 2},
    {0x1334, "dsp.ch4_dfs_hz", "Hz", Type::FLOAT, Access::RW, -20, 20, 0, nullptr, 0,
     "Channel 4 (RS) Barberpole single-sideband frequency shift", 2},
    {0x1302, "sub.mode", "", Type::ENUM, Access::RW, 0, 2, 0, kSubModeNames, static_cast<uint8_t>(sizeof(kSubModeNames) / sizeof(kSubModeNames[0])),
     "Subwoofer subharmonic resonator: bypass (LR4 LPF), zc (flipper), nccf (pitch-locked)", 0},
    {0x1303, "sub.dry_gain", "", Type::FLOAT, Access::RW, 0, 2, 0, nullptr, 0,
     "Subharmonic resonator dry LPF bass linear gain", 2},
    {0x1304, "sub.wet_gain", "", Type::FLOAT, Access::RW, 0, 2, 1, nullptr, 0,
     "Subharmonic resonator wet octave-down linear gain", 2},
    {0x1305, "sub.zc_lpf_hz", "Hz", Type::FLOAT, Access::RW, 50, 200, 100, nullptr, 0,
     "Zero-crossing flipper post-LPF smoothing cutoff", 1},
};
constexpr size_t kCount = sizeof(kTable) / sizeof(kTable[0]);

float currentMode() {
    if (!s_engine) return MODE_PAUSE;
    if (s_engine->getState() != NetworkState::CAST) return MODE_PAUSE;
    return s_engine->isToneTestMode() ? MODE_NOISE : MODE_CAST;
}

/** @brief Deferred reboot so the HID / console reply goes out first (action parameters). */
void scheduleRestart(bool to_bootloader) {
    static esp_timer_handle_t s_timer = nullptr;
    static bool s_boot = false;
    s_boot = to_bootloader;
    if (!s_timer) {
        esp_timer_create_args_t a = {};
        a.callback = [](void*) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
            if (s_boot) usb_reboot_to_rom_download();
#endif
            esp_restart();
        };
        a.name = "param_reboot";
        if (esp_timer_create(&a, &s_timer) != ESP_OK) return;
    }
    esp_timer_start_once(s_timer, 300000);
}

float kb(size_t bytes) { return static_cast<float>(bytes) / 1024.0f; }

}  // namespace

const char* setStatusName(SetStatus s) {
    switch (s) {
        case SetStatus::OK: return "ok";
        case SetStatus::CLAMPED: return "clamped";
        case SetStatus::READ_ONLY: return "read_only";
        case SetStatus::UNKNOWN: return "unknown";
        case SetStatus::INVALID: return "invalid";
    }
    return "?";
}

void bindEngine(EspNowBroadcastEngine* engine) { s_engine = engine; }

void publishStatus(const StatusSnapshot& s) {
    portENTER_CRITICAL(&s_status_mux);
    const uint32_t seq = s_status.seq + 1;
    s_status = s;
    s_status.seq = seq ? seq : 1;
    portEXIT_CRITICAL(&s_status_mux);
}

StatusSnapshot latestStatus() {
    portENTER_CRITICAL(&s_status_mux);
    const StatusSnapshot s = s_status;
    portEXIT_CRITICAL(&s_status_mux);
    return s;
}

size_t count() { return kCount; }
const Desc& descAt(size_t index) { return kTable[index < kCount ? index : 0]; }

const Desc* find(uint16_t id) {
    for (const Desc& d : kTable) {
        if (d.id == id) return &d;
    }
    return nullptr;
}

const Desc* findByText(const char* text) {
    if (!text || !*text) return nullptr;
    for (const Desc& d : kTable) {
        if (strcasecmp(d.name, text) == 0) return &d;
    }
    char* end = nullptr;
    const unsigned long v = strtoul(text, &end, 0);  // "0x0100" or "256"
    if (end && *end == '\0' && v <= 0xFFFF) return find(static_cast<uint16_t>(v));
    return nullptr;
}

bool get(uint16_t id, float& value) {
    const StatusSnapshot s = latestStatus();
    EspNowBroadcastEngine* e = s_engine;
    // Per-channel groups
    if (id >= 0x0140 && id < 0x0146) {
        const SinkPeerConfig* p = e ? e->getPeer(id - 0x0140) : nullptr;
        value = (p && p->status == PeerStatus::ONLINE) ? p->downlink_rssi : -127;
        return true;
    }
    if (id >= 0x0150 && id < 0x0156) {
        value = e ? e->getReplyLossPct(static_cast<uint8_t>(id - 0x0150)) : NAN;
        return true;
    }
    if (id >= 0x0160 && id < 0x0166) {
        value = e ? e->getRepliesMissed(static_cast<uint8_t>(id - 0x0160)) : 0;
        return true;
    }
    if (id >= 0x0130 && id < 0x0136) {
        const SinkPeerConfig* p = e ? e->getPeer(id - 0x0130) : nullptr;
        value = (p && p->status == PeerStatus::ONLINE) ? p->last_rssi : -127;
        return true;
    }
    if (id >= 0x1010 && id < 0x1016) {
        value = e ? e->getChannelGainNowDb(id - 0x1010) : 0;
        return true;
    }
    if (id >= 0x1320 && id < 0x1325) {
        value = e ? e->getDspChannelAzimuthDeg(id - 0x1320) : 0;
        return true;
    }
    if (id >= 0x1330 && id < 0x1335) {
        value = e ? e->getDspChannelDfsHz(id - 0x1330) : 0;
        return true;
    }
    if (id >= 0x0220 && id <= 0x0224) {
        bool voiced = false;
        float f0 = 0, fsub = 0, peak = 0, duck = 1;
        if (e) e->getDspSubDiagnostics(voiced, f0, fsub, peak, duck);
        switch (id) {
            case 0x0220: value = voiced ? 1 : 0; return true;
            case 0x0221: value = f0; return true;
            case 0x0222: value = fsub; return true;
            case 0x0223: value = peak; return true;
            case 0x0224: value = duck; return true;
            default: return false;
        }
    }
    if (id >= 0x1110 && id < 0x1146) {
        // Not getPtpStatus(): it has side effects (drift update, holdover count); these are cached per frame
        if (!e) { value = NAN; return true; }
        const uint8_t ch = static_cast<uint8_t>(id & 0x0F);
        if (ch >= MAX_SINK_NODES) return false;
        switch (id & 0xFFF0) {
            case 0x1110: value = e->getChannelTrimUs(ch); return true;
            case 0x1120: value = e->getCtrlRttUs(ch); return true;
            case 0x1130: value = e->getCtrlRttUs(ch) ? e->getCtrlDriftPpb(ch) / 1000.0f : NAN; return true;
            case 0x1140: {
                const int16_t err = e->getPlayoutErrUs(ch);
                value = (err == VSAF_PLAYOUT_ERR_UNKNOWN) ? NAN : err;
                return true;
            }
            default: return false;
        }
    }
    SourceTimingSnapshot t{};
    switch (id) {
        case 0x0100: value = s.cpu_pct; return true;
        case 0x0101: value = s.temp_c; return true;
        case 0x0102: value = static_cast<float>(esp_timer_get_time() / 1000000); return true;
        case 0x0103: value = kb(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)); return true;
        case 0x0104: value = kb(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)); return true;
        case 0x0105: value = kb(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)); return true;
        case 0x01F0: case 0x01F1: case 0x01F2: case 0x1F00: value = 0; return true;  // actions
        case 0x0110: value = s.peers_online; return true;
        case 0x0111: value = s.peers_mask; return true;
        case 0x0200: value = s.dsp_ms; return true;
        case 0x0201: value = s.enc_hq_ms; return true;
        case 0x0202: value = s.enc_mq_ms; return true;
        case 0x0203: if (e) e->getSourceTiming(t); value = t.overruns; return true;
        case 0x0204: if (e) e->getSourceTiming(t); value = t.deadline_skips; return true;
        case 0x0210: value = e ? e->getDspExtraLoadUs() : 0; return true;
        case 0x0211: value = e ? e->getEncoderCores() : 2; return true;
        case 0x0300: value = s.tx_end_ms; return true;
        case 0x0301: value = s.tx_pkts_s; return true;
        case 0x0302: if (e) e->getSourceTiming(t); value = t.tx_last_done_at.max / 1000.0f; return true;
        case 0x0303: value = e ? e->getTxRecoveryCount() : 0; return true;
        case 0x0304: value = e ? e->getWifiRestartCount() : 0; return true;
        case 0x0305: value = EspNowBroadcastEngine::getTxWedgeReboots(); return true;
        case 0x0306: value = e ? e->getTxMacErrorCount() : 0; return true;
        case 0x0307: value = e ? e->getTxNoMemCount() : 0; return true;
        case 0x0308: value = e ? e->getTxTimeoutCount() : 0; return true;
        case 0x0309: value = e ? e->getTxFailCount() : 0; return true;
        case 0x030A: value = e ? e->getTxDeadlineDropsCount() : 0; return true;
        case 0x1002: value = e ? e->getWifiChannelTarget() : 0; return true;  // incl. a change not yet applied
        case 0x1003: {
            value = NAN;
            if (!e) return true;
            for (size_t i = 0; i < sizeof(kPhyRates) / sizeof(kPhyRates[0]); ++i) {
                if (kPhyRates[i] == e->getWifiPhyRateTarget()) value = i;
            }
            return true;
        }
        case 0x1004: {
            // Level the radio uses for the current setting: the highest table entry not above it
            value = 0;
            const int8_t q = e ? e->getWifiTxPower() : 0;
            for (size_t i = 0; i < sizeof(kTxPowerQdbm); ++i) {
                if (kTxPowerQdbm[i] <= q) value = i;
            }
            return true;
        }
        case 0x0400: value = s.rms_dbfs; return true;
        case 0x0410: value = usb_audio_is_streaming() ? 1 : 0; return true;
        case 0x0411:
        case 0x0412: {
            int8_t m[3];
            int16_t v[3];
            uint32_t n = 0;
            usb_audio_get_fu_state(m, v, &n);
            if (id == 0x0412) {
                value = (m[0] || (m[1] && m[2])) ? 1 : 0;
            } else {
                const int16_t lo = std::min(v[0], std::max(v[1], v[2]));
                value = lo / 256.0f;
            }
            return true;
        }
        case 0x1000: value = e ? e->getMasterGainNowDb() : 0; return true;
        case 0x1005: value = (e && e->getMasterMute()) ? 1 : 0; return true;
        case 0x1001: value = currentMode(); return true;
        case 0x1100: value = e ? e->getPlayoutLatencyUs() / 1000.0f : 0; return true;
        case 0x1200: value = e ? e->getToneKind() : 0; return true;
        case 0x1202: value = e ? e->getCalSignal().satHz() : 1000; return true;
        case 0x1203: value = e ? e->getCalSignal().subHz() : 63; return true;
        case 0x1300: value = e ? e->getXoHpHz() : 100; return true;
        case 0x1306: value = e ? e->getXoHpOrder() : 2; return true;
        case 0x1307: value = e ? e->getXoLpHz() : 100; return true;
        case 0x1308: {
            value = NAN;
            const int o = e ? e->getXoLpOrder() : 4;
            for (size_t i = 0; i < sizeof(kLpOrders) / sizeof(kLpOrders[0]); ++i) {
                if (kLpOrders[i] == o) value = i;
            }
            return true;
        }
        case 0x1301: value = (e && !e->getDspHilbertBypass()) ? 1 : 0; return true;
        case 0x1302: value = e ? e->getDspSubMode() : 0; return true;
        case 0x1303: value = e ? e->getDspSubDryGain() : 0; return true;
        case 0x1304: value = e ? e->getDspSubWetGain() : 1; return true;
        case 0x1305: value = e ? e->getDspZcPostLpfHz() : 100; return true;
        case 0x1310: value = e ? e->getDspSatWet() : 1; return true;
        case 0x1311: value = e ? e->getDspSatSideGain() : 1; return true;
        case 0x1312: value = e ? e->getDspSatLfoHz() : 0; return true;
        case 0x1313: value = e ? e->getDspSatLfoSpanDeg() : 180; return true;
        default: return false;
    }
}

SetStatus set(uint16_t id, float value, float& applied) {
    const Desc* d = find(id);
    if (!d) return SetStatus::UNKNOWN;
    if (d->access == Access::RO) return SetStatus::READ_ONLY;
    if (!s_engine || std::isnan(value)) return SetStatus::INVALID;
    EspNowBroadcastEngine* e = s_engine;
    float v = std::clamp(value, d->min, d->max);
    if (d->type != Type::FLOAT) v = std::round(v);
    const SetStatus st = (v != value) ? SetStatus::CLAMPED : SetStatus::OK;
    const int iv = static_cast<int>(v);
    if (id >= 0x1010 && id < 0x1016) {
        e->setChannelGainDb(id - 0x1010, v);
    } else if (id >= 0x1110 && id < 0x1116) {
        e->setChannelTrimUs(static_cast<uint8_t>(id - 0x1110), static_cast<int16_t>(iv));
    } else if (id >= 0x1320 && id < 0x1325) {
        e->setDspChannelAzimuthDeg(id - 0x1320, v);
    } else if (id >= 0x1330 && id < 0x1335) {
        e->setDspChannelDfsHz(id - 0x1330, v);
    } else {
        switch (id) {
            case 0x01F0:
                e->resetStreamingCounters();
                e->resetErrorCounters();
                break;
            case 0x01F1: scheduleRestart(false); break;
            case 0x01F2: scheduleRestart(true); break;
            case 0x1F00:
                if (iv == 0) e->requestTxFaultInjection();
                else e->simulateTxWedge(iv);  // 1: healed by a Wi-Fi restart, 2: only by a reboot
                break;
            case 0x0210: e->setDspExtraLoadUs(static_cast<uint32_t>(iv)); break;
            case 0x0211: e->setEncoderCores(static_cast<uint32_t>(iv)); break;
            case 0x1002:
                e->requestWifiChannel(static_cast<uint8_t>(iv));  // at the next frame boundary
                e->lockChannel(true);
                break;
            case 0x1003:
                e->requestWifiPhyRate(iv <= 3 ? WIFI_PHY_MODE_HT20 : WIFI_PHY_MODE_11G, kPhyRates[iv]);
                break;
            case 0x1004:
                e->setWifiTxPower(kTxPowerQdbm[iv]);
                break;
            case 0x1000: e->setMasterGainDb(v); break;
            case 0x1005: e->setMasterMute(iv != 0); break;
            case 0x1001: {
                const auto m = static_cast<Mode>(iv);
                if (m == MODE_PAUSE) {
                    e->setToneTestMode(false);
                    e->transitionTo(NetworkState::IDLE);
                } else {
                    e->setToneTestMode(m == MODE_NOISE);
                    e->transitionTo(NetworkState::CAST);
                }
                break;
            }
            case 0x1100: e->setPlayoutLatencyUs(static_cast<uint32_t>(iv) * 1000U); break;
            case 0x1200: e->setToneKind(static_cast<uint8_t>(iv)); break;  // audio.source = noise plays it
            case 0x1202: case 0x1203: {
                // Sine frequencies: the pink / sine generator is configured as a whole
                const Audio::CalSignal& cs = e->getCalSignal();
                const float sat = (id == 0x1202) ? v : cs.satHz();
                const float sub = (id == 0x1203) ? v : cs.subHz();
                e->setCalSignal(cs.mode(), sat, sub);
                break;
            }
            case 0x1300: e->setXoHpHz(v); break;
            case 0x1306: e->setXoHpOrder(iv); break;
            case 0x1307: e->setXoLpHz(v); break;
            case 0x1308: e->setXoLpOrder(kLpOrders[iv]); break;
            case 0x1301: e->setDspHilbertBypass(iv == 0); break;
            case 0x1302: e->setDspSubMode(static_cast<uint8_t>(iv)); break;
            case 0x1303: e->setDspSubDryGain(v); break;
            case 0x1304: e->setDspSubWetGain(v); break;
            case 0x1305: e->setDspZcPostLpfHz(v); break;
            case 0x1310: e->setDspSatWet(v); break;
            case 0x1311: e->setDspSatSideGain(v); break;
            case 0x1312: e->setDspSatLfoHz(v); break;
            case 0x1313: e->setDspSatLfoSpanDeg(v); break;
            default:
                return SetStatus::READ_ONLY;
        }
    }
    if (d->access == Access::ACTION) {
        applied = v;
    } else {
        get(id, applied);  // the value now in effect (ramped values: where the ramp is right now)
    }
    return st;
}

bool parseValue(const Desc& d, const char* text, float& value) {
    if (!text || !*text) return false;
    if (d.type == Type::ENUM) {
        for (uint8_t i = 0; i < d.enum_count; ++i) {
            if (strcasecmp(text, d.enum_names[i]) == 0) {
                value = i;
                return true;
            }
        }
    }
    char* end = nullptr;
    const float v = strtof(text, &end);
    if (!end || *end != '\0') return false;
    value = v;
    return true;
}

void formatValue(const Desc& d, float value, char* out, size_t out_len) {
    if (std::isnan(value)) {
        snprintf(out, out_len, "-");
    } else if (d.type == Type::ENUM) {
        const int i = static_cast<int>(value);
        snprintf(out, out_len, "%s", (i >= 0 && i < d.enum_count) ? d.enum_names[i] : "?");
    } else if (d.type == Type::INT || d.decimals == 0) {
        snprintf(out, out_len, "%ld", static_cast<long>(std::lround(value)));
    } else {
        snprintf(out, out_len, "%.*f", static_cast<int>(d.decimals), static_cast<double>(value));
    }
}

}  // namespace Params
}  // namespace AudioNet
