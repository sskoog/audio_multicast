#include "usb_audio.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "tinyusb.h"
#include "tusb.h"
#include "soc/rtc_cntl_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include <atomic>

static const char *TAG = "USBaudio";

static std::atomic<bool> s_audio_alt_active(false);
static std::atomic<int64_t> s_last_audio_rx_us(0);


// -----------------------------------------------------------------------------
// Audio Class configuration
// -----------------------------------------------------------------------------
#define AUDIO_SAMPLE_RATE    48000
#define AUDIO_CHANNELS       2
#define AUDIO_BYTES_PER_SAMP 2
#define AUDIO_BPS            16

// Maximum packet size for ISO endpoint
// 48 kHz * 2 channels * 2 bytes = 192 bytes/ms
#define AUDIO_EP_SIZE        ((AUDIO_SAMPLE_RATE / 1000) * AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMP)

// Interface numbers
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_AUDIO_CONTROL,
    ITF_NUM_AUDIO_STREAMING,
    ITF_NUM_TOTAL
};

// Endpoint numbers
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82
#define EPNUM_AUDIO_OUT   0x03

// String descriptor indices
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC_INTERFACE,
    STRID_AUDIO_INTERFACE,
    STRID_MAC
};

// -----------------------------------------------------------------------------
// USB Descriptors
// -----------------------------------------------------------------------------
// USB Descriptors
// -----------------------------------------------------------------------------

#define TUD_AUDIO10_DESC_IAD_LEN 8
#define TUD_AUDIO10_DESC_IAD(_firstitf, _nitfs, _stridx) \
  TUD_AUDIO10_DESC_IAD_LEN, TUSB_DESC_INTERFACE_ASSOCIATION, _firstitf, _nitfs, TUSB_CLASS_AUDIO, AUDIO_FUNCTION_SUBCLASS_UNDEFINED, AUDIO_INT_PROTOCOL_CODE_V1, _stridx

#define TUD_AUDIO_SPEAKER_DESC_LEN ( \
  TUD_AUDIO10_DESC_IAD_LEN + \
  TUD_AUDIO10_DESC_STD_AC_LEN + \
  TUD_AUDIO10_DESC_CS_AC_LEN(1) + \
  TUD_AUDIO10_DESC_INPUT_TERM_LEN + \
  TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2) + \
  TUD_AUDIO10_DESC_OUTPUT_TERM_LEN + \
  TUD_AUDIO10_DESC_STD_AS_LEN + \
  TUD_AUDIO10_DESC_STD_AS_LEN + \
  TUD_AUDIO10_DESC_CS_AS_INT_LEN + \
  TUD_AUDIO10_DESC_TYPE_I_FORMAT_LEN(1) + \
  TUD_AUDIO10_DESC_STD_AS_ISO_EP_LEN + \
  TUD_AUDIO10_DESC_CS_AS_ISO_EP_LEN)

#define TUD_AUDIO_SPEAKER_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epout, _epsize) \
  /* Interface Association Descriptor (IAD) */ \
  TUD_AUDIO10_DESC_IAD(_itfnum, 2, _stridx),\
  /* Standard AC Interface Descriptor (4.3.1) */ \
  TUD_AUDIO10_DESC_STD_AC(_itfnum, 0x00, _stridx),\
  /* Class-Specific AC Interface Header Descriptor (4.3.2) */ \
  TUD_AUDIO10_DESC_CS_AC(0x0100, (TUD_AUDIO10_DESC_INPUT_TERM_LEN + TUD_AUDIO10_DESC_OUTPUT_TERM_LEN + TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2)), ((_itfnum)+1)),\
  /* Input Terminal Descriptor (4.3.2.1) */ \
  TUD_AUDIO10_DESC_INPUT_TERM(0x01, AUDIO_TERM_TYPE_USB_STREAMING, 0x00, 0x02, (AUDIO10_CHANNEL_CONFIG_LEFT_FRONT | AUDIO10_CHANNEL_CONFIG_RIGHT_FRONT), 0x00, 0x00),\
  /* Output Terminal Descriptor (4.3.2.2) */ \
  TUD_AUDIO10_DESC_OUTPUT_TERM(0x03, AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, 0x00, 0x02, STRID_AUDIO_INTERFACE),\
  /* Feature Unit Descriptor (4.3.2.5) */ \
  TUD_AUDIO10_DESC_FEATURE_UNIT(0x02, 0x01, 0x00, (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME), (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME), (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME)),\
  /* Standard AS Interface Descriptor (4.5.1) - Alt 0 (0 bandwidth) */ \
  TUD_AUDIO10_DESC_STD_AS_INT((uint8_t)((_itfnum)+1), 0x00, 0x00, 0x00),\
  /* Standard AS Interface Descriptor (4.5.1) - Alt 1 (Data streaming) */ \
  TUD_AUDIO10_DESC_STD_AS_INT((uint8_t)((_itfnum)+1), 0x01, 0x01, 0x00),\
  /* Class-Specific AS Interface Descriptor (4.5.2) */ \
  TUD_AUDIO10_DESC_CS_AS_INT(0x01, 0x01, AUDIO10_DATA_FORMAT_TYPE_I_PCM),\
  /* Type I Format Type Descriptor (2.2.5) */ \
  TUD_AUDIO10_DESC_TYPE_I_FORMAT(0x02, _nBytesPerSample, _nBitsUsedPerSample, 48000),\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor (4.6.1.1) */ \
  TUD_AUDIO10_DESC_STD_AS_ISO_EP(_epout, (uint8_t) ((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ADAPTIVE), _epsize, 0x01, 0x00),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor (4.6.1.2) */ \
  TUD_AUDIO10_DESC_CS_AS_ISO_EP(AUDIO10_CS_AS_ISO_DATA_EP_ATT_SAMPLING_FRQ, AUDIO10_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, 0x0001)

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_AUDIO_SPEAKER_DESC_LEN)

const uint8_t uac_cdc_configuration_descriptor[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // CDC
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC_INTERFACE, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    // Audio Speaker (UAC1)
    TUD_AUDIO_SPEAKER_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, STRID_AUDIO_INTERFACE, AUDIO_BYTES_PER_SAMP, AUDIO_BPS, EPNUM_AUDIO_OUT, AUDIO_EP_SIZE)
};

static const char *s_string_descriptors[] = {
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "Espressif",          // 1: Manufacturer
    "Node16 audio",       // 2: Product
    "123456",             // 3: Serials
    "CDC Console",        // 4: CDC Interface
    "Node16 audio",       // 5: Audio Interface
    NULL
};

static const tusb_desc_device_t s_device_descriptor = {
    .bLength = sizeof(s_device_descriptor),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,

    // Use Interface Association Descriptor (IAD) for composite CDC + Audio
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // Espressif VID
    .idProduct = 0x4002, // Custom PID
    .bcdDevice = 0x0100,

    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,

    .bNumConfigurations = 0x01
};

// -----------------------------------------------------------------------------
// Audio Stream Buffer
// -----------------------------------------------------------------------------

static StreamBufferHandle_t s_audio_stream_buf = NULL;
// 10ms worth of audio at 48kHz stereo 16-bit = 1920 bytes
// We keep a buffer of 40ms to handle jitter
#define AUDIO_STREAM_BUF_SIZE (192 * 40)

static int8_t s_mute[3] = {0, 0, 0}; // master, ch1, ch2
static int16_t s_volume[3] = {0, 0, 0}; // master, ch1, ch2 (in 1/256 dB)
static uint32_t s_current_sample_rate = 48000;

extern "C" bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void)rhport;
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    if (entityID == 0x02) { // Feature Unit ID
        if (channelNum > 2) channelNum = 0;
        if (ctrlSel == AUDIO10_FU_CTRL_MUTE) {
            if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR && p_request->wLength == 1) {
                s_mute[channelNum] = pBuff[0];
                return true;
            }
        } else if (ctrlSel == AUDIO10_FU_CTRL_VOLUME) {
            if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR && p_request->wLength == 2) {
                s_volume[channelNum] = (int16_t)tu_unaligned_read16(pBuff);
                return true;
            }
        }
    }
    return false;
}

extern "C" bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    if (entityID == 0x02) { // Feature Unit ID
        if (channelNum > 2) channelNum = 0;
        if (ctrlSel == AUDIO10_FU_CTRL_MUTE) {
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &s_mute[channelNum], 1);
        } else if (ctrlSel == AUDIO10_FU_CTRL_VOLUME) {
            switch (p_request->bRequest) {
                case AUDIO10_CS_REQ_GET_CUR:
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &s_volume[channelNum], sizeof(int16_t));
                case AUDIO10_CS_REQ_GET_MIN: {
                    int16_t min = -90 * 256; // -90 dB
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
                }
                case AUDIO10_CS_REQ_GET_MAX: {
                    int16_t max = 0; // 0 dB
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
                }
                case AUDIO10_CS_REQ_GET_RES: {
                    int16_t res = 256; // 1.0 dB
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
                }
                default:
                    return false;
            }
        }
    }
    return false;
}

extern "C" bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void)rhport;
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);

    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ) {
        if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
            if (p_request->wLength == 3) {
                s_current_sample_rate = tu_unaligned_read32(pBuff) & 0x00FFFFFF;
                ESP_LOGI(TAG, "Audio EP 0x%02x set sample rate: %lu Hz", ep, (unsigned long)s_current_sample_rate);
                return true;
            }
        }
    }
    return false;
}

extern "C" bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ) {
        uint8_t freq[3];
        freq[0] = (uint8_t)(s_current_sample_rate & 0xFF);
        freq[1] = (uint8_t)((s_current_sample_rate >> 8) & 0xFF);
        freq[2] = (uint8_t)((s_current_sample_rate >> 16) & 0xFF);

        switch (p_request->bRequest) {
            case AUDIO10_CS_REQ_GET_CUR:
            case AUDIO10_CS_REQ_GET_MIN:
            case AUDIO10_CS_REQ_GET_MAX:
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
            case AUDIO10_CS_REQ_GET_RES: {
                uint8_t res[3] = {1, 0, 0}; // 1 Hz resolution
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, res, sizeof(res));
            }
            default:
                return false;
        }
    }
    return false;
}

extern "C" bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex);
    uint8_t const alt = tu_u16_low(p_request->wValue);
    if (itf == ITF_NUM_AUDIO_STREAMING) {
        if (alt != 0) {
            s_audio_alt_active.store(true, std::memory_order_relaxed);
            ESP_LOGI(TAG, "Audio stream OPENED (Alt %u)", alt);
        } else {
            s_audio_alt_active.store(false, std::memory_order_relaxed);
            ESP_LOGI(TAG, "Audio stream CLOSED (Alt 0)");
        }
    }
    return true;
}

extern "C" bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex);
    uint8_t const alt = tu_u16_low(p_request->wValue);
    if (itf == ITF_NUM_AUDIO_STREAMING && alt == 0) {
        s_audio_alt_active.store(false, std::memory_order_relaxed);
        ESP_LOGI(TAG, "Audio stream CLOSED via EP close (Alt 0)");
    }
    return true;
}

extern "C" bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
    (void)rhport;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;

    uint8_t rx_buf[AUDIO_EP_SIZE];
    uint16_t read_bytes = tud_audio_read(rx_buf, n_bytes_received);

    if (s_audio_stream_buf && read_bytes > 0) {
        s_last_audio_rx_us.store(esp_timer_get_time(), std::memory_order_relaxed);
        s_audio_alt_active.store(true, std::memory_order_relaxed);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xStreamBufferSendFromISR(s_audio_stream_buf, rx_buf, read_bytes, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
    return true;
}

// -----------------------------------------------------------------------------
// Initialization & API
// -----------------------------------------------------------------------------

void usb_audio_init(void) {
    ESP_LOGI(TAG, "Initializing USB Audio (Composite CDC + UAC)...");

    s_audio_stream_buf = xStreamBufferCreate(AUDIO_STREAM_BUF_SIZE, 192);
    if (!s_audio_stream_buf) {
        ESP_LOGE(TAG, "Failed to create audio stream buffer");
        return;
    }

    tinyusb_config_t tusb_cfg = {};
    tusb_cfg.descriptor.device = &s_device_descriptor;
    tusb_cfg.descriptor.string = s_string_descriptors;
    tusb_cfg.descriptor.string_count = 6;
    tusb_cfg.descriptor.full_speed_config = uac_cdc_configuration_descriptor;
    tusb_cfg.descriptor.high_speed_config = uac_cdc_configuration_descriptor;
    tusb_cfg.phy.skip_setup = false;
    tusb_cfg.phy.self_powered = false;
    tusb_cfg.phy.vbus_monitor_io = -1; // -1 if not using
    tusb_cfg.task.size = 4096;
    tusb_cfg.task.priority = 5;
    tusb_cfg.task.xCoreID = 0; // Fix panic: No affinity can sometimes cause issues

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    ESP_ERROR_CHECK(ret);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB driver install failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TinyUSB driver installed successfully");
    }
}

size_t usb_audio_read_pcm(void* dest, size_t max_bytes) {
    if (!s_audio_stream_buf) return 0;
    return xStreamBufferReceive(s_audio_stream_buf, dest, max_bytes, 0);
}

bool usb_audio_is_streaming(void) {
    if (!s_audio_alt_active.load(std::memory_order_relaxed)) {
        return false;
    }
    int64_t last_rx = s_last_audio_rx_us.load(std::memory_order_relaxed);
    if (last_rx == 0) {
        return false;
    }
    int64_t now_us = esp_timer_get_time();
    // Inactive if no audio packet received for > 150 ms (15 missing 10ms frames)
    if ((now_us - last_rx) > 150000) {
        return false;
    }
    return true;
}

void usb_audio_clear_buffer(void) {
    if (s_audio_stream_buf) {
        xStreamBufferReset(s_audio_stream_buf);
    }
}


extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)rts;
    if (!dtr && itf == 0) {
        cdc_line_coding_t coding;
        tud_cdc_get_line_coding(&coding);
        if (coding.bit_rate == 1200) {
            ESP_LOGI(TAG, "Touch 1200bps detected! Rebooting into ROM download bootloader...");
#if defined(RTC_CNTL_OPTION1_REG) && defined(RTC_CNTL_FORCE_DOWNLOAD_BOOT)
            REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
#endif
            esp_restart();
        }
    }
}
