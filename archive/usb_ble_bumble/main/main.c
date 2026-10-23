/*
 * ESP32-C6 USB BLE 5.3 HCI Controller Firmware
 * Pure binary H4 transport over UART0 (COM121)
 * Direct NimBLE Controller HCI Transport Bridge with WS2812 Status LED Feedback
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "nimble/ble_hci_trans.h"
#include "os/os_mbuf.h"
#include "status_led.h"

#define HCI_UART_PORT           UART_NUM_0
#define HCI_UART_BAUDRATE       2000000
#define HCI_UART_BUF_SIZE       2048

#define HCI_H4_CMD              0x01
#define HCI_H4_ACL              0x02
#define HCI_H4_SCO              0x03
#define HCI_H4_EVT              0x04
#define HCI_H4_ISO              0x05

#define STATUS_LED_GPIO         8

static uint8_t s_rx_raw_buf[HCI_UART_BUF_SIZE];
static uint8_t s_rx_pkt_buf[HCI_UART_BUF_SIZE];

/* Controller -> Host HCI Event Callback */
static int controller_to_host_evt_cb(uint8_t *hci_ev, void *arg)
{
    if (!hci_ev) {
        return 0;
    }

    uint8_t param_len = hci_ev[1];
    uint16_t total_len = 2 + param_len;

    uint8_t h4_type = HCI_H4_EVT;
    uart_write_bytes(HCI_UART_PORT, (const char *)&h4_type, 1);
    uart_write_bytes(HCI_UART_PORT, (const char *)hci_ev, total_len);

    ble_hci_trans_buf_free(hci_ev);

    status_led_report_activity();
    return 0;
}

/* Controller -> Host ACL Callback */
static int controller_to_host_acl_cb(struct os_mbuf *om, void *arg)
{
    if (!om) {
        return 0;
    }

    uint8_t h4_type = HCI_H4_ACL;
    uart_write_bytes(HCI_UART_PORT, (const char *)&h4_type, 1);

    struct os_mbuf *m = om;
    while (m) {
        if (m->om_len > 0) {
            uart_write_bytes(HCI_UART_PORT, (const char *)m->om_data, m->om_len);
        }
        m = SLIST_NEXT(m, om_next);
    }

    os_mbuf_free_chain(om);

    status_led_report_activity();
    return 0;
}

static void uart_rx_task(void *pvParameters)
{
    size_t pkt_len = 0;
    size_t expected_len = 0;
    uint8_t pkt_type = 0;

    while (1) {
        int read_bytes = uart_read_bytes(
            HCI_UART_PORT,
            s_rx_raw_buf,
            sizeof(s_rx_raw_buf),
            pdMS_TO_TICKS(10)
        );

        if (read_bytes <= 0) {
            continue;
        }

        for (int i = 0; i < read_bytes; i++) {
            uint8_t b = s_rx_raw_buf[i];

            if (pkt_len == 0) {
                if (b == HCI_H4_CMD || b == HCI_H4_ACL || b == HCI_H4_ISO) {
                    pkt_type = b;
                    s_rx_pkt_buf[0] = b;
                    pkt_len = 1;
                    expected_len = 0;
                }
                continue;
            }

            s_rx_pkt_buf[pkt_len++] = b;

            if (expected_len == 0) {
                if (pkt_type == HCI_H4_CMD && pkt_len == 4) {
                    uint8_t param_len = s_rx_pkt_buf[3];
                    expected_len = 1 + 3 + param_len;
                } else if (pkt_type == HCI_H4_ACL && pkt_len == 5) {
                    uint16_t data_len = s_rx_pkt_buf[3] | (s_rx_pkt_buf[4] << 8);
                    expected_len = 1 + 4 + data_len;
                } else if (pkt_type == HCI_H4_ISO && pkt_len == 5) {
                    uint16_t iso_len = (s_rx_pkt_buf[3] | (s_rx_pkt_buf[4] << 8)) & 0x3FFF;
                    expected_len = 1 + 4 + iso_len;
                }
            }

            if (expected_len > 0 && pkt_len == expected_len) {
                if (pkt_type == HCI_H4_CMD) {
                    uint8_t *cmd = ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_CMD);
                    if (cmd) {
                        memcpy(cmd, &s_rx_pkt_buf[1], pkt_len - 1);
                        ble_hci_trans_hs_cmd_tx(cmd);
                    }
                } else if (pkt_type == HCI_H4_ACL) {
                    struct os_mbuf *om = os_msys_get_pkthdr(pkt_len - 1, 0);
                    if (om) {
                        os_mbuf_append(om, &s_rx_pkt_buf[1], pkt_len - 1);
                        ble_hci_trans_hs_acl_tx(om);
                    }
                }

                status_led_report_activity();

                pkt_len = 0;
                expected_len = 0;
                pkt_type = 0;
            }

            if (pkt_len >= sizeof(s_rx_pkt_buf)) {
                pkt_len = 0;
                expected_len = 0;
                pkt_type = 0;
            }
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Initialize WS2812 RGB Status LED (GPIO 8) */
    status_led_init(STATUS_LED_GPIO);

    uart_config_t uart_config = {
        .baud_rate = HCI_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(HCI_UART_PORT, HCI_UART_BUF_SIZE * 2, HCI_UART_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(HCI_UART_PORT, &uart_config);
    uart_set_pin(HCI_UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    /* Register direct HCI Host transport callbacks with the NimBLE Link Layer controller */
    ble_hci_trans_cfg_hs(controller_to_host_evt_cb, NULL, controller_to_host_acl_cb, NULL);

    xTaskCreatePinnedToCore(
        uart_rx_task,
        "hci_uart_rx",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        NULL,
        0
    );
}
