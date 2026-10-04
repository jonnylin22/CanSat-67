#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "range_test.h"

static const char *TAG = "range_test";
static QueueHandle_t s_rt_queue = NULL;
// gets macs for a,b, and peer defined to 48 bits
// need to manually define the macs of the esp32s
static const uint8_t s_mac_a[ESP_NOW_ETH_ALEN] = {0x34, 0xb7, 0xda, 0xf6, 0x38, 0x84};
static const uint8_t s_mac_b[ESP_NOW_ETH_ALEN] = {0xa8, 0x46, 0x74, 0x5c, 0x14, 0x98};
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];

//array of transmit packet countesr
static uint16_t s_tx_seq[RT_RATE_COUNT];

// struct tracking if a prev packet exists, what that packet is, how many received, and how many skipped between prev and curr
typedef struct {
    bool     have_last;
    uint16_t last_seq;
    uint32_t rx_count;
    uint32_t lost_count;
} rt_rx_stats_t;

// creates array of rx_stats
static rt_rx_stats_t s_rx_stats[RT_RATE_COUNT];

/* a seq gap this big means the peer's counter went backwards (reboot), not real loss */
#define RT_RESEQ_THRESHOLD 32768

// wifi setup same as example code but hardcode wifi channel and long rnage mode
static void rt_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
}

// sets the tx rate for the peer
static void rt_set_rate(rt_rate_mode_t mode)
{
    esp_now_rate_config_t rc = {0};
    switch (mode) {
        case RT_RATE_STD_1M:  rc.phymode = WIFI_PHY_MODE_11B; rc.rate = WIFI_PHY_RATE_1M_L;       break;
        case RT_RATE_STD_54M: rc.phymode = WIFI_PHY_MODE_11G; rc.rate = WIFI_PHY_RATE_54M;        break;
        case RT_RATE_LR_250K: rc.phymode = WIFI_PHY_MODE_LR;  rc.rate = WIFI_PHY_RATE_LORA_250K;  break;
        case RT_RATE_LR_500K: rc.phymode = WIFI_PHY_MODE_LR;  rc.rate = WIFI_PHY_RATE_LORA_500K;  break;
        default: return;
    }
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(s_peer_mac, &rc));
}

// callback function that checks if transmit info is null, then gets event info and attempts to queue it
// calls back on transmits
static void rt_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (tx_info == NULL) {
        return;
    }
    rt_event_t evt;
    evt.id = RT_EVENT_SEND_CB;
    memcpy(evt.info.send_cb.mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    evt.info.send_cb.status = status;
    xQueueSend(s_rt_queue, &evt, 0);
}

// callback function, checks if null data, creates receive event and fills info, allocates into memory and queues
static void rt_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len <= 0) {
        return;
    }
    rt_event_t evt;
    evt.id = RT_EVENT_RECV_CB;
    memcpy(evt.info.recv_cb.mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    evt.info.recv_cb.data = malloc(len);
    if (evt.info.recv_cb.data == NULL) {
        return;
    }
    memcpy(evt.info.recv_cb.data, data, len);
    evt.info.recv_cb.data_len = len;
    evt.info.recv_cb.rssi = recv_info->rx_ctrl->rssi;
    if (xQueueSend(s_rt_queue, &evt, 0) != pdTRUE) {
        free(evt.info.recv_cb.data);
    }
}

// fills info for transmit packet
static void rt_prepare_packet(rt_packet_t *pkt, rt_rate_mode_t mode)
{
    pkt->seq_num = s_tx_seq[mode]++;
    pkt->mode = (uint8_t)mode;
    pkt->send_timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    memset(pkt->payload, 0xAA, sizeof(pkt->payload));
}

/* TX task: cycle through every rate mode, RT_PACKETS_PER_MODE packets each, 1 Hz */
static void rt_tx_task(void *arg)
{
    static rt_packet_t pkt;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        for (int m = 0; m < RT_RATE_COUNT; m++) {
            rt_rate_mode_t mode = (rt_rate_mode_t)m;
            rt_set_rate(mode);

            for (int i = 0; i < RT_PACKETS_PER_MODE; i++) {
                rt_prepare_packet(&pkt, mode);
                esp_err_t err = esp_now_send(s_peer_mac, (uint8_t *)&pkt, sizeof(pkt));
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "send err: %s", esp_err_to_name(err));
                }
                vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
            }
        }
    }
}

/* Update one mode's loss stats from a newly received seq number */
static void rt_track_seq(rt_rx_stats_t *st, uint16_t seq)
{
    if (st->have_last) {
        uint16_t gap = (uint16_t)(seq - st->last_seq - 1);  /* uint16 math handles wraparound */
        if (gap > 0 && gap < RT_RESEQ_THRESHOLD) {
            st->lost_count += gap;
        }
    }
    st->last_seq = seq;
    st->have_last = true;
    st->rx_count++;
}

/* RX/event task */
static void rt_task(void *pvParameter)
{
    rt_event_t evt;

    printf("RT_HEADER,seq,mode,rssi,rx_total,lost_total\n");

    while (xQueueReceive(s_rt_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case RT_EVENT_SEND_CB:
                if (evt.info.send_cb.status != ESP_NOW_SEND_SUCCESS) {
                    ESP_LOGW(TAG, "tx fail (no ACK)");
                }
                break;

            case RT_EVENT_RECV_CB: {
                if (evt.info.recv_cb.data_len == sizeof(rt_packet_t)) {
                    rt_packet_t *rx = (rt_packet_t *)evt.info.recv_cb.data;
                    if (rx->mode < RT_RATE_COUNT) {
                        rt_rx_stats_t *st = &s_rx_stats[rx->mode];
                        rt_track_seq(st, rx->seq_num);
                        printf("RT,%u,%u,%d,%" PRIu32 ",%" PRIu32 "\n",
                               rx->seq_num, rx->mode, evt.info.recv_cb.rssi,
                               st->rx_count, st->lost_count);
                    } else {
                        ESP_LOGW(TAG, "bad mode field: %u", rx->mode);
                    }
                } else {
                    ESP_LOGW(TAG, "bad packet size: %d", evt.info.recv_cb.data_len);
                }
                free(evt.info.recv_cb.data);
                break;
            }
        }
    }
}

static esp_err_t rt_espnow_init(void)
{
    s_rt_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(rt_event_t));
    if (s_rt_queue == NULL) {
        ESP_LOGE(TAG, "Queue create fail");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(rt_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(rt_recv_cb));

    esp_now_peer_info_t peer = {0};
    peer.channel = 1;
    peer.ifidx = ESPNOW_WIFI_IF;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    xTaskCreate(rt_task, "rt_task", 4096, NULL, 5, NULL);
    xTaskCreate(rt_tx_task, "rt_tx_task", 4096, NULL, 4, NULL);
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    rt_wifi_init();

    // read this board's own MAC, then pick the other board as the peer
    uint8_t my_mac[ESP_NOW_ETH_ALEN];
    ESP_ERROR_CHECK(esp_read_mac(my_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "my MAC: " MACSTR, MAC2STR(my_mac));

    if (memcmp(my_mac, s_mac_a, ESP_NOW_ETH_ALEN) == 0) {
        memcpy(s_peer_mac, s_mac_b, ESP_NOW_ETH_ALEN);
    } else if (memcmp(my_mac, s_mac_b, ESP_NOW_ETH_ALEN) == 0) {
        memcpy(s_peer_mac, s_mac_a, ESP_NOW_ETH_ALEN);
    } else {
        ESP_LOGE(TAG, "this board's MAC isn't in the list, fix s_mac_a/s_mac_b");
        return;
    }

    rt_espnow_init();
}