/* Range Test
 * Header for the ESP-NOW range test.
 */

#ifndef RANGE_TEST_H
#define RANGE_TEST_H

#include <stdint.h>
#include "esp_now.h"

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   WIFI_IF_STA
#endif

#define ESPNOW_QUEUE_SIZE 10
#define RT_PACKET_SIZE 250
#define RT_PACKETS_PER_MODE 20
// for the defines it defines the mode as station mode, 
// has a event queue size of 10 with max packet size of 250 testing hardest case


typedef enum {
    RT_EVENT_SEND_CB,
    RT_EVENT_RECV_CB,
} rt_event_type;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} rt_event_send_cb_info;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
    int8_t rssi;
} rt_event_recv_cb_info;

typedef union {
    rt_event_send_cb_info send_cb;
    rt_event_recv_cb_info recv_cb;
} rt_event_info;

/* Callbacks post one of these to the range-test task. */
typedef struct {
    rt_event_type id;
    rt_event_info info;
} rt_event_t;

/* Which ESP-NOW rate mode this packet was sent under */
typedef enum {
    RT_RATE_STD_1M,
    RT_RATE_STD_54M,
    RT_RATE_LR_250K,
    RT_RATE_LR_500K,
    RT_RATE_COUNT,
} rt_rate_mode_t;

#define RT_HEADER_SIZE 7
// the test packet itself
typedef struct {
    uint16_t seq_num;                               
    uint8_t  mode;                                  
    uint32_t send_timestamp_ms;                     
    uint8_t  payload[RT_PACKET_SIZE - RT_HEADER_SIZE];
} __attribute__((packed)) rt_packet_t;

_Static_assert(sizeof(rt_packet_t) == RT_PACKET_SIZE, "rt_packet_t must be exactly 250 bytes");
