/*
 * CSI receiver for ESP32-C5
 *
 * Purpose:
 * 1) Connect to the 5 GHz hotspot.
 * 2) Stay on the AP's current channel.
 * 3) Enable CSI for legacy/HT/VHT packets.
 * 4) Filter CSI by the real sender MAC and print CSI to serial.
 * 5) Also receive ESP-NOW payloads from the sender: seq + tx_us.
 *
 * Notes:
 * - CONFIG_CSI_SEND_MAC must be the sender board's real STA MAC.
 * - This receiver assumes the sender payload is:
 *
 *   typedef struct __attribute__((packed)) {
 *       uint32_t seq;
 *       int64_t  tx_us;
 *   } csi_ping_t;
 *
 * Serial output:
 *   ESPNOW_DATA,seq,tx_us,rx_us,mac
 *   CSI_META,csi_pkt_count,seq,tx_us,rx_us
 *   CSI_DATA,pkt_count,mac,rssi,rate,noise_floor,channel,rx_local_timestamp_us,len,"[ ... ]"
 *
 * Important:
 * - CSI_META is the latest ESP-NOW metadata snapshot from the same sender MAC.
 * - It is useful for debugging timing, but it is not a guaranteed strict 1:1 mapping
 *   to the exact CSI packet currently being printed.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "rom/ets_sys.h"

#define WIFI_SSID "Xiaomi 15"
#define WIFI_PASS "xucongzxc"

#define CONFIG_WIFI_BAND_MODE WIFI_BAND_MODE_5G_ONLY
#define CONFIG_ESP_NOW_PHYMODE WIFI_PHY_MODE_HT40
#define CONFIG_ESP_NOW_RATE WIFI_PHY_RATE_MCS0_LGI

/* IMPORTANT:
 * Update this if the sender prints a different local MAC.
 * This should be the sender board's real STA MAC, not the hotspot BSSID.
 */
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x3c, 0xdc, 0x75, 0x82, 0x1e, 0xa8};

static const char *TAG = "csi_recv";
static bool g_wifi_connected = false;
static uint8_t g_ap_channel = 0;

/* ---------- Sender payload ---------- */
typedef struct __attribute__((packed)) {
    uint32_t seq;
    int64_t tx_us;
} csi_ping_t;

/* ---------- Latest ESP-NOW metadata snapshot ---------- */
typedef struct {
    bool valid;
    uint32_t seq;
    int64_t tx_us;
    int64_t rx_us;
    uint8_t mac[6];
} espnow_meta_t;

static espnow_meta_t g_last_meta = {0};
static portMUX_TYPE g_meta_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---------- Wi-Fi events ---------- */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Trying to connect to AP...");
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected from AP, retrying...");
        g_wifi_connected = false;
        g_ap_channel = 0;
        esp_wifi_connect();
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        g_wifi_connected = true;

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            g_ap_channel = ap_info.primary;
            ESP_LOGI(TAG, "Connected to AP - SSID: %s, Channel: %u, RSSI: %d",
                     ap_info.ssid, g_ap_channel, ap_info.rssi);
        }
    }
}

/* ---------- Wi-Fi init ---------- */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .scan_method = WIFI_FAST_SCAN,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "wifi_init finished.");
}

/* ---------- ESP-NOW recv callback ---------- */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int len)
{
    if (recv_info == NULL || recv_info->src_addr == NULL || data == NULL) {
        return;
    }

    if (memcmp(recv_info->src_addr, CONFIG_CSI_SEND_MAC, 6) != 0) {
        return;
    }

    if (len < (int)sizeof(csi_ping_t)) {
        return;
    }

    csi_ping_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    int64_t rx_us = esp_timer_get_time();

    portENTER_CRITICAL(&g_meta_lock);
    g_last_meta.valid = true;
    g_last_meta.seq = pkt.seq;
    g_last_meta.tx_us = pkt.tx_us;
    g_last_meta.rx_us = rx_us;
    memcpy(g_last_meta.mac, recv_info->src_addr, 6);
    portEXIT_CRITICAL(&g_meta_lock);

    ets_printf("ESPNOW_DATA,%lu,%lld,%lld," MACSTR "\n",
               (unsigned long)pkt.seq,
               (long long)pkt.tx_us,
               (long long)rx_us,
               MAC2STR(recv_info->src_addr));
}

/* ---------- ESP-NOW init ---------- */
static void wifi_esp_now_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peer = {
        .channel = 0, /* use current AP channel */
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,
        .ersu = false,
        .dcm = false,
    };

    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));
    ESP_LOGI(TAG, "ESP-NOW init finished on current channel.");
}

/* ---------- CSI callback ---------- */
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    static uint32_t pkt_count = 0;

    if (info == NULL || info->buf == NULL || info->len <= 0) {
        return;
    }

    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6) != 0) {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx = &info->rx_ctrl;
    int64_t rx_local_timestamp_us = esp_timer_get_time();

    espnow_meta_t meta_snapshot;
    memset(&meta_snapshot, 0, sizeof(meta_snapshot));

    portENTER_CRITICAL(&g_meta_lock);
    meta_snapshot = g_last_meta;
    portEXIT_CRITICAL(&g_meta_lock);

    if (meta_snapshot.valid &&
        memcmp(meta_snapshot.mac, info->mac, 6) == 0) {
        ets_printf("CSI_META,%lu,%lu,%lld,%lld\n",
                   (unsigned long)pkt_count,
                   (unsigned long)meta_snapshot.seq,
                   (long long)meta_snapshot.tx_us,
                   (long long)meta_snapshot.rx_us);
    }

    ets_printf("CSI_DATA,%lu," MACSTR ",%d,%d,%d,%d,%lld,%d,\"[%d",
               (unsigned long)pkt_count++,
               MAC2STR(info->mac),
               rx->rssi,
               rx->rate,
               rx->noise_floor,
               rx->channel,
               (long long)rx_local_timestamp_us,
               info->len,
               info->buf[0]);

    for (int i = 1; i < info->len; ++i) {
        ets_printf(",%d", info->buf[i]);
    }
    ets_printf("]\"\n");
}

/* ---------- CSI init ---------- */
static void wifi_csi_init(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t csi_config = {
        .enable = true,
        .acquire_csi_legacy = true,
        .acquire_csi_force_lltf = false,
        .acquire_csi_ht20 = true,
        .acquire_csi_ht40 = true,
        .acquire_csi_vht = true,
        .acquire_csi_su = true,
        .acquire_csi_mu = false,
        .acquire_csi_dcm = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg = 0,
        .dump_ack_en = false,
        .reserved = false,
    };

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI enabled. Waiting for packets from sender MAC " MACSTR,
             MAC2STR(CONFIG_CSI_SEND_MAC));
}

/* ---------- Main ---------- */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();

    uint8_t local_mac[6] = {0};
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, local_mac));
    ESP_LOGI(TAG, "Receiver local MAC: " MACSTR, MAC2STR(local_mac));

    for (int i = 0; i < 20 && !g_wifi_connected; ++i) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Waiting for Wi-Fi connection... (%d/20)", i + 1);
    }

    if (!g_wifi_connected) {
        ESP_LOGE(TAG, "Wi-Fi connection failed.");
        return;
    }

    ESP_LOGI(TAG, "Current AP channel: %u", g_ap_channel);

    wifi_bandwidth_t bw;
    ESP_ERROR_CHECK(esp_wifi_get_bandwidth(WIFI_IF_STA, &bw));
    ESP_LOGI(TAG, "Receiver negotiated bandwidth: %s",
         bw == WIFI_BW_HT40 ? "HT40" : "HT20");

    wifi_esp_now_init();
    wifi_csi_init();
}