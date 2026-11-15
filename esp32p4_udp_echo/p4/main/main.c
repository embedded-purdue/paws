/* ESP32-P4 Ethernet + UDP echo server.
 * For Waveshare ESP32-P4-WIFI6-DEV-KIT Rev1.2 (IP101GRI PHY, RJ45).
 * Requires ESP-IDF >= 5.5 and the espressif/ethernet_init component.
 *
 * For a direct Jetson <-> P4 cable:
 *   Jetson = 192.168.7.1/24
 *   P4     = 192.168.7.2/24
 *   UDP    = port 3333
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "ethernet_init.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#define UDP_PORT       3333
#define P4_IP_A        192
#define P4_IP_B        168
#define P4_IP_C        7
#define P4_IP_D        2
#define MAX_PACKET     512

static const char *TAG = "P4_UDP_ECHO";

static void ethernet_event_handler(void *arg, esp_event_base_t base,
                                   int32_t event_id, void *event_data)
{
    (void)arg;
    (void)base;
    (void)event_data;
    if (event_id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet link UP");
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "Ethernet link DOWN");
    }
}

static void udp_echo_task(void *arg)
{
    (void)arg;
    char rx[MAX_PACKET];

    for (;;) {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in listen_addr = {0};
        listen_addr.sin_family = AF_INET;
        listen_addr.sin_port = htons(UDP_PORT);
        listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sock, (struct sockaddr *)&listen_addr,
                 sizeof(listen_addr)) < 0) {
            ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Listening for UDP on port %d", UDP_PORT);

        for (;;) {
            struct sockaddr_in sender = {0};
            socklen_t sender_len = sizeof(sender);
            // No string assumptions: receive arbitrary bytes, up to MAX_PACKET.
            int count = recvfrom(sock, rx, sizeof(rx), 0,
                                 (struct sockaddr *)&sender, &sender_len);
            if (count < 0) {
                ESP_LOGE(TAG, "recvfrom() failed: errno=%d", errno);
                break;
            }

            char sender_ip[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &sender.sin_addr, sender_ip, sizeof(sender_ip));
            ESP_LOGI(TAG, "Received %d bytes from %s:%u", count,
                     sender_ip, (unsigned)ntohs(sender.sin_port));

            // Echo the original bytes to the sender's IP and UDP source port.
            int sent = sendto(sock, rx, (size_t)count, 0,
                              (struct sockaddr *)&sender, sender_len);
            if (sent != count) {
                ESP_LOGW(TAG, "sendto returned %d (expected %d), errno=%d",
                         sent, count, errno);
            } else {
                ESP_LOGI(TAG, "Echoed %d bytes", sent);
            }
        }
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Ethernet PHY and GPIO settings are preset in p4/sdkconfig.defaults.
    // Check menuconfig to confirm that IP101/RMII/clock input were applied.
    esp_eth_handle_t *eth_handles = NULL;
    uint8_t eth_count = 0;
    ESP_ERROR_CHECK(ethernet_init_all(&eth_handles, &eth_count));
    if (eth_count != 1) {
        ESP_LOGE(TAG, "Expected exactly one Ethernet port; found %u", eth_count);
        return;
    }

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&cfg);
    if (!netif) {
        ESP_LOGE(TAG, "Could not create Ethernet netif");
        return;
    }
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_eth_new_netif_glue(eth_handles[0])));

    // Direct cable has no DHCP server, so assign a static address.
    esp_err_t err = esp_netif_dhcpc_stop(netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_ERROR_CHECK(err);
    }
    esp_netif_ip_info_t ip_info = {0};
    IP4_ADDR(&ip_info.ip, P4_IP_A, P4_IP_B, P4_IP_C, P4_IP_D);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    IP4_ADDR(&ip_info.gw, 0, 0, 0, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));

    ESP_ERROR_CHECK(esp_event_handler_register(
        ETH_EVENT, ESP_EVENT_ANY_ID, ethernet_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handles[0]));

    ESP_LOGI(TAG, "P4 IPv4: " IPSTR ", UDP port: %d",
             IP2STR(&ip_info.ip), UDP_PORT);
    ESP_LOGI(TAG, "Waiting for UDP packets from computer or Jetson...");

    // UDP server can bind INADDR_ANY even before the physical link is up.
    BaseType_t task_ok = xTaskCreate(udp_echo_task, "udp_echo", 4096,
                                     NULL, 5, NULL);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UDP echo task");
    }
}
