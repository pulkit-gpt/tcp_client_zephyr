#include <zephyr/kernel.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>

#include <string.h>
#include <errno.h>

#include "max30102.h"


#define WIFI_SSID "MySSID"
#define WIFI_PSK "MyPasswd"

#define SERVER_IP "192.168.29.44"
#define SERVER_PORT 8080

static int tcp_sock = -1;

int wifi_connect(void)
{
    struct net_if *iface = net_if_get_default();
    struct wifi_connect_req_params cnx = {0};

    cnx.ssid = (uint8_t *)WIFI_SSID;
    cnx.ssid_length = strlen(WIFI_SSID);
    cnx.psk = (uint8_t *)WIFI_PSK;
    cnx.psk_length = strlen(WIFI_PSK);
    cnx.security = WIFI_SECURITY_TYPE_PSK;
    cnx.band = WIFI_FREQ_BAND_2_4_GHZ;
    cnx.channel = 0;
    cnx.timeout = SYS_FOREVER_MS;

    printk("Wi-Fi: connecting to %s\n", WIFI_SSID);

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT,
                       iface, &cnx, sizeof(cnx));

    if (ret) {
        printk("Wi-Fi connect request failed (%d)\n", ret);
        return ret;
    }

    return 0;
}

int tcp_send(const char *data)
{
    if (tcp_sock < 0) {
        return -ENOTCONN;
    }

    int ret = send(tcp_sock, data, strlen(data), 0);
    if (ret < 0) {
        printk("TCP send error: %d\n", errno);
        return ret;
    }

    return 0;
}

// int  max30102_data_Send(){
//     int count = 0;
//     while(count<150){
//         max30102_data_t d;
//         if (max30102_get_data(&d)) {
//             tcp_send("{\"max30102\":{\"red\":%lu,\"ir\":%lu,\"hr\":%.1f,\"spo2\":%.2f,\"pi\":%.2f}}\n",
//                      (unsigned long)d.red, (unsigned long)d.ir,
//                      d.hr, d.spo2, d.pi);
//             count++;
//         }
//         else {
//             printk("MAX30102 data not valid\n");
//         }
//         k_msleep(200);
//         max30102_update();
//     }
// }

static bool max_streaming = false;
static int max_stream_count = 0;


static void handle_command(char cmd)
{
    switch (cmd) {
    case '1':
        printk("Server requested camera data\n");
        tcp_send("{\"cmd\":\"camera\"}\n");
        break;

    case '2':
        printk("Server requested ECG data\n");
        tcp_send("{\"cmd\":\"ecg\"}\n");
        break;

    case '3':
        printk("Server requested temperature data\n");
        tcp_send("{\"cmd\":\"temperature\"}\n");
        break;

    case '4':
        printk("Server requested MAX data\n");
        tcp_send("{\"cmd\":\"MAX\"}\n");
        int lines = max30102_main(max_data);
        for(int i=0;i<lines;i++){
            tcp_send(max_data[i]);
            
        }
        break;

    default:
        printk("Server sent unknown command: %c\n", cmd);
        tcp_send("{\"error\":\"unknown\"}\n");
        break;
    }
}

void tcp_client_start(void)
{
    struct sockaddr_in addr;
    char rx_buf[256];

    tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tcp_sock < 0) {
        printk("TCP socket creation failed\n");
        return;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    net_addr_pton(AF_INET, SERVER_IP, &addr.sin_addr);

    printk("TCP: connecting to %s:%d\n", SERVER_IP, SERVER_PORT);

    if (connect(tcp_sock,
                (struct sockaddr *)&addr,
                sizeof(addr)) < 0) {
        printk("TCP connect failed (%d)\n", errno);
        close(tcp_sock);
        tcp_sock = -1;
        return;
    }

    printk("TCP connected\n");
    tcp_send("{\"status\":\"connected\"}\n");

    while (1) {

        printk("Waiting for TCP data...\n");
        int len = recv(tcp_sock, rx_buf, sizeof(rx_buf) - 1, 0);
        if (len > 0) {
            rx_buf[len] = '\0';
            handle_command(rx_buf[0]);
        } else {
            printk("TCP disconnected\n");
            break;
        }
    }

    close(tcp_sock);
    tcp_sock = -1;
}



int main(void)
{
    printk("System boot\n");

    if (wifi_connect() != 0) {
        printk("Wi-Fi setup failed\n");
        return 0;
    }

    k_sleep(K_SECONDS(5));

    printk("Starting TCP client\n");

    tcp_client_start();

    return 0;
}