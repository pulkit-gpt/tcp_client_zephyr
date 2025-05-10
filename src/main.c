#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_config.h>

#include <zephyr/net/socket.h>
#include <zephyr/net/tls_credentials.h>

#define SERVER_IP   "192.168.29.67"
#define SERVER_PORT 8080

#define WIFI_SSID   "MySSID"
#define WIFI_PSK    "MyPasswd"

void wifi_connect(void)
{
    struct net_if *iface = net_if_get_default();  /* Should be the wifi0 interface */
    struct wifi_connect_req_params cnx = {0};

    /* Fill Wi-Fi parameters */
    cnx.ssid = (uint8_t *)WIFI_SSID;
    cnx.ssid_length = strlen(WIFI_SSID);
    cnx.psk = (uint8_t *)WIFI_PSK;
    cnx.psk_length = strlen(WIFI_PSK);
    cnx.security = WIFI_SECURITY_TYPE_PSK;
    cnx.band = WIFI_FREQ_BAND_2_4_GHZ;
    cnx.channel = 0;  /* auto-select */
    cnx.timeout = SYS_FOREVER_MS;

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
                       &cnx, sizeof(cnx));
    if (ret) {
        printk("Wi-Fi connect request failed: %d\n", ret);
    } else {
        printk("Wi-Fi connection Successful\n");
    }
}


void func(char *buf) {
    printk("Received: %s\n", buf);
    switch (buf[0])
    {
    case '1':
        printk("Camera function\n");
        break;
    case '2':
        printk("ECG function\n");
        break;
    case '3':
        printk("Temperature function\n");
        break;
    case '4':
        printk("Heart rate function\n");
        break;
    default:
        printk("Unknown function\n");
        break;
    }
}


void tcp_client(void)
{
    int sock;
    struct sockaddr_in addr;
    char buf[256];

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        printk("Socket creation failed\n");
        return;
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    net_addr_pton(AF_INET, SERVER_IP, &addr.sin_addr);

    printk("Connecting to %s:%d...\n", SERVER_IP, SERVER_PORT);
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printk("Connect failed\n");
        close(sock);
        return;
    }
    printk("Connected!\n");

    const char *msg = "{\"why\":\"connect\"}";
    send(sock, msg, strlen(msg), 0);
    printk("Sent: %s\n", msg);

    while (1) {
        int len = recv(sock, buf, sizeof(buf) - 1, 0);
        if (len > 0) {
            buf[len] = '\0';
            func(buf);
        } else if (len == 0) {
            printk("Server closed the connection\n");
            break;
        } else {
            printk("recv() error: %d\n", errno);
            break;
        }
    }
    close(sock);
    printk("Disconnected.\n");
}


int main(){

    printk("Wi-Fi connection initialized\n");
    wifi_connect();
    printk("Starting TCP client\n");
    tcp_client();
    printk("Exiting...\n");
    return 0;
}