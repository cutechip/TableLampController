#ifndef _ESP_WIFIMANAGER_H
#define _ESP_WIFIMANAGER_H

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <memory>

extern "C" {
  #include "user_interface.h"
}
typedef enum {
    WIFICONFIG_UNKNOWN,
    WIFICONFIG_INIT_SUCCESS,
    WIFICONFIG_RECV_WIFI_INFO,
    WIFICONFIG_CONNECT_SUCCESS,
    WIFICONFIG_DISCONNECTED,
    WIFICONFIG_CONNECT_TIMEOUT
}wifi_connect_status_t;

typedef enum {
    ESPWIFI_IDLE,
    ESPWIFI_INIT,
    ESPWIFI_WAIT_RECVINFO,
    ESPWIFI_CHECK_CONNECT,
    ESPWIFI_WAIT_CONNECT
}wifi_connect_setp_t;

class ESPWiFiManager
{
    public:
        ESPWiFiManager(/* args */);
        ~ESPWiFiManager();

        
        
        void autoConnect();
        void autoConnect(char const *apName, char const *apPassword = NULL);
        void wifiHandle();
        void setConnectCallback(void (*connectCallback)(wifi_connect_status_t));
        void setTimeout(unsigned long seconds);
        void setHostname(String hostname);

    private:
        std::unique_ptr<DNSServer>        dnsServer;
        std::unique_ptr<ESP8266WebServer> server;

        String        _ssid                   = "";
        String        _pass                   = "";
        
        boolean _debug = true;
        const char*   _apName                 = "no-net";
        const char*   _apPassword             = NULL;
        uint32_t    _connectTimeout = 0;
        uint32_t    startConnectTick;
        const byte    DNS_PORT = 53;
        void (*_connectCallback)(wifi_connect_status_t) = NULL;

        uint8_t wifi_connect_status = 0;
        wifi_connect_setp_t wifi_setp = ESPWIFI_INIT;
        template <typename Generic>
        void          DEBUG_WM(Generic text);
        uint8_t connectLastWifi();
        void startConfigPortal(char const *apName, char const *apPassword);
        void checkWifiConnect();
        void saveWifiInfo();
        void setupConfigPortal(void);
        uint8_t waitForConnectResult();
        void startWPS();
        void getWifiList();
        boolean captivePortal();
        void handleNotFound();
        int  getRSSIasQuality(int RSSI);
        String toStringIp(IPAddress ip);
        boolean isIp(String str);

};




#endif