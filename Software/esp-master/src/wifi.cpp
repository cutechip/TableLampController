#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESPWiFiManager.h>
#include <wifi.h>
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ntp.h>

ESPWiFiManager wiFiManager;


#define AP_PREFIX  "MengXin-"
String apName;

wifi_connect_status_t wifi_status;

boolean wifi_connect_flag = false;



/**
 * @brief WIFI���ӻص�
 * 
 * @param status 
 */
void wifi_connect_callback(wifi_connect_status_t status)
{
    wifi_status = status;
    Serial1.printf("connect_callback:%d\r\n", status);
    if (status == WIFICONFIG_CONNECT_SUCCESS) 
    {
        wifi_connect_flag = true;
        ntp_update();
    }
    if (status == WIFICONFIG_DISCONNECTED)
    {
        wifi_connect_flag = false;
    }
}



/**
 * @brief �ж�wifi�Ƿ����ӳɹ�
 * 
 * @return boolean 
 */
boolean wifi_isconnect()
{
    return wifi_connect_flag;
}



/**
 * @brief ��ȡWIFI����״̬
 * 
 * @return wifi_connect_status_t 
 */
wifi_connect_status_t get_wifi_connect_status()
{
    return wifi_status;
}

void auto_connect_wifi()
{
    apName = (AP_PREFIX + String(ESP.getChipId()));
    wiFiManager.autoConnect(apName.c_str());
    
}




/**
 * @brief WIFi ��������
 * 
 */
void wifi_process_task()
{
    wiFiManager.wifiHandle();
}




/**
 * @brief WIFI��ʼ��
 * 
 */
void wifi_init()
{
    wiFiManager.setHostname("SmartModulator");
    wiFiManager.setConnectCallback(wifi_connect_callback);
    wiFiManager.setTimeout(180);    
}