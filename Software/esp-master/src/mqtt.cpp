#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESPWiFiManager.h>
#include <wifi.h>
#include <iotClient.h>
#include <mqtt.h>


#define PRODUCT_KEY     "woBxKx335QA6"
#define CUSTOM_STRING   "device"
#define MQTT_SERVER     "192.168.1.201"



typedef enum {
    MYMQTT_INIT,
    MYMQTT_WAIT_CONNECT,
    MYMQTT_CONNECTED
}mqtt_setp_t;


WiFiClient espClient;
PubSubClient client(espClient);

mqtt_setp_t mqtt_setp = MYMQTT_INIT;
uint32_t mqtt_tick = 0; 


String mqttClientId;
String mqttUsername;
String mqttPassword;
String deviceSn;


/**
 * @brief MQTT客户端初始化
 * 
 */
void mqtt_config_init()
{
    deviceSn = String(ESP.getChipId());
    mqttClientId = String(PRODUCT_KEY) + "|" + deviceSn;
    mqttUsername = String(CUSTOM_STRING) + "|" + String(PRODUCT_KEY) + "&" + String(ESP.getChipId()) + "&4294967295";
    mqttPassword = getIotPassword(mqttUsername);
    client.setServer(MQTT_SERVER, 1883);
    client.setCallback(mqtt_callback);
}



/**
 * @brief MQTT 任务
 * 
 */
void mqtt_task()
{
    static uint8_t connect_timeout_cnt = 0;
    if (!wifi_isconnect()) 
    {
        mqtt_setp = MYMQTT_INIT;
        return ;
    }
    switch (mqtt_setp)
    {
    case MYMQTT_INIT:
        Serial1.println();
        Serial1.print("mqttClientId:" + mqttClientId + "\r\n");
        Serial1.print("mqttUsername:" + mqttUsername + "\r\n");
        Serial1.print("mqttPassword:" + mqttPassword + "\r\n");
        client.connect(mqttClientId.c_str(), mqttUsername.c_str(), mqttPassword.c_str());
        mqtt_tick = millis();
        connect_timeout_cnt = 0;
        Serial1.printf("mqtt init");
        mqtt_setp = MYMQTT_WAIT_CONNECT;
        break;
    case MYMQTT_WAIT_CONNECT:
        if (millis() - mqtt_tick > 500)
        {
            mqtt_tick = millis();
            connect_timeout_cnt++;
            Serial1.printf("connect_timeout_cnt:%d\r\n", connect_timeout_cnt);
            if (client.connected())
            {
                Serial1.print("MQTT Success...");
                client.subscribe(("/sys/" +String(PRODUCT_KEY) + "/" + deviceSn + "/properties/set").c_str(), 0);
                client.subscribe(("/sys/" +String(PRODUCT_KEY) + "/" + deviceSn + "/properties/get").c_str(), 0);
                client.subscribe(("/sys/" +String(PRODUCT_KEY) + "/" + deviceSn + "/properties/report_reply").c_str(), 0);
                client.subscribe(("/sys/" +String(PRODUCT_KEY) + "/" + deviceSn + "/service/request").c_str(), 0);
                client.subscribe(("/sys/" +String(PRODUCT_KEY) + "/" + deviceSn + "/shadow/set_reply").c_str(), 0);
                client.subscribe(("/sys/" +String(PRODUCT_KEY) + "/" + deviceSn + "/shadow/down").c_str(), 0);
                mqtt_setp = MYMQTT_CONNECTED;
            }

            if (connect_timeout_cnt >= 20)
            {
                mqtt_setp = MYMQTT_INIT;
            }
        }
        
        break;
    case MYMQTT_CONNECTED:
        if (millis() - mqtt_tick > 3000)
        {
            mqtt_tick = millis();
            // client.publish("/ABC", "Hello");

        }
        client.loop();
        break;
    default:
        break;
    }
}