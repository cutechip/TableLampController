#include <Arduino.h>
#include <iotClient.h>
#include <sha1.h>
#include <ArduinoJson.h>
#include <slave.h>
#include <ControlCenter.h>
#include <sys_config.h>
#include <ui.h>


// 产品 secret
const uint8_t productSecret[] = {"uvf9e98hL2Mzrk4y"};
const char hexMap[] = {"0123456789abcdef"};

uint8_t password[41];


/**
 * @brief 获取MQTT连接密码
 * 
 * @param username 
 * @return String 
 */
String getIotPassword(String username)
{    
    Sha1.initHmac(productSecret, strlen((const char *)productSecret));
    Sha1.print(username);

    uint8_t *hash = Sha1.resultHmac();
    for (int i=0; i<20; i++) {
        password[2 * i] =  hexMap[hash[i]>>4];
        password[2 * i + 1] =  hexMap[hash[i]&0xf];
    }

    return String((const char *)password);
}







/**
 * @brief 属性设置
 * 
 * @param payload 
 */
void properties_set(byte* payload, unsigned int length)
{
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(payload);
    JsonObject &parameters = root["parameters"];
    JsonVariant sw = parameters["switch"];
    if (sw.success())
    {
        Serial1.printf("set switch...\r\n");
        bool sys_switch = parameters.get<bool>("switch");
        if (sys_switch)
        {
            Serial1.printf("on");
            master_switch(true);
            // ctrl_port(1, 1, 1);
        }else{
            // ctrl_port(0, 0, 0);
            master_switch(false);
            Serial1.printf("off");
        }   
    }

    // 寻找port1
    JsonVariant port1 = parameters["port1"];
    if (port1.success())
    {
        Serial1.printf("set port1...\r\n");
        JsonObject &port1P = parameters["port1"];
        sys_config_t config = get_sys_config();
        config.port1.sw = port1P.get<bool>("switch");
        config.port1.brightness = port1P.get<unsigned char>("brightness");
        config.port1.warm = port1P.get<unsigned char>("warm");
        Serial1.printf("\r\nbrightness:%d, warm:%d\r\n", config.port1.brightness, config.port1.warm);
        if (config.port1.sw) {
            ctrl_port(1, port1P.get<unsigned char>("brightness"), port1P.get<unsigned char>("warm"));
        }
        set_sys_config(config);
    }


    if (NULL != parameters["usbout"])
    {
        if (parameters["usbout"] == true)
        {
            slave_usb_pdout(1);
        }else{
            slave_usb_pdout(0);
        }   
    }
}






/**
 * @brief 服务请求
 * 
 * @param payload 
 * @param length 
 */
void service_request(byte* payload, unsigned int length)
{
    DynamicJsonBuffer jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(payload);
    JsonObject &parameters = root["parameters"];
    // 短信
    if (parameters.get<String>("identifier") == "shortMessage")
    {
        JsonObject &inputList = parameters["inputList"];
        String msg = inputList.get<String>("msg");
        set_short_message(msg.c_str());
    }
}

/**
 * @brief MQTT接收消息
 * 
 * @param topic 
 * @param payload 
 * @param length 
 */
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial1.print("Message arrived [");
    Serial1.print(topic);
    Serial1.print("] \r\n");
    uint8_t i;
    for (i = 0; i < length; i++) {
        Serial1.print((char)payload[i]);
    }
    Serial1.println();

    if (strstr(topic, "properties/set") != NULL)
    {
        properties_set(payload, length);
        return ;
    }

    if (strstr(topic, "service/request") != NULL)
    {
        service_request(payload, length);
    }
}