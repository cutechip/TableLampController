
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESPWiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <wifi.h>
#include <ArduinoJson.h>

#include <NetData.h>



word_t net_word;
WiFiClient wifiClient;
HTTPClient httpClient;
uint8_t get_weather_flag = 1;
uint8_t word_buf[64];
uint8_t chinese_buf[128];
uint8_t code_day = 99;

bool get_net_word()
{
    if (!wifi_isconnect()) 
    {
        return false;
    }
    String url = "http://192.168.1.201:5001/getWord";
    httpClient.begin(wifiClient, url);
    int httpResponseCode = httpClient.GET();
    if(httpResponseCode > 0) {
        String payload = httpClient.getString();
        Serial1.println(payload);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &root = jsonBuffer.parseObject(payload);
        String str1 = root.get<String>("word"); 
        String str2 = root.get<String>("chinese");  
        memset(word_buf, 0, sizeof(word_buf));
        memset(chinese_buf, 0, sizeof(chinese_buf)); 

        // 防止中文过长
        if (strlen(str1.c_str()) > 127) 
        {
            memcpy((char *)word_buf, str1.c_str(), 127);  
        } else {
            strcpy((char *)word_buf, str1.c_str());
            strcpy((char *)chinese_buf, str2.c_str());
        }
        
        return true;
    }

    return false;
}


/**
 * @brief 获取天气数据
 * 
 */
void get_weather_net_data()
{
    String url = "http://192.168.1.201:5001/getWeather";
    httpClient.begin(wifiClient, url);
    int httpResponseCode = httpClient.GET();
    if(httpResponseCode > 0) {
        String payload = httpClient.getString();
        Serial1.println(payload);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &root = jsonBuffer.parseObject(payload);
        code_day = root.get<unsigned char>("code_day"); 
    }
}

word_t get_random_word()
{
    net_word.word = (const char *)word_buf;
    net_word.chinese  = (const char *)chinese_buf;
    return net_word;
}


uint8_t get_weather()
{
    return code_day;
}


/**
 * @brief 处理网络数据请求
 * 
 */
void net_data_task()
{
    uint32_t tick = 0;
    uint8_t cnt = 0;
    if (!wifi_isconnect()) 
    {
        return ;
    }

    // 一分钟一次
    if (millis() - tick > 60000)
    {
        tick = millis();
        if (++cnt == 60)
        {
            cnt = 0;
            get_weather_flag = 1;
        }
    }

    if (get_weather_flag)
    {
        get_weather_flag = 0;
        // 一个小时更新一次天气
        get_weather_net_data();
    }


}