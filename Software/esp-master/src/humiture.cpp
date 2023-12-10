#include "humiture.h"
#include <AHT20.h>
#include <Wire.h>

AHT20 aht20;
float temperature, humidity;
/**
 * @brief 温湿度模块初始化
 * 
 */
void humiture_init()
{
    Wire.begin(0, 5);
    if (aht20.begin() == false)
    {
        Serial.println("AHT20 not detected. Please check wiring. Freezing.");
        // while (1);
    }
}





/**
 * @brief 获取温度
 * 
 * @return float 
 */
float get_temperature()
{
    return temperature;
}




/**
 * @brief 获取湿度
 * 
 * @return float 
 */
float get_humidity()
{
    return humidity;
}



/**
 * @brief 温湿度
 * 
 */
void humidity_task()
{
    static uint32_t humidity_tick = 0;
    if (millis() - humidity_tick > 5000)
    {
        humidity_tick = millis();

        temperature = aht20.getTemperature();
        humidity = aht20.getHumidity();

    }
}