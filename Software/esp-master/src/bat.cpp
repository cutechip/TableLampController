#include <Arduino.h>
#include <bat.h>





uint16_t get_adc()
{
    
    return analogRead(A0);
}




/**
 * @brief 获取电量等级
 * 
 * @return uint8_t 
 */
uint8_t get_bat_level()
{
    uint16_t adc_value = get_adc();
    if (adc_value > 510)
    {
        return 4;
    }
    else if (adc_value > 495)
    {
        return 3;
    }
    else if (adc_value > 480)
    {
        return 2;
    }
    else if (adc_value > 460)
    {
        return 1;
    }
    else {
        return 0;
    }
}