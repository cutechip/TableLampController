#include <Arduino.h>
#include <radar.h>
#define RADAR_IO  16


void radar_init()
{
    pinMode(RADAR_IO, INPUT_PULLDOWN_16);
}



/**
 * @brief ��ȡ�״�״̬
 * 
 * @return uint8_t 
 */
uint8_t get_radar_status()
{
    return digitalRead(RADAR_IO);
}

