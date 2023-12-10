#include <Arduino.h>
#include <lux.h>





uint8_t get_lux()
{
    uint16_t vcc_cache = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        //delay(1);
        vcc_cache += analogRead(A0);
    }
    vcc_cache = vcc_cache / 20;
    return vcc_cache;
}