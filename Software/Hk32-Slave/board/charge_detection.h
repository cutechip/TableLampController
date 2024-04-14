#ifndef _CHARGE_DETECTION_H
#define _CHARGE_DETECTION_H


void charge_detection_gpio_init(void);
void charge_detection_process(void);
uint8_t get_charge_status(void);

#endif