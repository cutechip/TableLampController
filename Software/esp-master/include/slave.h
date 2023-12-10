#ifndef _SLAVE_H
#define _SLAVE_H


void slave_get_bat();
void slave_task();
uint8_t get_bat_power();
void ctrl_port(uint8_t port_num, uint8_t value1, uint8_t value2);
void slave_usb_pdout(uint8_t status);

#endif