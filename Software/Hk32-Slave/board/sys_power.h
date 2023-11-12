#ifndef _SYS_POWER_H
#define _SYS_POWER_H

void pd_usb_gpio_init(void);
void sys_power_gpio_init(void);
void system_power_disable(void);
void system_power_enable(void);
void system_usb_disable(void);
void system_usb_enable(void);
uint8_t get_bat_leave(void);
#endif
