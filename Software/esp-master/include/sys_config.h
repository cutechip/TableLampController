#ifndef _SYS_CONFIG_H
#define _SYS_CONFIG_H


typedef struct {
    bool sw;
    uint8_t brightness;
    uint8_t warm;
} port_t;


typedef struct {
    bool nightStart;
    bool radar;
    uint8_t delayOff;
    bool usbOut;
    bool word;
    port_t port1;
    port_t port2;
    port_t port3;
    port_t port4;
} sys_config_t;



void read_config();
sys_config_t get_sys_config();
void set_sys_config(sys_config_t config);
void sys_congig_commit();

#endif