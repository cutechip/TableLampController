#include<Arduino.h>
#include<menu_handle.h>
#include<slave.h>
#include <sys_config.h>




void port1_enable(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port1.sw = value;
    if (config.port1.sw) ctrl_port(1, config.port1.brightness, config.port1.warm);
    else ctrl_port(1, 0, config.port1.warm);
    set_sys_config(config);
    
}

void port1_brightness(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port1.brightness = value;
    if (config.port1.sw) ctrl_port(1, config.port1.brightness, config.port1.warm);
    set_sys_config(config);
}

void port1_warm(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port1.warm = value;
    if (config.port1.sw) ctrl_port(1, config.port1.brightness, config.port1.warm);
    set_sys_config(config);
}


void port2_enable(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port2.sw = value;
    if (config.port2.sw) ctrl_port(2, config.port2.brightness, config.port2.warm);
    else ctrl_port(2, 0, config.port2.warm);
    set_sys_config(config);
}

void port2_brightness(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port2.brightness = value;
    if (config.port2.sw) ctrl_port(2, config.port2.brightness, config.port2.warm);
    set_sys_config(config);
}

void port2_warm(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port2.warm = value;
    if (config.port2.sw) ctrl_port(2, config.port2.brightness, config.port2.warm);
    set_sys_config(config);
}

void port3_enable(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port3.sw = value;
    if (config.port3.sw) ctrl_port(3, config.port3.brightness, config.port3.warm);
    else ctrl_port(3, 0, config.port3.warm);
    set_sys_config(config);
}

void port3_brightness(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port3.brightness = value;
    if (config.port3.sw) ctrl_port(3, config.port3.brightness, config.port3.warm);
    set_sys_config(config);
}


void port3_warm(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port3.warm = value;
    if (config.port3.sw) ctrl_port(3, config.port3.brightness, config.port3.warm);
    set_sys_config(config);
}


void port4_enable(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port4.sw = value;
    if (config.port4.sw) ctrl_port(4, config.port4.brightness, config.port4.warm);
    else ctrl_port(4, 0, config.port4.warm);
    set_sys_config(config);
}

void port4_brightness(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port4.brightness = value;
    if (config.port4.sw) ctrl_port(4, config.port4.brightness, config.port4.warm);
    set_sys_config(config);
}

void port4_warm(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.port4.warm = value;
    if (config.port4.sw) ctrl_port(4, config.port4.brightness, config.port4.warm);
    set_sys_config(config);
}



/**
 * @brief USB输出使能
 * 
 * @param value 
 */
void usb_pdout_enable(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.usbOut = value;
    slave_usb_pdout(value);
    set_sys_config(config);
}




/**
 * @brief 雷达是被检测
 * 
 * @param value 
 */
void radar_detection(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.radar = value;
    set_sys_config(config);
}



/**
 * @brief 延时关断
 * 
 * @param value 
 */
void delayed_shutdown(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.delayOff = value;
    set_sys_config(config);
}



/**
 * @brief 是否打开
 * 
 * @param value 
 */
void random_words_enable(uint8_t value)
{
    sys_config_t config = get_sys_config();
    config.word = value;
    set_sys_config(config);
}


/**
 * @brief 展示时间
 * 
 * @param value 
 */
void random_words_showtime(uint8_t value)
{
    
}