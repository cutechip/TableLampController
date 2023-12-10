#include <Arduino.h>
#include <ControlCenter.h>
#include <sys_config.h>
#include <slave.h>
#include <radar.h>

#define SLOW_SETP    4

bool masterSwitch = true;
bool lastMaster = false;
sys_config_t my_config;




/**
 * @brief 发送默认值到从机
 * 
 */
void control_default_value()
{
    my_config = get_sys_config();
    if (my_config.port1.sw) ctrl_port(1, my_config.port1.brightness, my_config.port1.warm);
    if (my_config.port2.sw) ctrl_port(2, my_config.port2.brightness, my_config.port2.warm);
    if (my_config.port3.sw) ctrl_port(3, my_config.port3.brightness, my_config.port3.warm);
    if (my_config.port4.sw) ctrl_port(4, my_config.port4.brightness, my_config.port4.warm);
    slave_usb_pdout(my_config.usbOut);
}

/**
 * @brief 控制主开关
 * 
 * @param switch 
 */
void master_switch(bool status)
{
    masterSwitch = status;
}



/**
 * @brief 主开关端口控制马上亮
 * 
 * @param enable 
 */
void master_switch_port(bool enable)
{
    my_config = get_sys_config();
    if (!enable)
    {
        ctrl_port(1, 0, my_config.port1.warm);
        ctrl_port(2, 0, my_config.port2.warm);
        ctrl_port(3, 0, my_config.port3.warm);
        ctrl_port(4, 0, my_config.port4.warm);
        return ;
    }

    if (my_config.port1.sw) ctrl_port(1, my_config.port1.brightness, my_config.port1.warm);
    if (my_config.port2.sw) ctrl_port(2, my_config.port2.brightness, my_config.port2.warm);
    if (my_config.port3.sw) ctrl_port(3, my_config.port3.brightness, my_config.port3.warm);
    if (my_config.port4.sw) ctrl_port(4, my_config.port4.brightness, my_config.port4.warm);
}


void radar_detection()
{
    static int16_t found_cnt = 0;
    static int16_t no_found_cnt = 0;
    static uint32_t radar_tick = 0;
    static bool found_flag = false;
    if (!my_config.radar) 
    {
        found_cnt = 0;
        return ;
    }
    if (millis() - radar_tick > 200)
    {
        // Serial1.printf("radar:%d, %d, %d", found_cnt, no_found_cnt, found_flag);
        radar_tick = millis();
        if(get_radar_status())
        {
            no_found_cnt = 0;
            if(++found_cnt < 3) return ;
            found_cnt = 3;
            if (!found_flag)
            {
                found_flag = true;
                master_switch_port(true);
            }
            return ;
        }
        found_cnt = 0;
        if(++no_found_cnt < 600) return ;
        no_found_cnt = 600;
        if (found_flag)
        {
            found_flag = false;
            master_switch_port(false);
        }
    }
    

}

/**
 * @brief 控制器的控制中心 开关， 亮度、雷达的管理
 * 
 */
void control_center_task()
{
    if (lastMaster != masterSwitch)
    {
        lastMaster = masterSwitch;
        master_switch_port(masterSwitch);
        // 主开关状态发生改变
    }
    if (!masterSwitch) return ;
    radar_detection();
}



