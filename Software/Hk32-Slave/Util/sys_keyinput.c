#include "include.h"



keyinput_dev_t *keynput_head = NULL;


/**
 * @brief 创建按键设备
 * 
 * @param keynput 按键设备
 */
void key_dev_create(keyinput_dev_t *keynput)
{
    keyinput_dev_t *keynput_tail = keynput_head;
    if (keynput_head == NULL)
    {
        keynput_head = keynput;
        return ;
    }

    while (keynput_tail->keyinput_dev_next != NULL)
        keynput_tail = keynput_tail->keyinput_dev_next;
    keynput_tail->keyinput_dev_next = keynput;
}


/**
 * @brief 按键处理
 * 
 */
void keyinput_process()
{    
    keyinput_dev_t  *dev;
    for (dev = keynput_head; dev != NULL; dev = dev->keyinput_dev_next) 
    {
        if (is_timeout(dev->key_timer,  0))
        {
            dev->key_timer = get_sys_tick();
            if (dev->get_gpio_val == NULL ) continue ;
            if (dev->get_gpio_val())
            {
                dev->hight_cnt++;
                dev->low_cnt = 0;
                if (dev->hight_cnt >= 10)
                {
                    if (!dev->key_press_flag)
                    {
                        dev->key_press_flag = 1;
                        dev->key_cnt++;
                        if (dev->key_press_process != NULL) dev->key_press_process(dev->key_cnt);
                    }
                }


                if (dev->long_press.long_press_max_leav >= 1 && dev->long_press_flag < 1)
                {
                    if (dev->hight_cnt >= (dev->long_press.long_press_time_1/10))
                    {
                        dev->long_press_flag = 1;           // 级别1
                        if (dev->key_long_press_process != NULL)  dev->key_long_press_process(dev->long_press_flag, dev->key_cnt);
                    }
                }

                if (dev->long_press.long_press_max_leav >= 2 && dev->long_press_flag < 2)
                {
                    if (dev->hight_cnt >= (dev->long_press.long_press_time_2/10))
                    {
                        dev->long_press_flag = 2;           // 级别2
                        if (dev->key_long_press_process != NULL)  dev->key_long_press_process(dev->long_press_flag, dev->key_cnt);
                    }
                } 
            }else{
                dev->low_cnt++;
                dev->hight_cnt = 0;
                // 按键出现短暂释放
                if (dev->low_cnt >= 5)
                {
                    dev->key_press_flag = 0;      
                }

                // 按键真正释放
                if (dev->low_cnt >= 50)
                {
                    if (dev->key_cnt > 0)
                    {
                        if (dev->key_reaease_process != NULL)  dev->key_reaease_process(dev->key_cnt, dev->long_press_flag);
                        dev->long_press_flag = 0;
                        dev->key_cnt = 0;
                    }
                }
            }
        }
    }
}
