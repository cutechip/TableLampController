#include "include.h"


static blink_dev_t *head = NULL;         /*头结点 -*/

/*
 * @brief       创建blink设备
 * @param[in]   dev    - 设备
 * @param[in]   ioctrl - IO控制函数
 * @return      none
 */
void blink_dev_create(blink_dev_t *dev, void (*ioctrl)(unsigned char enable))
{
    blink_dev_t  *tail = head;  
    memset(dev, 0, sizeof(blink_dev_t));
    dev->ioctrl = ioctrl;    
    dev->next = NULL;
    if (head == NULL) {
        head = dev;
        return;
    }
    while (tail->next != NULL)
        tail = tail->next;
    tail->next = dev;
}


/* 
 * @brief       blink 设备控制
 * @param[in]   name    - 设备名称
 * @param[in]   ontime - 开启时间(如果该值为0则永久关闭)
 * @param[in]   offtime- 关闭时间
 * @param[in]   repeats- 重复次数(0表示无限循环)
 * @return      none
 */
void blink_dev_ctrl(blink_dev_t *dev, unsigned int ontime, unsigned int offtime, unsigned int repeats, unsigned char finish_status)
{
    dev->ontime  = ontime;
    dev->offtime = offtime + ontime;                  
    dev->repeats = repeats;
    dev->finish_status = finish_status;
    dev->tick    = get_sys_tick();
    if (ontime  == 0) {
        dev->ioctrl(0);
        dev->enable  = 0;
    }        
}
/*
 * @brief	   忙判断
 */
unsigned char blink_dev_busy(blink_dev_t *dev)
{
    return dev->ontime;
}


/*
 * @brief       blink设备管理
 * @param[in]   none
 * @return      none
 */
void blink_dev_process(void)
{
    blink_dev_t  *dev;
    for (dev = head; dev != NULL; dev = dev->next) {
        if (dev->ontime == 0) {
            continue;
        } else if ((get_sys_tick() - dev->tick) < dev->ontime) {
            if (!dev->enable) {
                dev->enable = 1;
                dev->ioctrl(1);
            }
        } else if ((get_sys_tick() - dev->tick) < dev->offtime) {   
            if (dev->enable) {
                dev->enable = 0;
                dev->ioctrl(0);
            }
        } else {
            dev->tick = get_sys_tick();
            if (dev->repeats-- && dev->repeats == 0) {
					dev->ontime = 0;
					dev->ioctrl(dev->finish_status);
					dev->enable = 0;
				}
        }
    }
}

