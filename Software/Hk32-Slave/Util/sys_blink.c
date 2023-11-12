#include "include.h"


static blink_dev_t *head = NULL;         /*ͷ��� -*/

/*
 * @brief       ����blink�豸
 * @param[in]   dev    - �豸
 * @param[in]   ioctrl - IO���ƺ���
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
 * @brief       blink �豸����
 * @param[in]   name    - �豸����
 * @param[in]   ontime - ����ʱ��(�����ֵΪ0�����ùر�)
 * @param[in]   offtime- �ر�ʱ��
 * @param[in]   repeats- �ظ�����(0��ʾ����ѭ��)
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
 * @brief	   æ�ж�
 */
unsigned char blink_dev_busy(blink_dev_t *dev)
{
    return dev->ontime;
}


/*
 * @brief       blink�豸����
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

