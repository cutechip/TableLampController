/*
 * @Description: 
 * @Date: 2022-02-14 11:08:02
 * @LastEditTime: 2022-02-14 13:59:45
 * @FilePath: \drv_ctrl_code\UtilsLib\sys_util_blink.h
 */
#ifndef __SYS_UTIL_BLINK_H
#define __SYS_UTIL_BLINK_H


/*Blink �豸����*/
typedef struct blink_dev {
    unsigned short ontime;                                /*����ʱ��*/
    unsigned short offtime;                               /*�ر�ʱ��*/
    unsigned short repeats;                               /*�ظ�����*/                                
    unsigned char  enable;
    unsigned int   tick;
    unsigned char finish_status;                           /*��Ϻ��״̬*/
    void (*ioctrl)(unsigned char enable);                          /*io���ƽӿ�*/
    struct blink_dev *next;
}blink_dev_t;


void blink_dev_create(blink_dev_t *dev, void (*ioctrl)(unsigned char enable));

void blink_dev_ctrl(blink_dev_t *dev, unsigned int ontime, unsigned int offtime, unsigned int repeat, unsigned char finish_status);

unsigned char blink_dev_busy(blink_dev_t *dev);

void blink_dev_process(void);



#endif

