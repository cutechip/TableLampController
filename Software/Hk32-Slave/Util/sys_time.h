#ifndef _SYS_TIME_H
#define _SYS_TIME_H



/**
 * @brief ϵͳʱ��
 * 
 */
void sys_tick_count(void);


/**
 * @description: ��ȡϵͳ�δ��ʱ
 * @param {*}
 * @return {*}
 */
unsigned short int get_sys_tick(void);



/**
 * @description: �ж��Ƿ�ʱ
 * @param {unsigned long int} start ���㿪ʼ��ʱ��
 * @param {unsigned long int} timeout   ��ʱʱ��
 * @return {*}
 */
unsigned char is_timeout(unsigned short int start, unsigned short int timeout);


void delay_ms(uint16_t ms);

#endif

