#ifndef _SYS_TIME_H
#define _SYS_TIME_H



/**
 * @brief 系统时基
 * 
 */
void sys_tick_count(void);


/**
 * @description: 获取系统滴答计时
 * @param {*}
 * @return {*}
 */
unsigned short int get_sys_tick(void);



/**
 * @description: 判断是否超时
 * @param {unsigned long int} start 计算开始的时间
 * @param {unsigned long int} timeout   超时时长
 * @return {*}
 */
unsigned char is_timeout(unsigned short int start, unsigned short int timeout);


void delay_ms(uint16_t ms);

#endif

