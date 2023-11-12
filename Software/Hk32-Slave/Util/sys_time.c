#include "include.h"



static unsigned short  int sys_tick = 0;



/**
 * @brief 系统时基
 * 
 */
void sys_tick_count()
{
    sys_tick += 1;
}



/**
 * @description: 获取系统滴答计时
 * @param {*}
 * @return {*}
 */
unsigned short int get_sys_tick()
{
    return sys_tick;
}



/**
 * @description: 判断是否超时
 * @param {unsigned long int} start 计算开始的时间
 * @param {unsigned long int} timeout   超时时长
 * @return {*}
 */
unsigned char is_timeout(unsigned short int start, unsigned short int timeout)
{
    return ((uint16_t)(get_sys_tick() - start)) > timeout ?  1: 0;
}




void delay_ms(uint16_t ms)
{
	uint16_t tick = get_sys_tick();
	while(!is_timeout(tick, ms));

}
