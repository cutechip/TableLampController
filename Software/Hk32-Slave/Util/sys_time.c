#include "include.h"



static unsigned short  int sys_tick = 0;



/**
 * @brief ϵͳʱ��
 * 
 */
void sys_tick_count()
{
    sys_tick += 1;
}



/**
 * @description: ��ȡϵͳ�δ��ʱ
 * @param {*}
 * @return {*}
 */
unsigned short int get_sys_tick()
{
    return sys_tick;
}



/**
 * @description: �ж��Ƿ�ʱ
 * @param {unsigned long int} start ���㿪ʼ��ʱ��
 * @param {unsigned long int} timeout   ��ʱʱ��
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
