#include "include.h"

uint8_t  fac_us=0;	

void systick_init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	SysTick_Config(40000);
}


void SysTick_Handler(void)
{
	sys_tick_count();
}


// us级别延时函数
void delay_us(uint16_t us)
{
	uint32_t i=0;
	while(us--){
	i=10; while(i--);
	}
}
