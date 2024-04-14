#include "include.h"




void sys_power_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	system_power_enable();
}


void pd_usb_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	
}

/**
 * @brief ¹Ø»ú
 * 
 */
void system_power_disable()
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
}

void system_power_enable()
{
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
}


void system_usb_disable()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

void system_usb_enable()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
}


uint8_t get_bat_leave()
{
	uint16_t adc_value = get_adc_average(1, 5);
	if (adc_value > 2500) return 4;
	if (adc_value > 2300) return 3;
	if (adc_value > 2200) return 2;
	if (adc_value > 2100) return 1;
	return 0;

}

