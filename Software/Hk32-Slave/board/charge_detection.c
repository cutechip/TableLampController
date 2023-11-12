#include "include.h"




void charge_detection_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}


uint8_t get_charge_status()
{
	return !GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
}

void send_charge_detection(uint8_t status)
{
	uint8_t cmd = CHARGE_STATUS;
	usart_send_bytes(1, "\xAA", 1);
	usart_send_bytes(1, &cmd, 1);
	usart_send_bytes(1, &status, 1);
	usart_send_bytes(1, "\x00", 1);
	usart_send_bytes(1, "\x55", 1);
}

void charge_detection_process()
{
	static uint8_t last_status = 0;
	static uint8_t higth_cnt = 0;
	static uint8_t low_cnt = 0;
	if (get_charge_status())
	{
		low_cnt = 0;
		if (++higth_cnt >= 3)
		{
			higth_cnt = 3;
			if (!last_status) send_charge_detection(1);
			last_status = 1;
		}
	}else {
		higth_cnt = 0;
		if (++low_cnt >= 3)
		{
			low_cnt = 3;
			if (last_status) send_charge_detection(0);
			last_status = 0;
		}
		
	}

}


