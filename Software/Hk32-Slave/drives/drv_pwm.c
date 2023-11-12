#include "include.h"




void pwm_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD, ENABLE);
	
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_4);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_3);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Schmit = GPIO_Schmit_Disable;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}



void pwm_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructure.TIM_Period = 99;	
	TIM_TimeBaseStructure.TIM_Prescaler = 31;	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_Cmd(TIM1, ENABLE);	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


// 端口 2 4在改版中，下次更新会重新更改
void set_pwm_duty_cycle(uint8_t pwm_num, uint8_t duty)
{
	if (duty > 100) duty = 100;
	switch(pwm_num)
	{
		case 1:
			TIM_SetCompare1(TIM2, duty);	
		break;
		case 2:
			TIM_SetCompare4(TIM2, duty);	
		break;
		case 3:
//			TIM_SetCompare3(TIM2, duty);	
		break;
		case 4:
//			TIM_SetCompare1(TIM2, duty);	
		break;
		case 5:
			TIM_SetCompare1(TIM1, duty);	
		break;
		case 6:
			TIM_SetCompare2(TIM1, duty);	
		break;
		case 7:
//			TIM_SetCompare3(TIM1, duty);	
		break;
		case 8:
//			TIM_SetCompare3(TIM1, duty);	
		break;
	}
}
