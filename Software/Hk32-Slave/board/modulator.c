#include "include.h"

#define SLOW_SETP    1


typedef struct {
    uint8_t brightness;
    uint8_t warm;
} port_t;

port_t target_port[4];

int8_t brightness[4];
int8_t warm[4];
uint16_t total = 0;
void modulator_ctrl(uint8_t num, uint8_t val1, uint8_t val2)
{
	switch(num)
	{
		case 1:
			target_port[0].brightness = val1;
			target_port[0].warm = val2;
			break;
		case 2:
			target_port[1].brightness = val1;
			target_port[1].warm = val2;
			break;
		case 3:
			target_port[2].brightness = val1;
			target_port[2].warm = val2;
			break;
		case 4:
			target_port[3].brightness = val1;
			target_port[3].warm = val2;
			break;
	}

}



// 控制灯光亮度
void modulator_process()
{
	uint8_t i = 0;
	uint8_t refresh_flag = 0;
	
	// 共4个端口，判断目标值比现在值大或者小就进行缓加缓减操作 可达到缓慢开关灯效果
	for (i = 0; i < 4; i++)
	{
		refresh_flag = 0;
		if(warm[i] < target_port[i].warm)
		{
			warm[i] += SLOW_SETP;
			if (warm[i] > target_port[i].warm) warm[i] = target_port[i].warm;
			refresh_flag = 1;
		}
		if (brightness[i] < target_port[i].brightness)
		{
			brightness[i] += SLOW_SETP;
			if (brightness[i] > target_port[i].brightness) brightness[i] = target_port[i].brightness;
			refresh_flag = 1;
		}
		
		if (warm[i] > target_port[i].warm)
		{
			warm[i] -= SLOW_SETP;
			if (warm[i] < target_port[i].warm) warm[i] = target_port[i].warm;
			refresh_flag = 1;
		}
		
		if (brightness[i] > target_port[i].brightness)
		{
			brightness[i] -= SLOW_SETP;
			if (brightness[i] < target_port[i].brightness) brightness[i] = target_port[i].brightness;
			refresh_flag = 1;
		}
		
		if (refresh_flag)
		{
			total = 2 * brightness[i];
			set_pwm_duty_cycle(2 * i + 1, total - (target_port[i].warm * total / 100));
			set_pwm_duty_cycle(2 * i + 2, target_port[i].warm * total /100);
		}
		
	}	
}
