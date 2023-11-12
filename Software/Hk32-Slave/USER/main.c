#include "include.h"



// 控制灯光任务
sys_task_t modulator_task;

// 系统主任务，负责收发控制命令
sys_task_t system_task;

// 充电检测任务
sys_task_t charge_detection_task;



// 任务创建
void task_start()
{
	sys_task_create(&charge_detection_task, charge_detection_process, 50);
	sys_task_start(&charge_detection_task);
	sys_task_create(&modulator_task, modulator_process, 20);
	sys_task_start(&modulator_task);
	sys_task_create(&system_task, system_process, 50);
	sys_task_start(&system_task);
}
int main(void)
{
	systick_init();
	sys_board_gpio_init();
	pwm_init();
	adc_init();
	uart_init(1, 115200);
	task_start();
	slave_init();
	while (1)
	{
		sys_task_process();
	}
}




