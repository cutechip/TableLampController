#ifndef _SYSTEM_PROCESS_H
#define _SYSTEM_PROCESS_H


typedef enum {
	OBJ_1 = 1,			// 对象1
	OBJ_2,				// 对象 2
	OBJ_3,				// 对象 3
	OBJ_4,				// 对象4
	SYSTEM_PS,			// 电源
	BAT_VALUE,			// 电量
	CHARGE_STATUS,		// 充电状态
	PD_USB_STATUS		// PD-USB状态
}mode_t;

void slave_init(void);
void system_process(void);
void slave_put(unsigned char res);

#endif