#ifndef _SYSTEM_PROCESS_H
#define _SYSTEM_PROCESS_H


typedef enum {
	OBJ_1 = 1,			// ����1
	OBJ_2,				// ���� 2
	OBJ_3,				// ���� 3
	OBJ_4,				// ����4
	SYSTEM_PS,			// ��Դ
	BAT_VALUE,			// ����
	CHARGE_STATUS,		// ���״̬
	PD_USB_STATUS		// PD-USB״̬
}mode_t;

void slave_init(void);
void system_process(void);
void slave_put(unsigned char res);

#endif