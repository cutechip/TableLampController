#ifndef _INCLUDE_H
#define _INCLUDE_H

#include "hk32f030m.h" 

#include "stddef.h"
#include "stdio.h"
#include "string.h"



// 外设库驱动
#include "drv_systick.h"
#include "drv_usart.h"
#include "drv_adc.h"
#include "drv_gpio.h"
#include "drv_pwm.h"


#include "sys_power.h"
#include "charge_detection.h"
#include "modulator.h"


#include "system_process.h"

// 系统库文件
#include "sys_time.h"
#include "sys_task.h"
#include "sys_keyinput.h"
#include "sys_ringbuffer.h"
#include "sys_blink.h"




#endif
