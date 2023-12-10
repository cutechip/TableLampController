#include <Arduino.h>
#include <slave.h>


typedef enum {
	OBJ_1 = 1,			// 对象1
	OBJ_2,				// 对象 2
	OBJ_3,				// 对象 3
	OBJ_4,				// 对象4
	SYSTEM_PS,			// 电源
	BAT_VALUE,			// 电量
	CHARGE_STATUS,		// 充电状态
	PD_USB_STATUS		// PD-USB状态
}slave_mode_t;

uint32_t slave_tick = 0;


uint8_t slave_recv_buf[16];
uint8_t recv_index = 0;
uint8_t send_buf[5] = {0xAA, 0x00, 0x00, 0x00, 0x55};
uint8_t bat_value = 4;






void slave_recv_data(uint8_t mode)
{
    if (mode == BAT_VALUE)
	{
		Serial1.printf("get bat..");
		bat_value = slave_recv_buf[0];
	}
}






void slave_task()
{
    static uint8_t setp = 0;
	static uint8_t mode = 0;
	static uint16_t recv_tick = 0;
	static uint32_t get_data_tick = 0;
	uint8_t ch;
    if (!Serial.available()) return ;
	while(Serial.read(&ch, 1))
	{
		recv_tick = millis();
		switch(setp)
		{
			case 0:
				if (ch == 0xAA) setp++;
				break;
			case 1:
				mode = ch;
				setp++;
				break;
			case 2:
				slave_recv_buf[recv_index++] = ch;
				if (recv_index >= 2) setp++; 
				break;
			case 3:
				if (ch == 0x55) 
				{
					slave_recv_data(mode);
				}
				mode = 0;
				setp = 0;
				memset(slave_recv_buf, 0, recv_index);
				recv_index = 0;
				break;
		}
	}
	
	if (setp &&  (millis() - recv_tick > 500))
	{
		mode = 0;
		setp = 0;
		memset(slave_recv_buf, 0, recv_index);
		recv_index = 0;
	}


	if (millis() - get_data_tick > 60000)
	{
		get_data_tick = millis();
		slave_get_bat();
	}
}


/**
 * @brief 控制端口输出
 * 
 * @param port_num 
 * @param value1 
 * @param value2 
 */
void ctrl_port(uint8_t port_num, uint8_t value1, uint8_t value2)
{
	Serial1.printf("port_num:%d,value1:%d,value2:%d", port_num, value1, value2);
	Serial1.printf("\r\n");
    switch (port_num)
    {
    case 1:
        send_buf[1] = 0x01;
        break;
    case 2:
        send_buf[1] = 0x02;
        break;
    case 3:
        send_buf[1] = 0x03;
        break;
    case 4:
        send_buf[1] = 0x04;
        break;
    }
    send_buf[2] = value1;
    send_buf[3] = value2;
    Serial.write(send_buf, sizeof(send_buf));
}




/**
 * @brief PD USB
 * 
 * @param status 
 */
void slave_usb_pdout(uint8_t status)
{
	send_buf[1] = PD_USB_STATUS;
	send_buf[2] = status;
    send_buf[3] = 0x00;
    Serial.write(send_buf, sizeof(send_buf));
}


void slave_get_bat()
{
	send_buf[1] = BAT_VALUE;
	send_buf[2] = 0x00;
    send_buf[3] = 0x00;
    Serial.write(send_buf, sizeof(send_buf));
}


/**
 * @brief 获取电量
 * 
 */
uint8_t get_bat_power()
{
	return bat_value;
}