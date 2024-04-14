#include "include.h"





ring_buf_t slave_ringbuf;

static uint8_t recv_buf[10];
static uint8_t recv_index = 0;

static uint8_t send_buf[2]; 

unsigned char slave_ringbufer[256];


static unsigned char read_data(unsigned char *buf, unsigned int len)
{
    return ring_buf_get(&slave_ringbuf, buf, len);
}



void slave_put(unsigned char res)
{
    ring_buf_put(&slave_ringbuf, &res, 1);
}


void slave_init()
{
	ring_buf_init(&slave_ringbuf, slave_ringbufer, sizeof(slave_ringbufer));
	
}


void slave_send_data(uint8_t cmd, uint8_t *bytes)
{
	usart_send_bytes(1, "\xAA", 1);
	usart_send_bytes(1, &cmd, 1);
	usart_send_bytes(1, bytes, 2);
	usart_send_bytes(1, "\x44", 1);
}


void slave_recv_data(uint8_t mode)
{
	if (mode == OBJ_1)
	{
		modulator_ctrl(1, recv_buf[0], recv_buf[1]);
		return ;
	}
	
	if (mode == OBJ_2)
	{
		modulator_ctrl(2, recv_buf[0], recv_buf[1]);
		return ;
	}
	
	if (mode == OBJ_3)
	{
		modulator_ctrl(3, recv_buf[0], recv_buf[1]);
		return ;
	}
	
	if (mode == OBJ_4)
	{
		modulator_ctrl(4, recv_buf[0], recv_buf[1]);
		return ;
	}
	
	if (mode == SYSTEM_PS)
	{
		recv_buf[0] ? system_power_enable() : system_power_disable();
		return ;
	}
	
	if (mode == BAT_VALUE)
	{
		send_buf[0] = get_bat_leave();
		slave_send_data(BAT_VALUE, send_buf);
		return ;
	}
	
	if (mode == CHARGE_STATUS)
	{
		send_buf[0] = get_charge_status();
		slave_send_data(CHARGE_STATUS, send_buf);
		return ;
	}
	
	if (mode == PD_USB_STATUS)
	{
		recv_buf[0] ? system_usb_enable() : system_usb_disable();
		return ;
	}
	
}

void slave_recv_process()
{
	static uint8_t setp = 0;
	static uint8_t mode = 0;
	static uint16_t recv_tick = 0;
	uint8_t ch;
	while(read_data(&ch, 1))
	{
		recv_tick = get_sys_tick();
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
				recv_buf[recv_index++] = ch;
				if (recv_index >= 2) setp++; 
				break;
			case 3:
				if (ch == 0x55) 
				{
					slave_recv_data(mode);
				}
				mode = 0;
				setp = 0;
				memset(recv_buf, 0, recv_index);
				recv_index = 0;
				break;
			
		}
	}
	
	if (setp && is_timeout(recv_tick, 500))
	{
		mode = 0;
		setp = 0;
		memset(recv_buf, 0, recv_index);
		recv_index = 0;
	}
}

void system_process()
{
	slave_recv_process();
}

