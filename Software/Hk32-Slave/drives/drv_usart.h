#ifndef _UASRT_H
#define _USART_H



void uart_init(uint8_t uart_num, uint32_t bound);
void usart_send_bytes(uint8_t usart_num, uint8_t *byte, uint16_t len);


#endif

