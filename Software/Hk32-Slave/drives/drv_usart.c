#include "include.h"



void uart_init(uint8_t uart_num, uint32_t bound)
{
	USART_TypeDef* USARTx;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	switch (uart_num)
	{
		case 1:
			USARTx = USART1;
			break;
	}
	if(USARTx == USART1)
    {
		// 打开串口GPIO的时钟
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
		
		// 打开串口外设的时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	  
	  // 将USART信号连接到IO口上
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);

		// 将USART Tx的GPIO配置为推挽复用模式
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

	  // 将USART Rx的GPIO配置为复用模式
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		// 配置串口的工作参数
		// 配置波特率
		USART_InitStructure.USART_BaudRate = bound;
		// 配置 针数据字长
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		// 配置停止位
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		// 配置校验位
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		// 配置硬件流控制
		USART_InitStructure.USART_HardwareFlowControl = 
		USART_HardwareFlowControl_None;
		// 配置工作模式，收发一起
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		// 完成串口的初始化配置
		USART_Init(USARTx, &USART_InitStructure);
		
		// 使能串口接收中断
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);	
		
		// 使能串口
		USART_Cmd(USARTx, ENABLE);
		/* 配置USART为中断源 */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		/* 优先级*/
		NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
		/* 使能中断 */
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		/* 初始化配置NVIC */
		NVIC_Init(&NVIC_InitStructure);
	}
	
}




void usart_send_bytes(uint8_t usart_num, uint8_t *byte, uint16_t len)
{
	uint16_t i = 0;
	USART_TypeDef* USARTx;
	switch (usart_num)
	{
		case 1:
			USARTx = USART1;
			break;
	}
    while(len--) 
	{
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);	//等待发送完成
		USART_SendData(USARTx, *(byte + i));
		i++;
	}
}




void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	uint8_t res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		res = USART_ReceiveData(USART1);	
		slave_put(res);
	} 
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET)
    {
        USART_ReceiveData(USART1);
        USART_ClearFlag(USART1,USART_FLAG_ORE);
    }
} 


