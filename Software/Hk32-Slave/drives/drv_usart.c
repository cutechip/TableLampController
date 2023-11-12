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
		// �򿪴���GPIO��ʱ��
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
		
		// �򿪴��������ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	  
	  // ��USART�ź����ӵ�IO����
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);

		// ��USART Tx��GPIO����Ϊ���츴��ģʽ
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

	  // ��USART Rx��GPIO����Ϊ����ģʽ
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		// ���ô��ڵĹ�������
		// ���ò�����
		USART_InitStructure.USART_BaudRate = bound;
		// ���� �������ֳ�
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		// ����ֹͣλ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		// ����У��λ
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		// ����Ӳ��������
		USART_InitStructure.USART_HardwareFlowControl = 
		USART_HardwareFlowControl_None;
		// ���ù���ģʽ���շ�һ��
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		// ��ɴ��ڵĳ�ʼ������
		USART_Init(USARTx, &USART_InitStructure);
		
		// ʹ�ܴ��ڽ����ж�
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);	
		
		// ʹ�ܴ���
		USART_Cmd(USARTx, ENABLE);
		/* ����USARTΪ�ж�Դ */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		/* ���ȼ�*/
		NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
		/* ʹ���ж� */
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		/* ��ʼ������NVIC */
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
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);	//�ȴ��������
		USART_SendData(USARTx, *(byte + i));
		i++;
	}
}




void USART1_IRQHandler(void)                	//����1�жϷ������
{
	uint8_t res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
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


