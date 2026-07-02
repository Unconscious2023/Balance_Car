#include "bsp_lidar.h"



void Lidar_init(void)
{
	MX_USART3_UART_Init();
	LL_USART_EnableIT_RXNE(USART3); // Start receiving interrupt 启动接收中断
}

 
//发送一个字符 Send a character
void USART3_Send_U8(uint8_t ch)
{
	while (!LL_USART_IsActiveFlag_TXE(USART3))
	{
	};
	LL_USART_TransmitData8(USART3, ch);
}

//发送一个字符串 Send a string
/**
 * @Brief: UsART3发送数据 UsART3 sends data
 * @Note: 
 * @Parm: BufferPtr:待发送的数据（Data to be sent）  Length:数据长度（The length of the data）
 * @Retval: 
 */
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART3_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}


//中断调用 中断调用
void USART3_RX_deal(void)
{
	uint8_t Rx3_Temp;
	if (LL_USART_IsEnabledIT_RXNE(USART3))
	{
		Rx3_Temp = LL_USART_ReceiveData8(USART3); // Read information and clear interrupts 读取信息并清除中断
		recv_lidar_data(Rx3_Temp);
		//USART3_Send_U8(Rx3_Temp);
	}
}




