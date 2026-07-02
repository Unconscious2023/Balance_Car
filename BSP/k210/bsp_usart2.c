#include "bsp_usart2.h"


//USART2
void USART2_init(void)
{
	MX_USART2_UART_Init();
	LL_USART_EnableIT_RXNE(USART2); // Start receiving interrupt 启动接收中断
	

}


//发送一个字符 Send a character
void USART2_Send_U8(uint8_t ch)
{
	while (!LL_USART_IsActiveFlag_TXE(USART2))
	{
	};
	LL_USART_TransmitData8(USART2, ch);
}

//发送一个字符串 Send a string
/**
 * @Brief: UsART2发送数据 UsART2 sends data
 * @Note: 
 * @Parm: BufferPtr:待发送的数据（Data to be sent）  Length:数据长度（The length of the data）
 * @Retval: 
 */
void USART2_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART2_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}

//串口中断服务函数 Serial port interrupt service function
void USART2_RX_deal(void)
{
	uint8_t Rx2_Temp;
	if (LL_USART_IsEnabledIT_RXNE(USART2))
	{
		Rx2_Temp = LL_USART_ReceiveData8(USART2); // Read information and clear interrupts 读取信息并清除中断
//		USART2_Send_U8(Rx2_Temp);
		if(mode == K210_QR) //二维码模式 QR code mode
		{
			Deal_K210_QR(Rx2_Temp);
		}
		else if(mode == K210_SelfLearn) //自主学习模式 Self-learning mode
		{
			Deal_K210_self(Rx2_Temp);
		}
		else if(mode == K210_mnist) //识别数字模式 Recognizing digital patterns
		{
			Deal_K210_minst(Rx2_Temp);
		}
		else if(mode == K210_Line || mode == K210_Follow || mode == ChaseLine_Mode) //k210巡线、跟随 k210 line patrol and following
		{
			Deal_K210_AI(Rx2_Temp);
		}
	}
}
