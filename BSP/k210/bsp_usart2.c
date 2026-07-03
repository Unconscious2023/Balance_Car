#include "bsp_usart2.h"


//USART2
void USART2_init(void)
{
	MX_USART2_UART_Init();
	LL_USART_EnableIT_RXNE(USART2); // Start receiving interrupt       
	

}


//       Send a character
void USART2_Send_U8(uint8_t ch)
{
	while (!LL_USART_IsActiveFlag_TXE(USART2))
	{
	};
	LL_USART_TransmitData8(USART2, ch);
}

//        Send a string
/**
 * @Brief: UsART2     UsART2 sends data
 * @Note: 
 * @Parm: BufferPtr:       Data to be sent   Length:     The length of the data 
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

//         Serial port interrupt service function
void USART2_RX_deal(void)
{
	uint8_t Rx2_Temp;
	if (LL_USART_IsEnabledIT_RXNE(USART2))
	{
		Rx2_Temp = LL_USART_ReceiveData8(USART2); // Read information and clear interrupts          
//		USART2_Send_U8(Rx2_Temp);
		if(mode == K210_QR) //      QR code mode
		{
			Deal_K210_QR(Rx2_Temp);
		}
		else if(mode == K210_SelfLearn) //       Self-learning mode
		{
			Deal_K210_self(Rx2_Temp);
		}
		else if(mode == K210_mnist) //       Recognizing digital patterns
		{
			Deal_K210_minst(Rx2_Temp);
		}
		else if(mode == K210_Line || mode == K210_Follow || mode == ChaseLine_Mode) //k210      k210 line patrol and following
		{
			Deal_K210_AI(Rx2_Temp);
		}
	}
}
