#ifndef __BSP_LIDAR_H
#define __BSP_LIDAR_H


#include "AllHeader.h"


void Lidar_init(void);
void USART3_Send_U8(uint8_t ch);
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void USART3_RX_deal(void);


#endif

