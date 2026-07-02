#ifndef _BSP_IRTRACKING_H_
#define _BSP_IRTRACKING_H_

#include "AllHeader.h"


#define IR_X1_Pin 	GPIO_PIN_4
#define IR_X1_Port 	GPIOC

#define IR_X2_Pin 	GPIO_PIN_5
#define IR_X2_Port 	GPIOC

#define IR_X3_Pin 	GPIO_PIN_0
#define IR_X3_Port 	GPIOB

#define IR_X4_Pin 	GPIO_PIN_1
#define IR_X4_Port 	GPIOB


#define IN_X1 HAL_GPIO_ReadPin(IR_X1_Port, IR_X1_Pin)
#define IN_X2 HAL_GPIO_ReadPin(IR_X2_Port, IR_X2_Pin)
#define IN_X3 HAL_GPIO_ReadPin(IR_X3_Port, IR_X3_Pin)
#define IN_X4 HAL_GPIO_ReadPin(IR_X4_Port, IR_X4_Pin)


void irtracking_init(void);

#endif

