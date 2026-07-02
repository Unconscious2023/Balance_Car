#include "bsp_irtracking.h"

void irtracking_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	
	GPIO_InitStruct.Pin = IR_X1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_X1_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = IR_X2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_X2_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = IR_X3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_X3_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = IR_X4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_X4_Port, &GPIO_InitStruct);

}



