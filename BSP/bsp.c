#include "bsp.h"
#include "intsever.h"

void bsp_init(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); //                 

	delay_init();              // DWT  HAL   
	init_led_gpio();           //  LED
	init_beep();               //     

	Motor_start();             //        
	Encoder_Init_TIM3();       //      3(  )
	Encoder_Init_TIM4();       //      4(  )

	HAL_Delay(300);

	MPU6050_initialize();      //       
	DMP_Init();                // DMP   

	OLED_I2C_Init();           // OLED   
	Battery_init();            //       
}


// Chaseline       K210  
// Chaseline mode: only init K210 communication
void bsp_mode_init(void)
{
	USART2_init();             // K210   115200
	TIM6_Init();               // LED   +        
}

