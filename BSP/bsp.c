#include "bsp.h"
#include "intsever.h"

void bsp_init(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // 关闭外部中断，完成初始化后再使能

	delay_init();              // DWT作为HAL库时钟
	init_led_gpio();           // 关LED
	init_beep();               // 关蜂鸣器

	Motor_start();             // 启动电机定时器
	Encoder_Init_TIM3();       // 启动编码器3(左轮)
	Encoder_Init_TIM4();       // 启动编码器4(右轮)

	HAL_Delay(300);

	MPU6050_initialize();      // 陀螺仪初始化
	DMP_Init();                // DMP初始化

	OLED_I2C_Init();           // OLED初始化
	Battery_init();            // 电池电量检测
}


// Chaseline模式：只初始化K210通信
// Chaseline mode: only init K210 communication
void bsp_mode_init(void)
{
	USART2_init();             // K210串口 115200
	TIM6_Init();               // LED闪烁 + 电压检测定时器
}

