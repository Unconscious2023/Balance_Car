#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = ChaseLine_Mode;  // 默认巡线模式 Default line tracking mode


// 模式选择：默认Chaseline，按KEY确认启动
// Mode selection: default Chaseline, press KEY to confirm
void Mode_select(void)
{
	OLED_Draw_Line("Chaseline Mode", 1, true, true);
	OLED_Draw_Line("Press KEY start!", 2, false, true);

	while(!Key1_State(1));  // 等待按键 // Wait for key press

	Set_Mid_Angle();   // 设置机械中值
	Set_angle();       // 设置跌倒倾角
	Set_PID();         // 设置PID参数
}


// 仅处理Chaseline模式
void car_mode(int16_t cnt)
{
	(void)cnt;  // 不再需要手拧轮子切换模式
}


// 设置机械中值
void Set_Mid_Angle(void)
{
	Mid_Angle = -1;  // K210巡线的机械中值
}


// 设置跌倒倾角
void Set_angle(void)
{
	angle_max = 30;  // K210模式用30度
}


// 引入PID参数
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;

void Set_PID(void)
{
	// K210巡线PID参数
	Balance_Kp  = 12000;
	Balance_Kd  = 72;
	Velocity_Kp = 8000;
	Velocity_Ki = 40;
	Turn_Kp     = 2500;
	Turn_Kd     = 20;
}

