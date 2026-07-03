#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = ChaseLine_Mode;  //        Default line tracking mode


//        Chaseline  KEY    
// Mode selection: default Chaseline, press KEY to confirm
void Mode_select(void)
{
	OLED_Draw_Line("Chaseline Mode", 1, true, true);
	OLED_Draw_Line("Press KEY start!", 2, false, true);

	while(!Key1_State(1));  //      // Wait for key press

	Set_Mid_Angle();   //       
	Set_angle();       //       
	Set_PID();         //   PID  
}


//    Chaseline  
void car_mode(int16_t cnt)
{
	(void)cnt;  //             
}


//       
void Set_Mid_Angle(void)
{
	Mid_Angle = -1;  // K210       
}


//       
void Set_angle(void)
{
	angle_max = 30;  // K210   30 
}


//   PID  
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;

void Set_PID(void)
{
	// K210  PID  
	Balance_Kp  = 12000;
	Balance_Kd  = 72;
	Velocity_Kp = 8000;
	Velocity_Ki = 40;
	Turn_Kp     = 2500;
	Turn_Kd     = 20;
}

