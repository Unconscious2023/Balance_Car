#include "app_line.h"

#define Track_K210Speed    12     // Forward speed
#define Track_K210Slow     (4.5)     // Slow-down speed (curve predicted)
#define Track_K210Turn     (4.5)     // Turn speed
#define K210_Trun_KP       (5.2)    // Turn P gain
#define K210_Trun_KD       (0.15)  // Turn D gain
#define K210_Minddle       160    // Screen center (320/2)
#define K210_DEAD_ZONE     8      // Dead zone (pixels)


void Set_K210track_speed(void)
{
	// K210 Y zuobiao bianma sudu moshi:
	//   Y=0   -> zhuanwan, sudu 2
	//   Y>=200 -> yujian dao wandao, jiansu dao 2
	//   qita  -> zhengchang sudu 10
	uint16_t y = K210_data.k210_Y;

	if(y == 0)
		Move_X = Track_K210Turn;
	else if(y >= 200)
		Move_X = Track_K210Slow;
	else
		Move_X = Track_K210Speed;
}


int Turn_K210_PD(float gyro)
{
	float err = K210_data.k210_X - K210_Minddle;

	if(err > -K210_DEAD_ZONE && err < K210_DEAD_ZONE)
		return 0;

	return (int)(err * K210_Trun_KP + gyro * K210_Trun_KD);
}
