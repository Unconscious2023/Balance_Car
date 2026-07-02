#include "app_line.h"

#define Track_K210Speed    10     // zhixingsudu
#define Track_K210Slow      2     // yujiansu
#define Track_K210Turn      2     // zhuanwansudu
#define K210_Trun_KP       (5)    // zhuanxiang P
#define K210_Trun_KD       (0.1)  // zhuanxiang D
#define K210_Minddle       160    // pingmu zhongxian
#define K210_DEAD_ZONE     8      // siqu


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
