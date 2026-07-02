#include "app.h"

extern char showbuf[20];


void app_user(void)
{
	// K210巡线：OLED显示当前识别的线坐标
	// K210 line tracking: display recognized line coordinates on OLED
	sprintf(showbuf, "X=%d Y=%d  ", K210_data.k210_X, K210_data.k210_Y);
	OLED_Draw_Line(showbuf, 2, false, true);
}

