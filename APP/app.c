#include "app.h"

extern char showbuf[20];


void app_user(void)
{
	// K210   OLED          
	// K210 line tracking: display recognized line coordinates on OLED
	sprintf(showbuf, "X=%d Y=%d  ", K210_data.k210_X, K210_data.k210_Y);
	OLED_Draw_Line(showbuf, 2, false, true);
}

