#ifndef __APP_TRACKING_H_
#define __APP_TRACKING_H_


#include "AllHeader.h"

void Car_tracking(void);

//PID
void Set_IRtrack_speed(void);
void PID_track_get(void);
int Turn_IRTrack_PD(float gyro);

#endif


