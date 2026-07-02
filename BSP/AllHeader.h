/**
* @par Copyright (C): 2016-2026, Shenzhen Yahboom Tech
* @file         // ALLHeader.h
* @author       // lly
* @version      // V1.0
* @date         // 240628
* @brief        // 相关所有的头文件 All related header files
*/


#ifndef __ALLHEADER_H
#define __ALLHEADER_H


//C语言头文件
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

//HAL库STM32头文件
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif


//从这往下就是自定义的
#include "myenum.h"
#include "app_mode.h"

#include "bsp.h"
#include "delay.h"
#include "bsp_battery.h"
#include "bsp_beep.h"
#include "bsp_LED.h"
#include "bsp_timer.h"
#include "bsp_key.h"
#include "app.h"

//Usart
#include "bsp_usart.h"

//蓝牙
#include "bsp_bluetooth.h"
#include "app_bluetooth.h"

//K210
#include "bsp_usart2.h"
#include "app_k210.h"
#include "app_k210_ai.h"
#include "app_line.h"
#include "app_follow.h"

//PS2手柄控制
#include "bsp_ps2.h"
#include "app_ps2.h"

//雷达
#include "bsp_lidar.h"
#include "app_lidar.h"
#include "app_lidar_car.h"

//OLED
#include "bsp_oled.h"
#include "bsp_oled_i2c.h"
#include "oled_show.h"

//Mpu6050
#include "IOI2C.h"
#include "MPU6050.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

//Motor
#include "motor.h"
#include "encoder.h"
#include "app_motor.h"

//超声波
#include "bsp_ultrasonic.h"

//4路循迹
#include "bsp_irtracking.h"
#include "app_tracking.h"

//电磁循迹
#include "bsp_ele_track.h"
#include "app_ele_tracking.h"

//CCD
#include "bsp_ccd.h"
#include "app_ccd_tracking.h"

//平衡车整体控制
#include "app_control.h"
#include "pid_control.h"

//滤波算法
#include "filter.h"
#include "KF.h"


//CCD显示画面指针
extern u8 CCD_Zhongzhi, CCD_Yuzhi;
extern u8* CCDShowBuf;


//引出的通用变量
extern float Velocity_Left, Velocity_Right;
extern uint8_t GET_Angle_Way;
extern float Angle_Balance, Gyro_Balance, Gyro_Turn;
extern int Motor_Left, Motor_Right;
extern int Temperature;
extern float Acceleration_Z;
extern int Mid_Angle;
extern float Move_X, Move_Z;
extern float battery;
extern u8 lower_power_flag;
extern u32 g_distance;
extern u8 Flag_velocity;
extern enCarState g_newcarstate;
extern u8 Stop_Flag;
extern float Car_Target_Velocity, Car_Turn_Amplitude_speed;
extern int Mid_Angle;
extern Car_Mode mode;


#endif

