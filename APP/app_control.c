#include "app_control.h"


static u16 intstop_time = 0;
float battery = 12;  // 初始状态处于满电12V // Initial state: fully charged 12V


// 外部中断做延迟，至少10ms
// External interrupt delay, at least 10ms
void delay_time_int(u16 time)
{
	intstop_time = time * 2;
}


void set_time_int(u16 time)
{
	intstop_time = time;
}


u16 get_time_int(void)
{
	return intstop_time;
}


// MPU6050中断回调函数 — 平衡控制主循环，200Hz
// MPU6050 interrupt callback — main balance control loop, 200Hz
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int Encoder_Left, Encoder_Right;
	int Balance_Pwm, Velocity_Pwm, Turn_Pwm;

	if(GPIO_Pin == MPU6050_Int_Pin)
	{
		// 中断延迟计数器
		if(intstop_time > 0)
			intstop_time--;

		// ====== Chaseline: K210视觉巡线 ======
		Set_K210track_speed();  // 设定前进速度 Move_X = 15

		// ====== 读取姿态和编码器 ======
		Get_Angle(GET_Angle_Way);                         // 更新姿态，5ms一次
		Encoder_Left  = Read_Encoder(MOTOR_ID_ML);        // 左轮编码器
		Encoder_Right = -Read_Encoder(MOTOR_ID_MR);       // 右轮编码器(取反)
		Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);  // 编码器→速度

		// ====== 三层PID ======
		Balance_Pwm  = Balance_PD(Angle_Balance, Gyro_Balance);     // 直立环
		Velocity_Pwm = Velocity_PI(Encoder_Left, Encoder_Right);    // 速度环
		Turn_Pwm     = Turn_K210_PD(Gyro_Turn);                     // 转向环(K210巡线)

		// ====== 电机合成 ======
		Motor_Left  = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
		Motor_Right = Balance_Pwm + Velocity_Pwm - Turn_Pwm;

		// ====== 死区过滤 + PWM限幅 ======
		Motor_Left  = PWM_Ignore(Motor_Left);
		Motor_Right = PWM_Ignore(Motor_Right);
		Motor_Left  = PWM_Limit(Motor_Left,  2600, -2600);
		Motor_Right = PWM_Limit(Motor_Right, 2600, -2600);

		// ====== 拿起/放下检测(安全保护) ======
		if(Pick_Up(Acceleration_Z, Angle_Balance, Encoder_Left, Encoder_Right))
			Stop_Flag = 1;
		if(Put_Down(Angle_Balance, Encoder_Left, Encoder_Right))
			Stop_Flag = 0;

		// ====== 电机输出 ======
		if(Turn_Off(Angle_Balance, battery) == 0)
			Set_Pwm(Motor_Left, Motor_Right);
	}
}


/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{
	float gyro_x, gyro_y, accel_x, accel_y, accel_z;
	float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
	Temperature = Read_Temperature();      // 读取MPU6050内置温度传感器数据
	if(way == 1)                           // DMP的读取在数据采集中断读取
	{
		Read_DMP();                        // 读取加速度、角速度、倾角
		Angle_Balance = Pitch;             // 更新平衡倾角，前倾为正后倾为负
		Gyro_Balance  = gyro[0];           // 更新平衡角速度
		Gyro_Turn     = gyro[2];           // 更新转向角速度
		Acceleration_Z = accel[2];         // 更新Z轴加速度计
	}
	else
	{
		Gyro_X  = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);
		Gyro_Y  = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);
		Gyro_Z  = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);
		Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L);
		Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L);
		Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L);
		if(Gyro_X  > 32768)  Gyro_X  -= 65536;
		if(Gyro_Y  > 32768)  Gyro_Y  -= 65536;
		if(Gyro_Z  > 32768)  Gyro_Z  -= 65536;
		if(Accel_X > 32768)  Accel_X -= 65536;
		if(Accel_Y > 32768)  Accel_Y -= 65536;
		if(Accel_Z > 32768)  Accel_Z -= 65536;
		Gyro_Balance = -Gyro_X;
		accel_x = Accel_X / 1671.84;
		accel_y = Accel_Y / 1671.84;
		accel_z = Accel_Z / 1671.84;
		gyro_x  = Gyro_X  / 939.8;
		gyro_y  = Gyro_Y  / 939.8;
		if(GET_Angle_Way == 2)
		{
			Pitch = KF_X(accel_y, accel_z, -gyro_x) / PI * 180;  // 卡尔曼滤波
			Roll  = KF_Y(accel_x, accel_z,  gyro_y) / PI * 180;
		}
		else if(GET_Angle_Way == 3)
		{
			Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / PI;
			Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / PI;
			Pitch = -Complementary_Filter_x(Accel_Angle_x, Gyro_X / 16.4);  // 互补滤波
			Roll  = -Complementary_Filter_y(Accel_Angle_y, Gyro_Y / 16.4);
		}
		Angle_Balance = Pitch;
		Gyro_Turn     = Gyro_Z;
		Acceleration_Z = Accel_Z;
	}
}


/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration：Z-axis acceleration；Angle：The angle of balance；encoder_left：Left encoder count；encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起
入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
返回  值：1:小车被拿起  0：小车未被拿起
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right)
{
	static u16 flag, count0, count1, count2;
	if(flag == 0)
	{
		if(myabs(encoder_left) + myabs(encoder_right) < 50)
			count0++;
		else
			count0 = 0;
		if(count0 > 10)
			flag = 1, count0 = 0;
	}
	if(flag == 1)
	{
		if(++count1 > 200)  count1 = 0, flag = 0;
		if(Acceleration > 22000 && (Angle > (-20 + Mid_Angle)) && (Angle < (20 + Mid_Angle)))
			flag = 2;
	}
	if(flag == 2)
	{
		if(++count2 > 100)  count2 = 0, flag = 0;
		if(myabs(encoder_left + encoder_right) > 50)
		{
			flag = 0;
			return 1;
		}
	}
	return 0;
}

/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读数
返回  值：1：小车放下   0：小车未放下
**************************************************************************/
int Put_Down(float Angle, int encoder_left, int encoder_right)
{
	static u16 flag;
	if(Stop_Flag == 0)
		return 0;
	if(flag == 0)
	{
		if(Angle > (-10 + Mid_Angle) && Angle < (10 + Mid_Angle) && encoder_left == 0 && encoder_right == 0)
			flag = 1;
	}
	if(flag == 1)
	{
		if((encoder_left > 3 && encoder_left < 40) || (encoder_right > 3 && encoder_right < 40))
		{
			flag = 0;
			return 1;
		}
	}
	return 0;
}

