#include "app_control.h"


static u16 intstop_time = 0;
float battery = 12;  //         12V // Initial state: fully charged 12V


//           10ms
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


// MPU6050       —         200Hz
// MPU6050 interrupt callback — main balance control loop, 200Hz
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int Encoder_Left, Encoder_Right;
	int Balance_Pwm, Velocity_Pwm, Turn_Pwm;

	if(GPIO_Pin == MPU6050_Int_Pin)
	{
		//        
		if(intstop_time > 0)
			intstop_time--;

		// ====== Chaseline: K210     ======
		Set_K210track_speed();  //        Move_X = 15

		// ======          ======
		Get_Angle(GET_Angle_Way);                         //      5ms  
		Encoder_Left  = Read_Encoder(MOTOR_ID_ML);        //      
		Encoder_Right = -Read_Encoder(MOTOR_ID_MR);       //      (  )
		Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);  //    →  

		// ======   PID ======
		Balance_Pwm  = Balance_PD(Angle_Balance, Gyro_Balance);     //    
		Velocity_Pwm = Velocity_PI(Encoder_Left, Encoder_Right);    //    
		Turn_Pwm     = Turn_K210_PD(Gyro_Turn);                     //    (K210  )

		// ======      ======
		Motor_Left  = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
		Motor_Right = Balance_Pwm + Velocity_Pwm - Turn_Pwm;

		// ======      + PWM   ======
		Motor_Left  = PWM_Ignore(Motor_Left);
		Motor_Right = PWM_Ignore(Motor_Right);
		Motor_Left  = PWM_Limit(Motor_Left,  2600, -2600);
		Motor_Right = PWM_Limit(Motor_Right, 2600, -2600);

		// ======   /    (    ) ======
		if(Pick_Up(Acceleration_Z, Angle_Balance, Encoder_Left, Encoder_Right))
			Stop_Flag = 1;
		if(Put_Down(Angle_Balance, Encoder_Left, Encoder_Right))
			Stop_Flag = 0;

		// ======      ======
		if(Turn_Off(Angle_Balance, battery) == 0)
			Set_Pwm(Motor_Left, Motor_Right);
	}
}


/**************************************************************************
Function: Get angle
Input   : way The algorithm of getting angle 1 DMP  2 kalman  3 Complementary filtering
Output  : none
         
     way         1 DMP  2     3     
       
**************************************************************************/
void Get_Angle(u8 way)
{
	float gyro_x, gyro_y, accel_x, accel_y, accel_z;
	float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
	Temperature = Read_Temperature();      //   MPU6050         
	if(way == 1)                           // DMP            
	{
		Read_DMP();                        //             
		Angle_Balance = Pitch;             //                
		Gyro_Balance  = gyro[0];           //        
		Gyro_Turn     = gyro[2];           //        
		Acceleration_Z = accel[2];         //   Z     
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
			Pitch = KF_X(accel_y, accel_z, -gyro_x) / PI * 180;  //      
			Roll  = KF_Y(accel_x, accel_z,  gyro_y) / PI * 180;
		}
		else if(GET_Angle_Way == 3)
		{
			Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / PI;
			Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / PI;
			Pitch = -Complementary_Filter_x(Accel_Angle_x, Gyro_X / 16.4);  //     
			Roll  = -Complementary_Filter_y(Accel_Angle_y, Gyro_Y / 16.4);
		}
		Angle_Balance = Pitch;
		Gyro_Turn     = Gyro_Z;
		Acceleration_Z = Accel_Z;
	}
}


/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration Z-axis acceleration Angle The angle of balance encoder_left Left encoder count encoder_right Right encoder count
Output  : 1 picked up  0 No action
              
     Acceleration z     Angle       encoder_left        encoder_right       
      1:       0       
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
Input   : The angle of balance Left encoder count Right encoder count
Output  : 1 put down  0 No action
              
                       
      1        0      
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

