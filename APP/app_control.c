#include "app_control.h"


static u16 intstop_time =0 ;
float battery = 12;//初始状态处于满�?12v The initial state is fully charged 12v


//外部中断做延�?至少10ms的延�?此方法比delay准确
//External interrupt delay at least 10ms This method is more accurate than delay
void delay_time_int(u16 time)
{
	intstop_time = time*2; //�?5就是最终时�?//*5 is the final time
//	while(intstop_time);
}


void set_time_int(u16 time)
{
	intstop_time = time;
}

//返回时间 Return time
u16 get_time_int(void)
{
	return intstop_time;
}


//中断回调函数 Interrupt callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数 Pulse counting of left and right encoders
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平衡环PWM变量，速度环PWM变量，转向环PWM�?Balance loop PWM variable, speed loop PWM variable, steering loop PWM variable

  // 检查是否发生中断事�? Check if any interruption events have occurred
	if(GPIO_Pin == MPU6050_Int_Pin)
	{

			if(intstop_time>0)
			{
					intstop_time --;
			}

			// ChaseLine: K210发绝对目标, STM32直接执行
			if (mode == ChaseLine_Mode)
			{
				if (g_vision_input.confidence >= 15) {
					Vision_Set_Turn(g_vision_input.error);
				}
				Move_X = (float)(int8_t)g_vision_input.base_speed;
				g_newcarstate = enRUN;
				Car_Target_Velocity = Move_X;
			}
			Get_Angle(GET_Angle_Way);                     			//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果  //Updating the posture once every 5ms, a higher sampling frequency can improve the effectiveness of Kalman filtering and complementary filtering
			Encoder_Left=Read_Encoder(MOTOR_ID_ML);            					//读取左轮编码器的值，前进为正，后退为负   //Read the value of the left wheel encoder, forward is positive, backward is negative
			Encoder_Right=-Read_Encoder(MOTOR_ID_MR);           					//读取右轮编码器的值，前进为正，后退为负   //Read the value of the right wheel encoder, forward is positive, backward is negative
			Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right); //获取速度 Obtain speed

			Balance_Pwm=Balance_PD(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为�?  //Balance PID control gyro balance angular velocity polarity: forward tilt is positive, backward tilt is negative
			Velocity_Pwm=Velocity_PI(Encoder_Left,Encoder_Right);  //速度环PID控制	记住，速度反馈是正反馈   //Speed loop PID control. Remember, speed feedback is positive feedback



			//转向环PID控制
			Turn_Pwm = Turn_PD(Gyro_Turn);


			Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //计算左轮电机最终PWM Calculate the final PWM of the left wheel motor
			Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //计算右轮电机最终PWM Calculate the final PWM of the right wheel motor
																//PWM值正数使小车前进，负数使小车后退 Interrupt callback function

			//滤掉死区
			Motor_Left = PWM_Ignore(Motor_Left);
			Motor_Right = PWM_Ignore(Motor_Right);

			//PWM限幅 PWM limiting
			Motor_Left=PWM_Limit(Motor_Left,2600,-2600); //25khz->2592
			Motor_Right=PWM_Limit(Motor_Right,2600,-2600);

			// 差速闭�? BT 编码器差 Feedback
			if (mode == Bluetooth_Mode || mode == ChaseLine_Mode)
				Car_Diff_Turn(Gyro_Turn, Encoder_Left, Encoder_Right);

			//只有正常模式下检测小车的拿去和放�?姿态检�? Only in normal mode can the detection of the taking and lowering of the car be carried out (posture detection)
			if(mode == Bluetooth_Mode || mode == ChaseLine_Mode)
			{
				if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被拿起 Check if the car has been picked up
					Stop_Flag=1;	                           					//如果被拿起就关闭电机 If picked up, turn off the motor
				if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被放下 Check if the car has been lowered
					Stop_Flag=0;	                           					//如果被放下就启动电机 If it is put down, start the motor
			}

			if(Turn_Off(Angle_Balance,battery)==0)     					//如果不存在异�?		If there are no abnormalities
				Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存�?	Assign to PWM register
   }

}


/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角�?入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤�?返回  值：�?**************************************************************************/
void Get_Angle(u8 way)
{
	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度�?//Read the data from the MPU6050 built-in temperature sensor, which approximately represents the motherboard temperature.
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要�? //The reading of DMP is interrupted during data collection, strictly following the timing requirements
	{
		Read_DMP();                      	 //读取加速度、角速度、倾角  //Read acceleration, angular velocity, and tilt angle
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为�?//Update the balance tilt angle, with positive forward tilt and negative backward tilt
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为�? //Update the balance angular velocity, with positive forward tilt and negative backward tilt
		Gyro_Turn=gyro[2];                 //更新转向角速度 //Update steering angular velocity
		Acceleration_Z=accel[2];           //更新Z轴加速度�?//Update Z-axis accelerometer
	}
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪 //Read X-axis gyroscope
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪 //Read Y-axis gyroscope
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪 //Read Z-axis gyroscope
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度�?//Read X-axis accelerometer
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度�?//Read Y-axis accelerometer
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度�?//Read Z-axis accelerometer
		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换 Data type conversion can also be enforced through short type conversion
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换 Data type conversion can also be enforced through short type conversion
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换 Data type conversion
		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换 Data type conversion
		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换 Data type conversion
		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换 Data type conversion
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度 Update balance angular velocity
		accel_x=Accel_X/1671.84;
		accel_y=Accel_Y/1671.84;
		accel_z=Accel_Z/1671.84;
		gyro_x=Gyro_X/7510.0f;                            //Gyro→rad/s (±250°/s量程)
		gyro_y=Gyro_Y/7510.0f;                            //Gyro→rad/s (±250°/s量程)
		if(GET_Angle_Way==2)
		{
			 Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//卡尔曼滤�?Kalman filtering
			 Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;
		}
		else if(GET_Angle_Way==3)
		{
				Accel_Angle_x = atan2(Accel_Y,Accel_Z)*180/PI; //用Accel_Y和accel_y的参数得出的角度是一样的，只是边长不�?The angle obtained using Accel_Y and its parameters is the same, only the side length is different
				Accel_Angle_y = atan2(Accel_X,Accel_Z)*180/PI;

			 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X/16.4);//互补滤波 Complementary filtering
			 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y/16.4);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角    Update the balance tilt angle
		Gyro_Turn=Gyro_Z/131.0f;                          //更新转向角速度  Update steering angular velocity
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度�?Update Z-axis accelerometer
	}

}


/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration：Z-axis acceleration；Angle：The angle of balance；encoder_left：Left encoder count；encoder_right：Right encoder count
Output  : 1：picked up  0：No action
函数功能：检测小车是否被拿起
入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计�?返回  值：1:小车被拿�? 0：小车未被拿�?**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //第一�? Step 1
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<50)               //条件1，小车接近静�?Condition 1: The car is approaching a standstill
			count0++;
			else
			count0=0;
			if(count0>10)
			flag=1,count0=0;
	 }
	 if(flag==1)                                                      //进入第二�?Go to step 2
	 {
			if(++count1>200)       count1=0,flag=0;                       //超时不再等待2000ms，返回第一�?No more waiting for 2000ms after timeout, return to the first step
			if(Acceleration>22000&&(Angle>(-20+Mid_Angle))&&(Angle<(20+Mid_Angle)))   //条件2，小车是�?度附近被拿起 Condition 2, the car is picked up near 0 degrees
			flag=2;
	 }
	 if(flag==2)                                                       //第三�?Step 3
	 {
		  if(++count2>100)       count2=0,flag=0;                        //超时不再等待1000ms Timeout no longer waits 1000ms
	    if(myabs(encoder_left+encoder_right)>50)                       //条件3，小车的轮胎因为正反馈达到最大的转�?   Condition 3: The tires of the car reach their maximum speed due to positive feedback
      {
				flag=0;
				return 1;                                                    //检测到小车被拿�?Detected the car being picked up
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读�?返回  值：1：小车放�?  0：小车未放下
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag;//,count;
	 if(Stop_Flag==0)                     //防止误检    Prevent false positives
			return 0;
	 if(flag==0)
	 {
			if(Angle>(-10+Mid_Angle)&&Angle<(10+Mid_Angle)&&encoder_left==0&&encoder_right==0) //条件1，小车是�?度附近的 Condition 1, the car is around 0 degrees
			flag=1;
	 }
	 if(flag==1)
	 {
//		  if(++count>50)                     //超时不再等待 500ms  Timeout no longer waits 500ms
//		  {
//				count=0;flag=0;
//		  }
		 //增加灵敏�?Increase sensitivity
	    if((encoder_left>3&&encoder_left<40)||(encoder_right>3&&encoder_right<40)) //条件2，小车的轮胎在未上电的时候被人为转动  Condition 2: The tires of the car are manually rotated when not powered on
      {
				flag=0;
				return 1;                         //检测到小车被放�?Detected that the car has been lowered
			}
	 }
	return 0;
}
