#include "pid_control.h"





float Balance_K = 2.0; //2.0

float Velocity_K = 1.35;//1.35

float Turn_K = 1.0; //1.0







//   pid    ,   100  

//The PID below has been increased to 100 times for easy tuning



//   PD    

//Vertical loop PD control parameters

float Balance_Kp =9600;//  0-288  Range 0-288

float Balance_Kd =75; //  0-2  Range 0-2



//   PI    

//PI control parameters for speed loop

float Velocity_Kp=6000; //  0-72 6000  Range 0-72 6000

float Velocity_Ki=30;  //kp/200  30



//   PD    

//Steering ring PD control parameters

float Turn_Kp=1400; //                   ,        This can be adjusted according to one's own needs, but the balance can be left unadjusted, depending on the rotation speed

float Turn_Kd=30; //   0-2  Range 0-2



//    

//Forward speed

float Car_Target_Velocity=0; //0-10

//    

//Rotation speed

float Car_Turn_Amplitude_speed=0; //0-60



/**************************************************************************

Function: Absolute value function 

Input   : aNumber to be converted

Output  : unsigned int

         

    a         

          

**************************************************************************/	

int myabs(int a)

{ 		   

	int temp;

	if(a<0)  temp=-a;  

	else temp=a; 

	return temp;

}





/**************************************************************************

Function: Vertical PD control

Input   : Angle:angleGyroangular velocity

Output  : balanceVertical control PWM

      PD  		

    Angle:  Gyro   

     balance    PWM

**************************************************************************/	

int Balance_PD(float Angle,float Gyro)

{  

   float Angle_bias,Gyro_bias;

	 int balance;

	 Angle_bias=Mid_Angle-Angle;                       				//                Find the median angle and mechanical correlation for equilibrium

	 Gyro_bias=0-Gyro; 

	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //         PWM  PD     kp P   kd D    Calculate the motor PWM PD control for balance control kp is the P coefficient kd is the D coefficient

	

	if(mode == Weight_M)

		balance = balance*Balance_K;//   Load bearing

	

	 return balance;

}





/**************************************************************************

Function: Speed PI control

Input   : encoder_leftLeft wheel encoder readingencoder_rightRight wheel encoder reading

Output  : Speed control PWM

        PWM		

    encoder_left       encoder_right       

         PWM

**************************************************************************/

//           Target_Velocity    60

// To change the forward and backward speed, please modify Target_Velocity, for example, change it to 60

int Velocity_PI(int encoder_left,int encoder_right)

{  

    static float velocity,Encoder_Least,Encoder_bias,Movement;

	  static float Encoder_Integral;

	  //================         Remote control forward and backward part====================// 

									       	

		if(g_newcarstate==enRUN || g_newcarstate==enps2Fleft || g_newcarstate==enps2Fright)    	Movement=Car_Target_Velocity;	  //       Remote control forward signal

		else if(g_newcarstate==enBACK || g_newcarstate==enps2Bleft || g_newcarstate==enps2Bright)	Movement=-Car_Target_Velocity;  //       Remote control reverse signal

	

		else if(g_newcarstate==enAvoid)  Movement=-10; //                 Ultrasonic evasion signal operates at a fixed speed

		else if(g_newcarstate==enFollow)  Movement=10; //        Ultrasonic wave follows the signal

	   

		else  

				Movement=Move_X;

	

		

   //================  PI    Speed ??PI controller=====================//	

		Encoder_Least =0-(encoder_left+encoder_right);                    //        =        -            //Obtain the latest speed deviation=target speed (here zero) - measured speed (sum of left and right encoders) 

		Encoder_bias *= 0.84;		                                          //             //First order low-pass filter  

		Encoder_bias += Encoder_Least*0.16;	                              //              //First order low-pass filter to slow down speed changes

		Encoder_Integral +=Encoder_bias;                                  //          5ms //Integral offset time: 5ms

		Encoder_Integral=Encoder_Integral+Movement;                       //              //Receive remote control data and control forward and backward movement

		if(Encoder_Integral>8000)  	Encoder_Integral=8000;             //     //Integral limit

		if(Encoder_Integral<-8000)	  Encoder_Integral=-8000;            //    	 //Integral limit

		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //    	//Speed control

		

		if(Turn_Off(Angle_Balance,battery)==1) Encoder_Integral=0;//          //Clear points after motor shutdown



		if(mode == Weight_M)

			velocity = velocity*Velocity_K;//   Load bearing

		

	  return velocity;

} 







/**************************************************************************

Function: Turn control

Input   : Z-axis angular velocity

Output  : Turn control PWM

         

    Z    

         PWM

**************************************************************************/

float myTurn_Kd = 0;

int Turn_PD(float gyro)

{

	 static float Turn_Target,turn_PWM; 

	 float Kp=Turn_Kp,Kd;			//         Turn_Amplitude   To modify the steering speed, please modify Turn_Smplitude

	//===================         Remote control left and right rotation part=================//

	if(g_newcarstate==enLEFT || g_newcarstate==enps2Fleft || g_newcarstate==enps2Bleft)	        Turn_Target=-Car_Turn_Amplitude_speed;

	else if(g_newcarstate==enRIGHT || g_newcarstate==enps2Fright || g_newcarstate==enps2Bright)	  Turn_Target=Car_Turn_Amplitude_speed; 

	

	//          Left turn, right turn, fixed speed running

	else if(g_newcarstate == enTLEFT) Turn_Target=-50;

	else if(g_newcarstate == enTRIGHT) Turn_Target=50;

	else

	{

		Turn_Target=0; 

	}

	

	//         If it is remote control walking in a straight line

	if(g_newcarstate==enRUN || g_newcarstate==enBACK )

	{

		Kd=Turn_Kd; 

	} 

	else Kd=myTurn_Kd; 

	



  //===================  PD    Turn to PD controller=================//

	 turn_PWM=Turn_Target*Kp/100+gyro*Kd/100+Move_Z; //  Z      PD    Combining Z-axis gyroscope for PD control

	

	if(mode == Weight_M)

		turn_PWM = turn_PWM*Turn_K;//     Load bearing parameters

	

	 return turn_PWM;								 				 //   PWM         Steering ring PWM: Right turn is positive, left turn is negative

}









