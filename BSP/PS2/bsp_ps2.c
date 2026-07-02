#include "bsp_ps2.h"

/*********************************************************     
**********************************************************/	 
//使用的不是硬件SPI  Not using hardware SPI
#define DELAY_TIME  delay_us(5);

u16 Handkey;	// 按键值读取，零时存储。Key value reading, zero time storage.
u8 Comd[2]={0x01,0x42};	//开始命令。请求数据 //Start command. Request data
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组 //Data storage array
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	
 /**************************************************************************
Function function: The following is the initialization code for the PS2 receiver module
Entrance parameters: None
Return value: None
Key values and key specifications
Controller interface initialization
input  DI->PB14 
output DO->PB15    CS->PB12  CLK->PB13
函数功能：以下是PS2接收器模块的初始化代码
入口参数：无
返回  值：无
按键值与按键明
手柄接口初始化    输入  DI->PB14 
                 输出  DO->PB15    CS->PB12  CLK->PB13
**************************************************************************/
void PS2_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		__HAL_RCC_GPIOB_CLK_ENABLE();

		GPIO_InitStruct.Pin = PS_PIN_DI;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(PS_PORT_DI, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = PS_PIN_DO;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;
		HAL_GPIO_Init(PS_PORT_DO, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = PS_PIN_CS;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;
		HAL_GPIO_Init(PS_PORT_CS, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = PS_PIN_CLK;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;
		HAL_GPIO_Init(PS_PORT_CLK, &GPIO_InitStruct);

	
	
}
/**************************************************************************
Function function: Send commands to the controller
Entry parameter: CMD command
Return value: None
函数功能：向手柄发送命令
入口参数：CMD指令
返回  值：无
**************************************************************************/
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //输出一位控制位 Output a control bit
		}
		else DO_L;
		CLK_H;                        //时钟拉高 Clock up
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI) //为高电平的时候 When it is at a high level
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}
/**************************************************************************
Function function: Determine whether it is in red light mode, 0x41=Simulate green light, 0x73=Simulate red light
Entry parameter: CMD command
Return value: 0, Red light mode Other, Other modes
函数功能：判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
入口参数：CMD指令
返回  值：0，红灯模式  其他，其他模式
**************************************************************************/
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令 Start command
	PS2_Cmd(Comd[1]);  //请求数据 Request data
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;
}
/**************************************************************************
Function function: Read controller data
Entrance parameters: None
Return value: None
函数功能：读取手柄数据
入口参数：无
返回  值：无
**************************************************************************/
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令 Start command
	PS2_Cmd(Comd[1]);  //请求数据 Request data
	for(byte=2;byte<9;byte++)          //开始接受数据 Start accepting data
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME; 
			CLK_H;			
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}
/**************************************************************************
Function function: Process the data read from PS2, only processing the key part
Entry parameter: CMD command
Return value: None
//When only one button is pressed, it is pressed as 0, and when not pressed, it is pressed as 1
函数功能：对读出来的PS2的数据进行处理,只处理按键部分
入口参数：CMD指令
返回  值：无  
//只有一个按键按下时按下为0， 未按下为1
**************************************************************************/
u8 PS2_DataKey()
{
	u8 index;
	PS2_ClearData();
	PS2_ReadData();
	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1 These are 16 buttons that are pressed to 0 and not pressed to 1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下 No buttons pressed
}
/**************************************************************************
Function function: Send commands to the controller
Entrance parameters: Obtain the analog range of a joystick from 0 to 256
Return value: None
函数功能：向手柄发送命令
入口参数：得到一个摇杆的模拟量	 范围0~256
返回  值：无
**************************************************************************/
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}
//清除数据缓冲区
//Clear data buffer
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function function: joystick vibration function,
Calls:		 void PS2_Cmd(u8 CMD);
Entrance parameters: motor1: Right small vibration motor 0x00 off, other on
Motor 2: Left side large vibration motor 0x40~0xFF motor on, the larger the value, the greater the vibration
Return value: None
函数功能: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
入口参数: motor1:右侧小震动电机 0x00关，其他开
	        motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
返回  值:无
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  //开始命令 Start command
	PS2_Cmd(0x42);  //请求数据 Request data
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
/**************************************************************************
Function function: short poll
Entrance parameters: None
Return value: None
函数功能：short poll
入口参数：无
返回  值：无
**************************************************************************/
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function Function: Enter Configuration
Entrance parameters: None
Return value: None
函数功能：进入配置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function Function: Send Mode Settings
Entrance parameters: None
Return value: None
函数功能：发送模式设置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式  //analog=0x01;digital=0x00 Software sets the sending mode
	PS2_Cmd(0xEE); //Ox03锁存设置，即不可通过按键“MODE”设置模式。 // 0x03 latches the setting, that is, the mode cannot be set by pressing the "MODE" button.
				   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。 //0xEE does not latch the software settings, and the mode can be set by pressing the "MODE" button.
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function Function: Vibration Settings
Entrance parameters: None
Return value: None
函数功能：振动设置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function function: Complete and save configuration
Entrance parameters: None
Return value: None
函数功能：完成并保存配置
入口参数：无
返回  值：无
**************************************************************************/
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function function: Controller configuration initialization
Entrance parameters: None
Return value: None
函数功能：手柄配置初始化
入口参数：无
返回  值：无
**************************************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式 Enter configuration mode
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存 Configure the "traffic light" mode and choose whether to save it or not
	//PS2_VibrationMode();	//开启震动模式 Activate vibration mode
	PS2_ExitConfing();		//完成并保存配置 Complete and save configuration
}



