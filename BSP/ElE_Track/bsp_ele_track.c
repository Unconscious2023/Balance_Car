#include "bsp_ele_track.h"

int Sensor_Left_1,Sensor_Left_2,Sensor_Left_3;
int Sensor_Right_1,Sensor_Right_2,Sensor_Right_3;
int Sensor_Middle;
int ele_seat = 0;
static ADC_HandleTypeDef hadc2;

/**************************************************************************
Function: Electromagnetic sensor sampling initialization
Input parameters: None
Return value: None
Author:
函数功能：电磁传感器采样初始化
入口参数：无
返回  值：无
作    者：
**************************************************************************/

//PC0-3  pC4-5 pB0-1 做电磁传感器采集  pC4-5 pB0-1这几个接收右边,和红外巡线传感器引脚重定义
//PC0-3 pC4-5 pB0-1 for electromagnetic sensor collection pC4-5 pB0-1 these receive the right side and infrared line patrol sensor pin redefine
void  ele_Init(void)
{    
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_ADC2_CLK_ENABLE();

	//左边 left ADC 10,11,12
	GPIO_InitStruct.Pin = ELE_L1_Pin|ELE_L2_Pin|ELE_L3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	//中间 middle ADC 13
	GPIO_InitStruct.Pin = ELE_MID_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//右边 right ADC 14,15
	GPIO_InitStruct.Pin = ELE_R1_Pin |ELE_R2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//右边 ADC 8
	GPIO_InitStruct.Pin = ELE_R3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
	sConfig.Channel = ELE_L1_CH; //随便写一个管脚就行，用到也会初始化对应的管脚  Just write a pin and the corresponding pin will be initialized when used.
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;



	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


	//校准ADC2 Calibrating ADC2
	HAL_ADCEx_Calibration_Start(&hadc2);
}		

/**************************************************************************
Function: AD sampling
Input parameter: ADC1 channel
Return value: AD conversion result
函数功能：AD采样
入口参数：ADC1 的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc_ele(u8 ch)   
{
	u16 result;
	ADC_ChannelConfTypeDef sConfig;//通道初始化 Channel initialization
	sConfig.Channel=ch;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;		//采用周期239.5周期 Adopting cycle 239.5 cycles
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc2,&sConfig);
 
	HAL_ADC_Start(&hadc2);								//启动转换 Start the conversion
	HAL_ADC_PollForConversion(&hadc2,30);				//等待转化结束 Wait for the conversion to complete
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
	{
		result=HAL_ADC_GetValue(&hadc2);	//返回最近一次ADC1规则组的转换结果 Returns the most recent conversion result of the ADC1 rule group
	}
	return result;
}

//得到的数据做归一算法 The obtained data is normalized by the algorithm
int guiyi_way(void)
{
	int sum , Sensor;
	int Sensor_Left,Sensor_Right;
	
	//归一化处理 Normalization
//	sum=(Sensor_Left_1*1+Sensor_Left_3*100) 
//			+ Sensor_Middle *200 
//			+(Sensor_Right_1*300+Sensor_Right_3*399);  
//	Sensor_Left =  Sensor_Left_1+Sensor_Left_3;
//	Sensor_Right = Sensor_Right_1+Sensor_Right_3;
	
	sum=(Sensor_Left_3*1) 
			+ Sensor_Middle *100 
			+(Sensor_Right_1*199);  
	
		Sensor_Left =  Sensor_Left_3; //+ Sensor_Left_1;
  	Sensor_Right = Sensor_Right_1;// + Sensor_Right_3;
	 
	Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //求偏差 Find Deviation
	return Sensor;//返回目前的在磁场的位置 Returns the current position in the magnetic field
}

//获取传感器数据 Get sensor data
void getEleData(void)
{
	//小车屁股对着自己，从左到右数 The rear of the car is facing you, count from left to right
	
	
	Sensor_Left_1=Get_Adc_ele(ELE_L1_CH)>>4;                //采集左边电感的数据 Collect data of the inductor on the left
	Sensor_Left_3=Get_Adc_ele(ELE_L3_CH)>>4;                //采集左边电感的数据 Collect data of the inductor on the left
	
	Sensor_Middle=Get_Adc_ele(ELE_M1_CH)>>4;              //采集中间电感的数据 Collect data of the middle inductor
	
	Sensor_Right_1=Get_Adc_ele(ELE_R1_CH)>>4;               //采集右边电感的数据 Collect data on the inductor on the right
	Sensor_Right_3=Get_Adc_ele(ELE_R3_CH)>>4;               //采集右边电感的数据 Collect data on the inductor on the right

	//因为放大器不稳，滤波一下，正常是不会，巡线只用此3个 Because the amplifier is unstable, filter it. Normally it won't work. Only these 3 are used for line inspection.
	Sensor_Left_3 = deal_getdata(Sensor_Left_3);
	Sensor_Right_1 = deal_getdata(Sensor_Right_1);
	Sensor_Middle = deal_getdata(Sensor_Middle);
	
	ele_seat = guiyi_way();
	
}


int deal_getdata(int a)
{
	if(a<=10)
	{
		return 0;
	}
	else
		return a;
}

//数据显示在屏幕上 Data displayed on screen
void EleDataDeal(void)
{
	
	char ele_data[30]={'\0'};
	//getEleData();中断调用了 The interrupt call

//	sprintf(ele_data,"ele_seat:%d     ",ele_seat);
//	OLED_Draw_Line(ele_data, 1 , false, true);
	//	sprintf(ele_data,"MID:%d        ",Sensor_Middle);
//	OLED_Draw_Line(ele_data, 2 , false, true);
	
	sprintf(ele_data,"seat:%d  MID:%d   ",ele_seat,Sensor_Middle);
	OLED_Draw_Line(ele_data, 1 , false, true);
		
	sprintf(ele_data,"L1:%d  L3:%d     ",Sensor_Left_1,Sensor_Left_3);
	OLED_Draw_Line(ele_data, 2 , false, true);
		
	sprintf(ele_data,"R1:%d  R3:%d     ",Sensor_Right_1,Sensor_Right_3);
	OLED_Draw_Line(ele_data, 3 , false, true);
	

}



