#include "bsp_ccd.h"

char buf_CCD[20] = {'\0'};
u16 ADV[128]={0};
u8 CCD_Zhongzhi,CCD_Yuzhi;
static ADC_HandleTypeDef hadc2;
/**************************************************************************
Function function: Linear CCD initialization
Entrance parameters: None
Return value: None
函数功能：线性CCD初始化
入口参数：无
返回  值：无
**************************************************************************/
/*
*PF5 -> CLK
*PF4 -> CS
*PF6 ->AO
*/

void  ccd_Init(void)
{    
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// CLK,SI配置为输出  CLK, SI configured as output
	GPIO_InitStruct.Pin = CCD_SI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CCD_SI_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CCD_CLK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CCD_CLK_PORT, &GPIO_InitStruct);



	GPIO_InitStruct.Pin = CCD_AO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(CCD_AO_PORT, &GPIO_InitStruct);

	//adc引脚初始化  adc pin initialization
	CCD_MODE_ADC();
}	



void CCD_MODE_ADC(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	__HAL_RCC_ADC2_CLK_ENABLE();

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

	sConfig.Channel = CCD_ADC_CH;
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
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
函数功能：AD采样
入口参数：ADC的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc_CCD(u8 ch)   
{
	u16 result;
	ADC_ChannelConfTypeDef sConfig;//通道初始化 Channel initialization
	sConfig.Channel=ch;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;		//采用周期239.5周期 Adopting a cycle of 239.5 cycles
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc2,&sConfig);

	HAL_ADC_Start(&hadc2);								//启动转换 Start the conversion
	HAL_ADC_PollForConversion(&hadc2,30);				//等待转化结束  Wait for the conversion to complete
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
	{
		result=HAL_ADC_GetValue(&hadc2);	//返回最近一次ADC1规则组的转换结果  Returns the most recent conversion result of the ADC1 rule group
	}
	return result;
}

/**************************************************************************
Function Function: Delay
Entrance parameters: None
Return value: None
函数功能：延时
入口参数：无
返回  值：无
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++); 
}

/**************************************************************************
Function function: CCD data acquisition
Entrance parameters: None
Return value: None
函数功能：CCD数据采集
入口参数：无
返回  值：无
**************************************************************************/
 void RD_TSL(void) 
{
  u8 i=0,tslp=0;
  TSL_CLK=1;
  TSL_SI=0; 
  Dly_us();
      
  TSL_SI=1; 
  TSL_CLK=0;
  Dly_us();
	
	
  TSL_CLK=1;
  TSL_SI=0;
  Dly_us(); 
  for(i=0;i<128;i++)					//Read 128 pixel voltage values 读取128个像素点电压值
  { 
    TSL_CLK=0; 
    Dly_us();  //Adjust exposure time 调节曝光时间
		Dly_us();
		
    ADV[tslp]=(Get_Adc_CCD(CCD_ADC_CH))>>4; 
    ++tslp;
    TSL_CLK=1;
    Dly_us();	
		


  }  
}

//开始CCD采集并处理输出数据 Start CCD collection and processing of output data
void deal_data_ccd(void)
{
//		RD_TSL();  //获取图像时已经获取过了 Already acquired when acquiring the image
		Find_CCD_Zhongzhi();	 
}

/**************************************************************************
Function function: Linear CCD takes the median value
Entrance parameters: None
Return value: None
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值 Dynamic threshold algorithm, reading maximum and minimum values
     for(i=5;i<123;i++)   //两边各去掉5个点 Remove 5 points on each side
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值 min
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值 Calculate the threshold for extracting the centerline in this round
	 for(i = 5;i<118; i++)   //寻找左边跳变沿 Find the left jump edge
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿 Find the right jump edge
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置 Calculate the centerline position
	
	
	
//	printf("zhong: %d\r\n",CCD_Yuzhi);
//	printf("middle : %d\r\n",CCD_Zhongzhi);
	
//	sprintf(buf_CCD,"Yuzhi:%d ",CCD_Yuzhi);
//	OLED_Draw_Line(buf_CCD, 2 , false, true);
//	sprintf(buf_CCD,"Zhongzhi:%d ",CCD_Zhongzhi);
//	OLED_Draw_Line(buf_CCD, 3 , false, true);
//	
//	memset(buf_CCD,0,sizeof(buf_CCD));
	
	//根据实际情况 According to the actual situation
//	if(math_abs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大 Calculate the deviation from the midline, if it is too large
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值 Then take the last value
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差 Save the last deviation
	
}


uint8_t ADC_128X32[128] = {0};
// 返回128个像素点的ADV采集电压值，并将幅值压缩成128*32。 Returns the ADV collected voltage value of 128 pixels and compresses the amplitude into 128*32.
uint8_t* CCD_Get_ADC_128X32(void)
{
		RD_TSL();
    // 将8位AD值转化成5位AD值 Convert 8-bit AD value to 5-bit AD value
    for (int i = 0; i < 128; i++)
    {
        ADC_128X32[i] = ADV[i] >> 3;
    }
    return ADC_128X32;
}

void OLED_Show_CCD_Image(uint8_t* p_img)
{
    OLED_Clear();
    for (int i = 0; i < 128; i++)
    {
        if (p_img[i] < 32)
        {
            SSD1306_DrawPixel(i, p_img[i], SSD1306_COLOR_WHITE);
        }
    }
    OLED_Refresh();
}
