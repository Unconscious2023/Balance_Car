#ifndef _BSP_CCD_H
#define _BSP_CCD_H

#include "AllHeader.h"

#define TSL_SI    PBout(5)   //SI  
#define TSL_CLK   PBout(4)   //CLK 


#define CCD_SI_PIN   	GPIO_PIN_5
#define CCD_SI_PORT		GPIOB


#define CCD_CLK_PIN     GPIO_PIN_4
#define CCD_CLK_PORT	GPIOB


#define CCD_AO_PIN   	GPIO_PIN_4
#define CCD_AO_PORT		GPIOA

#define CCD_ADC_CH 		ADC_CHANNEL_4


u16 Get_Adc_CCD(u8 ch);
void Dly_us(void);
void RD_TSL(void); 
void ccd_Init(void);
void deal_data_ccd(void);

void  Find_CCD_Zhongzhi(void);
uint8_t* CCD_Get_ADC_128X32(void);
void OLED_Show_CCD_Image(uint8_t* p_img);

char binToHex_low(u8 num);
char binToHex_high(u8 num);
void slove_data(void);
void sendToPc(void);

void CCD_MODE_ADC(void);


uint8_t* CCD_Get_ADC_128X64(void);


#endif


