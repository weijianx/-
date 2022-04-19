#ifndef __DS18B20_H
#define __DS18B20_H

//#include "rtthread.h"
#include "gd32e230.h"


typedef struct {
	float	Ds18b20_temp;
		
}ds18b20_TypeDef;

void delay_us(uint32_t tim);
void delay_2us(void);
void delay_750us(void);
void delay_15us(void);
void delay_12us(void);
void delay_50us(void);
void delay_60us(void);

//IO方向设置
//#define DS18B20_IO_IN()  {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=8<<0;}
//#define DS18B20_IO_OUT() {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=3<<0;}
//#define DS18B20_DQ_OUT_H  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET) //数据端口	PA0
//#define DS18B20_DQ_OUT_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)  //数据端口	PA0
#define DS18B20_DQ_OUT_H  gpio_bit_write(GPIOA, GPIO_PIN_9, SET) //数据端口	PA0
#define DS18B20_DQ_OUT_L gpio_bit_write(GPIOA, GPIO_PIN_9, RESET)  //数据端口	PA0
//IO操作函数											   
//#define	DS18B20_DQ_OUT PAout(0) //数据端口	PA0
//#define	DS18B20_DQ_IN  PAin(0)  //数据端口	PA0
//#define	DS18B20_DQ_IN  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)  //数据端口	PA0
#define	DS18B20_DQ_IN  gpio_input_bit_get(GPIOA, GPIO_PIN_9)  //数据端口	PA0

void DS18B20_Output(void);
void DS18B20_Input(void);
void DS18B20_Init(void);			//初始化DS18B20
int DS18B20_Get_Temp(void);	//获取温度
void DS18B20_Start(void);		//开始温度转换
void DS18B20_Write_Byte(uint8_t dat);//写入一个字节
uint8_t DS18B20_Read_Byte(void);		//读出一个字节
uint8_t DS18B20_Read_Bit(void);		//读出一个位
uint8_t DS18B20_Check(void);			//检测是否存在DS18B20
void DS18B20_Rst(void);			//复位DS18B20 


#endif


