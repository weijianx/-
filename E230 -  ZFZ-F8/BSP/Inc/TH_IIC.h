#ifndef _TH_IIC_H
#define _TH_IIC_H

//#include "BSP.h"
#include "gd32e230.h"

////x:取值0~5，代表GPIOA~GPIOF;y:取值0~15，代表Pin0~Pin15
//#define G(x)			        ((GPIO_TypeDef *)(0x50000000 + 0x400 * x ))     //0~5代表A~F
//#define P(x)			        ((uint16_t)(1 << x))			        //Pin 0~15
//#define PORT_CLOCK(x)                   ((uint32_t)(0x00020000 << x))              
//#define SDA_IO_IN(x, y)                 {G(x)->MODER &= ~(0x03 << 2 * y);}
//#define SDA_IO_OUT(x, y)                {G(x)->MODER |= 0x01 << 2 * y; G(x)->OTYPER |= 0x00 << y; G(x)->OSPEEDR |= 0x11 << 2 * y;}
//#define	SDA_IN_READ(x, y)               (G(x)->IDR & P(y)) ? GPIO_PIN_SET  : GPIO_PIN_RESET   
//#define	SDA_IO_H(x, y)                  {G(x)->BSRR = P(y);}
//#define	SDA_IO_L(x, y)                  {G(x)->BRR = P(y);}
//#define SCL_IO_H(x, y)                  {G(x)->BSRR = P(y);}
//#define SCL_IO_L(x, y)                  {G(x)->BRR = P(y);}

//#define SCL_I2C_H                       SCL_IO_H(x, y1)
//#define SCL_I2C_L                       SCL_IO_L(x, y1)
//#define SDA_I2C_IN                      SDA_IO_IN(x, y2)
//#define SDA_I2C_OUT                     SDA_IO_OUT(x, y2)
//#define SDA_I2C_H                       SDA_IO_H(x, y2)
//#define SDA_I2C_L                       SDA_IO_L(x, y2)
//#define SDA_I2C_READ                    SDA_IN_READ(x, y2)


//iic配置为SCL为PB6/SDA为PB7
//#define SDA_IN()  gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_7)		//配置PB7为输入模式
//#define SDA_OUT() gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7)             //配置PB7为输出模式

#define Set_IIC_SCL			gpio_bit_set(GPIOB,GPIO_PIN_6)
#define Reset_IIC_SCL		gpio_bit_reset(GPIOB,GPIO_PIN_6)
#define Set_IIC_SDA			gpio_bit_set(GPIOB,GPIO_PIN_7)
#define Reset_IIC_SDA		gpio_bit_reset(GPIOB,GPIO_PIN_7)
#define READ_SDA			gpio_input_bit_get(GPIOB,GPIO_PIN_7)
#define Write_SDA(x)		gpio_bit_write(GPIOB, GPIO_PIN_7, x?SET:RESET)


void TH_IIC_Init(void);

void Get_Channal_Pin(uint8_t sChannel);

void Delay_1Us(uint32_t cnt);
void Delay_2Us(uint32_t cnt);
void Delay_4Us(uint32_t cnt);

void Delay_1Ms(uint32_t cnt);

void TH_IIC_Start(void);
void TH_IIC_Stop(void);

void TH_IIC_Ack(void);
void TH_IIC_NAck(void);
void TH_IIC_Send_Byte(uint8_t txd);
uint8_t TH_IIC_Wait_Ack(void);
uint8_t TH_IIC_Read_Byte(uint8_t ack);


#endif


