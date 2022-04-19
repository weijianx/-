//#include "BSP.h"
#include "TH_IIC.h"


uint8_t x, y1, y2;


void Delay_1Us(uint32_t cnt)
{
//    cnt = cnt  ;      //如果是2,50度以上会跳为0度，如果为4会读不出温度
//    while (cnt--);      //16M的主频，延时要修改
	cnt = cnt * 1;

    while (cnt--);
}


void Delay_2Us(uint32_t cnt)
{
//    cnt = cnt  ;      //如果是2,50度以上会跳为0度，如果为4会读不出温度
//    while (cnt--);      //16M的主频，延时要修改
	cnt = cnt * 5;

    while (cnt--);
}



void Delay_4Us(uint32_t cnt)
{
//    cnt = cnt  ;      //如果是2,50度以上会跳为0度，如果为4会读不出温度
//    while (cnt--);      //16M的主频，延时要修改
	cnt = cnt * 16;

    while (cnt--);
}


void Delay_1Ms(uint32_t cnt)
{
    cnt = cnt * 940;

    while (cnt--);
   
  
}



/*******************************************************************************
// 名称         : Get_GPIO_PIN
// 创建日期     : 2017-07-29
// 作者         : 梅梦醒
// 功能         : 通过通道号获得对应的GPIO和PIN
// 输入参数     : 无
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 通道：0-PA12;1-PA5;2-PB3;3-PB4
// 修改内容     : 
********************************************************************************/
void Get_Channal_Pin(uint8_t sChannel)
{
    switch(sChannel)
    {     
    case 0:
        x = 0;
        y1 = 9;
        y2 = 10;
        break;
        
    case 1:
        x = 1;
        y1 = 3;
        y2 = 4;
        break; 
        
    case 2:
        x = 1;
        y1 = 5;
        y2 = 12;
        break;

    case 3:
        x = 1;
        y1 =13;
        y2 = 14;
        break ;
        
    default:
        break;
    }
}



//******************************************************************************
// 名称         : IIC_Init()
// 创建日期     : 2017-11-07
// 功能         : IIC GPIO配置
// 输入参数     : 无
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 
// 修改内容     :
//******************************************************************************
void TH_IIC_Init(void)
{
	
	rcu_periph_clock_enable(RCU_GPIOB);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
	
	gpio_bit_write(GPIOB, GPIO_PIN_6, RESET);
	Delay_1Us(10000);
	gpio_bit_write(GPIOB, GPIO_PIN_6, SET);
	
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
	
	gpio_bit_write(GPIOB, GPIO_PIN_7, RESET);
	Delay_1Us(10000);
	gpio_bit_write(GPIOB, GPIO_PIN_7, SET);
//    GPIOA->MODER &= 0xFFF3FFFF;            
//    GPIOA->MODER |= 0x00040000;           //PA9通用输出模式
//    GPIOA->OTYPER &= ~(1<<9);             //推挽输出
//    GPIOA->OSPEEDR |= (3<<18);            //高速  
//    GPIOA->PUPDR &= 0xFFF3FFFF;           //上拉
//    GPIOA->PUPDR |= 0x00040000;  
//    GPIOA->BRR = GPIO_PIN_9;            //拉低再拉高，出现故障时复位一下总线
//    Delay_1Us(10000);
//    GPIOA->BSRR = GPIO_PIN_9;
//    
//    GPIOA->MODER &= 0xFFCFFFFF;            
//    GPIOA->MODER |= 0x00100000;           //PA10通用输出模式
//    GPIOA->OTYPER &= ~(1<<10);             //推挽输出
//    GPIOA->OSPEEDR |= (3<<20);            //高速  
//    GPIOA->PUPDR &= 0xFFCFFFFF;           //上拉
//    GPIOA->PUPDR |= 0x00100000; 
//    GPIOA->BRR = GPIO_PIN_10;
//    Delay_1Us(10000);
//    GPIOA->BSRR = GPIO_PIN_10;

}

void SDA_IN()
{
	rcu_periph_clock_enable(RCU_GPIOB);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
}


void SDA_OUT()
{
	rcu_periph_clock_enable(RCU_GPIOB);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
	
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

}

/**@brief       产生IIC起始信号
* @param[in]    无
* @param[out]   无
* @return       无
* - None
* @note         
*/
void TH_IIC_Start(void)
{
    SDA_OUT();     //sda
    Set_IIC_SDA;
    Set_IIC_SCL;
//    GPIO_SetBits(GPIOC, GPIO_Pin_9);//del frank cl
    Delay_4Us(1);
//    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    Reset_IIC_SDA;//START:when CLK is high,DATA change form high to low
    Delay_4Us(1);
    Reset_IIC_SCL;
}

//void TH_IIC_Stop(void)
//{
//    SDA_I2C_OUT;
//    SCL_I2C_L;
//    SDA_I2C_L; 
//    Delay_1Us(4);
//    SCL_I2C_H;    
//    SDA_I2C_H;
//    Delay_1Us(4);
//}
/**@brief       产生IIC停止信号
* @param[in]    无
* @param[out]   无
* @return       无
* - None
* @note         
*/
void TH_IIC_Stop(void)
{
    SDA_OUT();//sda
    Reset_IIC_SCL;
    Reset_IIC_SDA;//STOP:when CLK is high DATA change form low to high
    Delay_4Us(1);
    Set_IIC_SCL;
//    Set_IIC_SDA;//I2C change frank cl
    Delay_4Us(1);
	Set_IIC_SDA;//I2C
}


/**@brief       等待应答信号到来
* @param[in]    无
* @param[out]   无
* @return       1，接收应答失败  0，接收应答成功
* - None
* @note         
*/
uint8_t TH_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN();      //SDA
    Set_IIC_SDA;
    Delay_1Us(1);
    Set_IIC_SCL;
    Delay_1Us(1);

    while (READ_SDA)
    {
        ucErrTime++;

        if (ucErrTime > 250)
        {
            TH_IIC_Stop();
            return 1;
        }
    }

    Reset_IIC_SCL;

    return 0;
}


/**@brief       产生ACK应答
* @param[in]    无
* @param[out]   无
* @return       无
* - None
* @note         
*/
void TH_IIC_Ack(void)
{
    Reset_IIC_SCL;
    SDA_OUT();
    Reset_IIC_SDA;
    Delay_2Us(1);
    Set_IIC_SCL;
    Delay_2Us(1);
    Reset_IIC_SCL;
}

/**@brief       不产生ACK应答
* @param[in]    无
* @param[out]   无
* @return       无
* - None
* @note         
*/
void TH_IIC_NAck(void)
{
    Reset_IIC_SCL;
    SDA_OUT();
    Set_IIC_SDA;
    Delay_2Us(1);
    Set_IIC_SCL;
    Delay_2Us(1);
    Reset_IIC_SCL;
}


/**@brief       IIC发送一个字节
* @param[in]    无
* @param[out]   无
* @return       无
* - None
* @note         
*/
void TH_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    Reset_IIC_SCL;

    for (t = 0;t < 8;t++)
    {
        Write_SDA((txd&0x80) >> 7);
        txd <<= 1;
        Delay_2Us(1);   
        Set_IIC_SCL;
        Delay_2Us(1);
        Reset_IIC_SCL;
        Delay_2Us(1);
    }
}


/**@brief       读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
* @param[in]    无
* @param[out]   无
* @return       无
* - None
* @note         
*/
uint8_t TH_IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN();//SDA?????

    for (i = 0;i < 8;i++)
    {
        Reset_IIC_SCL;
        Delay_2Us(1);
        Set_IIC_SCL;
        receive <<= 1;

        if (READ_SDA)receive++;

        Delay_1Us(1);
    }

    if (!ack)
        TH_IIC_NAck();
    else
        TH_IIC_Ack(); 

    return receive;
}
