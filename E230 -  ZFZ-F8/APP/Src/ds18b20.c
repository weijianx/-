#include "ds18b20.h"



void delay_us(uint32_t tim)
{
	tim = tim*1;
	while(tim--);	
}


void delay_2us(void)
{
	uint32_t tim = 7;
	while(tim--);	
}


/**
 * 名称         : delay_750us(void)
 * 创建日期     : 
 * 作者         : 
 * 功能         : 750微秒延时
 * 输入参数     : 无
 * 输出参数     : 无
 * 返回结果     : 无
 * 注意和说明   : 无
 * 修改内容     : 无 
 */
void delay_750us(void)
{
	uint32_t tim=4000;
	while(tim--);	
}

/**
 * 名称         : delay_15us(void)
 * 创建日期     : 
 * 作者         : 
 * 功能         : 15微秒延时
 * 输入参数     : 无
 * 输出参数     : 无
 * 返回结果     : 无
 * 注意和说明   : 无
 * 修改内容     : 无 
 */
void delay_15us(void)
{
	uint32_t tim=78;
	while(tim--);	
}

/**
 * 名称         : delay_12us(void)
 * 创建日期     : 
 * 作者         : 
 * 功能         : 12微秒延时
 * 输入参数     : 无
 * 输出参数     : 无
 * 返回结果     : 无
 * 注意和说明   : 无
 * 修改内容     : 无 
 */
void delay_12us(void)
{
	uint32_t tim=62;
	while(tim--);	
}


/**
 * 名称         : delay_50us(void)
 * 创建日期     : 
 * 作者         : 
 * 功能         : 50微秒延时
 * 输入参数     : 无
 * 输出参数     : 无
 * 返回结果     : 无
 * 注意和说明   : 无
 * 修改内容     : 无 
 */
void delay_50us(void)
{
	uint32_t tim=277;
	while(tim--);	
}

/**
 * 名称         : delay_60us(void)
 * 创建日期     : 
 * 作者         : 
 * 功能         : 60微秒延时
 * 输入参数     : 无
 * 输出参数     : 无
 * 返回结果     : 无
 * 注意和说明   : 无
 * 修改内容     : 无 
 */
void delay_60us(void)
{
	uint32_t tim=330;
	while(tim--);	
}


/**
* @brief        ds18b20引脚输出配置
* @retval       None
*/
void DS18B20_Output(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
		/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO_PIN_9);
	
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

}

/**
* @brief        ds18b20引脚输入配置
* @retval       None
*/
void DS18B20_Input(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
		/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT , GPIO_PUPD_PULLDOWN, GPIO_PIN_9);
	
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

}

/**
* @brief       初始化DS18B20的IO口 DQ 同时检测DS的存在 
* @retval      None
*/
void DS18B20_Init(void)
{
	DS18B20_Output();
	
	DS18B20_Rst();
	DS18B20_Check();
}

//复位DS18B20
void DS18B20_Rst(void)	   
{                 
	DS18B20_Output();   //设置为输出
	DS18B20_DQ_OUT_L;  	//拉低DQ

	delay_750us();
	
	DS18B20_DQ_OUT_H;  	//DQ=1
	delay_15us();

}

//等待DS18B20的回应
//返回1:未检测到DS18B20的存在
//返回0:存在
uint8_t DS18B20_Check(void) 	   
{   
	uint8_t retry=0;
	DS18B20_Input();    //设置为输入 
    while (DS18B20_DQ_IN&&retry<200)
	{
		retry++;
		delay_us(5);
//		delay_us(1);

	};	 
	if(retry>=200)return 1;
	else retry=0;
    while (!DS18B20_DQ_IN&&retry<240)
	{
		retry++;
		delay_us(5);
//		delay_us(1);
	};
	if(retry>=240)return 1;	    
	return 0;
}


/**
* @brief		从DS18B20读取一个位
* @para			NULL
* @retval		返回值：1/0      
* - None		
* @note			读取数据时要延时长一点，否则，读取高温时数据不准确
*/
uint8_t DS18B20_Read_Bit(void) 
{
	uint8_t data;
	DS18B20_Output();   //设置为输出
	DS18B20_DQ_OUT_L; 
//	delay_2us();
	delay_us(5);

	DS18B20_DQ_OUT_H; 
	DS18B20_Input();    //设置为输入
//	delay_12us();
	delay_us(15);

	if(DS18B20_DQ_IN)data=1;
	else data=0;
//	delay_50us();
	delay_us(54);
     
	return data;
}

//从DS18B20读取一个字节
//返回值：读到的数据
uint8_t DS18B20_Read_Byte(void)   
{        
	uint8_t i,j,dat;
	dat=0;
	for (i=1;i<=8;i++) 
	{
        j=DS18B20_Read_Bit();
        dat=(j<<7)|(dat>>1);
    }						    
	return dat;
}

//写一个字节到DS18B20
//dat：要写入的字节
void DS18B20_Write_Byte(uint8_t dat)     
 {             
    uint8_t j;
    uint8_t testb;
    DS18B20_Output();     //设置为输出
    for (j=1;j<=8;j++) 
	{
        testb=dat&0x01;
        dat=dat>>1;
        if(testb)       // 写1
        {
            DS18B20_DQ_OUT_L;

//			delay_2us();
			delay_us(5);
            DS18B20_DQ_OUT_H;

			delay_60us();            
        }
        else            //写0
        {
            DS18B20_DQ_OUT_L;

			delay_60us();
               
            DS18B20_DQ_OUT_H;
			delay_us(5);
//			delay_2us();
                          
        }
    }
}
 
/**
* @brief		开始温度转换
* @retval		None
*/
void DS18B20_Start(void)
{   						               
    DS18B20_Rst();	   
    DS18B20_Check();	 
    DS18B20_Write_Byte(0xcc);// skip rom
    DS18B20_Write_Byte(0x44);// convert
}

/**
* @brief		从ds18b20得到温度值
* @retval		浮点型：温度值 （-55~125） 
*/

uint16_t temp;
uint32_t f_tem;

int DS18B20_Get_Temp(void)
{
//    uint8_t temp;
    uint8_t TL,TH;
    int16_t tem;
	
	int temperature;
	
    DS18B20_Start ();           //开始转换
    DS18B20_Rst();
    DS18B20_Check();	 
    DS18B20_Write_Byte(0xcc);   // skip rom
    DS18B20_Write_Byte(0xbe);   // convert	    
    TL=DS18B20_Read_Byte();     // LSB   
    TH=DS18B20_Read_Byte();     // MSB   
    if(TH>7)
    {
        TH=~TH;
        TL=~TL; 
        temp=0;//温度为负  
    }else temp=1;//温度为正	  	  
    tem=TH; //获得高八位
    tem<<=8;    
    tem+=TL;//获得底八位
    temperature=(int)tem*6250;//转换     
	if(temp)return temperature; //返回温度值
	else return -temperature;    

//	tem = TH<<8;
//	tem = tem | TL;
//	if(tem < 0 )
//		f_tem = (~tem+1) * 0.625;
////		f_tem = (~tem+1) * 0.0625;
//	else
//		f_tem = tem * 0.625;
////		f_tem = tem * 0.0625;
//	
//	return f_tem;
}


