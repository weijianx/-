/*!
    \file  main.c
    \brief USART HyperTerminal Interrupt
    
    \version 2018-06-19, V1.0.0, firmware for GD32E230
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e230.h"
#include "gd32e230c_eval.h"
#include "systick.h"
#include <stdio.h>

#include "timer.h"
#include "type.h"
#include "para.h"

#include "algorithm.h"



extern BitAction UartRecvFrameOK;

extern UserTypeDef UserPara;
extern FlagStatus UartRecvFlag;
FlagStatus StaChangeFlag = RESET;

extern uint16_t Current_PositiveTime ;
extern uint16_t Current_NegativeTime ; 
extern uint16_t tim_1min_cnt; 
extern uint16_t tim_cnt;
extern BitAction  PulseFlag;


uint8_t transmitter_buffer[] = "\n\rUSART interrupt test\n\r";


#define LED_PORT        GPIOA
#define LED_GREEN_PIN   GPIO_PIN_6      // 红灯 PA6
#define LED_RED_PIN     GPIO_PIN_5      //绿灯PA5


void Led_Control(uint8_t color)  // 1 绿色  0红色
{       
  if(color)  //1 绿色
  {
    gpio_bit_write(LED_PORT,LED_RED_PIN,RESET);
    gpio_bit_write(LED_PORT,LED_GREEN_PIN,SET);
    
//    LED_PORT->BRR = LED_RED_PIN; 
//    LED_PORT->BSRR = LED_GREEN_PIN;   
  }
  else      // 0  红色
  {     
     gpio_bit_write(LED_PORT,LED_GREEN_PIN,RESET);
     gpio_bit_write(LED_PORT,LED_RED_PIN,SET);
//      LED_PORT->BRR = LED_GREEN_PIN; 
//      LED_PORT->BSRR = LED_RED_PIN;  
  }
}

//GPIO配置
void GPIO_Configuration(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
   
   	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_9);
	
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);
   
	   	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO_PIN_5 | GPIO_PIN_6);
	
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_5 | GPIO_PIN_6);

      
}

void DRV_IWDG_Init(void)
{
	rcu_osci_on(RCU_IRC40K);
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC40K));
	
	/*	FWDG clock is independent lsi 40KHz, DIV128 IS 312.5Hz, that is 3.2 ms per clock
	reload value is 1000, delay time 2500 * 3.2ms = 8 s	*/
	fwdgt_config( 2500, FWDGT_PSC_DIV256 );
	fwdgt_enable();
	
}



void rs485_Init(void)
{
	/* enable the GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
	/* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
	
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_4);
	
	gpio_bit_reset(GPIOA,GPIO_PIN_4);

}

/*!
    \brief      initialize the key GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ZF_gpio_init(void)
{
    /* enable the TAMPER key gpio clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* configure button pin as input */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
}

/*!
    \brief      initialize the EXTI configuration of the key
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ZF_exti_init(void)
{
    /* enable the CFGCMP clock */
    rcu_periph_clock_enable(RCU_CFGCMP);
    /* enable and set key EXTI interrupt to the specified priority */
    nvic_irq_enable(EXTI0_1_IRQn, 0U);

    /* connect key EXTI line to key GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN1);

    /* configure key EXTI line */
    exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_1);
}

uint16_t cnt_1 = 0;
void EXTI0_1_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_1)) {
		cnt_1++;
//        Led_Control(0);
        exti_interrupt_flag_clear(EXTI_1);
    }

}

void delayms()
{
	uint16_t i=0xFF,j=0xFF;
	for(;i>0;i--)
		for(;j>0;j--);
}


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t read_cnt = 0;
uint16_t read_level;
uint16_t pulse_level = 0;
uint8_t Time_1min_flag = 0;
extern  uint8_t signs,initial;

int main(void)
{
	//中断向量设置，APP升级
	nvic_vector_table_set(FLASH_BASE, 0x4000);  
	  
	uint8_t i, j;
	uint8_t w_cnt = 0;
	uint8_t uTemp[4];
	uint8_t total_temp[16] = {0};
	uint8_t ch[] = "dggsgawegf\r\n";
	uint8_t Z_F_Zhuan;
	
	
	nvic_irq_enable(USART1_IRQn, 3);
     
    /* initilize the com */
    com_gpio_init();
    com_usart_init();
    
	GPIO_Configuration();
	
	ZF_gpio_init();
	ZF_exti_init();
	
    /* enable USART TBE interrupt */  
//    usart_interrupt_enable(USART1, USART_INT_TBE);
    
	usart_interrupt_enable(USART1, USART_INT_RBNE);	
	
	
	DRV_IWDG_Init();
	
//	timer_config();
	TIMER2_Init();
	TIMER15_Init();
		
	ReadPara(); 

	Led_Control(1);
	
    while (1)
	{
		fwdgt_counter_reload();


		MBASC_Function();
//		UARTx_SendData(ch, sizeof(ch));
//		Delay_Ms(1000);
		
		if(Time_5s_flag)  //10s 时间到 更新 数据  
        {
         
          
			Time_5s_flag = 0;
//         Led_Control(0);
			UserPara.RotateSpeed = pulse_level * 12;
//         if(pulse_level == 0)
//         {
//            i++;
//            
//         }
//         else
//         {
//            i = 0;
//            read_level = pulse_level;
//         }
//         

//         if(i < 2)
//         {
////            if( i >= 1 )
////             {
////             
////                signs = 0;
////                initial = 0;
////             }
//            
//            UserPara.RotateSpeed = read_level * 12;
//            
//         }
         
//         else
//         {
//            i = 0;
//           read_level = 0;
//           UserPara.RotateSpeed = 0;
//         }
           
			w_cnt++;
			if(w_cnt >= 72)
			{
//				w_cnt = 0;
				usart_disable(USART1);

				long32Array(UserPara.PositiveTimeBase, uTemp);                         // 更新  正转时间   单位：分钟
//				Flash_Write_MultiBytes(POSITIVE_ROTATE_TIME_BASE, uTemp, 4);  
				for(j=0; j<4; j++)
				{
					total_temp[j] = uTemp[j];
				}
				long32Array(UserPara.NegativeTimeBase, uTemp);                         // 更新  反转时间   单位：分钟
	//			Flash_Write_MultiBytes(NEGATIVE_ROTATE_TIME_BASE, uTemp, 4);  
				for(j=0; j<4; j++)
				{
					total_temp[j+4] = uTemp[j];
				}
				long32Array(UserPara.TotalPulse, uTemp);                               // 更新  总脉冲数     单位：HZ   
	//			Flash_Write_MultiBytes(PULSE_TOTAL_BASE, uTemp, 4);
				for(j=0; j<4; j++)
				{
					total_temp[j+8] = uTemp[j];
				}
			}

         
			 if(UserPara.DirSta != Stall )//正转或者反转时      20200410 增加
			 {
				UserPara.WorkTime = UserPara.PositiveTimeBase + UserPara.NegativeTimeBase + (UserPara.Duration + 30)/60;    //计算总时间 单位分钟
			 }
			 else                  //停转时
			 {
				UserPara.WorkTime = UserPara.PositiveTimeBase + UserPara.NegativeTimeBase ;    //计算总时间 单位分钟
			 
			 }
			 
			 
			 UserPara.WorkTime = UserPara.WorkTime/6;    // 更新  累计运行时间   单位转换  分钟--> 0.1h
			 long32Array(UserPara.WorkTime, uTemp);
			 if(w_cnt >= 72)
			 {
				w_cnt = 0;
	//			Flash_Write_MultiBytes(WORK_TIME_BASE, uTemp, 4);  
				for(j=0; j<4; j++)
				{
					total_temp[j+12] = uTemp[j];
				}
				Flash_Write_MultiBytes(POSITIVE_ROTATE_TIME_BASE, total_temp, 16); 
				usart_enable(USART1);
			 }
			 
			 if(UserPara.DirSta==1)// 正转
			 {
				 UserPara.Duration = UserPara.PositiveTimeBase - Current_PositiveTime; //  旋转方向持续时间    
			 }
			 else if(UserPara.RotateSta==2)// 反转
			 { 
				UserPara.Duration = UserPara.NegativeTimeBase - Current_NegativeTime;//  旋转方向持续时间   
			 }
			 else
			 {
				UserPara.Duration = (tim_1min_cnt + 30) / 60;//  停转     不计停转时间
			 }
			 long32Array(UserPara.Duration, uTemp);       // 更新  当前状态运行时间            单位 100ms = 0.1s
	//         Flash_Write_MultiBytes(DURATION_BASE, uTemp, 4); 
		}
        
		if(Time_1s_flag)    //1s 判断一次 更新当前状态
		{     
			Time_1s_flag = 0;
//          Led_Control(1);
			read_cnt = cnt_1;
			cnt_1 = 0;
			pulse_level = Get_filter(read_cnt);

			UserPara.TotalPulse += read_cnt;  //计总脉冲        
				
			if(read_cnt)
			{            
				PulseFlag = Bit_SET;  //有脉冲   转动中
				Time_1min_flag = 0;
			}
				
			Time_1min_flag++;
			  
			if((Time_1min_flag > 59) && (read_cnt == 0))
			{
				PulseFlag = Bit_RESET; //无脉冲  停转
			}
			  
			Z_F_Zhuan = gpio_input_bit_get(GPIOA,GPIO_PIN_9);
			if(Z_F_Zhuan == 1) //正转    //  低电平 正转   修改正反转方向，与协议一致  20200527        
			{
				Led_Control(0);   //红灯
			}
			else
			{
				Led_Control(1);  //反转  绿灯
			}
//          UserPara.RotateSpeed  = 0;
//          for(i = 0; i<3;i++ )
//          {
//            UserPara.RotateSpeed +=Pulse100msCntBuf[i];                       
//          }
//          UserPara.RotateSpeed *= 20;  //计算旋转速度  1s脉冲数*10 =10秒* 6 = 1分钟  单位：转每分
       
			if(PulseFlag)  //有脉冲  在转动
            {           
                            
                //Z_F_Zhuan = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
                
                if(Z_F_Zhuan == 1) //正转
                {
                    if(UserPara.DirSta == Reversal) //上一次状态是  反转
                    {
                        Current_PositiveTime = UserPara.PositiveTimeBase;
                        UserPara.NegativeTimeBase += (tim_1min_cnt + 30)/60;          //计算反转时间  +30意思是超过半分钟算1分钟
                        tim_1min_cnt = 0;  //清0当前状态计时
                        tim_cnt = 0;                    }
                    if(UserPara.RotateSta == STA_STOP)   //上一次状态是  停转
                    {
                        Current_PositiveTime = UserPara.PositiveTimeBase;
                        tim_1min_cnt = 0;  //清0当前状态计时
                          tim_cnt = 0;
                    }
                                            
                    UserPara.DirSta = Foreward;   // ==1  //旋转方向  正转
                             
                    if(!((tim_cnt +1)%60))//60s到  更新一次正转时间
                    {
                        UserPara.PositiveTimeBase += 1;        //60s 更新一次正转时间  
                        tim_1min_cnt = 0;
                    }
                }
                else               //反转
                {
                    if(UserPara.DirSta == Foreward) //上一次状态是  正转
                    {
                        Current_NegativeTime = UserPara.NegativeTimeBase;
                        UserPara.PositiveTimeBase += (tim_1min_cnt + 30)/60;          //计算正转时间
                        tim_1min_cnt = 0;  //清0当前状态计时
                        tim_cnt = 0;
                    }
                    if(UserPara.RotateSta == STA_STOP)   //上一次状态是  停转
                    {
                        Current_NegativeTime = UserPara.NegativeTimeBase;
                        tim_1min_cnt = 0;  //清0当前状态计时
                          tim_cnt = 0;
                    }
                                            
                    UserPara.DirSta = Reversal;   // ==0  //旋转方向  反转
                    
                    if(!((tim_cnt +1)%60))//60s到  更新一次反转时间
                    {
                        UserPara.NegativeTimeBase += 1;          //60s  更新一次反转时间  
                        tim_1min_cnt = 0;
                    }                   
                }
                
                UserPara.RotateSta = STA_WORK;                      //旋转状态   转动中
                
            }
            else     //无脉冲  停止
            {
                    if(UserPara.DirSta == Foreward)                      //上一次状态是  正转
                    {
                        UserPara.PositiveTimeBase += (tim_1min_cnt + 30)/60;             //计算正转时间  
                        tim_1min_cnt = 0;                                       //清0当前状态计时
                    }           
                     if(UserPara.DirSta == Reversal)                //上一次状态是  反转
                    {
                        UserPara.NegativeTimeBase += (tim_1min_cnt + 30)/60;              //计算反转时间
                        tim_1min_cnt = 0;                                      //清0当前状态计时
                    }

                                  
                UserPara.DirSta = Stall; // ==0  //旋转方向 
                UserPara.RotateSta = STA_STOP;     //旋转状态   停转
                
            }  
		}
	}
	
}

/*!
    \brief      initilize the com GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void com_gpio_init(void)
{
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_2);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_3);
}

/*!
    \brief      initilize the USART configuration of the com
    \param[in]  none
    \param[out] none
    \retval     none
*/
extern UserTypeDef UserPara;
void com_usart_init(void)
{
//	rs485_Init();
	
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);

	flash_read_multi(BAUDRATE, &(UserPara.Baudrate), 1);
    /* USART configure */
    usart_deinit(USART1);
    switch(UserPara.Baudrate)
	{
		case 01:
			usart_baudrate_set(USART1, 2400U);
			break;
		
		case 02:
			usart_baudrate_set(USART1, 4800U);
			break;
		
		case 03:
			usart_baudrate_set(USART1, 9600U);
			break;
		
		case 04:
			usart_baudrate_set(USART1, 19200U);
			break;
		
		case 05:
			usart_baudrate_set(USART1, 38400U);
			break;

		case 06:
			usart_baudrate_set(USART1, 57600U);
			break;
		
		case 07:
			usart_baudrate_set(USART1, 115200U);
			break;
		default:
			usart_baudrate_set(USART1, 9600U);
			break;
	
	}
//	usart_baudrate_set(USART1, 9600U);
	
	rs485_Init();
	
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);

    usart_enable(USART1);
}




/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t) ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
    return ch;
}

