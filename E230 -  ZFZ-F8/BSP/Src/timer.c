#include "timer.h"
#include "type.h"



/*!
    \brief      TIMER configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void timer_config(void)
//{
//   timer_oc_parameter_struct timer_ocintpara;
//    timer_parameter_struct timer_initpara;
//	timer_ic_parameter_struct timer_icinitpara;

//    rcu_periph_clock_enable(RCU_TIMER0);
//	rcu_periph_clock_enable(RCU_GPIOA);
////	rcu_periph_clock_enable(RCU_AF);
//	
////	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8 | GPIO_PIN_9);
////	gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_8 | GPIO_PIN_9);
////	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
//	
//	timer_deinit(TIMER0);

//    /* TIMER configuration */
//    timer_initpara.prescaler         = 71;
//    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
//    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
//    timer_initpara.period            = 65535;
//    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
//    timer_initpara.repetitioncounter = 0;
//    timer_init(TIMER0,&timer_initpara);
//	
////	*(uint32_t *)(TIMER0 + 0x18) |= (1<<4 | 1<<12);			//滤波控制
////	*(uint32_t *)(TIMER0 + 0x20) &= ~(1<<1);				//ch0输入上升沿
////	*(uint32_t *)(TIMER0 + 0x18) |= 1;						//ch0输入 is0映射在ci0fe0
////	
////	*(uint32_t *)(TIMER0 + 0x20) |= (1<<5);					//ch1输入下降沿
////	*(uint32_t *)(TIMER0 + 0x18) |= (2<<8);					//ch1输入 is1映射在ci0fe1
////	
////	*(uint32_t *)(TIMER0 + 0x0c) |= (3<<1);					//使能ch0 ch1比较捕获中断
////	*(uint32_t *)(TIMER0 + 0x20) |= (1|1<<4);				//使能ch0 ch1比较捕获
////	
////	*(uint32_t *)(TIMER0 + 0x00) |= 1;						//计数器使能
//	
////	
//	timer_channel_input_struct_para_init(&timer_icinitpara);    //将输入捕获结构体参数变为初始值
//	timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;    //通道输入极性
//	timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI; //通道输入模式选择
//	timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;           //通道输入预分频器
//	timer_icinitpara.icfilter    = 0x00;                         //通道输入捕获滤波
//	timer_input_capture_config(TIMER0, TIMER_CH_0, &timer_icinitpara);
//	

//	timer_channel_input_struct_para_init(&timer_icinitpara);    //将输入捕获结构体参数变为初始值
//	timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;    //通道输入极性
//	timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI; //通道输入模式选择
//	timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;           //通道输入预分频器
//	timer_icinitpara.icfilter    = 0x00;                         //通道输入捕获滤波
//	timer_input_capture_config(TIMER0, TIMER_CH_1, &timer_icinitpara);
//	
//	
//    /* auto-reload preload enable */
//    timer_auto_reload_shadow_enable(TIMER0);
////    timer_primary_output_config(TIMER0, ENABLE);
//	timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH0);
//	timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH1);
//	
//	timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH0);
//	timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH1);
//	
//	nvic_irq_enable(TIMER0_Channel_IRQn, 2);
//	
////	TIMER_CNT(TIMER0) = 0;
//	timer_enable(TIMER0);
//	
//}


void TIMER2_Init(void)
{
	timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);

    /* TIMER configuration */
    timer_initpara.prescaler         = 7199;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* CH0 configuration in PWM mode1 */
//    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_LOW;
//    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
//    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);

//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 100);
//    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);
//    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

	nvic_irq_enable(TIMER2_IRQn, 1);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
//    timer_primary_output_config(TIMER0, ENABLE);
	
	timer_interrupt_enable(TIMER2, TIMER_INT_UP);
	timer_enable(TIMER2);

	
}



void TIMER15_Init(void)
{
	timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER15);

    timer_deinit(TIMER15);

    /* TIMER configuration */
    timer_initpara.prescaler         = 7199;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER15,&timer_initpara);

    /* CH0 configuration in PWM mode1 */
//    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_LOW;
//    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
//    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);

//    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 100);
//    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);
//    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

	nvic_irq_enable(TIMER15_IRQn, 2);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER15);
//    timer_primary_output_config(TIMER0, ENABLE);
	
	timer_interrupt_enable(TIMER15, TIMER_INT_UP);
	timer_enable(TIMER15);

	
}



uint16_t tim_cnt = 0;   //总时间累加
uint16_t tim_1min_cnt = 0;   //1min时间累加

uint8_t Time_5s_flag = 0;
uint8_t Time_1s_flag = 0;
uint16_t CH0_cnt = 0;
uint16_t CH1_cnt = 0;

uint16_t CH_cnt = 0;

extern UserTypeDef UserPara;
BitAction  PulseFlag = Bit_RESET;                                         //开始装填数组标志

UserTypeDef status;


//uint8_t Time_1min_flag = 0;
uint8_t first_20s = 0;   //开机后的前20秒
extern uint16_t Current_pulse;  //当前脉冲数

uint16_t pulse_flag0 = 0;           //正传标志位
uint16_t pulse_flag1 = 0;			//反转标志位


uint16_t status_flag = 0;			//运行状态标志位

uint16_t timeD_flag = 0;			//中断时间差标志位

void TIMER0_Channel_IRQHandler(void)
{

	if(timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH0))
	{

		timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH0);
		
		if(pulse_flag0 >= pulse_flag1)
		{
			pulse_flag0++;
			CH0_cnt++;

		}
		
		else
		{
			pulse_flag1 = 0;
		}
		
		timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH1);
		
	}

	if(timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH1))
	{

		timer_interrupt_disable(TIMER0, TIMER_INT_FLAG_CH0);

		timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH1);
		
		if(pulse_flag1 >= pulse_flag0)
		{

			pulse_flag1++;

			CH1_cnt++;
				
		}
		
		else
		{			
			pulse_flag0 = 0;
		}	

		timer_interrupt_enable(TIMER0, TIMER_INT_FLAG_CH0);
	}


}


//uint16_t tim_cnt1=0;
void TIMER2_IRQHandler(void)
{
	uint16_t read_pulse = 0;  //当前脉冲数
	uint16_t i;
	
//	uint8_t ch[] = "awegf\r\n";
	if(timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP))
	{
		timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
		tim_cnt++;
		if(!((tim_cnt+1)%1))  //1s
		{
			Time_1s_flag = 1;

		}

		if(!((tim_cnt +1)%5))//5s计时
		{
			Time_5s_flag = 1;
		}


	}


}


uint16_t AutoUpLoad_counter=0;
uint16_t AutoUpLoad_flag=0;
uint16_t count_flag = 0;
void TIMER15_IRQHandler(void)
{
	uint16_t read_pulse = 0;  //当前脉冲数
	uint16_t i;
	uint8_t ttmp[4];
	
//	uint8_t ch[] = "awegf\r\n";
	if(timer_interrupt_flag_get(TIMER15, TIMER_INT_FLAG_UP))
	{
		timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);
		AutoUpLoad_counter++;

		if(!((AutoUpLoad_counter +1)%5))
		{
			count_flag++;
			if(count_flag == 2)
			{
				count_flag = 0;
				AutoUpLoad_counter = 0;
				AutoUpLoad_flag = 1;
			}
			

		}
	}


}


