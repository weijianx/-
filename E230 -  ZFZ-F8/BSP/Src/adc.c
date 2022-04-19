#include "adc.h"



__IO uint16_t ad_value[220];


//各个时钟初始化，GPIO的初始化
void rcu_config_inject(void)
{
/* enable GPIOC clock */
    rcu_periph_clock_enable(RCU_GPIOA);
	/* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA);
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC);
    /* enable timer1 clock */
//    rcu_periph_clock_enable(RCU_TIMER2);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
}

/*!
    \brief      GPIO configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* configure PA2(ADC channel2) as analog input */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_2);
}

/*!
    \brief      DMA configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
    dma_parameter_struct dma_init_struct;

    /* initialize DMA channel0 */
    dma_deinit(DMA_CH0);
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr  = (uint32_t)ad_value;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.number       = 220;
    dma_init_struct.periph_addr  = (uint32_t)&(ADC_RDATA);
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH0, &dma_init_struct);
    
    /* configure DMA mode */
    dma_circulation_enable(DMA_CH0);
    dma_memory_to_memory_disable(DMA_CH0);
    
    /* enable DMA channel0 */
    dma_channel_enable(DMA_CH0);
}

/*!
    \brief      ADC configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_config(void)
{
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1);

    /* ADC regular channel config */
    adc_regular_channel_config(0, ADC_CHANNEL_2, ADC_SAMPLETIME_55POINT5);

    /* ADC external trigger enable */
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    /* ADC external trigger source config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_T0_CH0);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* enable ADC interface */
    adc_enable();
    /* ADC calibration and reset calibration */
    adc_calibration_enable();
    /* ADC DMA function enable */
    adc_dma_mode_enable();
}


/*!
    \brief      display ADC value
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void display_adc_value(void)
//{
//    int ix,iy,i;
//    int deta = 200 /4 * 3.3;


//    for(ix = x_offset + 6 + 1,i = 0;ix < x_offset + 6 +220;ix++,i++){
//        for(iy = 110;iy < 310-1;iy++){
//            if(iy == (310 - ad_value[ i ] * deta / 0x0FFF)){
//                /* set the pixel */
//                lcd_draw_point(ix,iy,YELLOW);
//            }else{
//                /* set the pixel */
//                lcd_draw_point(ix,iy,BLUE);
//            }
//        }
//    }

//}




