#ifndef TIMER_H
#define TIMER_H

#include "gd32e230.h"

extern uint8_t Time_5s_flag;
extern uint8_t Time_1s_flag;
extern uint16_t AutoUpLoad_counter;
extern uint16_t AutoUpLoad_flag;

void timer_config(void);
void TIMER2_Init(void);
void TIMER15_Init(void);
void counter_pulse(void);


#endif


