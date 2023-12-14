#include <stdbool.h>
#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include "touch.h"

void delay_ms(uint32_t ms);
void Servo_posimove(int Init_duty, int Direction,int delay);
void Servo_negamove(int Init_duty, int Direction, int delay);

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Enable the peripheral clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Change to TIM3

    // Configure PA0 as alternate function push-pull (TIM3_CH1) - Change to TIM3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure TIM3 - Change to TIM3
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 20ms period for 50Hz PWM
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // Assuming a 72MHz clock
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); // Change to TIM3

    // Configure TIM3_CH1 (PA0) as PWM output - Change to TIM3
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; // Initial duty cycle for the servo (adjust as needed)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // Change to TIM3
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); // Change to TIM3

    // Enable TIM3 - Change to TIM3
    TIM_Cmd(TIM3, ENABLE);

    while (1)
    {
        // Vary the duty cycle from 5% to 10% (adjust as needed)
        Servo_negamove(2500, 100, 5);
        Servo_posimove(500,   100, 5);
    }
}

void Servo_posimove(int Init_duty, int Direction, int delay){
      for (int dutyCycle = Init_duty; dutyCycle <= 1500; dutyCycle += Direction)
    {
        TIM_SetCompare1(TIM3, dutyCycle); // Change to TIM3
        delay_ms(100*delay);  // motor 돌아가는 시간
    }
}

void Servo_negamove(int Init_duty, int Direction,int delay){
      for (int dutyCycle = Init_duty; dutyCycle >= 1500; dutyCycle -= Direction)
    {
        TIM_SetCompare1(TIM3, dutyCycle); // Change to TIM3
        delay_ms(100*delay);  // motor 돌아가는 시간
    }
}

void delay_ms(uint32_t ms)
{
    volatile uint32_t nCount;
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    nCount = (RCC_Clocks.HCLK_Frequency / 10000) * ms;
    for (; nCount != 0; nCount--)
        ;
}
