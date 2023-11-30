#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "core_cm3.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"
#include "misc.h"
#include "stm32f10x_tim.h"

void RCC_Configure(void);
void GPIO_Configure(void);
void TIM_Configure(void);
void NVIC_Configure(void);
void TIM2_IRQHandler();
void delayMotor();

//LCD variable
uint16_t x, y;
uint16_t led1 = 0, led2 = 0, five_count = 0, isOn = 0, motorflag = 0;
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

//Motor variable
uint16_t curDeg = 700, minDeg = 700, maxDeg = 2600;


void RCC_Configure(void) {
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE); // LED : PD2, PD3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); // Servo Motor : PB0

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); // TIMER2 Enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); // TIMER3 Enable
        
}

void GPIO_Configure(void) {
  
	GPIO_InitTypeDef GPIOD_InitStructure;
        GPIO_InitTypeDef GPIOB_InitStructure;
        
        //LED part
	GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Enable LED : PD2, PD3
	GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIOD_InitStructure);
  
        //Servo Motor part
	GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_0;              // Enable Servo Motor : PB0
	GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIOB_InitStructure);
        
}

void TIM_Configure(void) {
    
    //https://www.disca.upv.es/aperles/arm_cortex_m3/curset/STM32F4xx_DSP_StdPeriph_Lib_V1.0.1/html/struct_t_i_m___time_base_init_type_def.html
    //TIM2 Configure
    TIM_TimeBaseInitTypeDef TIM2_InitStructure;

    TIM2_InitStructure.TIM_Prescaler = 7200;
    TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; // set upcounter mode
    TIM2_InitStructure.TIM_Period = 10000; // ARR 
    TIM2_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM2, &TIM2_InitStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    //TIM3 Configure
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    uint16_t prescale;
    uint16_t period = 20000;
    uint16_t frequency = 50;

    prescale = (uint16_t) (SystemCoreClock / period / frequency);

    TIM3_InitStructure.TIM_Period = period;         
    TIM3_InitStructure.TIM_Prescaler = prescale;
    TIM3_InitStructure.TIM_ClockDivision = 0;
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Down;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // us

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    
}

void change(uint16_t per) { // 서보모터 각도 변경
  
	int pwm_pulse = per;

	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = pwm_pulse; 

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
        
}

void NVIC_Configure(void) {

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
        
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

}

void TIM2_IRQHandler() {
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) { // 우선 순위 체크
		
		change((uint16_t)curDeg);
		
		if(isOn) { // LED가 켜져있으면
			
			curDeg += 100;
			if (curDeg > maxDeg) {
				curDeg = minDeg;
			}
			five_count++;
		    // LED 1 : 1sec toggle
		    if(led1 == 0) {
				GPIO_ResetBits(GPIOD,GPIO_Pin_2); // LED1 On
				led1 = 1;
		    } else {
				GPIO_SetBits(GPIOD,GPIO_Pin_2);   // LED1 Off
				led1 = 0;
		 	}
		    // 5초마다
		  	if(five_count % 5 ==0) { // five_count 가 5의 배수이면 = 5초 단위
				// toggle
				if(led2==0){ 
					GPIO_ResetBits(GPIOD,GPIO_Pin_3); // LED2 On
					led2 = 1; 
				} else{
					GPIO_SetBits(GPIOD,GPIO_Pin_3);   // LED2 Off
					led2 = 0;
				}
				five_count = 0;
		    }
		  
		} else { // toggle != 0
			
			curDeg -= 100;
			if (curDeg < minDeg) {
				curDeg = maxDeg;
			}
		    GPIO_SetBits(GPIOD,GPIO_Pin_2);
		  	GPIO_SetBits(GPIOD,GPIO_Pin_3);
		}
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	}
}

int main(void) {

	SystemInit();
	RCC_Configure();
	GPIO_Configure();
    TIM_Configure();
	NVIC_Configure();
    /////////////////
	LCD_Init();
	Touch_Configuration();
	Touch_Adjust();
	LCD_Clear(GREEN);


	LCD_ShowString(80,80,"THUR_Team01", BLACK, WHITE); // Print Team
	
	LCD_ShowString(130,210,"BUT",BLACK,WHITE); // Make Button
	LCD_DrawRectangle(60, 140, 200, 280);
	LCD_ShowString(80,100,"OFF",BLACK,WHITE);	

	while (1) {
		
	    Touch_GetXY(&x,&y,1); // x, y 좌표 읽어옴
	    Convert_Pos(x,y,&x,&y); // 좌표 변환
	    if(60 <= x && x <= 200 && 140 <= y && y <= 280) { // 사각형 범위 내 (버튼) 누르면 동작 1
		  if (isOn == 0) { //on 출력
			LCD_ShowString(80,100,"    ",BLACK,WHITE); // erase text
			isOn=1;
			LCD_ShowString(80,100,"ON",BLACK,WHITE);   // print on
		  
		  } else { //off 출력
			LCD_ShowString(80,100,"    ",BLACK,WHITE); // erase text
			isOn=0;
			LCD_ShowString(80,100,"OFF",BLACK,WHITE);  // print on
		  }
		}
		
	}
	
	return 0;
}