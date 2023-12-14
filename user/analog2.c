#include <stdbool.h>
#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "astm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include "touch.h"

// 집에서 고쳐온 부분(ADC1 & ADC2 test) ref : @wlqrkrhtlvek

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void TIM_Configure(void);
void Servo_Change(uint16_t per);
void EXTI_Configure(void);
void USART2_Init(void);
void NVIC_Configure(void);
void TIM2_IRQHandler();
void USART2_IRQHandler(void);
void EXTI4_IRQHandler(void);
void ADC_Configure(void);
void ADC1_2_IRQHandler(void);
void TIM2_IRQHandler(void);

/*-------------Timer Status---------------*/

#define NO_TIMER 0
#define TWENTY_MINUTES 20

uint16_t analogData_1, analogData_2; // @wlqrkrhtlvek
uint16_t lightValue, x, y, time_count;
int isSpin, isStart, led;
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};
int timer_status = NO_TIMER; // TODO: 온도차 판단 결과에 따라 10도 초과일 경우 ONE_MINUTES_30_SEC, 10도 이하일 경우 THREE_MINUTES로 설정
// timer 초기화 시 NO_TIMER로 상태 재설정
int current_time = 0;
int isOpen = 0; // 창문 및 환풍기 가동 flag

/*-------------Configure---------------*/
void RCC_Configure(void)
{
    // USART2 : PA2, PA3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); // 아날로그 센서 1, 2 : PC2, PC3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  // ADC1 ENABLE
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);  // ADC2 ENABLE @wlqrkrhtlvek
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); // TIMER2 Enable
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); // TIMER3 Enable
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //Servo Motor part
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // Enable Servo Motor : PB0
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ADC_1 Part
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     // 아날로그 센서 1 : PC2 - channel - 12
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PC2 ->  ADC (Input)
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ADC_2 Part
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;     // 아날로그 센서 2 : PC3 - channel - 13 @wlqrkrhtlvek
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PB13 ->  ADC2 (Input)
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // (Temp) Button
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // KEY1 (PC4) - Digital Input
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = (GPIO_Mode_IPU);
    GPIO_Init(GPIOC, &GPIO_InitStructure);

     // Motor Part
     // 릴레이 모듈 등 여러가지를 연결하려면 번거롭기 때문에 불가피하게 LED의 전원 상태로
     // 전원이 정상적으로 인가되었는지 확인
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Enable Motor : PD2, PD3(나중에 OR을 통해 2개까지 조작 가능)
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* USART2 pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // Tx -> PA2
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50Mhz
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function
   GPIO_Init(GPIOA, &GPIO_InitStructure); // PA2
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // Rx -> PA3
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50Mhz
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // Pull-Up
   GPIO_Init(GPIOA, &GPIO_InitStructure); // PA3
  
}

void TIM_Configure(void) {
    
    //TIM2 Configure (1sec count)
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

void USART2_Init(void) {
    USART_InitTypeDef USART2_InitStructure;

   // Enable the USART2 peripheral
   USART_Cmd(USART2, ENABLE);
   // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
   USART2_InitStructure.USART_BaudRate = 9600;
   USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART2_InitStructure.USART_StopBits = USART_StopBits_1;
   USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // CTS, RTS x
   USART2_InitStructure.USART_Parity = USART_Parity_No;
   USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_Init(USART2, &USART2_InitStructure); 
   
   // TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument lightValue 'Receive Data register not empty interrupt'
   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Rx -> interrupt enable
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // Button S1(PC4)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // Timer 2
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   NVIC_Init(&NVIC_InitStructure);

    // Timer 3
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
   NVIC_Init(&NVIC_InitStructure);

    //ADC part
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    // USART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void ADC_Configure(void){ // ************************************************** 1208 수정
   
    ADC_InitTypeDef ADC_12; // ADC12 채널 -> PC2
   
    ADC_12.ADC_ContinuousConvMode = ENABLE; // ContinuousConv : 값을 계속해서 입력받음
    ADC_12.ADC_DataAlign = ADC_DataAlign_Right; // Little Endian
    ADC_12.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_12.ADC_Mode = ADC_Mode_Independent; // Independent 
    ADC_12.ADC_NbrOfChannel = 1; // 사용하는 channel의 개수
    ADC_12.ADC_ScanConvMode = DISABLE; 
    
    // ADC1 - PC2
    ADC_Init(ADC1, &ADC_12);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5); // 중간에 숫자를 뭘로 해야 하는지 기억이 안 남
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // ADC2 - PC3 @wlqrkrhtlvek
    ADC_Init(ADC2, &ADC_12);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);

    //ADC multi channel로 여러 센서의 값을 받아오는 법 알아보기***********************
}

/*-------------Handler---------------*/

void ADC1_2_IRQHandler(void) {  // 아날로그 센서값을 받아옴 @wlqrkrhtlvek

  // ADC_1 value(PC2)
  if(ADC_GetITStatus(ADC1,ADC_IT_EOC) != RESET) {
    analogData_1 = ADC_GetConversionValue(ADC1);
  }
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

  // ADC_2 value(PC3)
  if(ADC_GetITStatus(ADC2,ADC_IT_EOC) != RESET) {
    analogData_2 = ADC_GetConversionValue(ADC2);
  }
  ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);

}

void TIM2_IRQHandler() { // 1초에 한번씩 수행
   if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) {
        if (timer_status != NO_TIMER){
            current_time++;
        }
      if (timer_status == TWENTY_MINUTES){ // 20초 경과시 timeout
            if (current_time >= TWENTY_MINUTES){
                timer_status = NO_TIMER;
                current_time = 0;
                isOpen = 0;
            } 
        }
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   }
}

void checkBright(void){
    if (lightValue <= 2500){ // 밝을 때
        timer_status = TWENTY_MINUTES;
    }
}

/*-------------main---------------*/
 int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM_Configure();
    EXTI_Configure();
    USART2_Init();
    ADC_Configure();
    NVIC_Configure();
    
    GPIO_SetBits(GPIOD,GPIO_Pin_2); 
    GPIO_SetBits(GPIOD,GPIO_Pin_3); 
    GPIO_SetBits(GPIOD,GPIO_Pin_7); 
    // --------------LCD Part----------------
    LCD_Init();
    Touch_Configuration();
    LCD_Clear(WHITE); 

    LCD_ShowString(80, 40, "start", BLACK, WHITE);

    // @wlqrkrhtlvek (LCD part)

    while (1) {

      LCD_ShowString(80, 100, "ADC1 Part", BLACK, WHITE);
      LCD_ShowString(80, 200, "ADC2 Part", BLACK, WHITE);
      LCD_ShowNum(80, 60,  current_time, 2, BLACK, WHITE);
      LCD_ShowNum(80, 120, analogData_1, 4, BLACK, WHITE);
      LCD_ShowNum(80, 220, analogData_2, 4, BLACK, WHITE);
      
    }
    return 0;
}