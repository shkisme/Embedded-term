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


/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void TIM_Configure(void);
//void Servo_Change(uint16_t per);
void EXTI_Configure(void);
void USART2_Init(void);
void NVIC_Configure(void);
void TIM2_IRQHandler();
void USART2_IRQHandler(void);
void EXTI4_IRQHandler(void);
void ADC_Configure(void);
void ADC1_2_IRQHandler(void);
void TIM2_IRQHandler(void);


uint16_t value, x, y, time_count;
int isspin, led;
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

// Servo Motor variable
uint16_t curDeg = 700, minDeg = 700, maxDeg = 2600;
//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
    // USART2 : PA2, PA3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); // 아날로그 센서 1 : PC2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  // ADC1 ENABLE
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     // 아날로그 센서 1 : PC2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PC2 ->  ADC (Input)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
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

void Servo_Change(uint16_t per) { // 서보 모터의 각도를 변환하는 함수
  
	int pwm_pulse = per;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = pwm_pulse; 

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
        
}

void EXTI_Configure(void)
{
    // Interrupt 신호를 보내는 입력 핀(버튼 1)에 대해 EXTI Controller Setting
    EXTI_InitTypeDef EXTI_InitStructure;

   // TODO: Select the GPIO pin (button) used as EXTI Line using function 'GPIO_EXTILineConfig'
   // TODO: Initialize the EXTI using the structure 'EXTI_InitTypeDef' and the function 'EXTI_Init'
   
    /* Button 1(PC4) is pressed */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4; ////
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

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
	
	// TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

    // Timer 3
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

    //ADC part
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
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

// start of Handler
/////////////////////////////////////////////////////

void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET){
        // the most recent received data by the USART2 peripheral
        word = USART_ReceiveData(USART2);
        if(word == 'a'){
          // TODO implement*********************
          //LCD에 received message text를 띄움
          LCD_ShowChar(80, 160, ' ', 20, BLACK, WHITE);
          LCD_ShowString(80, 160, "start", BLACK, WHITE);
        }
        // clear 'Read data register not empty' flag
    	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
    USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

void EXTI4_IRQHandler(void){ /////////w35879083759279047204723907420
    if(EXTI_GetITStatus(EXTI_Line4) != RESET) { // KEY1
      
        if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_RESET) {
            //모터가 켜져있는 상태에서는 꺼지고 꺼져있는 상태에서는 꺼짐
          if(isspin == 1){
            isspin = 0;
        } else isspin = 1;
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void ADC_Configure(void){
	
    ADC_InitTypeDef ADC_12; // ADC12 채널 -> PC2
	
	ADC_12.ADC_ContinuousConvMode = ENABLE; // ???? ??? ??????? ContinuousConv ????
    ADC_12.ADC_DataAlign = ADC_DataAlign_Right; // Right ???? ??? ?��??? ????
    ADC_12.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_12.ADC_Mode = ADC_Mode_Independent; // Independent ??? ????
    ADC_12.ADC_NbrOfChannel = 1; // ????? ??? 1??
    ADC_12.ADC_ScanConvMode = DISABLE; // ???? ??��? ??? ???? ??????
    
	ADC_Init(ADC1, &ADC_12); // ????
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC1,ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    //ADC multi channel로 여러 센서의 값을 받아오는 법 알아보기***********************
}

void ADC1_2_IRQHandler(void) {
// 디버깅으로 할 때는 정상적으로 동작하는데 
  //디버깅 없이 할 때는 LCD가 동작하지 않음
  //대체 뭐가 문제임
  if(ADC_GetITStatus(ADC1,ADC_IT_EOC) != RESET) { // ?�� ???? ??
	  
    value = (int)ADC_GetConversionValue(ADC1); // ADC1?? ???? ???? value?? ???
    
  }
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  
}

void TIM2_IRQHandler() {
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) { // ?�� ???? ??
		
                if(led == 0) {
                            GPIO_ResetBits(GPIOD,GPIO_Pin_3); // LED2 On
                            led = 1;
                } else {
                            GPIO_SetBits(GPIOD,GPIO_Pin_3);   // LED2 Off
                            led = 0;
                    }
          
		if(isspin) { // DC ????? ??????????
                        Servo_Change((uint16_t)curDeg);
			curDeg += 100;
			if (curDeg > maxDeg) {
				curDeg = minDeg;
			}
		}
                time_count++;
                    if(time_count % 10 == 0) {
			time_count = 0;
		    }
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

/////////////////////////////////////////////////////
//end of Handler

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
    
    GPIO_SetBits(GPIOD,GPIO_Pin_7); 
    GPIO_SetBits(GPIOD,GPIO_Pin_2); 
    GPIO_SetBits(GPIOD,GPIO_Pin_3); 
    // --------------LCD Part----------------
    LCD_Init();
    Touch_Configuration();
    //Touch_Adjust();
    LCD_Clear(WHITE);

    LCD_ShowString(80,80,"THUR_Team01",BLACK,WHITE); // ?? ??? ???
    
    
    while (1) {
      //Touch_GetXY(&x, &y, 1); // ext?? 1?? ??????��? "??????" ????? 1???? ?????? ???
      //Convert_Pos(x, y, &x, &y); // ??? ???	
      
      if(isspin == 1){ // on -> off
                //GPIO_ResetBits(GPIOD,GPIO_Pin_2);   // LED1 Off
                LCD_ShowString(80, 100, "   ", BLACK, WHITE);
                LCD_ShowString(80, 100, "ON", BLACK, WHITE);
      } else{     // off -> on
          //GPIO_SetBits(GPIOD,GPIO_Pin_2); // LED1 On
          LCD_ShowString(80, 100, "   ", BLACK, WHITE);
          LCD_ShowString(80, 100, "OFF", BLACK, WHITE);
      }
      LCD_ShowNum(120,200, time_count, 2, BLACK, WHITE);
      LCD_ShowNum(80, 120, value / 64, 4, BLACK, WHITE); // (80, 120) ????? ???? ???? ?? ???
	if( x != 0 || y != 0 ) { // ????? ????? 0?? ????
          LCD_DrawCircle(x, y, 3);							      // ??? ????? ?????? 3??? ?? ????
          LCD_ShowNum(80, 120, value, 4, BLACK, WHITE); // (80, 120) ????? ???? ???? ?? ???
          
        }
        
        
      
    }
    return 0;
}
