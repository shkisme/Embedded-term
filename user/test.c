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
void Servo_Change(uint16_t per);
void EXTI_Configure(void);
void USART2_Init(void);
void NVIC_Configure(void);
void TIM2_IRQHandler(void);
void USART2_IRQHandler(void);
void EXTI4_IRQHandler(void);
void ADC_Configure(void);
void ADC1_2_IRQHandler(void);
void TIM2_IRQHandler(void);
void startVen(void);
void stopVen(void);
void startMotor(void);
void stopMotor(void);
void LED_On(void);
void LED_Off(void);
void resetTimer(void);

/*-------------Timer Status---------------*/
#define NO_TIMER 0
#define TWENTY_SEC 20 // 20 sec
int timer_status = NO_TIMER;
int current_time = 0;

int SMOKE_FLAG = 2400; // 연기 농도가 높은지 판단하는 값
int LIGHT_FLAG = 2900; // 밝기 농도 판단

uint16_t analogData_1, analogData_2, lightValue;
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

// flag
bool isSpin = false;
bool isStart = false; // 전체 시스템 가동
bool isOpen = false; // 창문 및 환풍기 가동
bool isLed = false; // LED 작동

// Servo Motor variable
uint16_t curDeg = 700, minDeg = 700, maxDeg = 2600;

// message
int message_index = 0;
char msg1[] = "System Start!\r\n";
char msg2[] = "System End...\r\n";
char msg3_open[] = "Window is open\r\n";
char msg3_close[] = "Window is close\r\n";
char msg4_smoke_good[] = "Smoke concentration good\r\n";
char msg4_smoke_bad[] = "Smoke concentration bad. need Ventilation\r\n";
char msg5[] = "Start Ventilation!\r\n";
char msg6[] = "Stop Ventilation.\r\n";

/*-------------Configure---------------*/
void RCC_Configure(void)
{
    // USART2 : PA2, PA3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); // 아날로그 센서 1 : PC2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);  // ADC1 ENABLE
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);  // ADC2 ENABLE
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
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ADC_2 Part
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;     // 아날로그 센서 2 : PC3 - channel - 13 @wlqrkrhtlvek
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PB13 ->  ADC2 (Input)
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

void ADC_Configure(void){
	
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

    // ADC2 - PC3
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
void USART2_IRQHandler() { // 폰에서 값 받아옴
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET){
        // the most recent received data by the USART2 peripheral
        word = USART_ReceiveData(USART2);
        if(word == '1'){ // 자동 환기 시스템 작동 시작
          // TODO implement*********************
          // LCD에 received message text를 띄움
          isStart = true; 
          message_index = 1;
        } else if (word == '2'){ // 시스템 전체 종료
            isStart = false;
            message_index = 2;
        } else if (word == '3'){ // 창문 상태 조회
            message_index = 3;
        } else if (word == '4'){ // 연기 상태 조회
            message_index = 4;
        } else if (word == '5'){ // 수동 환기 시작
            if (isOpen == false){  // 닫혀있을때
                isOpen = true;
                resetTimer();
                message_index = 5;
                // TODO: 서보모터, DC 모터 작동 함수 호출
                startMotor();
            }
        } else if (word == '6'){ // 수동 환기 멈춤 
            if (isOpen == true){  // 열려있을때
                isOpen = false;
                resetTimer();
                message_index = 6;
                // TODO: 서보모터, DC 모터 중지 함수 호출
                stopMotor();
            }
        }
        // clear 'Read data register not empty' flag
    	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

void sendDataUART2(uint16_t data) {
   /* Wait till TC is set */
   while ((USART2->SR & USART_SR_TC) == 0);
   USART_SendData(USART2, data);
}

void EXTI4_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line4) != RESET) { // KEY1
        if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_RESET) {
            //모터가 켜져있는 상태에서는 꺼지고 꺼져있는 상태에서는 꺼짐
          if(isSpin == true){
            GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_15);   // Motor Off
            isSpin = false;
        } else {
            GPIO_SetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_15);   // Motor On
            isSpin = true;
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void ADC1_2_IRQHandler(void) {  // 아날로그 센서값을 받아옴 @wlqrkrhtlvek

  // ADC_1 value(PC2) - 연기 센서
  if(ADC_GetITStatus(ADC1,ADC_IT_EOC) != RESET) {
    analogData_1 = ADC_GetConversionValue(ADC1);
    if (analogData_1 <= SMOKE_FLAG){ // 연기 상태 나쁠 때
        startVen();
    }
  }
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

  // ADC_2 value(PC3) - 조도 센서
  if(ADC_GetITStatus(ADC2,ADC_IT_EOC) != RESET) {
    analogData_2 = ADC_GetConversionValue(ADC2);
    if (analogData_2 >= LIGHT_FLAG){ // 밝을 때
        startMotor();
    }
  }
  ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);

}

void TIM2_IRQHandler() { // 1초에 한번씩 수행
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) {
        if (timer_status != NO_TIMER){
            current_time++;
        }
		if (timer_status == TWENTY_SEC){
            if (current_time >= TWENTY_SEC){ // 20초 경과시 timeout
                timer_status = NO_TIMER;
                current_time = 0;
                isLed = false;
                // TODO : 모터 멈춤 함수 호출
                stopMotor();
            } 
        }
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

// TODO : 연기 감지하면 호출
void startVen(){ // 환기 시작
    // TODO : 모터 작동 함수 호출
    startMotor();
    // TODO : LED 빨간불 켜기 함수 호출
    LED_On();    
}

void startMotor(){ // 모터 작동
    timer_status = TWENTY_SEC;
    current_time = 0;
}

void stopMotor(){ // 모터 중지
    if (isLed == true){
        LED_Off();
    }
}

void stopVen(){ // 환기 멈춤
    
}

void LED_On(){ // LED 켬
    isLed = true;
}

void LED_Off(){ // LED 끔
    isLed = false;
}

void resetTimer(){
    timer_status = NO_TIMER; // 타이머 초기화
    current_time = 0;        // 타이머 초기화   
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
    // --------    
    LCD_Init();

    while(true){
        LCD_Clear(WHITE);
        LCD_ShowString(20, 40,  "1 : START SYSTEM", BLACK, WHITE);
        LCD_ShowString(20, 60,  "2 : STOP SYSTEM", BLACK, WHITE);
        LCD_ShowString(20, 80,  "3 : SHOW WINDOW STATUS", BLACK, WHITE);
        LCD_ShowString(20, 100, "4 : SHOW SMOKE VALUE", BLACK, WHITE);
        LCD_ShowString(20, 120, "5 : OPEN WINDOW", BLACK, WHITE);
        LCD_ShowString(20, 140, "6 : CLOSE WINDOW", BLACK, WHITE);
        while (isStart == false){}
        while (isStart == true) {
          LCD_ShowString(80, 200, "start", BLACK, WHITE);
          if (message_index == 1){
            for(int i = 0; msg1[i] != '\0'; i++){
                    sendDataUART2(msg1[i]);
            }
          } else if (message_index == 2){
            for(int i = 0; msg2[i] != '\0'; i++){
                    sendDataUART2(msg2[i]);
            }
            break;
          } else if (message_index == 3){
            if (isOpen == true){
                for(int i = 0; msg3_open[i] != '\0'; i++){
                    sendDataUART2(msg3_open[i]);
                }
            }
            if (isOpen == false){
                for(int i = 0; msg3_close[i] != '\0'; i++){
                    sendDataUART2(msg3_close[i]);
                }
            }
          } else if (message_index == 4){
            if ((int)analogData_1 <= SMOKE_FLAG){ // 연기 농도 나쁠 때
                for(int i = 0; msg4_smoke_bad[i] != '\0'; i++){
                    sendDataUART2(msg4_smoke_bad[i]);
                }
            }
            else{ // 연기 농도 괜찮을 때
                for(int i = 0; msg4_smoke_good[i] != '\0'; i++){
                    sendDataUART2(msg4_smoke_good[i]);
                }
            }
          } else if (message_index == 5){
            for(int i = 0; msg5[i] != '\0'; i++){
                    sendDataUART2(msg5[i]);
            }
          } else if (message_index == 6){
            for(int i = 0; msg6[i] != '\0'; i++){
                    sendDataUART2(msg6[i]);
            }
          }
        message_index = 0;

        if(isSpin == true){ // on -> off
            //GPIO_ResetBits(GPIOD,GPIO_Pin_2);   // LED1 Off
            LCD_ShowString(80, 240, " ON", BLACK, WHITE);
        } else {     // off -> on
            //GPIO_SetBits(GPIOD,GPIO_Pin_2); // LED1 On
            LCD_ShowString(80, 240, "OFF", BLACK, WHITE);
        }
            LCD_ShowNum(120,200, current_time, 2, BLACK, WHITE);

            LCD_ShowString(20, 0, "YEON1 : ", BLACK, WHITE);
            LCD_ShowString(20, 20, "JODO2 : ", BLACK, WHITE);
            
            LCD_ShowNum(80, 260, analogData_1, 4, BLACK, WHITE);
            LCD_ShowNum(80, 280, analogData_2, 4, BLACK, WHITE);
        }
    }
    return 0;
}
