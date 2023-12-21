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
void RCC_Configure(void);       // RCC Clock 인가
void GPIO_Configure(void);      // GPIO 핀 설정
void TIM_Configure(void);       // Timer 설정
void Servo_posimove(int Init_duty, int Direction,int delay);    // 서보모터 반시계 방향으로 회전
void Servo_negamove(int Init_duty, int Direction, int delay);   // 서보모터 시계 방향으로 회전
void delay_ms(uint32_t ms);     // 서보모터에 인가하는 딜레이
void USART2_Init(void);         // USART2 초기 설정
void NVIC_Configure(void);      // Interrupt 초기 설정
void USART2_IRQHandler(void);   // USART2 Interrupt 발생 시 처리 함수
void ADC_Configure(void);       // ADC 초기 설정
void ADC1_2_IRQHandler(void);   // ADC값을 주기적으로 받아오는 함수
void TIM2_IRQHandler(void);     // Timer2 Interrupt 발생 시 처리 함수
void startVen(void);            // 환기를 시작하는 함수
void startMotor(void);          // 모터 회전을 시작하는 함수
void stopMotor(void);           // 모터 회전을 중지하는 함수
void LED_On(void);              // LED 불빛을 켜는 함수
void LED_Off(void);             // LED 불빛을 끄는 함수
void resetTimer(void);          // 타이머 카운트를 중지하는 함수

/*-------------Timer Status---------------*/
#define NO_TIMER 0              // Timer를 사용하지 않을 때의 State
#define TWENTY_SEC 10           // 20 sec
int timer_status = NO_TIMER;    // 타이머의 상태를 표시하는 변수
int current_time = 0;           //현재 타이머가 센 시간 값

int SMOKE_FLAG = 1000;          // 연기 농도가 높은지 판단하는 값
int LIGHT_FLAG = 0;             // 밝기 농도 판단 하는 값 , 2900; 

uint16_t analogData_1, analogData_2; // 아날로그 데이터 저장 변수
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

// flag
bool isSpin  = false;           // 창문 및 환풍기 가동
bool isStart = false;           // 전체 시스템 가동
bool isLed   = false;           // LED 작동
bool enableTimer = false;       // Timer 작동

bool isWindowOpening = false;   // 창문 여는중 상태
bool isWindowClosing = false;   // 창문 닫는중 상태
bool isClosed = true;           // 창문 상태

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
char msg5_fin[] = "Window Opened successfully.\r\n";
char msg6[] = "Stop Ventilation.\r\n";
char msg6_fin[] = "Window Closed successfully.\r\n";

/*-------------Configure---------------*/
void RCC_Configure(void) {

    // Enable clock for GPIOA and USART2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE); // Enables clock for GPIOA
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // Enables clock for USART2, USART2 : PA2, PA3

    // Enable clock for GPIOB, GPIOC, and GPIOD
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Enables clock for GPIOB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // Enables clock for GPIOC (Analog Sensor 1: PC2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // Enables clock for GPIOD
    
    // Enable ADC1 and ADC2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enables clock for ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); // Enables clock for ADC2

    // Enable AFIO (Alternate Function I/O)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // Enables clock for Alternate Function I/O

    // Enable TIMER2 and TIMER3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Enables clock for TIMER2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Enables clock for TIMER3
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Servo Motor Configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // Connects to TIM3_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function, Push-Pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO operating speed
    GPIO_Init(GPIOA, &GPIO_InitStructure); // Initializes GPIOA with the specified settings

    // ADC_1 Configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     // Connects to Analog Sensor 1: PC2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PC2 ->  ADC (Input)                                                                                                                                                                                   
    GPIO_Init(GPIOC, &GPIO_InitStructure); // Initializes GPIOC with the specified settings

    // ADC_2 Configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;     // 아날로그 센서 2 : PC3 - channel - 13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PC3 ->  ADC2 (Input)
    GPIO_Init(GPIOC, &GPIO_InitStructure); // Initializes GPIOC with the specified settings

    // Motor Configuration
    // 릴레이 모듈 등 여러가지를 연결하려면 번거롭기 때문에 불가피하게 LED의 전원 상태로 전원이 정상적으로 인가되었는지 확인
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14; // Enable Motor : PD12, PD14 (나중에 OR을 통해 2개까지 조작 가능)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // Output, Push-Pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO operating speed
    GPIO_Init(GPIOD, &GPIO_InitStructure); // Initializes GPIOD with the specified settings

    /* USART2 Pin Configuration */
    // TX (Transmit)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // Connects to TX (PA2)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO operating speed
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function, Push-Pull mode
    GPIO_Init(GPIOA, &GPIO_InitStructure); // Initializes GPIOA with the specified settings for TX
    
    // RX (Receive)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // Connects to RX (PA3)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO operating speed
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // Input, Pull-Down mode
    GPIO_Init(GPIOA, &GPIO_InitStructure); // Initializes GPIOA with the specified settings for RX
  
}

void TIM_Configure(void) {
    
    // TIM2 Configure (1-second count)
    TIM_TimeBaseInitTypeDef TIM2_InitStructure;

    TIM2_InitStructure.TIM_Prescaler = 7200; // Set the prescaler value for 1Hz (assuming a 72MHz clock)
    TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; // Set upcounter mode
    TIM2_InitStructure.TIM_Period = 10000; // Auto-Reload Register (ARR) value
    TIM2_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // Set clock division

    TIM_TimeBaseInit(TIM2, &TIM2_InitStructure); // Initialize TIM2 with the specified settings
    TIM_ARRPreloadConfig(TIM2, ENABLE); // Enable auto-reload preload
    TIM_Cmd(TIM2, ENABLE); // Enable TIM2 counter
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // Enable TIM2 update interrupt
    
    // TIM3 Configure (PWM setup for servo motor)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Configure TIM3 - Change to TIM3
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // Set 20ms period for 50Hz PWM
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // Assuming a 72MHz clock, adjust prescaler accordingly
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;  // Set clock division
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // Set upcounter mode
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); // Initialize TIM3 with the specified settings

    // Configure TIM3_CH1 (PA6) as PWM output - Change to TIM3
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // Set PWM mode
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // Enable PWM output
    TIM_OCInitStructure.TIM_Pulse = 1500; // Initial duty cycle for the servo (adjust as needed)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // Set output polarity
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // Initialize TIM3_CH1 with the specified settings
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); // Enable TIM3_CH1 preload

    TIM_Cmd(TIM3, ENABLE); // Enable TIM3 counter
    
}

// Init_duty: Initial duty cycle value
// Direction: Incremental value for duty cycle change
// delay: Multiplier for delay time
void Servo_posimove(int Init_duty, int Direction, int delay){ 
    for (int dutyCycle = Init_duty; dutyCycle <= 1500; dutyCycle += Direction) {
        TIM_SetCompare1(TIM3, dutyCycle); // Set the compare value for PWM (TIM3)
        delay_ms(45*delay);  // motor 돌아가는 시간동안 delay
    }
    TIM_SetCompare1(TIM3, 1500); // Set the duty cycle to the center position after movement
}

void Servo_negamove(int Init_duty, int Direction, int delay){
    for (int dutyCycle = Init_duty; dutyCycle >= 1500; dutyCycle -= Direction) {
        TIM_SetCompare1(TIM3, dutyCycle); // Set the compare value for PWM (TIM3)
        delay_ms(45*delay);  // motor 돌아가는 시간동안 delay
    }
    TIM_SetCompare1(TIM3, 1500); // Set the duty cycle to the center position after movement
}

void delay_ms(uint32_t ms) {
    volatile uint32_t nCount; // Variable for counting delay cycles
    RCC_ClocksTypeDef RCC_Clocks; // Structure to hold RCC clock frequencies

    RCC_GetClocksFreq(&RCC_Clocks); // Get the system clock frequencies

    // Calculate the number of cycles needed for the delay based on milliseconds
    nCount = (RCC_Clocks.HCLK_Frequency / 10000) * ms;

    // Loop to create the delay for the calculated number of cycles
    for (; nCount != 0; nCount--)
        ;
}

void USART2_Init(void) {

    USART_InitTypeDef USART2_InitStructure;

    // Enable the USART2 peripheral
    USART_Cmd(USART2, ENABLE);

    // Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART2_InitStructure.USART_BaudRate = 9600; // Set baud rate to 9600
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b; // 8-bit data length
    USART2_InitStructure.USART_StopBits = USART_StopBits_1; // 1 stop bit
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control (CTS, RTS disabled)
    USART2_InitStructure.USART_Parity = USART_Parity_No; // No parity bit
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Enable both Rx and Tx modes
    USART_Init(USART2, &USART2_InitStructure); // Initialize USART2 with the specified settings

    // Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument lightValue 'Receive Data register not empty interrupt'
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enable receive data register not empty interrupt (Rx interrupt)
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;

    // Set the priority grouping to 2 bits for preemption priority and 2 bits for subpriority
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // Timer 2 Interrupt Configuration
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; // Timer 2 IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable Timer 2 IRQ
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Preemption priority for Timer 2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // Subpriority for Timer 2
    NVIC_Init(&NVIC_InitStructure); // Initialize NVIC settings for Timer 2

    // Timer 3 Interrupt Configuration
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; // Timer 3 IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable Timer 3 IRQ
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Preemption priority for Timer 3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // Subpriority for Timer 3
    NVIC_Init(&NVIC_InitStructure); // Initialize NVIC settings for Timer 3

    // ADC Interrupt Configuration for ADC1 and ADC2
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn; // ADC1 and ADC2 IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable ADC1 and ADC2 IRQ
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // Preemption priority for ADC1 and ADC2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // Subpriority for ADC1 and ADC2
    NVIC_Init(&NVIC_InitStructure); // Initialize NVIC settings for ADC1 and ADC2

    // USART2 Interrupt Configuration
    // Enable USART2 IRQ using 'NVIC_EnableIRQ'
    NVIC_EnableIRQ(USART2_IRQn); // Enable USART2 IRQ
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; // USART2 IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // Preemption priority for USART2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // Subpriority for USART2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable USART2 IRQ
    NVIC_Init(&NVIC_InitStructure); // Initialize NVIC settings for USART2

}

void ADC_Configure(void){

    ADC_InitTypeDef ADC_12; // Configuration structure for ADC1 and ADC2
   
    // Common ADC settings for ADC1 and ADC2
    ADC_12.ADC_ContinuousConvMode = ENABLE; // Continuous conversion mode: 값을 계속해서 입력받음
    ADC_12.ADC_DataAlign = ADC_DataAlign_Right; // Data alignment: right aligned (Little Endian)
    ADC_12.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // No external trigger for conversion
    ADC_12.ADC_Mode = ADC_Mode_Independent; // Independent mode operation
    ADC_12.ADC_NbrOfChannel = 1; // 사용하는 channel의 개수
    ADC_12.ADC_ScanConvMode = DISABLE; // Single channel conversion mode
    
    // ADC1 Configuration for PC2 (ADC_Channel_12)
    ADC_Init(ADC1, &ADC_12); // Initialize ADC1 with the specified settings
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5); // Configure regular conversion for ADC1, PC2 (ADC_Channel_12)
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // Enable End of Conversion (EOC) interrupt for ADC1
    ADC_Cmd(ADC1, ENABLE); // Enable ADC1
    ADC_ResetCalibration(ADC1); // Reset calibration for ADC1
    while(ADC_GetResetCalibrationStatus(ADC1)); // Wait for reset calibration to complete for ADC1
    ADC_StartCalibration(ADC1); // Start calibration for ADC1
    while(ADC_GetCalibrationStatus(ADC1)); // Wait for calibration to complete for ADC1
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // Start ADC1 conversion using software trigger

    // ADC2 Configuration for PC3 (ADC_Channel_13)
    ADC_Init(ADC2, &ADC_12); // Initialize ADC2 with the specified settings
    ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SampleTime_239Cycles5); // Configure regular conversion for ADC2, PC3 (ADC_Channel_13)
    ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE); // Enable End of Conversion (EOC) interrupt for ADC2
    ADC_Cmd(ADC2, ENABLE); // Enable ADC2
    ADC_ResetCalibration(ADC2); // Reset calibration for ADC2
    while(ADC_GetResetCalibrationStatus(ADC2)); // Wait for reset calibration to complete for ADC2
    ADC_StartCalibration(ADC2); // Start calibration for ADC2
    while(ADC_GetCalibrationStatus(ADC2));  // Wait for calibration to complete for ADC2
    ADC_SoftwareStartConvCmd(ADC2, ENABLE); // Start ADC2 conversion using software trigger

}

/*-------------Handler---------------*/
void USART2_IRQHandler() {
    uint16_t word; // 받은 데이터를 저장할 변수

    if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET){ // Check if USART2 'Read data register not empty' interrupt is pending
        word = USART_ReceiveData(USART2); // the most recent received data by the USART2 peripheral
        if(word == '1'){ // 자동 환기 시스템 작동 시작

          isStart = true; 
          message_index = 1;

        } 
        else if (word == '2'){ // 시스템 전체 종료

           message_index = 2;
            
        } 
        else if (word == '3'){ // 창문 상태 조회
          
            message_index = 3;

        } 
        else if (word == '4'){ // 연기 상태 조회

            message_index = 4;

        } 
        else if (word == '5' && isStart == true){ // 수동 환기 시작

            if (isSpin == false){
                message_index = 5;
                current_time = 0;
                enableTimer = false;
                startMotor(); // DC 모터 작동 함수
            }

        } 
        else if (word == '6'){ // 수동 환기 멈춤 

            if (isSpin == true){
                message_index = 6;
                current_time = 0;
                enableTimer = true;
                stopMotor(); // DC 모터 작동 중지 함수
            }

        }
        USART_ClearITPendingBit(USART2,USART_IT_RXNE); // clear 'Read data register not empty' flag
    }
}

void sendDataUART2(uint16_t data) {
   /* Wait till TC is set */
   while ((USART2->SR & USART_SR_TC) == 0);
   USART_SendData(USART2, data);
}

void ADC1_2_IRQHandler(void) {

    // ADC_1 value (PC2) - Smoke 센서
    if(ADC_GetITStatus(ADC1,ADC_IT_EOC) != RESET) {
        analogData_1 = ADC_GetConversionValue(ADC1);

        if ((int)analogData_1 <= SMOKE_FLAG && enableTimer == true && timer_status == NO_TIMER){ // 연기 상태 나쁠 때
            startVen();
        }
    }
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC); // Clear the EOC (End of Conversion) interrupt flag for ADC1

    // ADC_2 value (PC3) - 조도 센서
    if(ADC_GetITStatus(ADC2,ADC_IT_EOC) != RESET) {
        analogData_2 = ADC_GetConversionValue(ADC2);

        if ((int)analogData_2 <= LIGHT_FLAG && enableTimer == true && timer_status == NO_TIMER){ // 밝을 때
            startVen();
        }
    }
    ADC_ClearITPendingBit(ADC2, ADC_IT_EOC); // Clear the EOC (End of Conversion) interrupt flag for ADC2

}

void TIM2_IRQHandler() { // 1초에 한번씩 수행
   if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) {
        if (timer_status != NO_TIMER && enableTimer == true ){
            current_time++;
        }

        if (timer_status == TWENTY_SEC  && enableTimer == true){
            if (current_time >= TWENTY_SEC){ // 20초 경과시 timeout
                timer_status = NO_TIMER;
                current_time = 0;
                isLed = false;
                stopMotor();
            } 
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the update interrupt flag for TIM2
    }
}

// 환기 시작 함수
void startVen() {
    timer_status = TWENTY_SEC;
    current_time = 0;
    isLed = false;

    startMotor();    
    LED_On();    
}

// DC 모터 작동 함수
void startMotor() { 
    isSpin = true;

    if(isClosed) isWindowOpening = true;

    isWindowOpening = true; // ?

    if (isLed == false){
        LED_On();
    }
    
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);   // Motor On

}

// DC 모터 중지 함수
void stopMotor() { 
    isSpin = false;

    if(!isClosed) isWindowClosing = true;

    if (isLed == true){
        LED_Off();
    }

    GPIO_SetBits(GPIOD, GPIO_Pin_12);     // Motor Off

}

// 결론 : 모터는 Set으로 켜고 LED는 Reset으로 킨다. vise versa

void LED_On(void) {
    isLed = true;
    GPIO_SetBits(GPIOD, GPIO_Pin_14); 
}

void LED_Off(void) {
    isLed = false;
    GPIO_ResetBits(GPIOD, GPIO_Pin_14); 
}

/*-------------main---------------*/
int main(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM_Configure();
    USART2_Init();
    ADC_Configure();
    NVIC_Configure();
    
    GPIO_SetBits(GPIOD, GPIO_Pin_12);    // Motor on
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);  // LED on
    LCD_Init();

    while(true){
        LCD_Clear(WHITE);
        LCD_ShowString(20, 40,  "1 : START SYSTEM", BLACK, WHITE);
        LCD_ShowString(20, 60,  "2 : STOP SYSTEM", BLACK, WHITE);
        LCD_ShowString(20, 80,  "3 : SHOW WINDOW STATUS", BLACK, WHITE);
        LCD_ShowString(20, 100, "4 : SHOW SMOKE VALUE", BLACK, WHITE);
        LCD_ShowString(20, 120, "5 : OPEN WINDOW", BLACK, WHITE);
        LCD_ShowString(20, 140, "6 : CLOSE WINDOW", BLACK, WHITE);

        while (isStart == false) {} // 시스템 시작할 때까지 wait

        for(int i = 0; msg1[i] != '\0'; i++) sendDataUART2(msg1[i]);

        while (isStart == true) {
            if (message_index == 2){ // 시스템 종료
                isStart = false;
                for(int i = 0; msg2[i] != '\0'; i++) sendDataUART2(msg2[i]);
                break;
            } 
            else if (message_index == 3){ // 창문 상태 확인
                if (isSpin == true){
                    for(int i = 0; msg3_open[i] != '\0'; i++) sendDataUART2(msg3_open[i]);
                }
                if (isSpin == false){
                    for(int i = 0; msg3_close[i] != '\0'; i++) sendDataUART2(msg3_close[i]);
                }
            } 
            else if (message_index == 4){ // 연기 상태 확인
                if ((int)analogData_1 <= SMOKE_FLAG){ // 연기 농도 나쁠 때
                    for(int i = 0; msg4_smoke_bad[i] != '\0'; i++) sendDataUART2(msg4_smoke_bad[i]);
                }
                else{                                 // 연기 농도 괜찮을 때
                    for(int i = 0; msg4_smoke_good[i] != '\0'; i++) sendDataUART2(msg4_smoke_good[i]);
                }
            } 
            else if (message_index == 5){ // 수동 환기 시작
                for(int i = 0; msg5[i] != '\0'; i++) sendDataUART2(msg5[i]);
            } 
            else if (message_index == 6){ // 수동 환기 중지
                for(int i = 0; msg6[i] != '\0'; i++) sendDataUART2(msg6[i]);
            }
            message_index = 0;

            //
            if(isWindowOpening == true && isWindowClosing == false && isClosed == true){ // open

              Servo_posimove(500, 100, 1); // open
              isClosed = false;
              isWindowOpening = false;
              for(int i = 0; msg5_fin[i] != '\0'; i++) sendDataUART2(msg5_fin[i]);

            } 
            else if(isWindowOpening == false && isWindowClosing == true && isClosed == false){ // close
              
              Servo_negamove(2500, 100, 1); // close
              isClosed = true;
              isWindowClosing = false;
              for(int i = 0; msg6_fin[i] != '\0'; i++) sendDataUART2(msg6_fin[i]);

            }
            //

            if(isSpin == true) LCD_ShowString(20, 240, " OPEN", BLACK, WHITE);
            else               LCD_ShowString(20, 240, "CLOSE", BLACK, WHITE);
            LCD_ShowNum(120,200, current_time, 2, BLACK, WHITE);

            LCD_ShowString(20, 0,  "Smoke Degree : ", BLACK, WHITE);
            LCD_ShowNum(140, 260, (int)analogData_1, 4, BLACK, WHITE);
            //LCD_ShowString(20, 20, "JODO2 : ", BLACK, WHITE);
            //LCD_ShowNum(80, 280, (int)analogData_2, 4, BLACK, WHITE);
        }
        stopMotor();

        if(isClosed == false){
            Servo_negamove(2500, 100, 1); // close
            isClosed = true;
            isWindowClosing = false;
        }

        timer_status = NO_TIMER;
        enableTimer = false;
        current_time = 0;
        isWindowOpening = false;
        isWindowClosing = false;
        isClosed = true;
    }
    return 0;
}
