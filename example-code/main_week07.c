#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void NVIC_Configure(void);
void EXTI15_10_IRQHandler(void);
void Delay(void);
void sendDataUART1(uint16_t data);
int flag = 1; 
int doUartSend = 0;
//  1 : 1->2->3->4
// -1 : 4->3->2->1

void RCC_Configure(void)
{    // 사용하고자 하는 포트에 Clock 인가(A, B, C, D, USART1, AFIO Enable)
    // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
    /* UART TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   /* Button S1, S2, S3 port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   /* LED port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
   /* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
   /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}
void GPIO_Configure(void)
{
    // 각 핀(KEY1,2,3/LED1,2,3,4/UART)에 대한 입출력 설정
    
    GPIO_InitTypeDef GPIOA9_InitStructure;
    GPIO_InitTypeDef GPIOA10_InitStructure;
    GPIO_InitTypeDef GPIOB_InitStructure;
    GPIO_InitTypeDef GPIOC_InitStructure;
    GPIO_InitTypeDef GPIOD_InitStructure;

   // TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'
   
    /* Button KEY1, KEY2, KEY3 pin setting */
    //KEY1 : PC4 / KEY2 / PB10, KEY3 / PC13
    GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIOB_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // default speed
    GPIOB_InitStructure.GPIO_Mode = (GPIO_Mode_IPU);
    GPIO_Init(GPIOB, &GPIOB_InitStructure);

    GPIOC_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_13;
    GPIOC_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOC_InitStructure.GPIO_Mode = (GPIO_Mode_IPU);
    GPIO_Init(GPIOC, &GPIOC_InitStructure);

    /* LED pin setting*/
    GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
    GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIOD_InitStructure);
    /* UART pin setting */
    //TX
    GPIOA9_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIOA9_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA9_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIOA9_InitStructure);
   //RX
    GPIOA10_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIOA10_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOA10_InitStructure.GPIO_Mode = (GPIO_Mode_IPU);
    GPIO_Init(GPIOA, &GPIOA10_InitStructure);

}

void EXTI_Configure(void)
{
    // Interrupt 신호를 보내는 입력 핀(버튼123)에 대해 EXTI Controller Setting
    // USART입력도 Interrupt 신호를 보내지만 이건 USART1_Init()에서 따로 구현
    EXTI_InitTypeDef EXTI_InitStructure;

   // TODO: Select the GPIO pin (Joystick, button) used as EXTI Line using function 'GPIO_EXTILineConfig'
   // TODO: Initialize the EXTI using the structure 'EXTI_InitTypeDef' and the function 'EXTI_Init'
   
    /* Button 1(PC4) is pressed */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4; ////
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Button 2 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
    EXTI_InitStructure.EXTI_Line = EXTI_Line10; ////
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
   
   /* Button 3 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13; ////
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

   // NOTE: do not select the UART GPIO pin used as EXTI Line here
}

void USART1_Init(void)
{
    // Putty와 시리얼통신을 하기 위한 USART 설정 Setting(Week6과 유사함)
    // USART_ITConfig() 함수를 통해 RX로 들어오는 신호가 Interrupt로 동작하도록 설정
   USART_InitTypeDef USART_InitStructure;

   // Enable the USART1 peripheral
   USART_Cmd(USART1, ENABLE);
    USART_InitStructure.USART_BaudRate = 9600; // Baud Rate
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // word length : 8bit
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // stop bit 1bit
    USART_InitStructure.USART_Parity = USART_Parity_No ; // no parity bits
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // rx&tx mode
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

   // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
   USART_Init(USART1,&USART_InitStructure); //USART1에적용
   
   // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
   // RX로 들어오는 신호가 Interrupt로 동작하도록 설정해줌
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void) {
    // Interrupt : KEY1, 2, 3, RX
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

   // TODO: Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'
   
    // Button S1(PC4)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Button S2(PB10)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Button S3(PC13)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // UART1
   // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
    // Interrupt가 들어왔을 때의 동작 구현 함수

    uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
       // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);

        // TODO implement
        if(word == 'a'){
            flag =  1;
        } else if(word == 'b'){
            flag = -1;
        }

        // clear 'Read data register not empty' flag
       USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

void EXTI4_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line4) != RESET) { // KEY1
        if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_RESET) {
            flag = 1; // forward
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void EXTI15_10_IRQHandler(void) { // when the button is pressed

    if (EXTI_GetITStatus(EXTI_Line10) != RESET) { //KEY2
      if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == Bit_RESET) {
         // TODO implement
            flag = -1;
      }
      EXTI_ClearITPendingBit(EXTI_Line10);
   }
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) { //KEY3
      if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET) {
         // TODO implement
         // send data at Main while
            doUartSend = 1;
      }
        EXTI_ClearITPendingBit(EXTI_Line13);
   }
}

void Delay(void) {
   int i;

   for (i = 0; i < 2000000; i++) {}
}

void sendDataUART1(uint16_t data) {
   /* Wait till TC is set */
   while ((USART1->SR & USART_SR_TC) == 0);
   USART_SendData(USART1, data);
}

int main(void)
{

    SystemInit();

    RCC_Configure();

    GPIO_Configure();

    EXTI_Configure();

    USART1_Init();

    NVIC_Configure();

    int i = 0;
    int iter = 0;
    uint16_t led_array[4][4] = { //LED 웨이브 구현 array
        {0,     UINT16_MAX,      UINT16_MAX,      UINT16_MAX},
        {UINT16_MAX,     0,      UINT16_MAX,      UINT16_MAX},
        {UINT16_MAX,     UINT16_MAX,      0,      UINT16_MAX},
        {UINT16_MAX,     UINT16_MAX,      UINT16_MAX,      0}
    };
    char msg[] = "TEAM01\r\n";
    
    while (1) {
       // TODO: implement 
        if(doUartSend == 1){
            for(i = 0; msg[i] != '\0'; i++){
                sendDataUART1(msg[i]);
            }
            doUartSend = 0;
        }
       GPIOD->ODR = ((GPIO_ODR_ODR2 & led_array[iter][0]) |
                     (GPIO_ODR_ODR3 & led_array[iter][1]) |
                     (GPIO_ODR_ODR4 & led_array[iter][2]) |
                     (GPIO_ODR_ODR7 & led_array[iter][3]));
       // Delay
       Delay();
        if(flag == 1) iter++;
        else iter--;

        if(iter == 4) iter = 0;
        if(iter == -1) iter = 3;
    }
    return 0;
}