#include <stdbool.h>
#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"

uint16_t value, x, y;
int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};

//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); //  조도센서 PC2
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); // ADC1 ENABLE
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // 조도센서 PC2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PC2 -> 아날로그 IN 모드
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

  
  
}

void ADC_Configure(void) {
	
    ADC_InitTypeDef ADC_12; // ADC12 사용 -> PC2
	
	ADC_12.ADC_ContinuousConvMode = ENABLE; // 연속 변환 필요하므로 ContinuousConv 활성화
    ADC_12.ADC_DataAlign = ADC_DataAlign_Right; // Right 부터 신호 읽도록 설정
    ADC_12.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_12.ADC_Mode = ADC_Mode_Independent; // Independent 모드 설정
    ADC_12.ADC_NbrOfChannel = 1; // 사용할 채널 1개
    ADC_12.ADC_ScanConvMode = DISABLE; // 단일 채널로 변환 하므로 비활성화
    
	ADC_Init(ADC1, &ADC_12); // 초기화
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC1,ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_Init(&NVIC_InitStructure);
    
}

void ADC1_2_IRQHandler() {
	
  if(ADC_GetITStatus(ADC1,ADC_IT_EOC) != RESET) { // 우선 순위 체크
	  
    value = ADC_GetConversionValue(ADC1); // ADC1로 얻은 값을 value에 할당
	
  }
  
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  
}



int main(void)
{

    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    NVIC_Configure();
    
    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);

    
    LCD_ShowString(80,80,"THU_Team11",BLACK,WHITE); // 조 번호 출력
    
    while (1) {
      Touch_GetXY(&x,&y,1); // ext에 1을 넣으므로써 "인터럽트" 발생시 1개의 동그라미 출력
	  								 // 0 넣으면 동그라미 터치 후 손 뗄 때 2번 발생
	  
      Convert_Pos(x,y,&x,&y); // 좌표 변환
      
	  if(x !=0 || y !=0) { // 하나라도 좌표가 0이 아니면
          LCD_DrawCircle(x,y,3);							      // 해당 좌표에 반지를 3짜리 원 생성
		  
          LCD_ShowNum(80,120,value,4,BLACK,WHITE); // (80, 120) 자리에 조도 센서 값 출력
          LCD_ShowNum(80,140,x,3,BLACK,WHITE); 	  // (80,140) 에 X 값 출력
          LCD_ShowNum(80,160,y,3,BLACK,WHITE);      // (80, 160)에 Y 값 출력
        }
      
    }
    return 0;
}
