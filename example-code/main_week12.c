#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"

#define LUX_THRESHOLD 3500

//void Init(void);
void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void DMA_Configure(void);

const int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

// volatile unsigned 32bits
volatile uint32_t ADC_Value[1]; // ADC 값을 저장할 공간을 항상 참조하도록 volatile 키워드를 이용해 선언 


void RCC_Configure(void) {
    // DMA1, ADC1, Port C에 CLOCK 인가
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

void GPIO_Configure(void) {
    // PC2 (조도 센서) Configuration
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;
    
    // ADC1 설정
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // reference : week10_source_code
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5);
    // ADC_ITConfig(ADC1,ADC_IT_EOC, ENABLE); // No Interrupt
    ADC_DMACmd(ADC1, ENABLE); // interrupt가 아닌 DMA 사용 -> ADC-DMACmd 호출
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void DMA_Configure(void) {
	DMA_InitTypeDef DMA_Instructure; // DMA 설정을 위한 구조체

	DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;    			// 어디에 있는 걸 가져올지(ADC->DR = conversion된 데이터 결과 쓰여짐)
	DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value[0];				// 가져온 걸 어디에 쓸지
	DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;					        // 주변 장치에서 데이터를 읽어올 수 있도록 DIR_PeripheralSRC 설정
	DMA_Instructure.DMA_BufferSize = 1;							                // ADC 변수는 1개만 사용하기 때문에 버퍼 크기 1로 설정
	DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				// 주변 장치가 1개 이므로 PeripheralInc Disable
	DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Disable;					    // 한 메모리 주소 안에서만 값을 바꾸므로, 메모리 주소 레지스터 값을 증가시킬 필요 없음 -> Disable
	DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	    // 데이터 크기는 word 크기로 설정
	DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;				// 메모리 데이터 크기도 word 크기로 설정
	DMA_Instructure.DMA_Mode = DMA_Mode_Circular;						        // 센서 값을 계속해서 읽어야 하므로 Circular Mode
	DMA_Instructure.DMA_Priority = DMA_Priority_High;					        // 우선 순위는 원활한 처리를 위해 high로 설정
	DMA_Instructure.DMA_M2M = DMA_M2M_Disable;						            // 사용하지 않으므로 Disable

	DMA_Init(DMA1_Channel1, &DMA_Instructure);						            // 초기화
	DMA_Cmd(DMA1_Channel1, ENABLE);								                // 활성화
}

int main(){
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    DMA_Configure();
    // LCD Initialize
    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);	
    LCD_ShowString(80,80,"THUR_Team01", BLACK, WHITE); // Print Team

    uint8_t light = 0;		    // 현재 센서 상태
    uint8_t before_light = 2;   // 이전 센서 상태
    int back_color = WHITE; 	// 배경 색깔
    
    while(1){
		
        light = ADC_Value[0] < LUX_THRESHOLD; // Threshold 값보다 ADC 값이 작으면(=밝다는 뜻) 1 값 대입
		
        if (before_light != light){	      // 이전의 센서 상태와 지금 센서 상태가 다르다면(=값 변경을 감지하면)
		  // 밝으면 회색 배경, 어두우면 흰 배경
		  if(light) back_color = GRAY;
		  else 	    back_color = WHITE;

          LCD_Clear(back_color); 
          LCD_ShowString(80,80,"THUR_Team01", BLACK, back_color); // Print Team
          LCD_ShowNum(80,100, ADC_Value[0], 4, BLACK, back_color); // 조도 센서 값 출력

          before_light = light; // 이전 센서 상태 <- 현재 센서 상태
        }
      }
}