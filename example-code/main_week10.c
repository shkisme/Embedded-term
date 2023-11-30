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
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); //  �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� PC2
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); // ADC1 ENABLE
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� PC2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // PC2 -> �뜝�럥�맶�뜝�럥堉껃뜝�럥爰뗥뜝�럩援꿨뜝�럥�맶�뜝�럥堉®뭐癒��뵰占쎄뎡 IN �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占�
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);

  
  
}

void ADC_Configure(void) {
	
    ADC_InitTypeDef ADC_12; // ADC12 �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占� -> PC2
	
	ADC_12.ADC_ContinuousConvMode = ENABLE; // �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩�꼶 �뜝�럥�맶�뜝�럥堉뤷뜝�럩�뮲�뜝�럩援꿨뜝�럥�맶�뜝�럥堉ｆ쾬�꼻�삕嶺뚮슡�뫒占쎄뎡 ContinuousConv �뜝�럩�꽎�뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩�꼨
    ADC_12.ADC_DataAlign = ADC_DataAlign_Right; // Right �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩源� �뜝�럥�맶�뜝�럥堉��뜝�럥荑덂뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援�
    ADC_12.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_12.ADC_Mode = ADC_Mode_Independent; // Independent �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援�
    ADC_12.ADC_NbrOfChannel = 1; // �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占� 嶺뚳옙熬곻옙占쎌맶�뜝�럥吏쀥뜝�럩援� 1�뜝�럥�맶�뜝�럥吏쀥뜝�럩援�
    ADC_12.ADC_ScanConvMode = DISABLE; // �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� 嶺뚳옙熬곻옙占쎌맶�뜝�럥堉®춯�슡�뫒占쎄뎡 �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩�꼶 �뜝�럥�맶�뜝�럥堉ｆ쾬�꼻�삕嶺뚮슡�뫒占쎄뎡 �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩�꽎�뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩�꼨
    
	ADC_Init(ADC1, &ADC_12); // �뜝�럥�맶�뜝�럥堉뤹뭐癒��뵰占쎄뎡�뜝�럩�꼨
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
	
  if(ADC_GetITStatus(ADC1,ADC_IT_EOC) != RESET) { // �뜝�럥�맶�뜝�럥�쑋�뜝�럡�맖 �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� 嶺뚳퐢�샑野껓옙
	  
    value = ADC_GetConversionValue(ADC1); // ADC1�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� value�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�룞�삕�뜝�룞�삕占쎌냷�뜝�럩援�
	
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

    
    LCD_ShowString(80,80,"THU_Team11",BLACK,WHITE); // �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩源� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占�
    
    while (1) {
      Touch_GetXY(&x,&y,1); // ext�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� 1�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥堉꾢슖�돦裕뉒뙴�쉻�삕占쎄뎡 "�뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥堉잏춯�슡�뫒占쎄뎡�뜝�럥諭�" �뜝�럥�맶�뜝�럥�넰�뜝�럡�븫�뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� 1�뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥�냺�뜝�럩逾쎾뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占�
	  								 // 0 �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥�냺�뜝�럩逾쎾뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿰뇖�궪�삕 �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� 2�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥�넰�뜝�럡�븫�뜝�럩援�
	  
      Convert_Pos(x,y,&x,&y); // �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�걫 �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럩�꼶
      
	  if(x !=0 || y !=0) { // �뜝�럥�맶�뜝�럥堉ｅ뜝�럥爰뗥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�걫�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� 0�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥堉껃뜝�럥鍮띸춯琉우뒩占쎄뎡
          LCD_DrawCircle(x,y,3);							      // �뜝�럥�맶�뜝�럥�냻�뜝�럥�냷�뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�걫�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� 3嶺뚯쉸萸울옙�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援�
		  
          LCD_ShowNum(80,120,value,4,BLACK,WHITE); // (80, 120) �뜝�럥�맶�뜝�럥�넃嶺뚮ㅏ�뒩占쎄뎡�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占�
          LCD_ShowNum(80,140,x,3,BLACK,WHITE); 	  // (80,140) �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� X �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占�
          LCD_ShowNum(80,160,y,3,BLACK,WHITE);      // (80, 160)�뜝�럥�맶�뜝�럥吏쀥뜝�럩援� Y �뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶占쎈쐻�뜝占�
        }
      
    }
    return 0;
}
