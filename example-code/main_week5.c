#include "stm32f10x.h"

// I used PD13, PD15 in this code (can changed)

// Base Address
#define RCC_APB2_BASE 0x40021000
#define GPIO_A_BASE   0x40010800
#define GPIO_B_BASE   0x40010C00
#define GPIO_C_BASE   0x40011000
#define GPIO_D_BASE   0x40011400

// RCC Clock Setting & Enable(Port B, C, D)
#define RCC_APB2_OFFSET 0x18
#define RCC_APB2 ((volatile unsigned *)(RCC_APB2_BASE + RCC_APB2_OFFSET)) // clock
#define RCC_APB2_ABCD_EN 0x00000038 // Enable Port BCD

// Port Configuration(Low/High)
// Low Port  : 0~7
// High Port : 8~15
#define GPIO_ADDR_L_OFFSET 0x00
#define GPIO_ADDR_H_OFFSET 0x04

// Port x's Base Address + Offset
#define GPIO_B_CRH ((volatile unsigned *)(GPIO_B_BASE + GPIO_ADDR_H_OFFSET))
#define GPIO_C_CRL ((volatile unsigned *)(GPIO_C_BASE + GPIO_ADDR_L_OFFSET))
#define GPIO_C_CRH ((volatile unsigned *)(GPIO_C_BASE + GPIO_ADDR_H_OFFSET))
#define GPIO_D_CRL ((volatile unsigned *)(GPIO_D_BASE + GPIO_ADDR_L_OFFSET))
#define GPIO_D_CRH ((volatile unsigned *)(GPIO_D_BASE + GPIO_ADDR_H_OFFSET))

// Using Port Bit
#define PORT_BIT_2 0x00000004
#define PORT_BIT_3 0x00000008
#define PORT_BIT_4 0x00000010
#define PORT_BIT_7 0x00000080
#define PORT_BIT_10 0x00000400
#define PORT_BIT_13 0x00002000
#define PORT_BIT_15 0x00008000

//Port Input Data Register Control
#define GPIO_IDR_OFFSET 0x08
#define GPIO_B_IDR ((volatile unsigned *)(GPIO_B_BASE + GPIO_IDR_OFFSET)) // High
#define GPIO_C_IDR ((volatile unsigned *)(GPIO_C_BASE + GPIO_IDR_OFFSET)) // Low & High

//Port Output Data Register Control
#define GPIO_BSRR_OFFSET 0x10
#define GPIO_BRR_OFFSET 0x14
#define GPIO_D_BSRR ((volatile unsigned *)(GPIO_D_BASE + GPIO_BSRR_OFFSET))// Bit Set
#define GPIO_D_BRR  ((volatile unsigned *)(GPIO_D_BASE + GPIO_BRR_OFFSET)) // Bit Reset

// Button1 : PC4
// Button2 : PB10
// Button3 : PC13

void delay() {
  int i;
  for (i = 0; i < 10000000; i++){}
}

int main(void)
{
  *RCC_APB2 |= RCC_APB2_ABCD_EN; // Clock Enable (Port B, C, D)

*GPIO_B_CRH &= ~0x00000F00; // B Port Reset Before Setting
*GPIO_B_CRH |=  0x00000800;  // Use Pin 10

*GPIO_C_CRL &= ~0x000F0000; // C Port Reset Before Setting(Low)
*GPIO_C_CRL |=  0x00080000;  // Use Pin 4

*GPIO_C_CRH &= ~0x00F00000; // C Port Reset Before Setting(High)
*GPIO_C_CRH |=  0x00800000;  // Use Pin 13

*GPIO_D_CRH &= ~0xF0F00000; // D Port Reset Before Setting
*GPIO_D_CRH |=  0x30300000;  // Use Pin 13, 15

  // Reset All Output Port
  *GPIO_D_BRR |= (PORT_BIT_13|PORT_BIT_15); // Reset D Port's Pin 13, 15
int flag;
while(1){
  
  flag = 0;
  if(!(*GPIO_C_IDR & PORT_BIT_4))  // KEY1 Button Pushed
        flag=1;
  if(!(*GPIO_B_IDR & PORT_BIT_10)) // KEY2 Button Pushed
        flag=2;
  if(!(*GPIO_C_IDR & PORT_BIT_13)) // KEY3 Button Pushed
        flag=3;
    
switch(flag){
  case 0:
    break;
  //Because Buttons Of Board are in 'Pull-Up', Bit '0' means Pushed.
  case 1: // KEY1 Button(PC4)  Pushed
    //motor 1, 2 rotate
    *GPIO_D_BSRR  |= (PORT_BIT_13 | PORT_BIT_15); // Set  Pin 13, 15
    
    delay();
    *GPIO_D_BRR   |= (PORT_BIT_13 | PORT_BIT_15);  // Reset Pin 13, 15
    break;
  case 2: // KEY2 Button(PB10) Pushed
    //motor 1 rotate
    *GPIO_D_BSRR  |= PORT_BIT_13;  // Set   Pin 13
    
    delay();
    *GPIO_D_BRR   |= (PORT_BIT_13 | PORT_BIT_15);  // Reset Pin 13, 15
    break;
  case 3: // KEY3 Button(PC13) Pushed
    //motor 2 rotate
    *GPIO_D_BSRR  |= PORT_BIT_15; // Set   Pin 15
    
    delay();
    *GPIO_D_BRR |= (PORT_BIT_13 | PORT_BIT_15);  // Reset Pin 13, 15
    break;
  
}

  }
  return 0;
}
