#include "stm32f10x.h"

// Base Address
#define RCC_APB2_BASE 0x40021000
#define GPIO_A_BASE 0x40010800
#define GPIO_B_BASE 0x40010C00
#define GPIO_C_BASE 0x40011000
#define GPIO_D_BASE 0x40011400

// RCC Clock Setting & Enable(Port A, B, C, D)
#define RCC_APB2_OFFSET 0x18
#define RCC_APB2 ((volatile unsigned *)(RCC_APB2_BASE + RCC_APB2_OFFSET)) // clock
#define RCC_APB2_ABCD_EN 0x0000003C

// Port Configuration(Low/High)
// Low Port  : 0~7
// High Port : 8~15
#define GPIO_ADDR_L_OFFSET 0x00
#define GPIO_ADDR_H_OFFSET 0x04

// Port x's Base Address + Offset
#define GPIO_A_CRL ((volatile unsigned *)(GPIO_A_BASE + GPIO_ADDR_L_OFFSET))
#define GPIO_B_CRH ((volatile unsigned *)(GPIO_B_BASE + GPIO_ADDR_H_OFFSET))
#define GPIO_C_CRL ((volatile unsigned *)(GPIO_C_BASE + GPIO_ADDR_L_OFFSET))
#define GPIO_C_CRH ((volatile unsigned *)(GPIO_C_BASE + GPIO_ADDR_H_OFFSET))
#define GPIO_D_CRL ((volatile unsigned *)(GPIO_D_BASE + GPIO_ADDR_L_OFFSET))

// Using Port Bit
#define PORT_BIT_0 0x00000001
#define PORT_BIT_2 0x00000004
#define PORT_BIT_3 0x00000008
#define PORT_BIT_4 0x00000010
#define PORT_BIT_7 0x00000080
#define PORT_BIT_10 0x00000400
#define PORT_BIT_13 0x00002000

//Port Input Data Register Control
#define GPIO_IDR_OFFSET 0x08
#define GPIO_A_IDR ((volatile unsigned *)(GPIO_A_BASE + GPIO_IDR_OFFSET)) // Low
#define GPIO_B_IDR ((volatile unsigned *)(GPIO_B_BASE + GPIO_IDR_OFFSET)) // High
#define GPIO_C_IDR ((volatile unsigned *)(GPIO_C_BASE + GPIO_IDR_OFFSET)) // Low & High

//Port Output Data Register Control
#define GPIO_BSRR_OFFSET 0x10
#define GPIO_BRR_OFFSET 0x14
#define GPIO_D_BSRR ((volatile unsigned *)(GPIO_D_BASE + GPIO_BSRR_OFFSET))// Bit Set
#define GPIO_D_BRR  ((volatile unsigned *)(GPIO_D_BASE + GPIO_BRR_OFFSET)) // Bit Reset


int main(void)
{
  *RCC_APB2 |= RCC_APB2_ABCD_EN; // Clock Enable

  *GPIO_A_CRL &= ~0x0000000F; // A Port Reset Before Setting
  *GPIO_A_CRL |= 0x00000008;  // A Port Configuration

  *GPIO_B_CRH &= ~0x00000F00; // B Port Reset Before Setting
  *GPIO_B_CRH |= 0x00000800;  // B Port Configuration

  *GPIO_C_CRL &= ~0x000F0000; // C Port Reset Before Setting(Low)
  *GPIO_C_CRL |= 0x00080000;  // C Low Port Configuration

  *GPIO_C_CRH &= ~0x00F00000; // C Port Reset Before Setting(High)
  *GPIO_C_CRH |= 0x00800000;  // C High Port Configuration

  *GPIO_D_CRL &= ~0xF00FFF00; // D Port Reset Before Setting
  *GPIO_D_CRL |= 0x30033300;  // D Port Configuration

  // Reset All LED Off
  *GPIO_D_BSRR |= (PORT_BIT_2|PORT_BIT_3|PORT_BIT_4|PORT_BIT_7); // Reset D Port's Pin 2,3,4,7

  int flag = 0;
  while(1){

    //Because Buttons Of Board are in 'Pull-Up', Bit '0' means Pushed.
    if(!(*GPIO_C_IDR & PORT_BIT_4))  // KEY1 Button Pushed
          flag=1;
    if(!(*GPIO_B_IDR & PORT_BIT_10)) // KEY2 Button Pushed
          flag=2;
    if(!(*GPIO_C_IDR & PORT_BIT_13)) // KEY3 Button Pushed
          flag=3;
    if(!(*GPIO_A_IDR & PORT_BIT_0))  // KEY4 Button Pushed
          flag=4;

    switch(flag){
    case 1:
      *GPIO_D_BRR |= (PORT_BIT_2 | PORT_BIT_7); // LED 1, 4 ON
      break;
    case 2:
      *GPIO_D_BSRR |= (PORT_BIT_2 | PORT_BIT_7); // LED 1, 4 OFF
      break;
    case 3:
      *GPIO_D_BRR |= (PORT_BIT_3 | PORT_BIT_4); // LED 2, 3 ON
      break;
    case 4:
      *GPIO_D_BSRR |= (PORT_BIT_3 | PORT_BIT_4); // LED 2, 3 OFF
      break;
    }
  }


  return 0;
}
