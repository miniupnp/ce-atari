#include "floppyhelpers.h"
#include "stm32f10x.h"                       // STM32F103 definitions

// set GPIOB as --- CNF1:0 -- 00 (push-pull output), MODE1:0 -- 11 (output 50 Mhz)
void FloppyOut_Enable(void )
{
    GPIOB->CRH &= ~(0x00000fff); 
    GPIOB->CRH |=  (0x00000737);            // (0x00000333)
    FloppyIndex_Enable();
    FloppyMFMread_Enable();     
}

// set GPIOB as --- CNF1:0 -- 01 (floating input), MODE1:0 -- 00 (input)
void FloppyOut_Disable(void)
{
    GPIOB->CRH &= ~(0x00000fff); 
    GPIOB->CRH |=  (0x00000444); 
    FloppyIndex_Disable();  
    FloppyMFMread_Disable();    
}

// enable / disable TIM1 CH1 output on GPIOA_8
void FloppyMFMread_Enable(void)
{
    GPIOA->CRH &= ~(0x0000000f); 
    GPIOA->CRH |= (0x0000000f);         // open drain
//GPIOA->CRH |= (0x0000000b);           // push pull
}

void FloppyMFMread_Disable(void)
{
    GPIOA->CRH &= ~(0x0000000f);
    GPIOA->CRH |= (0x00000004);
}

// enable / disable TIM2 CH4 output on GPIOB_11
void FloppyIndex_Enable(void)
{
    GPIOB->CRH &= ~(0x0000f000);
    GPIOB->CRH |= (0x0000f000);         // open drain
//GPIOB->CRH |= (0x0000b000);           // push pull
}

void FloppyIndex_Disable(void)
{
    GPIOB->CRH &= ~(0x0000f000);
    GPIOB->CRH |= (0x00004000);
}
