
#include "std_types.h"
#include "SYSTEM_CTL.h"
#include "GPIO_REG.h"
#include "GPIO_INT.h"
/**
 * main.c
 */
volatile unsigned long int * RCC = 0x400FE060;
int main(void)
{ *RCC |= (1<<11);
*RCC &= ~(1<<22);
*RCC |= (1<<4);
*RCC &= ~(1<<5);
*RCC |= (1<<13);
GPIO_vidBusSet(PORTF, AHB);
    GPIO_vidClkSet(PORTF, ENABLE);
    GPIO_vidDirectionModeSet(PORTF, 0x02, MODE_OUT);
    GPIO_vidPadSet(PORTF, 0x02, PAD_NPU_NPD, DRIVE_2_m, DIGITAL);
    GPIO_vidDirectionModeSet(PORTF, 0x01, MODE_IN);
        GPIO_vidPadSet(PORTF, 0x01, PAD_PU, DRIVE_2_m, DIGITAL);
    //GPIO_vidWrite(PORTF, 0x01, 0x01);




while(1)
{
    //u8 out=GPIO_u8Read(PORTF, 0x01);
    //if(out==0x01)
    GPIO_vidWrite(PORTF, 0x02, 0x02);
    //else
        //GPIO_vidWrite(PORTF, 0x02, 0x00);
}


	return 0;
}
