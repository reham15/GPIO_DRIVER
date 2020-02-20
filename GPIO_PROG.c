/*
 * GPIO_PROG.c
 *
 *  Created on: Feb 19, 2020
 *      Author: yssh
 */

#include "GPIO_INT.h"
#include "GPIO_REG.h"
#include "SYSTEM_CTL.h"

static u32 ports[12] ={
                       GPIO_PORT_A_APB_BASE, GPIO_PORT_A_AHB_BASE,
                       GPIO_PORT_B_APB_BASE, GPIO_PORT_B_AHB_BASE,
                       GPIO_PORT_C_APB_BASE, GPIO_PORT_C_AHB_BASE,
                       GPIO_PORT_D_APB_BASE, GPIO_PORT_D_AHB_BASE,
                       GPIO_PORT_E_APB_BASE, GPIO_PORT_E_AHB_BASE,
                       GPIO_PORT_F_APB_BASE, GPIO_PORT_F_AHB_BASE
};

void GPIO_vidBusSet(gpioPort localPort, gpioBus localBus)
{
    GPIOHBCTL &= ~(1 << localPort);
    GPIOHBCTL |= ( localBus << localPort);
}
gpioBus GPIO_BusGet(gpioPort localPort)
{
    return (gpioBus)((GPIOHBCTL >> localPort) & 1);
}


u32 GPIO_u32GetPortAdd(gpioPort localPort)
{
    gpioBus bus  = GPIO_BusGet(localPort);
    return ports[(2*localPort)+ bus];
}

void GPIO_vidClkSet(gpioPort localPort, ctrl localAction)
{
    RCGCGPIO &= ~(1 << localPort);
    RCGCGPIO |= ( localAction << localPort);
}
ctrl GPIO_clkGet(gpioPort localPort)
{
    return (ctrl)((RCGCGPIO >> localPort) & 1);
}


void GPIO_vidDirectionModeSet(gpioPort localPort, u8 localPins, gpioMode localMode)
{
    volatile u32* reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOAFSEL);
    u32 data = *reg;
    data &= ~ localPins;
    if (localMode == MODE_AF)
        data |= (0xFF & localPins);
    else
        data |= (0x00 & localPins);
    *reg = data;

    reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIODIR);
    data = *reg;
    if (localMode == MODE_OUT)
        data |= (0xFF & localPins);
    else if (localMode == MODE_IN)
        data |= (0x00 & localPins);
    *reg = data;
}

u8 GPIO_u8DirectionGet(gpioPort localPort, u8 localPins)
{
    u32 reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIODIR));
    return (u8)(reg & localPins);
}
u8 GPIO_u8ModeGet(gpioPort localPort, u8 localPins)
{
    u32 reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIOAFSEL));
    return (u8)(reg & localPins);
}

void GPIO_vidPadSet(gpioPort localPort, u8 localPins, gpioPad localPad, gpioDrive localstr, gpioSig localSignal)
{
    volatile u32 * reg, * regSLR, * regAnlg;

    u32 data;
    reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIODEN);
    regAnlg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOAMSEL);
    if (localSignal == DIGITAL)
    {
        MASK_SET(reg, localPins, data);
        MASK_CLR(regAnlg, localPins, data);
    }
    else
    {
        MASK_CLR(reg, localPins, data);
        MASK_SET(regAnlg, localPins, data);
    }

    switch(localstr)
    {
    case DRIVE_2_m:
        reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIODR2R);
        break;
    case DRIVE_4_m:
        reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIODR4R);
        break;
    case DRIVE_8_m:
        reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIODR8R);
        break;
    case DRIVE_8_m_Slew:
        reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIODR8R);
        regSLR = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOSLR);
        /* Slew Rate */
        MASK_SET(regSLR, localPins, data);
        break;
    }
    MASK_SET(reg, localPins, data);


    reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOPUR);
    MASK_CLR(reg, localPins, data);
    reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOPDR);
    MASK_CLR(reg, localPins, data);
    reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOODR);
    MASK_CLR(reg, localPins, data);
    switch(localPad)
    {
    case PAD_PU:
        reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOPUR);
        MASK_SET(reg, localPins, data);
        break;

    case PAD_PD:
        reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOPDR);
        MASK_SET(reg, localPins, data);
        break;

    case PAD_OD:
        reg = (u32*) (GPIO_u32GetPortAdd(localPort) + GPIOODR);
        MASK_SET(reg, localPins, data);
        break;

    case PAD_NPU_NPD:
        /* nothing else to do  */
        break;
    }
}


u8 GPIO_u8PadDriveStrGet(gpioPort localPort, u8 localPins, gpioDrive localstr)
{
    u32 reg;
    switch (localstr)
    {
    case DRIVE_2_m:
        reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIODR2R));
        break;

    case DRIVE_4_m:
        reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIODR4R));
        break;

    case DRIVE_8_m:
        reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIODR8R));
        break;
    case DRIVE_8_m_Slew:
        reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIOSLR));
        break;
    }
    return (u8)(reg & localPins);
}


u8 GPIO_u8PadOpenDrainGet(gpioPort localPort, u8 localPins)
{
    u32 reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIOODR));
    return (u8)(reg & localPins);
}

u8 GPIO_u8PadPullUpGet(gpioPort localPort, u8 localPins)
{
    u32 reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIOPUR));
    return (u8)(reg & localPins);
}

u8 GPIO_u8PadPullDownGet(gpioPort localPort, u8 localPins)
{
    u32 reg = *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIOPDR));
    return (u8)(reg & localPins);
}


void GPIO_vidWrite(gpioPort localPort, u8 localPins, u8 localData)
{
    volatile u32* reg = (u32*)(GPIO_u32GetPortAdd(localPort) + GPIODATA + (u32)(localPins << 2) );
    *reg = localData;
}

u8 GPIO_u8Read(gpioPort localPort, u8 localPins)
{
    return *((volatile u32*)(GPIO_u32GetPortAdd(localPort) + GPIODATA + (u32)(localPins << 2)));
}

