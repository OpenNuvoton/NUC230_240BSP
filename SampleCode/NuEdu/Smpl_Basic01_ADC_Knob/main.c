/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/08/18 11:54a $
 * @brief    NuEdu Basic01 Volume Knob Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "NuEdu-Basic01.h"

void UART0_Init(void);
void Write_LED_Color_Flash(uint32_t Speed);
void Write_LED_Bar(uint32_t Number);

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t Volume;
    uint32_t LED_Value;

    //Initial System
    SYS_Init();

    //Initial UART
    UART0_Init();

    //Open Volume Knob Device
    Open_ADC_Knob();

    printf("Volume Knob Value:\n");

    while(1)
    {
        //Get Volume Knob Data
        Volume = Get_ADC_Knob();                    //Volume Range: 0 ~ 4095
        printf("%d\n", Volume);

        //Use Volume Control to LED Flash Speed
        Write_LED_Color_Flash(Volume);

        //Show Volume scale on LED Bar
        LED_Value = Volume * (12 + 1) / 4096;       //LED Bar Count Range: 0 ~ 12
        Write_LED_Bar(LED_Value);

        //Close Volume Knob Device when you want
//      if(Volume_Data==4095)
//          Close_Volume_Knob();
    }
}

void UART0_Init(void)
{
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    SYS_UnlockReg();
    if(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk))
    {
        CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);                         //Enable XTAL's 12 MHz
        SystemCoreClockUpdate();
    }
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_EnableModuleClock(UART0_MODULE);
    SYS_LockReg();

    UART_Open(UART0, 115200);
//  printf("\nUART Open\n");
}

#define _LED_Color_Count    3
#define _LED_B              PA14
#define _LED_R              PA12
#define _LED_G              PA13

void Write_LED_Color_Flash(uint32_t Speed)
{
    uint32_t i;
    static uint32_t LED_Count = 0;
    volatile uint32_t *ptrLED[_LED_Color_Count] = {&_LED_B, &_LED_R, &_LED_G};

    //Delay Time Control
    if(Speed > 4096)  Speed = 4096;
    for(i = 0; i < ((4096 - Speed) * 100); i++);

    for(i = 0; i < _LED_Color_Count; i++)
    {
        if(LED_Count == i)
            *ptrLED[i] = 0; //LED ON
        else
            *ptrLED[i] = 1; //LED OFF
    }

    LED_Count++;
    if(LED_Count == _LED_Color_Count)
        LED_Count = 0;
}

#define _LED_Bar_Count      8
#define _LED1               PB4
#define _LED2               PB5
#define _LED3               PD14
#define _LED4               PD15
#define _LED5               PE7
#define _LED6               PE15
#define _LED7               PE14
#define _LED8               PD5

void Write_LED_Bar(uint32_t Number)
{
    uint32_t i;
    volatile uint32_t *ptrLED[_LED_Bar_Count] = {&_LED1, &_LED2, &_LED3, &_LED4, &_LED5, &_LED6,
                                                 &_LED7, &_LED8
                                                };

    for(i = 0; i < _LED_Bar_Count; i++)
    {
        if(Number > i)
            *ptrLED[i] = 0; //LED ON
        else
            *ptrLED[i] = 1; //LED OFF
    }
}
