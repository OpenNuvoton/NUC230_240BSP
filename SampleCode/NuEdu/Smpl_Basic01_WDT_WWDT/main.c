/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/08/18 11:54a $
 * @brief    NuEdu-SDK-NUC240 WDT_WWDT Sample Code
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01.h"

#define WDT_alarm 1
#define WWDT_alarm 2
#define initial_Buzzer() GPIO_SetMode(PE, BIT5, GPIO_PMD_OUTPUT)
#define RKEY_INPUT PB14
#define BKEY_INPUT PC13
#define Buzzer_ON  PE5=0
#define Buzzer_OFF PE5=1
#define RLED_ON    LED_On(1);
#define GLED_ON    LED_On(4);
#define RLED_OFF   LED_On(0);
#define GLED_OFF   LED_On(0);

uint32_t alarm_time = 1000000;  //1s
uint32_t WDT_wait;
uint32_t WWDT_wait = 0;
uint32_t Alarm = 0;

void WDT_Buzzer(void)
{
    Buzzer_ON;
    RLED_ON;
    CLK_SysTickDelay(alarm_time);
    Buzzer_OFF;
    RLED_OFF;
}
void WWDT_Buzzer(void)
{
    Buzzer_ON;
    GLED_ON;
    CLK_SysTickDelay(alarm_time);
    Buzzer_OFF;
    GLED_OFF;
}
void WDT_IRQHandler(void)
{
    /*WDT Interrupt Status is Set*/
    if(WDT_GET_TIMEOUT_INT_FLAG())
    {
        // Reset WDT and clear time out flag
        WDT_CLEAR_TIMEOUT_INT_FLAG();
        Alarm = WDT_alarm;
        printf("WDT interrupt !!!\n");
        WDT_RESET_COUNTER();            //Reset WDT Timer Counter
        WDT_wait = 0;
    }

    /*WWDT Interrupt Status is Set*/
    if(WWDT_GET_INT_FLAG())
    {
        // Reset WWDT and clear time out flag
        WWDT_CLEAR_INT_FLAG();
        if(WWDT_wait == 1)                  //Check black key status
        {
            Alarm = WWDT_alarm;
            printf("WWDT interrupt !!!\n");
            WWDT_wait = 0;
        }
        WWDT_RELOAD_COUNTER();      //Reset WWDT Timer Counter
    }
}

void initial_KEY_INPUT(void)
{
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);
    GPIO_SetMode(PC, BIT13, GPIO_PMD_INPUT);
}

void SYST_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz HXT, HIRC and LIRC */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk);

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));
    SystemCoreClockUpdate();

    /*Set UART clock*/
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));

    /*Set WDT clock*/
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, 0);

    /*Set WWDT clock*/
    CLK_EnableModuleClock(WWDT_MODULE);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL2_WWDT_S_HCLK_DIV2048, 0);

    /* Set PB.0 and PB.1 multi-function pins for UART0 RXD, UART0 TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    initial_Buzzer();
    Initial_LED();
    initial_KEY_INPUT();

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    //Initial System
    SYST_Init();
    SYS_UnlockReg();

    UART0_Init();
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_3CLK, FALSE, FALSE); //Initial WDT (timeout interval 1.638 s)
    WWDT_Open(WWDT_PRESCALER_768, 0x20, TRUE);      //Initial WWDT (timeout interval 4.19 s)
    WDT_EnableInt();                                                            //Enable WDT (share with WWDT) timeout interrupt
    NVIC_EnableIRQ(WDT_IRQn);

    printf("WDT & WWDT Sample code!!!\n");
    while(1)
    {
        if(RKEY_INPUT == 0)
        {
            WDT_wait = 1;
            while(WDT_wait);            //Wait for WDT interrupt
        }
        if(BKEY_INPUT == 0)
        {
            WWDT_wait = 1;
        }

        /*Check WDT or WWDT alarm*/
        if(Alarm == WDT_alarm)
        {
            WDT_Buzzer();
            Alarm = 0;
        }
        else if(Alarm == WWDT_alarm)
        {
            WWDT_Buzzer();
            Alarm = 0;
        }
        WDT_RESET_COUNTER();            //Reset WDT
    }
}
