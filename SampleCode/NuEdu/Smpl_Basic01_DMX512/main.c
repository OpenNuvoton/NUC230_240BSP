/****************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 15/09/08 9:38a $
 * @brief    NUC230_240 Series UART Interface Controller Driver Sample Code
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"
//#include "NuEdu-Basic01.h"
#include "Segment.h"
#include "DMX512.h"


#define PLL_CLOCK           50000000


uint32_t u32SysTimerCounter = 0;
uint32_t u32SysTimerKeyCounter = 0;
uint32_t u32SysTimerDMX512SendCounter = 0;


#define DMX512_ROLE_NONE 0
#define DMX512_ROLE_MASTER 1
#define DMX512_ROLE_SLAVE 2
uint8_t u8DMX512Role;
uint16_t    u16DMX512TransChannel;
uint16_t    u16DMX512RecChannel;
uint8_t u8DMX512TransData[DMX512_CHANNEL_NUM + 1] ;
uint8_t  u8DMX512RevBuffer[2 * (DMX512_CHANNEL_NUM + 1)];
uint16_t u8DMX512RevBufferStart;

uint8_t u8KeyGet;
uint8_t  u87SegRam[2];
uint8_t  u87SegRamPosition = 0;
uint8_t u8DMX512SendTriggerEnable = 0;

uint32_t volatile TimerCounter = 0;
uint32_t TimerDisplayCounter = 0;

#define KEY1 PC13
#define KEY2 PE8
#define KEY3 PC15
#define KEY4 PC14

#define KEY_NO_KEY 0
#define KEY_MASTER 1
#define KEY_CHANNEL_SELECT  2
#define KEY_UP  3
#define KEY_DOWN 4

#define TRUE 1
#define FALSE 0

#define DMX512_SEGMENT_DISPLAY_CHANNEL_BASE  0//100

#define SYSTICK_TIMER 10000  //SysTick timer=10ms

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
//yhtang
    //SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk);
    //SYS->GPB_MFP |= SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD;


}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baud-rate */
    UART_Open(UART0, 250000);
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART1_RST);

    /* Configure UART0 and set UART0 Baud-rate */
    UART_Open(UART1, 115200);
}

//yhtang  add  for DMX512

__STATIC_INLINE void CLK_SysTickInit(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}



void SysTick_Handler(void)
{
    u32SysTimerCounter++;
}


void Initial_Key_Input(void)
{
    GPIO_SetMode(PC, BIT13, GPIO_PMD_INPUT);
    GPIO_SetMode(PE, BIT8,  GPIO_PMD_INPUT);
    GPIO_SetMode(PC, BIT14, GPIO_PMD_INPUT);
    GPIO_SetMode(PC, BIT15, GPIO_PMD_INPUT);
}




uint8_t Get_Key_Input(void)
{
    unsigned char temp = KEY_NO_KEY;
    if(KEY1 == 0)
        temp = KEY_MASTER;


    if(KEY2 == 0)
        temp = KEY_CHANNEL_SELECT;


    if(KEY3 == 0)
        temp = KEY_DOWN;


    if(KEY4 == 0)
        temp = KEY_UP;

    return   temp;
}



void TMR0_IRQHandler(void)
{
	  TimerCounter = TimerCounter+1;
    // clear Timer0 interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}


void Key_Process(uint8_t bKey)
{
    uint16_t    Temp;

    if(bKey == KEY_NO_KEY)
    {
        return;
    }

    switch(bKey)
    {
        case KEY_MASTER:
#if 0//def SUPPORT_DEBUGINFO
            printf("\n process key_master");
#endif
            if(u8DMX512Role != DMX512_ROLE_MASTER)
            {
                u8DMX512Role = DMX512_ROLE_MASTER;
                u16DMX512TransChannel = 1;
                u87SegRam[0] = SEG_CHAR_H;
                u87SegRam[1] = u16DMX512TransChannel;
                u8DMX512TransData[0] = DMX512_START_CODE;
                for(Temp = 1; Temp < (DMX512_CHANNEL_NUM + 1); Temp++)
                {
                    u8DMX512TransData[Temp] = Temp;
                }
                DMX512_SendInit();
                DMX512_SendData();
                u8DMX512SendTriggerEnable = 1;
                u32SysTimerDMX512SendCounter = u32SysTimerCounter;
                LED_On(u8DMX512TransData[u16DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
                //configure UART as transmitter
            }
            else
            {
                u8DMX512Role = DMX512_ROLE_SLAVE;
                u87SegRam[0] = SEG_CHAR_S;
                u87SegRam[1] = SEG_CHAR_NULL;
                u8DMX512SendTriggerEnable = 0;
                u16DMX512RecChannel = 1;
                //configure UART as receiver
                DMX512_ReceiveInit();
            }

            break;
        case   KEY_CHANNEL_SELECT:
#if 0 //def SUPPORT_DEBUGINFO
            printf("\n process KEY_CHANNEL_SELECT");
#endif
            if(u8DMX512Role == DMX512_ROLE_MASTER)
            {
                u16DMX512TransChannel++;
                if(u16DMX512TransChannel > 9)
                {
                    u16DMX512TransChannel = 1;
                }
                u87SegRam[0] = SEG_CHAR_H;
                u87SegRam[1] = u16DMX512TransChannel;
                LED_On(u8DMX512TransData[u16DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
            }
            else if(u8DMX512Role == DMX512_ROLE_SLAVE)
            {
                u16DMX512RecChannel++;
                if(u16DMX512RecChannel > 9)
                {
                    u16DMX512RecChannel = 1;
                }
                u87SegRam[0] = SEG_CHAR_S;
                u87SegRam[1] = u16DMX512RecChannel;
            }
            break;
        case KEY_UP:
#if 0//def SUPPORT_DEBUGINFO
            printf("\n process KEY_up");
#endif
            if(u8DMX512Role == DMX512_ROLE_MASTER)
            {
                u87SegRam[0] = SEG_CHAR_H;
                u87SegRam[1] = u16DMX512TransChannel;

                u8DMX512TransData[u16DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]++;
                LED_On(u8DMX512TransData[u16DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
            }
            break;
        case KEY_DOWN:
#if 0//def SUPPORT_DEBUGINFO
            printf("\n process KEY_down");
#endif
            if(u8DMX512Role == DMX512_ROLE_MASTER)
            {
                u87SegRam[0] = SEG_CHAR_H;
                u87SegRam[1] = u16DMX512TransChannel;
                u8DMX512TransData[u16DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]--;
                LED_On(u8DMX512TransData[u16DMX512TransChannel + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
            }
            break;
        default:
            break;

    }
}


int main(void)
{
    uint8_t u8KeyScan;
    uint8_t u8KeyScanPrev = KEY_NO_KEY;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
    UART1_Init();
#if 0//def SUPPORT_DEBUGINFO
    printf("\n DMX512 INIT BEGIN");
#endif

    Initial_Key_Input();
    Open_Seven_Segment();
    Initial_LED();
    //configure UART0 to do DMX512 communication,250kbps,8bit+2stop bit
    UART0_Init();
#ifdef SUPPORT_DEBUGINFO
    GPIO_FOR_SIGNAL_OSCILLOSCOPE_INITIAL;
    GPIO_FOR_SIGNAL_OSCILLOSCOPE_HIGH;
#endif
    //TIMER0 USED TO  do  7-SEGMENT displaying refresh
    //Enable Timer0 clock and select Timer0 clock source
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);
    //Initial Timer0 to periodic mode with 5000Hz=200us
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 5000);
    //Enable Timer0 interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    //Start Timer0
    TIMER_Start(TIMER0);

    //configure SysTick
    CLK_SysTickInit(SYSTICK_TIMER);//set SysTick time

    u8DMX512Role = DMX512_ROLE_NONE;
    u87SegRam[0] = 0;
    u87SegRam[1] = 0;
    u32SysTimerKeyCounter = u32SysTimerCounter;

    while(1)
    {
        if((u32SysTimerCounter - u32SysTimerKeyCounter) >= 20) //20*SYSTICK_TIMER
        {
            u32SysTimerKeyCounter = u32SysTimerCounter;
            u8KeyScan = Get_Key_Input();
            if(u8KeyScan == KEY_NO_KEY)
            {
                u8KeyGet = u8KeyScanPrev;
            }
            u8KeyScanPrev = u8KeyScan;
            if(u8KeyGet != KEY_NO_KEY)
            {
                Key_Process(u8KeyGet);
                u8KeyGet = KEY_NO_KEY;
            }
        }
        if(TimerDisplayCounter != TimerCounter)     //TIMER1 USED TO TRIGGER 7SEGMENT DISPLAY
        {
            TimerDisplayCounter = TimerCounter;
            u87SegRamPosition = ~u87SegRamPosition;
            if(u87SegRamPosition)
            {
                Show_Seven_Segment(u87SegRam[0], 1);
            }
            else
            {
                Show_Seven_Segment(u87SegRam[1], 2);
            }
        }
        if(u8DMX512SendTriggerEnable)
        {
            if((u32SysTimerCounter - u32SysTimerDMX512SendCounter) >= 10) //per 10*SYSTICK_TIMER resend data
            {
                u32SysTimerDMX512SendCounter = u32SysTimerCounter;
                DMX512_SendData();
            }
        }
        if(u8DMX512Role == DMX512_ROLE_SLAVE)
        {
            if((u32SysTimerCounter - u32SysTimerDMX512MTBF) > 100) //100*SYSTICK_TIMER
            {
                printf("\n DMX512 MTBF timer out !!!!!");
                RestartDMX512Receive();
            }
            if(u8DMX512ReceiveDone)
            {
                u8DMX512ReceiveDone = 0;
                u87SegRam[0] = SEG_CHAR_S;
                u87SegRam[1] = u16DMX512RecChannel;
                if(u16ReceiveBufferArea)
                {
                    u8DMX512RevBufferStart = 0;
                }
                else
                {
                    u8DMX512RevBufferStart = 513;
                }
                LED_On(u8DMX512RevBuffer[u16DMX512RecChannel + u8DMX512RevBufferStart + DMX512_SEGMENT_DISPLAY_CHANNEL_BASE]);
            }

        }
    }

}







