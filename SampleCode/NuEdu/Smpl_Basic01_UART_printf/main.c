/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/09/02 3:49p $
 * @brief    NuEdu Basic01 UART printf Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdarg.h>

#include "NUC230_240.h"
#include "NuEdu-Basic01.h"
void SendChar_ToUART(int ch);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
#ifdef DEBUG_ENABLE_UART1
    /* Set PB.4 and PB.5 multi-function pins for UART1 RXD, UART1 TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD;
#else
    /* Set PB.0 and PB.1 multi-function pins for UART0 RXD, UART0 TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
#endif

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple printf() function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void printf_UART(uint8_t *str, ...);
void printInteger(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i = 15, j;

    *(print_buf + i) = '\0';
    j = u32Temp >> 31;
    if(j)
        u32Temp = ~u32Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + u32Temp % 10;
        u32Temp = u32Temp / 10;
    }
    while(u32Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    printf_UART(print_buf + i);
}
void printHex(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i = 15;
    uint32_t temp;

    *(print_buf + i) = '\0';
    do
    {
        i--;
        temp = u32Temp % 16;
        if(temp < 10)
            *(print_buf + i) = '0' + temp;
        else
            *(print_buf + i) = 'a' + (temp - 10) ;
        u32Temp = u32Temp / 16;
    }
    while(u32Temp != 0);
    printf_UART(print_buf + i);
}
void printf_UART(uint8_t *str, ...)
{
    va_list args;
    va_start(args, str);
    while(*str != '\0')
    {
        if(*str == '%')
        {
            str++;
            if(*str == '\0') return;
            if(*str == 'd')
            {
                str++;
                printInteger(va_arg(args, int));
            }
            else if(*str == 'x')
            {
                str++;
                printHex(va_arg(args, int));
            }
        }
        SendChar_ToUART(*str++);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    uint32_t u32Key, i = 0;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Key and LED GPIO type */
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);
    Initial_Key_Input();
    Initial_LED();

    /* Init UART to 115200-8n1 for print message */
#ifdef DEBUG_ENABLE_UART1
    UART_Open(UART1, 115200);
#else
    UART_Open(UART0, 115200);
#endif

    printf("+-----------------------------------------+\n");
    printf("|    NUC240 UART Sample Code      |\n");
    printf("+-----------------------------------------+\n");

    while(1)
    {
        /* Detect Key status */
        u32Key = Get_Key_Input();
        if(PB14 == 0)
        {
            LED_On(i);
            printf("+----------------------------------+\n");
            printf("|    Standard printf function:%d   |\n", i++);
            printf("+----------------------------------+\n");
        }
        if(u32Key & 0x01)
        {
            LED_On(i);
            printf_UART("+------------------------------+\n");
            printf_UART("|  Simple printf function:%d   |\n", i--);
            printf_UART("+------------------------------+\n");
        }
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
