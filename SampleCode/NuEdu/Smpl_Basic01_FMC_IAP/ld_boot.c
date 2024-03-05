/**************************************************************************//**
 * @file     ld_boot.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/09/02 11:56a $
 * @brief    FMC VECMAP sample program (LDROM code)
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include "NUC230_240.h"
#include "map.h"
#include "NuEdu-Basic01.h"

__STATIC_INLINE void BranchTo(uint32_t u32Address)
{
    FUNC_PTR        *func;
    FMC_SetVectorPageAddr(u32Address);
    func = (FUNC_PTR *)(*(uint32_t *)(u32Address + 4));
    printf("branch to address 0x%x\n", (int)func);
    printf("\n\nChange VECMAP and branch to user application...\n");
    while(!(UART0->FSR & UART_FSR_TX_EMPTY_Msk));
    __set_MSP(*(uint32_t *)u32Address);
    func();
}
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

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

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t        au32Version[2];

    volatile int    loop;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

    printf("\n\n\n");
    printf("+---------------------------------------------------+\n");
    printf("|       Boot loader program running on LDROM        |\n");
    printf("+---------------------------------------------------+\n");

    au32Version[0] = FMC_Read(USER_AP0_ENTRY + 0x1000);
    au32Version[1] = FMC_Read(USER_AP1_ENTRY + 0x1000);

    printf("|               APROM Version Check                 |\n");
    printf("+---------------------------------------------------|\n");
    printf("|        Version for Application No.0:0x%x          |\n", au32Version[0]);
    printf("|        Version for Application No.1:0x%x          |\n", au32Version[1]);
    printf("+---------------------------------------------------|\n");
    printf("|                  Boot Selection                   |\n");
    if((au32Version[0] >= au32Version[1]) & (au32Version[0] != 0xFFFFFFFF))
    {
        printf("|AP0 has latest program and then system jumps to AP0|\n");
        BranchTo(USER_AP0_ENTRY);
    }
    else if(au32Version[1] != 0xFFFFFFFF)
    {
        printf("|AP1 has latest program and then system jumps to AP1|\n");
        BranchTo(USER_AP1_ENTRY);
    }
    if((au32Version[0] <= au32Version[1]) & (au32Version[1] != 0xFFFFFFFF))
    {
        printf("|AP1 has latest program and then system jumps to AP1|\n");
        BranchTo(USER_AP1_ENTRY);
    }
    else if(au32Version[0] != 0xFFFFFFFF)
    {
        printf("|AP0 has latest program and then system jumps to AP0|\n");
        printf("+---------------------------------------------------+\n");
        BranchTo(USER_AP0_ENTRY);
    }
    printf("| Don't find any program on APROM                   |\n");
    printf("| Please implement ISP function to update APROM     |\n");
    printf("+---------------------------------------------------+\n");
    while(1);
}



