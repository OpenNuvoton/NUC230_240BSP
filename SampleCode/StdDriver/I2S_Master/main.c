/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/04/16 2:17p $
 * @brief    NUC230_240 Series I2S Driver Sample Code
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC230_240.h"

uint32_t g_u32TxValue;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for 12 MHz clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock rate as 72 MHz from PLL */
    CLK_SetCoreClock(72000000);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2S_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set GPA multi-function pins for I2S MCLK. */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA15_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA15_I2S_MCLK;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA15_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PA15_I2S_MCLK;
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PA15_Msk);
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PA15_I2S_MCLK;

    /* Set multi function pin for I2S: GPC0, GPC1, GPC2, GPC3 */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk | SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC2_Msk | SYS_GPC_MFP_PC3_Msk);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_I2S_LRCLK | SYS_GPC_MFP_PC1_I2S_BCLK | SYS_GPC_MFP_PC2_I2S_DI | SYS_GPC_MFP_PC3_I2S_DO;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC0_Msk | SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC2_Msk | SYS_ALT_MFP_PC3_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PC0_I2S_LRCLK | SYS_ALT_MFP_PC1_I2S_BCLK | SYS_ALT_MFP_PC2_I2S_DI | SYS_ALT_MFP_PC3_I2S_DO;
}

void I2S_IRQHandler()
{
    /* Write 4 Tx values to TX FIFO */
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32DataCount, u32RxValue1, u32RxValue2;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (master mode)           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x55005501, 0x55025503, ..., 0x55FE55FF, wraparound\n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PC0)\n      I2S_BCLK(PC1)\n");
    printf("      I2S_DI (PC2)\n      I2S_DO (PC3)\n\n");
    printf("  NOTE: Connect with a I2S slave device.\n");
    printf("        This sample code will transmit a TX value 50000 times, and then change to the next TX value.\n");
    printf("        When TX value or the received value changes, the new TX value or the current TX value and the new received value will be printed.\n");

    /* select source from HXT (12 MHz) */
    CLK_SetModuleClock(I2S_MODULE, CLK_CLKSEL2_I2S_S_HXT, 0);

    /* Master mode, 16-bit word width, stereo mode, I2S format. Set TX and RX FIFO threshold to middle value. */
    I2S_Open(I2S, I2S_MODE_MASTER, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);
    NVIC_EnableIRQ(I2S_IRQn);

    /* Initiate data counter */
    u32DataCount = 0;
    /* Initiate Tx value and Rx value */
    g_u32TxValue = 0x55005501;
    u32RxValue1 = 0;
    u32RxValue2 = 0;
    /* Enable Tx threshold level interrupt */
    I2S_EnableInt(I2S, I2S_IE_TXTHIE_Msk);

    printf("Start I2S ...\nTx value: 0x%X\n", g_u32TxValue);

    while(1) {
        if((I2S->STATUS & I2S_STATUS_RXEMPTY_Msk) == 0) {
            u32DataCount++;
            u32RxValue2 = I2S_READ_RX_FIFO(I2S);
            if(u32RxValue1 != u32RxValue2) {
                u32RxValue1 = u32RxValue2;
                printf("Tx value: 0x%X;  Rx value: 0x%X\n", g_u32TxValue, u32RxValue1);
            }
            if(u32DataCount >= 50000) {
                g_u32TxValue = 0x55005500 | ((g_u32TxValue + 0x00020002) & 0x00FF00FF); /* g_u32TxValue: 0x55005501, 0x55025503, ..., 0x55FE55FF */
                printf("Tx value: 0x%X\n", g_u32TxValue);
                u32DataCount = 0;
            }
        }
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
