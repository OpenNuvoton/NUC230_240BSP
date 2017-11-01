/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 4 $
 * $Date: 15/05/08 2:53p $
 * @brief    Smartcard UART mode demo for NUC230_240 series MCU
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"

#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t au8TxBuf[] = "Hello World!";

/*---------------------------------------------------------------------------------------------------------*/
/* This function use to enable smartcard module UART mode and set baudrate.                                */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate)
{
    uint32_t u32ClkSrc, u32Clk, u32Div;

    if(sc == SC0)
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC0_S_Msk) >> CLK_CLKSEL3_SC0_S_Pos;
    else if(sc == SC1)
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC1_S_Msk) >> CLK_CLKSEL3_SC1_S_Pos;
    else // SC2
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC2_S_Msk) >> CLK_CLKSEL3_SC2_S_Pos;

    // Get smartcard module clock
    if(u32ClkSrc == 0)
        u32Clk = __HXT;
    else if(u32ClkSrc == 1)
        u32Clk = CLK_GetPLLClockFreq();
    else
        u32Clk = __HIRC;

    if(sc == SC0)
        u32Clk /= (CLK->CLKDIV1 & CLK_CLKDIV1_SC0_N_Msk);
    else if(sc == SC1)
        u32Clk /= ((CLK->CLKDIV1 & CLK_CLKDIV1_SC1_N_Msk) >> CLK_CLKDIV1_SC1_N_Pos);
    else // SC2
        u32Clk /= ((CLK->CLKDIV1 & CLK_CLKDIV1_SC2_N_Msk) >> CLK_CLKDIV1_SC2_N_Pos);

    // Calculate divider for target baudrate
    u32Div = (u32Clk + (u32baudrate >> 1) - 1) / u32baudrate - 1;

    sc->CTL = SC_CTL_SC_CEN_Msk | SC_CTL_SLEN_Msk;  // Enable smartcard interface and stop bit = 1
    sc->UACTL = SCUART_CHAR_LEN_8 | SCUART_PARITY_NONE | SC_UACTL_UA_MODE_EN_Msk; // Enable UART mode, disable parity and 8 bit per character
    sc->ETUCR = u32Div;

    return(u32Clk / u32Div);
}

/*---------------------------------------------------------------------------------------------------------*/
/* This function is to write data into transmit FIFO to send data out.                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SCUART_Write(SC_T* sc, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;

    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++) {
        while(SCUART_GET_TX_FULL(sc));  // Wait 'til FIFO not full
        sc->THR = pu8TxBuf[u32Count];    // Write 1 byte to FIFO
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief       IRQ Handler for SmartCard 0/1/2 Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The SC012_IRQHandler is default IRQ of SmartCard, declared in startup_NUC230_240.s.
 */
void SC012_IRQHandler(void)
{
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    UART_WRITE(UART0, SCUART_READ(SC0));

    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12 MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    
    /* System optimization when CPU runs at 72MHz */
    FMC->FATCON |= 0x50;

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable UART and module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Enable SC0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_SC0_EN_Msk ;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_PLL;

    /* Select SC0 module clock source */
    CLK->CLKSEL3 &= ~CLK_CLKSEL3_SC0_S_Msk ;
    CLK->CLKSEL3 |= CLK_CLKSEL3_SC0_S_HXT ;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
    
    /* Set PA multi-function pins for SC UART mode */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA2_Msk | SYS_GPA_MFP_PA3_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA2_SC0_CLK | SYS_GPA_MFP_PA3_SC0_DAT;
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PA2_Msk | SYS_ALT_MFP1_PA3_Msk);
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PA2_SC0_CLK | SYS_ALT_MFP1_PA3_SC0_DAT;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("This sample code demos smartcard interface UART mode\n");
    printf("Please connect SC0 CLK pin(PA.2) as TX pin with SC0 DAT pin(PA.3) as RX pin\n");
    printf("Hit any key to continue\n");
    getchar();

    // Open smartcard interface 0 in UART mode.
    SCUART_Open(SC0, 115200);
    
    // Enable receive interrupt
    SCUART_ENABLE_INT(SC0, SC_IER_RDA_IE_Msk);
    NVIC_EnableIRQ(SC012_IRQn);

    SCUART_Write(SC0, au8TxBuf, sizeof(au8TxBuf));

    while(1);
}
