/*
  * FreeModbus Libary: LPC214X Port
  * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

//#include <LPC214X.h>
#include <NUC230_240.h>
#include "port.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


static void     prvvUARTTxReadyISR(void);
static void     prvvUARTRxISR(void);

/* ----------------------- Start implementation -----------------------------*/
void  vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    __disable_irq();

    if(xRxEnable)
    {
        UART_ENABLE_INT(UART0, UART_IER_RDA_IEN_Msk);
    }
    else
    {
        UART_DISABLE_INT(UART0, UART_IER_RDA_IEN_Msk);
    }
    if(xTxEnable)
    {
        UART_ENABLE_INT(UART0, UART_IER_THRE_IEN_Msk);
    }
    else
    {
        UART_DISABLE_INT(UART0, UART_IER_THRE_IEN_Msk);
    }

    NVIC_EnableIRQ(UART02_IRQn);
    __enable_irq();
}

void
vMBPortClose(void)
{
    /* Disable Interrupt */
    UART_DISABLE_INT(UART0, UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk);
    NVIC_DisableIRQ(UART02_IRQn);
}

BOOL
xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
    if(ucPORT != 0) return FALSE;

    /* Step 1. GPIO initial */
    /* Set PB multi-function pins for UART0 RXD, TXD */
    /* PB.0 --> UART0 RX, PB.1 --> UART0 TX */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Step 2. Enable and Select UART clock source*/
    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* UART0 clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_HIRC;

    /* Step 3. Select Operation mode */
    SYS_ResetModule(UART0_RST);
    UART0->FCR |= 1 << 2;       //Tx FIFO Reset
    UART0->FCR |= 1 << 1;       //Rx FIFO Reset

    /* Step 4. Set Baud-Rate to 38400*/
    /* Configure UART0 and set UART0 Baud-rate: Baud-rate 38400, 8 bits length, None parity , 1 stop bit*/
    UART_Open(UART0, ulBaudRate);

    switch(eParity)
    {
        case MB_PAR_NONE:
            UART0->LCR &= ~UART_PARITY_EVEN;
            break;
        case MB_PAR_EVEN:
            UART0->LCR &= ~UART_PARITY_EVEN;
            UART0->LCR |= UART_PARITY_EVEN;
            break;
        case MB_PAR_ODD:
            UART0->LCR &= ~UART_PARITY_EVEN;
            UART0->LCR |= UART_PARITY_ODD;
            break;
    }

    UART0->LCR &= ~UART_LCR_WLS_Msk;
    switch(ucDataBits)
    {
        case 5:
            UART0->LCR |= UART_WORD_LEN_5;      //5 bits Data Length
            break;
        case 6:
            UART0->LCR |= UART_WORD_LEN_6;      //6 bits Data Length
            break;
        case 7:
            UART0->LCR |= UART_WORD_LEN_7;      //7 bits Data Length
            break;
        case 8:
            UART0->LCR |= UART_WORD_LEN_8;      //8 bits Data Length
            break;
    }

    return TRUE;

}

BOOL
xMBPortSerialPutByte(CHAR ucByte)
{

    UART0->THR = (uint8_t) ucByte;
    while(UART_GET_TX_EMPTY(UART0) != 0x00); //check Tx Empty

    return TRUE;
}

BOOL
xMBPortSerialGetByte(CHAR * pucByte)
{

    while(UART_GET_RX_EMPTY(UART0) != 0x00); //check Rx Empty
    *pucByte = UART0->RBR;
    return TRUE;
}


void UART02_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_ISR_RDA_INT_Msk))
    {

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0))
        {
            prvvUARTRxISR();
        }

    }
    else if(UART_GET_INT_FLAG(UART0, UART_ISR_THRE_INT_Msk))
    {

        prvvUARTTxReadyISR();

    }
}



/*
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
static void
prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/*
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void
prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}
