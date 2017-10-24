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
 * File: $Id: porttimer.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
//#include <LPC214X.h>
#include <NUC230_240.h>
#include "port.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"



/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit(USHORT usTim1Timerout50us)
{
    /* Enable Timer0 clock source */
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Select Timer0 clock source as external 12M */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0);

    /* Reset IP TMR0 */
    SYS_ResetModule(TMR0_RST);

    /* Select timer0 Operation mode as period mode */
    TIMER0->TCSR &= ~TIMER_TCSR_MODE_Msk;
    TIMER0->TCSR |= TIMER_PERIODIC_MODE;

    /* Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);        // Set Prescale [0~255]
    TIMER_SET_CMP_VALUE(TIMER0, 12 * 50 * usTim1Timerout50us);  // Set TCMPR [0~16777215]

    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);
    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    return TRUE;
}


void
vMBPortTimersEnable()
{
    /* Reset Timer0 counter */
    TIMER0->TCSR |= TIMER_TCSR_CRST_Msk;
    /* Enable Timer0 */
    TIMER0->TCSR |= TIMER_TCSR_CEN_Msk;
}

void
vMBPortTimersDisable()
{

    /* Disable Timer0 */
    TIMER0->TCSR &= ~TIMER_TCSR_CEN_Msk;
//      /* Reset Timer0 counter */
    TIMER0->TCSR |= TIMER_TCSR_CRST_Msk;

}

void TMR0_IRQHandler(void)
{
    /* Clear Timer0 interrupt flag */
    TIMER0->TISR |= TIMER_TISR_TIF_Msk;

    (void)pxMBPortCBTimerExpired();
}
