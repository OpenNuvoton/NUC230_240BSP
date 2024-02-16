/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/09/02 11:55a $
 * @brief    NUC200 Series I2S Controller Sample Code
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01.h"
uint32_t u32LEDEanble;
void IrDA_Code_Exe(uint8_t* IR_CODE1)
{
    if((IR_CODE1[0] == 0x00) & (IR_CODE1[1] == 0xFF))
    {
        if((IR_CODE1[2] == 0x10) & (IR_CODE1[3] == 0xEF))
        {
            LED_On(++u32LEDEanble);
        }
        else if((IR_CODE1[2] == 0x14) & (IR_CODE1[3] == 0xEB))
        {
            LED_On(--u32LEDEanble);
        }
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint8_t au8IR_CODE[4];

    SYS_Init();
    UART0_Init();
    printf("IrDA TEST\n\r");
    Initial_Key_Input();
//      IrDA_NEC_Rx_Init();

//      IrDA_NEC_Tx_Init();
    IrDA_NEC_TxRx_Init();
    printf("+-----------------------------------------+\n");
    printf("|             IrDA NEC Sample Code      |\n");
    printf("+-----------------------------------------+\n");

    au8IR_CODE[0] = 0x00;
    au8IR_CODE[1] = ~au8IR_CODE[0];

    while(1)
    {
        /* Detect Key status */

        if(Get_Key_Input() & 0x01)
        {
            au8IR_CODE[2] = 0x10;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }
        if(Get_Key_Input() & 0x02)
        {
            au8IR_CODE[2] = 0x14;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }
    }
}
