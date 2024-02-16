/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/08/18 11:54a $
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

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int temp;
    SYS_Init();
    Initial_Timer_Port();
    Initial_Timer2_Count();
    UART0_Init();
    while(1)
    {
        if(temp != Get_Timer2_Count())
        {
            temp = Get_Timer2_Count();
            printf("temp=%d\n\r", temp);
        }

    }
}
