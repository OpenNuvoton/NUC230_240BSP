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
    int i, j;
    SYS_Init();
    Open_Seven_Segment();
    while(1)
    {
        for(i = 0; i < 10; i++)
        {
            Show_Seven_Segment(i, 1);
            for(j = 0; j < 500; j++)
                CLK_SysTickDelay(1000);
        }
        for(i = 0; i < 10; i++)
        {
            Show_Seven_Segment(i, 2);
            for(j = 0; j < 500; j++)
                CLK_SysTickDelay(1000);
        }
    }
}
