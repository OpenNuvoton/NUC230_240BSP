
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/08/18 11:54a $
 * @brief    PWM sample for NANO100 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01.h"


int32_t main(void)
{
    int i, j;
    //SYS_Init();
    Initial_LED();
    while(1)
    {
        for(i = 9; i > 0; i--)
        {
            Write_LED_Bar(i);
            for(j = 0; j < 600; j++)
                CLK_SysTickDelay(1000);
        }
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


