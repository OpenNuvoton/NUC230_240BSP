/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/08/18 11:54a $
 * @brief    NUC200 Series LED Controller Sample Code
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
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
    while(1)
    {
        for(i = 0; i < 9; i++)
        {
            Write_LED_Bar(i);
            for(j = 0; j < 600; j++)
                CLK_SysTickDelay(1000);
        }
    }
}
