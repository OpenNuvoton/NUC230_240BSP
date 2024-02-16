/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
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

    SYS_Init();
    Open_Buzzer();
    Initial_Key_Input();
    while(1)
    {
        if(Get_Key_Input() == 0x01)
            Write_Buzzer(0, 0, 0);
        if(Get_Key_Input() == 0x02)
            Write_Buzzer(1, 1000, 50);
        if(Get_Key_Input() == 0x04)
            Write_Buzzer(1, 10000, 50);
        if(Get_Key_Input() == 0x08)
            Write_Buzzer(1, 100000, 50);
    }
}
