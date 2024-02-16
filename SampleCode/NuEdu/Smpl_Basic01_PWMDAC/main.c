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
    unsigned int temp, old_temp;
    SYS_Init();
    Initial_PWM_LED();
    Initial_PWM_DAC();
    Open_ADC_Knob();
    while(1)
    {
        //Get Volume Knob Data
        temp = Get_ADC_Knob();                 //Volume Range: 0 ~ 4095


        if(temp != old_temp)
        {
            Write_LED_Bar((temp * (8 + 1) / 4096));
            Write_PWMDAC(1, 2 + temp * (100) / 4096);
            old_temp = temp;
        }
    }
}
