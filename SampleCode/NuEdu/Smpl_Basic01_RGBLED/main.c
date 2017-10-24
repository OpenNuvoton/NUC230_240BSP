/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/08/18 11:54a $
 * @brief    NUC200 Series RGBLED Controller Sample Code
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
    unsigned int temp;
    SYS_Init();
    Initial_PWM_LED();

//Open ADC Device
    Open_ADC_Knob();
    while(1)
    {
        //Get Volume Knob Data
        temp = Get_ADC_Knob();                  //Volume Range: 0 ~ 4095
        temp = temp * (100 + 1) / 4096;
        PWM_LED(0x01, 3000, temp, 3000, temp, 3000, temp);
    }
}
