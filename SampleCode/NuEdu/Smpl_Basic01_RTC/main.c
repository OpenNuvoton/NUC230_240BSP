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
    S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;
    unsigned int temp;
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 2;
    sWriteRTC.u32Day        = 6;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 30;
    sWriteRTC.u32Second     = 55;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
    SYS_Init();
    UART0_Init();
    printf("RTC TEST\n\r");

    RTC_Init();
    RTC_SET(sWriteRTC);

    while(1)
    {
        RTC_GetDateAndTime(&sReadRTC);
        if(sReadRTC.u32Second != temp)
        {
            printf("    %d/%02d/%02d %02d:%02d:%02d\n",
                   sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);
            temp = sReadRTC.u32Second;

        }
    }

}
