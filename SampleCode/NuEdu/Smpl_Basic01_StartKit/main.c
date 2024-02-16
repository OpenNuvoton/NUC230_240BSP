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
    int i, j, k;
    unsigned int temp;
    unsigned int MidDid;
    SYS_Init();
    Open_ADC_Knob();
    Initial_PWM_LED();
    Open_Buzzer();
    Initial_Key_Input();
    Open_Seven_Segment();
    UART0_Init();
    Open_SPI_Flash();


    /* Read MID & DID */
    MidDid = SpiFlash_ReadMidDid();
    printf("\nMID and DID = 0x%x", MidDid);
    k = 1;
    while(MidDid != 0xef14);

    I2C_EEPROM_Init();
    I2C_EEPROM_Write(0x0010, 0x55);
    temp = I2C_EEPROM_Read(0x0010);

    while(temp != 0x55);

    while(1)
    {
        if(PB14 != 0)
        {
            for(i = 0; i < 10; i++)
            {
                if(i < 9)
                //led test
                Write_LED_Bar(i);
                //adc test
                temp = Get_ADC_Knob();                  //Volume Range: 0 ~ 4095
                temp = temp * (100) / 4096;
                PWM_LED(0x07, 3000, temp, 3000, temp, 3000, temp);

                Show_Seven_Segment(i, k);
                for(j = 0; j < 100; j++)
                    CLK_SysTickDelay(1000);

            }
            k = k + 1;
            if(k == 3) k = 1;
            //buzzer test
            if(Get_Key_Input() == 0x01)
                Write_Buzzer(0, 0, 0);
            if(Get_Key_Input() == 0x02)
                Write_Buzzer(1, 1000, 50);
            if(Get_Key_Input() == 0x04)
                Write_Buzzer(1, 10000, 50);
            if(Get_Key_Input() == 0x08)
                Write_Buzzer(1, 50000, 50);
        }
    }
}
