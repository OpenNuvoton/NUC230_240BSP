/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/09/04 2:22p $
 * @brief    NUC200 Series I2S Controller Sample Code
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
    int i, u32Data;
    SYS_Init();
    UART0_Init();
    printf("Hello World.\n");
    printf("PLL Clock = %d Hz\n", CLK_GetPLLClockFreq());
    printf("Core Clock = %d Hz\n\n", CLK_GetHCLKFreq());
    printf("+-------------------------------------------------------+\n");
    printf("|                  I2C Sample Code with EEPROM 24LC64   |\n");
    printf("+-------------------------------------------------------+\n");

    /* Initial I2C */
    I2C_EEPROM_Init();

    /* I2C EEPROM Write/Read test */
    for(i = 0; i < 2; i++)
    {
        printf("\n\nAddress = 0x0010, Write Data = %xh", (i * 2 + 3));
        I2C_EEPROM_Write(0x0010, (i * 2 + 3));

        u32Data = I2C_EEPROM_Read(0x0010);
        printf("\nAddress = 0x0010, Read Data = %xh", u32Data);
        if(u32Data != (i * 2 + 3))
        {
            printf("I2C Byte Write/Read Failed, Data 0x%x\n", u32Data);
            return -1;
        }
    }

    printf("\n\nI2C Access EEPROM Test OK\n");
	
	while(1);
}
