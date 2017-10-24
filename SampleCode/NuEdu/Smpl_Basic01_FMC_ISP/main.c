/******************************************************************************
 * @file     sample_ld.c
 * @brief    Nano1xx FMC Driver Sample code
 * @version  1.0.1
 * @date     04, September, 2012
 *
 * @note
 * Copyright (C) 2012-2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01.h"
#define LDROM_BASE          0x00100000
#define LDROM_SIZE          0x10000
#define PAGE_SIZE           512
extern uint32_t loaderImageBase;
extern uint32_t loaderImageLimit;

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t  i32Err;
    uint32_t u32Data, i, u32ImageSize, j, *pu32Loader;
    SYS_Init();

    UART0_Init();
    Initial_LED();

    /* Unlock protected registers */
    SYS_UnlockReg();


    /* Enable FMC ISP function */
    FMC_Open();



    /* Enable LDROM update */
    FMC_EnableLDUpdate();

    printf("  Erase LD ROM ............................... ");
    /* Page Erase LDROM */
    for(i = 0; i < 4096; i += PAGE_SIZE)
        FMC_Erase(LDROM_BASE + i);

    /* Erase Verify */
    i32Err = 0;
    for(i = LDROM_BASE; i < (LDROM_BASE + 4096); i += 4)
    {
        u32Data = FMC_Read(i);
        if(u32Data != 0xFFFFFFFF)
        {
            printf(" u32Data = 0x%x\n", u32Data);
            i32Err = 1;
        }
    }
    if(i32Err)
        printf("[FAIL]\n");
    else
        printf("[OK]\n");


    printf("  Program LD ROM test ........................ ");

    /* Program LD ROM and read out data to compare it */
    for(i = LDROM_BASE; i < (LDROM_BASE + 4096); i += 4)
    {
        FMC_Write(i, i);
    }

    i32Err = 0;
    for(i = LDROM_BASE; i < (LDROM_BASE + 4096); i += 4)
    {
        u32Data = FMC_Read(i);
        if(u32Data != i)
        {
            i32Err = 1;
        }
    }
    if(i32Err)
        printf("[FAIL]\n");
    else
        printf("[OK]\n");


    /* Check LD image size */
    u32ImageSize = (uint32_t)&loaderImageLimit - (uint32_t)&loaderImageBase;
    if(u32ImageSize == 0)
    {
        printf("  ERROR: Loader Image is 0 bytes!\n");
        goto lexit;
    }

    if(u32ImageSize > 4096)
    {
        printf("  ERROR: Loader Image is larger than 4KBytes!\n");
        goto lexit;
    }


    printf("  Program Simple LD Code ..................... ");
    pu32Loader = (uint32_t *)&loaderImageBase;
    for(i = 0; i < u32ImageSize; i += PAGE_SIZE)
    {
        FMC_Erase(LDROM_BASE + i);
        for(j = 0; j < PAGE_SIZE; j += 4)
        {
            FMC_Write(LDROM_BASE + i + j, pu32Loader[(i + j) / 4]);
        }
    }

    /* Verify loader */
    i32Err = 0;
    for(i = 0; i < u32ImageSize; i += PAGE_SIZE)
    {
        for(j = 0; j < PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(LDROM_BASE + i + j);
            if(u32Data != pu32Loader[(i + j) / 4])
                i32Err = 1;

            if(i + j >= u32ImageSize)
                break;
        }
    }

    if(i32Err)
    {
        printf("[FAIL]\n");
    }
    else
    {
        printf("[OK]\n");

        /* Reset CPU to boot to LD mode */
        printf("\n  >>> Reset to LD mode <<<\n");
        while(1)
        {
            i++;
            if(i > 8) i = 1;
            for(j = 0; j < 10; j++)
                CLK_SysTickDelay(100000);
            LED_On(1 << i);

            if(PB14 == 0)
            {

                FMC_SetBootSource(1);

                // do chip reset
                SYS_ResetCPU();
            }
        }
    }


lexit:

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");
}

/*** (C) COPYRIGHT 2012 Nuvoton Technology Corp. ***/


