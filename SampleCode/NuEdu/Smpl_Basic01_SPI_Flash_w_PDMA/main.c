/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/08/18 11:54a $
 * @brief    NuEdu-SDK-NUC240 SPI_Flash_w_PDMA Sample Code
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01.h"

#define TEST_NUMBER                 10 /* page numbers */
#define CH1                     1
#define CH2                     2

unsigned char   SrcArray[256];
unsigned char DestArray[256];
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    unsigned int u32ByteCount;
    unsigned int u32PageNumber;
    unsigned int u32ProgramFlashAddress = 0;
    unsigned int u32VerifyFlashAddress = 0;
    unsigned int MidDid;

    PDMA_T *PDMA_CH1, *PDMA_CH2;

    // PDMA Channel 1/2 control registers
    PDMA_CH1 = (PDMA_T *)((uint32_t) PDMA1_BASE);
    PDMA_CH2 = (PDMA_T *)((uint32_t) PDMA2_BASE);

    /* Initial system */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    printf("Hello World.\n");
    printf("PLL Clock = %d Hz\n", CLK_GetPLLClockFreq());
    printf("Core Clock = %d Hz\n\n", CLK_GetHCLKFreq());
    printf("+-------------------------------------------------------+\n");
    printf("|    NUC230/240 Series SPI_Flash_w_PDMA Sample Code     |\n");
    printf("+-------------------------------------------------------+\n");

    /* Open 7-Seg */
    Open_Seven_Segment();

    /* Open SPI for Serial Flash */
    Open_SPI_Flash();

    /* Initial PDMA Channels */
    Init_PDMA_CH1_for_SPI2_TX((uint32_t)SrcArray);
    Init_PDMA_CH2_for_SPI2_RX((uint32_t)DestArray);

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Read MID & DID */
    MidDid = SpiFlash_w_PDMA_ReadMidDid();
    printf("\nMID and DID = %x", MidDid);

    /* Erase SPI Flash */
    SpiFlash_w_PDMA_ChipErase();
    printf("\nFlash Erasing... ");

    /* Wait ready */
    SpiFlash_w_PDMA_WaitReady();
    printf("Done!");

    /* Fill the Source Data and clear Destination Data Buffer */
    for(u32ByteCount = 0; u32ByteCount < 256; u32ByteCount++)
    {
        SrcArray[u32ByteCount] = u32ByteCount;
        DestArray[u32ByteCount] = 0;
    }

    u32ProgramFlashAddress = 0;
    u32VerifyFlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        printf("\n\nTest Page Number = %d", u32PageNumber);
        Show_Seven_Segment(u32PageNumber, 1);
        CLK_SysTickDelay(200000);

        /*=== Program SPI Flash ===*/
        printf("\n Flash Programming... ");

        /* Trigger PDMA specified Channel */
        PDMA_CH1->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);

        /* Page Program */
        SpiFlash_w_PDMA_PageProgram(u32ProgramFlashAddress, 256);
        SpiFlash_w_PDMA_WaitReady();
        u32ProgramFlashAddress += 0x100;
        printf("Done!");

        /*=== Read Back and Compare Data ===*/
        printf("\n Flash Verifying... ");

        /* Trigger PDMA specified Channel */
        PDMA_CH2->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);

        /* Page Read */
        SpiFlash_w_PDMA_ReadData(u32VerifyFlashAddress, 256);
        u32VerifyFlashAddress += 0x100;

        for(u32ByteCount = 0; u32ByteCount < 256; u32ByteCount++)
        {
            //printf("\nByteCount:%d, %d\n", u32ByteCount, DestArray[u32ByteCount]);
            if(DestArray[u32ByteCount] != u32ByteCount)
            {
                /* Error */
                printf("SPI Flash R/W Fail!");
                while(1);
            }
        }

        /* Clear Destination Data Buffer */
        for(u32ByteCount = 0; u32ByteCount < 256; u32ByteCount++)
            DestArray[u32ByteCount] = 0;
        printf("Done!");
    }

    printf("\n\nSPI Flash Test Ok!");
    printf("\n\n");

    while(1);

}
