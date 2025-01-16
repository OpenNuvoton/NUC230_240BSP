#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_SPI_Flash.h"


void Initial_SPI2_GPIO(void)
{
    /* Set PD0, PD1, PD2 and PD3 for SPI2 */
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD0_Msk | SYS_GPD_MFP_PD1_Msk | SYS_GPD_MFP_PD2_Msk | SYS_GPD_MFP_PD3_Msk);
    SYS->GPD_MFP |= SYS_GPD_MFP_PD0_SPI2_SS0 | SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;

}

void Open_SPI_Flash(void)
{

    /* Init GPIO for SPI2 */
    Initial_SPI2_GPIO();

    /* Enable SPI2 IP clock */
    CLK->APBCLK |= CLK_APBCLK_SPI2_EN_Msk;

    /* Configure SPI2 as a master, MSB first, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    //SPI2->CNTRL = SPI_CNTRL_MASTER_MODE | SPI_CNTRL_MSB_FIRST | SPI_CNTRL_CLK_IDLE_LOW | SPI_CNTRL_TX_FALLING |
    //            SPI_CNTRL_RX_RISING | SPI_CNTRL_TX_BIT_LEN(32);

    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_0, 32, 2000000);
    /* Disable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_DisableAutoSS(SPI2);
    SPI_SET_SS0_HIGH(SPI2);

}

// **************************************
// For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x4015
unsigned int SpiFlash_ReadJedecID(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 8 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);

    // send Command: 0x9F, Read JEDEC ID
    au32SourceData = 0x9F;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI2, 24);
    // receive
    au32SourceData = 0x0;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
    // dump Rx register
    au32DestinationData = SPI2->RX[0];

    return (au32DestinationData & 0xFFFFFF);

}

// **************************************
void SpiFlash_ChipErase(void)
{
    unsigned int au32SourceData;

    // configure transaction length as 8 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI2, 8);
    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);
    // send Command: 0xC7, Chip Erase
    au32SourceData = 0xC7;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
}

// **************************************
unsigned int SpiFlash_ReadStatusReg1(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 16 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(16);
    SPI_SET_DATA_WIDTH(SPI2, 16);
    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);
    // send Command: 0x05, Read status register 1
    au32SourceData = 0x0500;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
    // dump Rx register
    au32DestinationData = SPI2->RX[0];

    return (au32DestinationData & 0xFF);

}


// **************************************
unsigned int SpiFlash_ReadStatusReg2(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 16 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(16);
    SPI_SET_DATA_WIDTH(SPI2, 16);
    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);
    // send Command: 0x35, Read status register 2
    au32SourceData = 0x3500;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
    // dump Rx register
    au32DestinationData = SPI2->RX[0];

    return (au32DestinationData & 0xFF);

}


// **************************************
void SpiFlash_WaitReady(void)
{
    unsigned int ReturnValue;

    do
    {
        ReturnValue = SpiFlash_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    }
    while(ReturnValue != 0); // check the BUSY bit

}

// **************************************
void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int Counter;

    // configure transaction length as 8 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI2, 8);
    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);
    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);
    // send Command: 0x02, Page program
    au32SourceData = 0x02;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI2, 24);
    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI2, 8);
    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // send   data to program
        au32SourceData = DataBuffer[Counter];
        SPI2->TX[0] = au32SourceData;
        SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

        // wait
        while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}
    }

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
}


// **************************************
void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;
    unsigned int Counter;

    // configure transaction length as 8 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI2, 8);
    // /CS: active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI2);

    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI2, 24);
    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI2, 8);

    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // receive
        au32SourceData = 0x0;
        SPI2->TX[0] = au32SourceData;
        SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

        // wait
        while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

        // dump Rx register
        au32DestinationData = SPI2->RX[0];
        DataBuffer[Counter] = (unsigned char) au32DestinationData;
    }

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
}


