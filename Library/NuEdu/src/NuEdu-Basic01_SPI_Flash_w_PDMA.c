#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_SPI_Flash_w_PDMA.h"

#define SPI_FLASH_PORT  SPI2

#define TEST_NUMBER         1   /* page numbers */
#define TEST_LENGTH         256 /* length */
#define CH1                     1
#define CH2                     2

#define MODE_PER2MEM        1
#define MODE_MEM2PER        2

volatile    uint32_t    PDMA_CH1_INT_Flag;
volatile    uint32_t    PDMA_CH2_INT_Flag;


void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GCR->GCRISR;

    /* CH1 */
    if(status & 0x2)
    {
        if(PDMA_GET_CH_INT_STS(1) & 0x2)
        {
            PDMA_CH1_INT_Flag = 1;
            PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_BLKD_IF_Msk);
        }
        /* CH2 */
    }
    else if(status & 0x4)
    {
        if(PDMA_GET_CH_INT_STS(2) & 0x2)
        {
            PDMA_CH2_INT_Flag = 2;
            PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_BLKD_IF_Msk);
        }
    }

}

void Initial_SPI2_GPIO(void)
{
    /* Set PD0, PD1, PD2 and PD3 for SPI2 */
    SYS->GPD_MFP |= SYS_GPD_MFP_PD0_SPI2_SS0 | SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;

}

// **************************************
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
void Init_PDMA_CH1_for_SPI2_TX(uint32_t u32SrcAddr)
{
    uint32_t SPI2_TX;
    PDMA_T *PDMA_CH1;

    // PDMA Channel 1 control registers
    PDMA_CH1 = (PDMA_T *)((uint32_t) PDMA1_BASE);

    // SPI2 TX register
    SPI2_TX = SPI2_BASE + 0x20;

    // Enable DMA IP clock
    CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;

    // Enable Channel 1 clock
    PDMA_GCR->GCRCSR |= ((1 << CH1) << 8);

    // Set Channel 1 for SPI2_TX
    PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI2_TXSEL_Msk) | (1 << PDMA_PDSSR0_SPI2_TXSEL_Pos);

    // Set Transfer Byte Count
    PDMA_CH1->BCR = TEST_LENGTH;

    // Set Source Address
    PDMA_CH1->SAR = u32SrcAddr;

    // Set Destination Address
    PDMA_CH1->DAR = SPI2_TX;

    // Set Transfer Width = 8 bits, Source Direction = INC, Destination Direction = FIX and Mode = Memory to Peripheral
    PDMA_CH1->CSR = (PDMA_CH1->CSR & ~(PDMA_CSR_APB_TWS_Msk | PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk | PDMA_CSR_MODE_SEL_Msk)) |
                    (PDMA_WIDTH_8 | PDMA_SAR_INC | PDMA_DAR_FIX | (MODE_MEM2PER << PDMA_CSR_MODE_SEL_Pos));

    // Enable Transfer Block Done Interrupt
    PDMA_CH1->IER = (PDMA_CH1->IER & ~(PDMA_IER_TABORT_IE_Msk | PDMA_IER_BLKD_IE_Msk)) | PDMA_IER_BLKD_IE_Msk;

}

// **************************************
void Init_PDMA_CH2_for_SPI2_RX(uint32_t u32DstAddr)
{
    uint32_t SPI2_RX;
    PDMA_T *PDMA_CH2;

    // PDMA Channel 2 control registers
    PDMA_CH2 = (PDMA_T *)((uint32_t) PDMA2_BASE);

    // SPI2 RX register
    SPI2_RX = SPI2_BASE + 0x10;

    // Enable PDMA IP clock
    CLK->AHBCLK |= CLK_AHBCLK_PDMA_EN_Msk;

    // Enable Channel 2 clock
    PDMA_GCR->GCRCSR |= ((1 << CH2) << 8);

    // Set Channel 2 for SPI2_RX
    PDMA_GCR->PDSSR0 = (PDMA_GCR->PDSSR0 & ~PDMA_PDSSR0_SPI2_RXSEL_Msk) | (2 << PDMA_PDSSR0_SPI2_RXSEL_Pos);

    // Set Transfer Byte Count
    PDMA_CH2->BCR = TEST_LENGTH;

    // Set Source Address
    PDMA_CH2->SAR = SPI2_RX;

    // Set Destination Address
    PDMA_CH2->DAR = u32DstAddr;

    // Set Transfer Width = 8 bits, Source Direction = FIX, Destination Direction = INC and Mode = Peripheral to Memory
    PDMA_CH2->CSR = (PDMA_CH2->CSR & ~(PDMA_CSR_APB_TWS_Msk | PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk | PDMA_CSR_MODE_SEL_Msk)) |
                    (PDMA_WIDTH_8 | PDMA_SAR_FIX | PDMA_DAR_INC | (MODE_PER2MEM << PDMA_CSR_MODE_SEL_Pos));

    // Enable Transfer Block Done Interrupt
    PDMA_CH2->IER = (PDMA_CH2->IER & ~(PDMA_IER_TABORT_IE_Msk | PDMA_IER_BLKD_IE_Msk)) | PDMA_IER_BLKD_IE_Msk;

}

// **************************************
// For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x14
// For W26X16, Manufacturer ID: 0xEF; Device ID: 0x14
unsigned int SpiFlash_w_PDMA_ReadMidDid(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // /CS: active
    SPI_SET_SS0_LOW(SPI2);

    // send Command: 0x90, Read Manufacturer/Device ID
    au32SourceData = 0x90;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI2, 24);
    // send 24-bit '0', dummy
    au32SourceData = 0x0;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 16 bits
    //SPI2->CNTRL = SPI2->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(16);
    SPI_SET_DATA_WIDTH(SPI2, 16);
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

    return (au32DestinationData & 0xffff);

}

// **************************************
void SpiFlash_w_PDMA_ChipErase(void)
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
    au32SourceData = 0xc7;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);

}

// **************************************
unsigned int SpiFlash_w_PDMA_ReadStatusReg1(void)
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
unsigned int SpiFlash_w_PDMA_ReadStatusReg2(void)
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
void SpiFlash_w_PDMA_WaitReady(void)
{
    unsigned int ReturnValue;

    do
    {
        ReturnValue = SpiFlash_w_PDMA_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    }
    while(ReturnValue != 0); // check the BUSY bit

}

// **************************************
void SpiFlash_w_PDMA_PageProgram(unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // /CS: active

    SPI_SET_SS0_LOW(SPI2);
    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI2);

    // /CS: active
    SPI_SET_SS0_LOW(SPI2);

    // send Command: 0x02, Page program
    au32SourceData = 0x02;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // enable SPI PDMA
    SPI_FLASH_PORT->DMA = (SPI_FLASH_PORT->DMA & ~(SPI_DMA_RX_DMA_GO_Msk | SPI_DMA_TX_DMA_GO_Msk)) | SPI_DMA_TX_DMA_GO_Msk;

    // SPI go
    PDMA_CH1_INT_Flag = 0;
    SPI_FLASH_PORT->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait PDMA done
    while(1)
    {
        if(PDMA_CH1_INT_Flag)
        {
            PDMA_CH1_INT_Flag = 0;
            break;
        }
    }

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);

}


// **************************************
void SpiFlash_w_PDMA_ReadData(unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // /CS: active
    SPI_SET_SS0_LOW(SPI2);

    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI2->TX[0] = au32SourceData;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // enable SPI PDMA
    SPI_FLASH_PORT->DMA = (SPI_FLASH_PORT->DMA & ~(SPI_DMA_RX_DMA_GO_Msk | SPI_DMA_TX_DMA_GO_Msk)) | SPI_DMA_RX_DMA_GO_Msk;

    // SPI go
    PDMA_CH2_INT_Flag = 0;
    SPI_FLASH_PORT->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait PDMA done
    while(1)
    {
        if(PDMA_CH2_INT_Flag)
        {
            PDMA_CH2_INT_Flag = 0;
            break;
        }
    }

    // wait
    while(SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI2->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI2);
}


