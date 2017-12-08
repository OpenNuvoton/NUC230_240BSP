/**************************************************************************//**
 * @file     NuEdu-Basic01_EEPROM.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/08/18 11:54a $
 * @brief    NUC200 Series EEPROM Library
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuEdu-Basic01_EEPROM.h"
#define I2C_EEPROM I2C1
#define I2C_EEPROM_IRQn I2C1_IRQn

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr = 0x50;
uint8_t g_au8TxData[3];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2CHandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_EEPROM->I2CSTATUS;

    if(I2C_EEPROM->I2CTOC & I2C_I2CTOC_TIF_Msk)
    {
        /* Clear I2C Timeout Flag */
        I2C_EEPROM->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    }
    else
    {
        if(s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Initial Function                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
__INLINE void I2C_PIN_Init(void)
{
    /* Set GPA10,11 multi-function pins for I2C1 SDA and SCL */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA10_Msk | SYS_GPA_MFP_PA11_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA10_I2C1_SDA | SYS_GPA_MFP_PA11_I2C1_SCL;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA10_Msk | SYS_ALT_MFP_PA11_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PA10_I2C1_SDA | SYS_ALT_MFP_PA11_I2C1_SCL;

    /* Enable I2C1 clock */
    CLK->APBCLK |= CLK_APBCLK_I2C1_EN_Msk;

    /* Reset I2C1 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C1_RST_Msk;
}

void I2C_EEPROM_Init(void)
{
    I2C_PIN_Init();
    /* Enable I2C Controller */
    //I2C_EEPROM->I2CON |= I2C_I2CON_ENS1_Msk;

    /* I2C clock divider, I2C Bus Clock = PCLK / (4*120) = 100kHz */
    //I2C_EEPROM->I2CLK = I2C_I2CLK_DIV4(u8Divider);
    I2C_Open(I2C_EEPROM, 100000);
    /* Enable I2C interrupt and set corresponding NVIC bit */
    I2C_EEPROM->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C_EEPROM_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_EEPROM->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_EEPROM->I2CDAT = g_au8TxData[g_u8DataLen++];
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_STA | I2C_I2CON_STO_SI);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen != 2)
        {
            I2C_EEPROM->I2CDAT = g_au8TxData[g_u8DataLen++];
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_EEPROM->I2CDAT = ((g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = I2C_EEPROM->I2CDAT;
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_STO_SI);
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        //printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_EEPROM->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_EEPROM->I2CDAT = g_au8TxData[g_u8DataLen++];
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_STA | I2C_I2CON_STO_SI);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen != 3)
        {
            I2C_EEPROM->I2CDAT = g_au8TxData[g_u8DataLen++];
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_STO_SI);
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        //printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write I2C EEPROM                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_EEPROM_Write(uint16_t u16Address, uint8_t u8Data)
{
    g_au8TxData[0] = u16Address >> 8;
    g_au8TxData[1] = u16Address & 0xFF;
    g_au8TxData[2] = u8Data;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2CHandlerFn = (I2C_FUNC)I2C_MasterTx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_STA);

    /* Wait I2C Tx Finish */
    while(g_u8EndFlag == 0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Read I2C EEPROM                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t I2C_EEPROM_Read(uint16_t u16Address)
{
    g_au8TxData[0] = u16Address >> 8;
    g_au8TxData[1] = u16Address & 0xFF;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2CHandlerFn = (I2C_FUNC)I2C_MasterRx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_I2CON_STA);

    /* Wait I2C Tx Finish */
    while(g_u8EndFlag == 0);

    return g_u8RxData;
}




