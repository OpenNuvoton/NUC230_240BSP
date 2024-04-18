/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Use CAN Silent mode to listen to CAN bus communication test in Basic mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"

#define PLL_CLOCK       50000000

/*---------------------------------------------------------------------------*/
/* Function Declare                                                          */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);

/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000)        /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n") ;
        }
    }
    else if(u8IIDRstatus != 0)
    {
        CAN_CLR_INT_PENDING_BIT(CAN0, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
    }
    else if(CAN0->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;    /* Write '0' to clear */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Reset message interface parameters                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ResetIF(CAN_T *tCAN, uint8_t u8IF_Num)
{
    if(u8IF_Num > 1)
        return;
    tCAN->IF[u8IF_Num].CREQ     = 0x0;          // set bit15 for sending
    tCAN->IF[u8IF_Num].CMASK    = 0x0;
    tCAN->IF[u8IF_Num].MASK1    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].MASK2    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].ARB1     = 0x0;          // ID15~0
    tCAN->IF[u8IF_Num].ARB2     = 0x0;          // MsgVal, eXt, xmt, ID28~16
    tCAN->IF[u8IF_Num].MCON     = 0x0;          // DLC
    tCAN->IF[u8IF_Num].DAT_A1   = 0x0;          // data0,1
    tCAN->IF[u8IF_Num].DAT_A2   = 0x0;          // data2,3
    tCAN->IF[u8IF_Num].DAT_B1   = 0x0;          // data4,5
    tCAN->IF[u8IF_Num].DAT_B2   = 0x0;          // data6,7
}

/*---------------------------------------------------------------------------*/
/* Show Message Function                                                     */
/*---------------------------------------------------------------------------*/
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;

    /* Show the message information */
    printf("Read ID=0x%X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType ? "EXT" : "STD", Msg->DLC);
    for(i = 0; i < Msg->DLC; i++)
        printf("%X,", Msg->Data[i]);
    printf("\n\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CAN module clock */
    CLK_EnableModuleClock(CAN0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set GPD multi-function pins for CAN0 RXD and TXD */
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP |= SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init CAN                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_SilentMode_Init(CAN_T *tCAN)
{
    printf("CAN monitoring baud rate(bps): %d\n\n", CAN_GetCANBitRate(tCAN));

    /* Enable the Silent Mode */
    CAN_EnterTestMode(tCAN, CAN_TEST_BASIC_Msk | CAN_TEST_SILENT_Msk);

    /* Enable CAN interrupt */
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk | CAN_CON_SIE_Msk);

    /* Set Interrupt Priority */
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    /* Enable External Interrupt */
    NVIC_EnableIRQ(CAN0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Disable CAN Clock and Reset it                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_STOP(void)
{
    /* Disable CAN0 Clock and Reset it */
    SYS_ResetModule(CAN0_RST);
    CLK_DisableModuleClock(CAN0_MODULE);
}

/*----------------------------------------------------------------------------*/
/* Some description about how to create test environment                      */
/*----------------------------------------------------------------------------*/
void Note_Configure(void)
{
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                     CAN Basic + Silent Mode Sample Code                     |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                              |\n");
    printf("|    The sample code needs three boards. Use CAN Silent mode to listen to     |\n");
    printf("|    CAN Bus communication test. The sample code must be set up on the node   |\n");
    printf("|    of CAN communication transmission. Users can use the sample codes        |\n");
    printf("|    ' CAN_BasicMode_Transmit ' and ' CAN_BasicMode_Receive ' as the CAN      |\n");
    printf("|    communication transmission network.                                      |\n");
    printf("|    Note: Users need to confirm whether the transmission rate matches.       |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                Pin Configure                                |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                                         CAN_TXD(Any board)  |\n");
    printf("|                                CANBUS                   CAN_RXD(Any board)  |\n");
    printf("|                                  ||    CAN_H   |-----------|                |\n");
    printf("|                                  || <--------->|           |<------         |\n");
    printf("|                                  ||            |    CAN    |CAN_TX          |\n");
    printf("|      CAN_TXD                     ||    CAN_L   |Transceiver|                |\n");
    printf("|      CAN_RXD                     || <--------->|           |------>         |\n");
    printf("|         |-----------|   CAN_H    ||            |           |CAN_RX          |\n");
    printf("|  ------>|           |<---------> ||            |-----------|                |\n");
    printf("|   CAN_TX|   CAN     |            ||                                         |\n");
    printf("|         |Transceiver|            ||                     CAN_TXD(Any board)  |\n");
    printf("|  <------|           |   CAN_L    ||                     CAN_RXD(Any board)  |\n");
    printf("|   CAN_RX|           |<---------> ||    CAN_H   |-----------|                |\n");
    printf("|         |-----------|            || <--------->|           |<------         |\n");
    printf("|                                  ||            |    CAN    |CAN_TX          |\n");
    printf("|                                  ||    CAN_L   |Transceiver|                |\n");
    printf("|                                  || <--------->|           |------>         |\n");
    printf("|                                  ||            |           |CAN_RX          |\n");
    printf("|                                  ||            |-----------|                |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");
}

/*----------------------------------------------------------------------------*/
/* Set the CAN speed                                                          */
/*----------------------------------------------------------------------------*/
void SelectCANSpeed(CAN_T *tCAN)
{
    int32_t i32Item;
    uint32_t u32BaudRate = 0;

    printf("Please select CAN speed you desired\n");
    printf("[0] 1000Kbps\n");
    printf("[1]  500Kbps\n");
    printf("[2]  250Kbps\n");
    printf("[3]  125Kbps\n");
    printf("[4]  100Kbps\n");
    printf("[5]   50Kbps\n");

    PE->PMD = (GPIO_PMD_OUTPUT << 3*2);
    PE3 = 0;

    /* Set target baud-rate and operation mode */
    i32Item = GetChar();
    printf("%c\n", i32Item);
    if(i32Item == '1')
        u32BaudRate = CAN_Open(tCAN,  500000, CAN_BASIC_MODE);
    else if(i32Item == '2')
        u32BaudRate = CAN_Open(tCAN,  250000, CAN_BASIC_MODE);
    else if(i32Item == '3')
        u32BaudRate = CAN_Open(tCAN,  125000, CAN_BASIC_MODE);
    else if(i32Item == '4')
        u32BaudRate = CAN_Open(tCAN,  100000, CAN_BASIC_MODE);
    else if(i32Item == '5')
        u32BaudRate = CAN_Open(tCAN,   50000, CAN_BASIC_MODE);
    else
        u32BaudRate = CAN_Open(tCAN, 1000000, CAN_BASIC_MODE);

    if(u32BaudRate > 1000000)
        printf("Set CAN bit rate is fail\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    CAN_T *tCAN;
    STR_CANMSG_T rMsg[5];
    uint8_t i;

    tCAN = (CAN_T *) CAN0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable CAN transceiver for Nuvoton board */
    /* CAN0 */
    PB->PMD = GPIO_PMD_OUTPUT << 3*2;
    PB3 = 0;

    /* Some description about how to create test environment */
    Note_Configure();

    /* Configuring the Bit Timing */
    SelectCANSpeed(tCAN);

    /* CAN interface initialization */
    CAN_SilentMode_Init(tCAN);

    for(i = 0; i < 5; i++)
    {
        CAN_ResetIF(tCAN, 1);
        while(CAN_Receive(tCAN, 0, &rMsg[i]) == FALSE);
    }
    for(i = 0; i < 5; i++)
        CAN_ShowMsg(&rMsg[i]);

    /* Disable CAN */
    CAN_Close(tCAN);

    /* Disable CAN Clock and Reset it */
    CAN_STOP();

    while(1);
}
