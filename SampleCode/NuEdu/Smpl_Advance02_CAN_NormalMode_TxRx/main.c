/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 4 $
 * $Date: 15/08/18 6:03p $
 * @brief    NUC200 Series CAN Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NUC230_240.h"

#define PLL_CLOCK       48000000

/* Declare a CAN message structure */
CAN_T *tCAN0 = (CAN_T *)CAN0;
CAN_T *tCAN1 = (CAN_T *)CAN1;

/* Define the FIFO depth for message objects */
#define CAN_FIFO_DEPTH      8

/* Define the message RAM ID uses for Tx */
#define TX_MSG_OBJ_ID           31

/* */
#define MAX_FIFO_MESSAGES   40

typedef struct
{
    uint8_t Head;
    uint8_t Tail;
    STR_CANMSG_T Msg[MAX_FIFO_MESSAGES];
} MSG_BUF_T;

typedef MSG_BUF_T *PMSG_BUF_T;

MSG_BUF_T rrMsg0 = {0}; /* Receive FIFO buffer for CAN0 */
MSG_BUF_T rrMsg1 = {0}; /* Receive FIFO buffer for CAN1 */

uint8_t ResetCAN0 = FALSE;  /* It will be TRUE if bus off, reset CAN to start again */
uint8_t ResetCAN1 = FALSE;

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);

void CAN_ShowMsg(STR_CANMSG_T* Msg);
void GetMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg);
void SendMsgObj(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
void SetMsgObj_for_Rx(CAN_T *tCAN);
void SendMsgObj_to_Tx(CAN_T *tCAN);

uint32_t CountsInFifo(PMSG_BUF_T pMsgBuf);
uint8_t PutToFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg);
uint8_t GetFromFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000)         /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;       /* Clear RxOK status*/

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;       /* Clear TxOK status*/

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n");
            ResetCAN0 = TRUE;
        }
        else if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n");
            ResetCAN0 = TRUE;
        }
        else if((CAN0->ERR & CAN_ERR_TEC_Msk) != 0)
        {
            printf("Transmit error!\n");
        }
        else if((CAN0->ERR & CAN_ERR_REC_Msk) != 0)
        {
            printf("Receive error!\n");
        }
    }
    else if((u8IIDRstatus >= 0x1) || (u8IIDRstatus <= 0x20))
    {
        STR_CANMSG_T rrMsg;

        uint32_t i;
        uint32_t u32NewDataReg;

        u32NewDataReg = tCAN0->NDAT1 & ((1 << CAN_FIFO_DEPTH) - 1);

        if(u32NewDataReg)
        {
            for(i = 0; (u32NewDataReg != 0) && (i < CAN_FIFO_DEPTH); i++)
            {
                if(u32NewDataReg & 1)
                {
                    GetMsgObj(tCAN0, i, &rrMsg);
                    CAN_ShowMsg(&rrMsg);
                    PutToFifo(&rrMsg0, &rrMsg);
                }
                u32NewDataReg >>= 1;
            }
        }
        else
        {
            CAN_CLR_INT_PENDING_BIT(CAN0, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
        }
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
/*  Show Message Function                                                    */
/*---------------------------------------------------------------------------*/
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;

    /* Show the message information */
    printf("\nRead ID=0x%X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType ? "EXT" : "STD", Msg->DLC);
    for(i = 0; i < Msg->DLC; i++)
        printf("%X,", Msg->Data[i]);

    printf("\n");

}

/*---------------------------------------------------------------------------*/
/*  CAN Message FIFO Buffer Functions                                        */
/*---------------------------------------------------------------------------*/
void ResetFifo(PMSG_BUF_T pMsgBuf)
{
    pMsgBuf->Head = 0;
    pMsgBuf->Tail = 0;

}

/* Return numbers of messages in FIFO */
uint32_t CountsInFifo(PMSG_BUF_T pMsgBuf)
{
    uint8_t h, t;

    /* This method can safely get the difference without disable interrupt */
    h = pMsgBuf->Head;
    t = pMsgBuf->Tail;

    return (h >= t) ? (h - t) : (h + MAX_FIFO_MESSAGES - t);

}

uint8_t PutToFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg)
{
    if(CountsInFifo(pMsgBuf) >= MAX_FIFO_MESSAGES - 1)
        return FALSE;

    memcpy(&(pMsgBuf->Msg[pMsgBuf->Head]), pMsg, sizeof(STR_CANMSG_T));

    if(++(pMsgBuf->Head) >= MAX_FIFO_MESSAGES)
        pMsgBuf->Head = 0;

    return TRUE;

}

uint8_t GetFromFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg)
{
    if(CountsInFifo(pMsgBuf) == 0)
        return FALSE;

    memcpy(pMsg, &(pMsgBuf->Msg[pMsgBuf->Tail]), sizeof(STR_CANMSG_T));

    if(++(pMsgBuf->Tail) >= MAX_FIFO_MESSAGES)
        pMsgBuf->Tail = 0;

    return TRUE;

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC 22M clock ready */
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

    /* Set PD multi-function pins for CAN0_TX, CAN0_RX */
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP |= SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baud-rate */
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
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void Note_Configure()
{
    printf("\n\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|   About CAN sample code configure                                          |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN           |\n");
    printf("|   Before execute it, please check description as below                     |\n");
    printf("|                                                                            |\n");
    printf("|   1.CAN0_TX and CAN0_RX should be connected to your CAN transceiver        |\n");
    printf("|   2.Using two module board and connect to the same CAN BUS                 |\n");
    printf("|   3.Check the terminal resistor of bus is connected                        |\n");
    printf("|   4.Using UART0 as print message port                                      |\n");
    printf("|                                                                            |\n");
    printf("|  |--------|        |-----------|   CANBUS  |-----------|        |--------| |\n");
    printf("|  |        |------->|           |<--------->|           |<-------|        | |\n");
    printf("|  |        |CAN0_TX |    CAN    |   CAN_H   |   CAN     |CAN0_TX |        | |\n");
    printf("|  | NUC2XX |        |Transceiver|           |Transceiver|        | NUC2XX | |\n");
    printf("|  |        |<-------|           |<--------->|           |------->|        | |\n");
    printf("|  |        |CAN0_RX |           |   CAN_L   |           |CAN0_RX |        | |\n");
    printf("|  |--------|        |-----------|           |-----------|        |--------| |\n");
    printf("|  |                                                              |          |\n");
    printf("|  |                                                              |          |\n");
    printf("|  V                                                              V          |\n");
    printf("| UART0                                                           UART0      |\n");
    printf("|(print message)                                             (print message) |\n");
    printf("+----------------------------------------------------------------------------+\n");

}

/*----------------------------------------------------------------------------*/
/*  Check the real baud-rate                                                  */
/*----------------------------------------------------------------------------*/
void BaudRateCheck(uint32_t u32BaudRate, uint32_t u32RealBaudRate)
{
    /* Get Core Clock Frequency */
    SystemCoreClockUpdate();

    if(u32BaudRate != u32RealBaudRate)
    {
        printf("\nSet CAN baud-rate is fail\n");
        printf("Real baud-rate value(bps): %d\n", u32RealBaudRate);
        printf("CAN baud-rate calculation equation as below:\n");
        printf("CAN baud-rate(bps) = Fin/(BPR+1)*(Tseg1+Tseg2+3)\n");
        printf("where: Fin: System clock freq.(Hz)\n");
        printf("       BRP: The baud rate prescale. It is composed of BRP (CAN_BTIME[5:0]) and BRPE (CAN_BRPE[3:0]).\n");
        printf("       Tseg1: Time Segment before the sample point. You can set tseg1 (CAN_BTIME[11:8]).\n");
        printf("       Tseg2: Time Segment ater the sample point. You can set tseg2 (CAN_BTIME[14:12]).\n");

        if(SystemCoreClock % u32BaudRate != 0)
            printf("\nThe BPR does not calculate, the Fin must be a multiple of the CAN baud-rate.\n");
        else
            printf("\nThe BPR does not calculate, the (Fin/(CAN baud-rate)) must be a multiple of the (Tseg1+Tseg1+3).\n");
    }
    else
        printf("\nReal baud-rate value(bps): %d\n", u32RealBaudRate);

}

uint32_t InitCAN(CAN_T *tCAN, uint32_t u32BaudRate)
{
    uint8_t u8Tseg1, u8Tseg2;
    uint32_t u32Brp;
    uint32_t u32Value;

    // CAN do Initialization
    tCAN ->CON |= CAN_CON_INIT_Msk | CAN_CON_CCE_Msk;

    // Update system clock for CAN
    SystemCoreClockUpdate();
    u32Value = SystemCoreClock;

    // Set BTIME and BRPE registers
    u8Tseg1 = 3;
    u8Tseg2 = 2;
    while(1)
    {
        if(((u32Value % (u8Tseg1 + u8Tseg2 + 3)) == 0) | (u8Tseg1 >= 15))
            break;

        u8Tseg1++;
        if((u32Value % (u8Tseg1 + u8Tseg2 + 3)) == 0)
            break;

        if(u8Tseg2 < 7)
            u8Tseg2++;
    }
    u32Brp  = u32Value / (u32BaudRate) / (u8Tseg1 + u8Tseg2 + 3) - 1;

    u32Value = ((uint32_t)u8Tseg2 << CAN_BTIME_TSEG2_Pos) |
               ((uint32_t)u8Tseg1 << CAN_BTIME_TSEG1_Pos) |
               (u32Brp & CAN_BTIME_BRP_Msk) |
               (CAN0->BTIME & CAN_BTIME_SJW_Msk);
    tCAN->BTIME = u32Value;
    tCAN->BRPE = (u32Brp >> 6) & 0x0F;

    // CAN Initialization finished
    tCAN->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
    while(tCAN->CON & CAN_CON_INIT_Msk);

    // Enable CAN interrupt
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk | CAN_CON_SIE_Msk);
    NVIC_SetPriority(CAN0_IRQn, 1);
    NVIC_EnableIRQ(CAN0_IRQn);

    // Return baudrate
    return (SystemCoreClock / (u8Tseg1 + u8Tseg2 + 3) / (u32Brp + 1));

}

/*----------------------------------------------------------------------------*/
/*  Set the CAN speed                                                         */
/*----------------------------------------------------------------------------*/
void SetCANSpeed(CAN_T *tCAN)
{
    uint32_t BaudRate = 0, RealBaudRate = 0;

    /* Set CAN baud rate to 1Mbps */
    BaudRate = 125000;
    RealBaudRate = InitCAN(tCAN,  BaudRate);

    /* Check the real baud-rate is OK */
    BaudRateCheck(BaudRate, RealBaudRate);

}

/*----------------------------------------------------------------------------*/
/*  Test Menu                                                                 */
/*----------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                                                                   |\n");
    printf("|     1. Master: Transmit messages from CAN0                        |\n");
    printf("|     2. Slave: Receive messages thru CAN0                          |\n");
    printf("|     Please select 1 (Master) or 2 (Slave).                        |\n");
    printf("|                                                                   |\n");
    printf("+-------------------------------------------------------------------+\n");

}

/*----------------------------------------------------------------------------*/
/*  Configure Message Objects for Tx and Rx                                   */
/*----------------------------------------------------------------------------*/
void SetRxMsgObj(CAN_T *tCAN, uint32_t u32MsgNum, uint8_t u8FifoLast)
{
    tCAN->IF[0].CMASK = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_MASK_Msk | CAN_IF_CMASK_ARB_Msk |
                        CAN_IF_CMASK_CONTROL_Msk | CAN_IF_CMASK_DATAA_Msk | CAN_IF_CMASK_DATAB_Msk;

    tCAN->IF[0].ARB1 = 0;
    tCAN->IF[0].ARB2 = CAN_IF_ARB2_MSGVAL_Msk;
    tCAN->IF[0].MASK1 = 0;
    tCAN->IF[0].MASK2 = 0;

    tCAN->IF[0].MCON = CAN_IF_MCON_UMASK_Msk | CAN_IF_MCON_RXIE_Msk |
                       ((u8FifoLast) ? CAN_IF_MCON_EOB_Msk : 0);

    tCAN->IF[0].DAT_A1  = 0;
    tCAN->IF[0].DAT_A2  = 0;
    tCAN->IF[0].DAT_B1  = 0;
    tCAN->IF[0].DAT_B2  = 0;

    tCAN->IF[0].CREQ = 1 + u32MsgNum;

    while((tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk));

}

void GetMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg)
{
    tCAN->STATUS &= (~CAN_STATUS_RXOK_Msk);

    /* read the message contents*/
    tCAN->IF[0].CMASK = CAN_IF_CMASK_MASK_Msk
                        | CAN_IF_CMASK_ARB_Msk
                        | CAN_IF_CMASK_CONTROL_Msk
                        | CAN_IF_CMASK_CLRINTPND_Msk
                        | CAN_IF_CMASK_TXRQSTNEWDAT_Msk
                        | CAN_IF_CMASK_DATAA_Msk
                        | CAN_IF_CMASK_DATAB_Msk;

    tCAN->IF[0].CREQ = 1 + u8MsgObj;

    /*Wait*/
    while(tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk) {};

    if((tCAN->IF[0].ARB2 & CAN_IF_ARB2_XTD_Msk) == 0)
    {
        /* standard ID*/
        pCanMsg->IdType = CAN_STD_ID;
        pCanMsg->Id     = (tCAN->IF[0].ARB2 & CAN_IF_ARB2_ID_Msk) >> 2;
    }
    else
    {
        /* extended ID*/
        pCanMsg->IdType = CAN_EXT_ID;
        pCanMsg->Id  = (((tCAN->IF[0].ARB2) & 0x1FFF) << 16) | tCAN->IF[0].ARB1;
    }

    pCanMsg->FrameType = !((tCAN->IF[0].ARB2 & CAN_IF_ARB2_DIR_Msk) >> CAN_IF_ARB2_DIR_Pos);
    pCanMsg->DLC     = tCAN->IF[0].MCON & CAN_IF_MCON_DLC_Msk;
    pCanMsg->Data[0] = tCAN->IF[0].DAT_A1 & CAN_IF_DAT_A1_DATA0_Msk;
    pCanMsg->Data[1] = (tCAN->IF[0].DAT_A1 & CAN_IF_DAT_A1_DATA1_Msk) >> CAN_IF_DAT_A1_DATA1_Pos;
    pCanMsg->Data[2] = tCAN->IF[0].DAT_A2 & CAN_IF_DAT_A2_DATA2_Msk;
    pCanMsg->Data[3] = (tCAN->IF[0].DAT_A2 & CAN_IF_DAT_A2_DATA3_Msk) >> CAN_IF_DAT_A2_DATA3_Pos;
    pCanMsg->Data[4] = tCAN->IF[0].DAT_B1 & CAN_IF_DAT_B1_DATA4_Msk;
    pCanMsg->Data[5] = (tCAN->IF[0].DAT_B1 & CAN_IF_DAT_B1_DATA5_Msk) >> CAN_IF_DAT_B1_DATA5_Pos;
    pCanMsg->Data[6] = tCAN->IF[0].DAT_B2 & CAN_IF_DAT_B2_DATA6_Msk;
    pCanMsg->Data[7] = (tCAN->IF[0].DAT_B2 & CAN_IF_DAT_B2_DATA7_Msk) >> CAN_IF_DAT_B2_DATA7_Pos;

}

void SendMsgObj(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg)
{
    if(u32MsgNum < 16)
        while(tCAN->TXREQ1 & (1 << u32MsgNum));
    else
        while(tCAN->TXREQ2 & (1 << (u32MsgNum - 16)));

    /* update the contents needed for transmission*/
    tCAN->IF[1].CMASK = 0xF7; /* 0xF3;  CAN_CMASK_WRRD_Msk | CAN_CMASK_MASK_Msk | CAN_CMASK_ARB_Msk
                                    | CAN_CMASK_CONTROL_Msk | CAN_CMASK_DATAA_Msk  | CAN_CMASK_DATAB_Msk ; */

    if(pCanMsg->IdType == CAN_STD_ID)
    {
        /* standard ID*/
        tCAN->IF[1].ARB1 = 0;
        tCAN->IF[1].ARB2 = (((pCanMsg->Id) & 0x7FF) << 2) | CAN_IF_ARB2_DIR_Msk | CAN_IF_ARB2_MSGVAL_Msk;
    }
    else
    {
        /* extended ID*/
        tCAN->IF[1].ARB1 = (pCanMsg->Id) & 0xFFFF;
        tCAN->IF[1].ARB2 = ((pCanMsg->Id) & 0x1FFF0000) >> 16 | CAN_IF_ARB2_DIR_Msk
                           | CAN_IF_ARB2_XTD_Msk | CAN_IF_ARB2_MSGVAL_Msk;
    }

    if(pCanMsg->FrameType)
        tCAN->IF[1].ARB2 |=   CAN_IF_ARB2_DIR_Msk;
    else
        tCAN->IF[1].ARB2 &= (~CAN_IF_ARB2_DIR_Msk);

    tCAN->IF[1].DAT_A1 = ((uint16_t)pCanMsg->Data[1] << 8) | pCanMsg->Data[0];
    tCAN->IF[1].DAT_A2 = ((uint16_t)pCanMsg->Data[3] << 8) | pCanMsg->Data[2];
    tCAN->IF[1].DAT_B1 = ((uint16_t)pCanMsg->Data[5] << 8) | pCanMsg->Data[4];
    tCAN->IF[1].DAT_B2 = ((uint16_t)pCanMsg->Data[7] << 8) | pCanMsg->Data[6];

    tCAN->IF[1].MCON   =  CAN_IF_MCON_NEWDAT_Msk | pCanMsg->DLC | CAN_IF_MCON_TXIE_Msk | CAN_IF_MCON_EOB_Msk;
    tCAN->IF[1].CREQ   = 1 + u32MsgNum;

    /* Wait */
    while(tCAN->IF[1].CREQ & CAN_IF_CREQ_BUSY_Msk) {};

}

void SetMsgObj_for_Rx(CAN_T *tCAN)
{
    /* Use several message objects as FIFO buffer to receive CAN messages */
    int i;

    for(i = 0; i < CAN_FIFO_DEPTH - 1; i++)
        SetRxMsgObj(tCAN, i, 0);

    SetRxMsgObj(tCAN, i, 1);

}

/*----------------------------------------------------------------------------*/
/*  Send Tx Msg by Normal Mode Function (With Message RAM)                    */
/*----------------------------------------------------------------------------*/
void SendMsgObj_to_Tx(CAN_T *tCAN)
{
    STR_CANMSG_T tMsg;

    /* Send a 29-bit Extended Identifier message */
    tMsg.FrameType = CAN_DATA_FRAME;
    tMsg.IdType   = CAN_EXT_ID;
    tMsg.Id       = 0x7FF01;
    tMsg.DLC      = 4;
    tMsg.Data[0]  = 0xA1;
    tMsg.Data[1]  = 0xB2;
    tMsg.Data[2]  = 0xC3;
    tMsg.Data[3]  = 0xD4;

    SendMsgObj(tCAN, TX_MSG_OBJ_ID, &tMsg);

    printf("\nMsgObj(%d), Send EXT_ID:0x7FF01, Data[A1,B2,C3,D4] done!\n", TX_MSG_OBJ_ID);

}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    char c;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Enable CAN transceiver on NuEdu-Advance02 V1.1 thru NuEdu-EVB-NUC240 v2.0 main board */
    GPIO_SetMode(PA, BIT0, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_PMD_OUTPUT);
    PA0 = 0;
    PA1 = 0;

    /* Set PE.8 as Input for key button */
    GPIO_SetMode(PE, BIT8,  GPIO_PMD_INPUT);

    /* Some description about how to create test environment */
    Note_Configure();

    /* Configuring the Bit Timing */
    SetCANSpeed(tCAN0);

    /* Test Menu */
    TestItem();
    c = getchar();
    putchar(c);
    switch(c)
    {
        case '1':
            printf("\nCAN0 as Master\n");
            while(1)
            {
                if(PE8 == 0)
                {
                    SendMsgObj_to_Tx(tCAN0);
                    CLK_SysTickDelay(200000);
                }
            }
        case '2':
            printf("\nCAN0 as Slave\n");
            SetMsgObj_for_Rx(tCAN0);
            break;
        default:
            printf("\nPlease press reset key to re-select!!!");
            break;
    }

    while(1);

}
