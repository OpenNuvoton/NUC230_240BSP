/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 2 $
 * $Date: 4/15/15 11:45a $
 * @brief    NUC230_240 Series CAN Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"


#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK       72000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);
void CAN_CLR_INT_PENDING_BIT(CAN_T *tCAN, uint8_t u32MsgNum);
int32_t CAN_ReadMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg);
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR);
void CAN_ResetIF(CAN_T *tCAN, uint8_t u8IF_Num);
void CAN_ShowMsg(STR_CANMSG_T* Msg);
void SYS_Init(void);
void UART0_Init(void);
void Note_Configure(void);
void TestItem(void);
void CAN_Init(CAN_T *tCAN, uint32_t u32kbps);
uint32_t GetCANBitRate(CAN_T  *tCAN);
void SelectCANSpeed(CAN_T  *tCAN);
void CAN_STOP(void);
static uint32_t CAN_GetFreeIF(CAN_T *tCAN);
int32_t CAN_SetMsgObjMask(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMASK_T* MaskMsg);
int32_t CAN_SetRxMsgObj(CAN_T  *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint8_t u8singleOrFifoLast);
void Test_SetMaskFilter(CAN_T *tCAN);

/*---------------------------------------------------------------------------*/
/*  Callback function                                                        */
/*---------------------------------------------------------------------------*/

/* Declare a CAN message structure */
STR_CANMSG_T rrMsg;


/*---------------------------------------------------------------------------------------------------------*/
/* Clear interrupt pending bit                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_CLR_INT_PENDING_BIT(CAN_T *tCAN, uint8_t u32MsgNum)
{
    uint32_t u32MsgIfNum = 0;
    uint32_t u32IFBusyCount = 0;

    while(u32IFBusyCount < 0x10000000) {
        if((tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk) == 0) {
            u32MsgIfNum = 0;
            break;
        } else if((tCAN->IF[1].CREQ  & CAN_IF_CREQ_BUSY_Msk) == 0) {
            u32MsgIfNum = 1;
            break;
        }

        u32IFBusyCount++;
    }

    tCAN->IF[u32MsgIfNum].CMASK = CAN_IF_CMASK_CLRINTPND_Msk | CAN_IF_CMASK_TXRQSTNEWDAT_Msk;
    tCAN->IF[u32MsgIfNum].CREQ = 1 + u32MsgNum;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Gets the message                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t CAN_ReadMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg)
{
    if(!CAN_GET_NEW_DATA_IN_BIT(tCAN, u8MsgObj)) {
        return FALSE;
    }

    tCAN->STATUS &= (~CAN_STATUS_RXOK_Msk);

    /* read the message contents*/
    tCAN->IF[1].CMASK = CAN_IF_CMASK_MASK_Msk
                        | CAN_IF_CMASK_ARB_Msk
                        | CAN_IF_CMASK_CONTROL_Msk
                        | CAN_IF_CMASK_CLRINTPND_Msk
                        | CAN_IF_CMASK_TXRQSTNEWDAT_Msk
                        | CAN_IF_CMASK_DATAA_Msk
                        | CAN_IF_CMASK_DATAB_Msk;

    tCAN->IF[1].CREQ = 1 + u8MsgObj;

    while(tCAN->IF[1].CREQ & CAN_IF_CREQ_BUSY_Msk) {
        /*Wait*/
    }

    if((tCAN->IF[1].ARB2 & CAN_IF_ARB2_XTD_Msk) == 0) {
        /* standard ID*/
        pCanMsg->IdType = CAN_STD_ID;
        pCanMsg->Id     = (tCAN->IF[1].ARB2 & CAN_IF_ARB2_ID_Msk) >> 2;
    } else {
        /* extended ID*/
        pCanMsg->IdType = CAN_EXT_ID;
        pCanMsg->Id  = (((tCAN->IF[1].ARB2) & 0x1FFF) << 16) | tCAN->IF[1].ARB1;
    }

    pCanMsg->DLC     = tCAN->IF[1].MCON & CAN_IF_MCON_DLC_Msk;
    pCanMsg->Data[0] = tCAN->IF[1].DAT_A1 & CAN_IF_DAT_A1_DATA0_Msk;
    pCanMsg->Data[1] = (tCAN->IF[1].DAT_A1 & CAN_IF_DAT_A1_DATA1_Msk) >> CAN_IF_DAT_A1_DATA1_Pos;
    pCanMsg->Data[2] = tCAN->IF[1].DAT_A2 & CAN_IF_DAT_A2_DATA2_Msk;
    pCanMsg->Data[3] = (tCAN->IF[1].DAT_A2 & CAN_IF_DAT_A2_DATA3_Msk) >> CAN_IF_DAT_A2_DATA3_Pos;
    pCanMsg->Data[4] = tCAN->IF[1].DAT_B1 & CAN_IF_DAT_B1_DATA4_Msk;
    pCanMsg->Data[5] = (tCAN->IF[1].DAT_B1 & CAN_IF_DAT_B1_DATA5_Msk) >> CAN_IF_DAT_B1_DATA5_Pos;
    pCanMsg->Data[6] = tCAN->IF[1].DAT_B2 & CAN_IF_DAT_B2_DATA6_Msk;
    pCanMsg->Data[7] = (tCAN->IF[1].DAT_B2 & CAN_IF_DAT_B2_DATA7_Msk) >> CAN_IF_DAT_B2_DATA7_Pos;

    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if(u32IIDR == 1) {
        printf("Msg-0 INT and Callback\n");
        CAN_ReadMsgObj(tCAN, 0, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR == 5 + 1) {
        printf("Msg-5 INT and Callback \n");
        CAN_ReadMsgObj(tCAN, 5, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR == 31 + 1) {
        printf("Msg-31 INT and Callback \n");
        CAN_ReadMsgObj(tCAN, 31, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000) {      /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/

            printf("RxOK INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/

            printf("TxOK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_EWARN_Msk) {
            printf("EWARN INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk) {
            printf("BOFF INT\n") ;
        }
    } else if(u8IIDRstatus != 0) {
        printf("=> Interrupt Pointer = %d\n", CAN0->IIDR - 1);

        CAN_MsgInterrupt(CAN0, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN0, ((CAN0->IIDR) - 1)); /* Clear Interrupt Pending */
    } else if(CAN0->WU_STATUS == 1) {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;    /* Write '0' to clear */
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN1_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN1->IIDR;

    if(u8IIDRstatus == 0x00008000) {      /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN1->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/

            printf("RxOK INT\n") ;
        }

        if(CAN1->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/

            printf("TxOK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN1->STATUS & CAN_STATUS_EWARN_Msk) {
            printf("EWARN INT\n") ;
        }

        if(CAN1->STATUS & CAN_STATUS_BOFF_Msk) {
            printf("BOFF INT\n") ;
        }
    } else if(u8IIDRstatus != 0) {
        printf("=> Interrupt Pointer = %d\n", CAN1->IIDR - 1);

        CAN_MsgInterrupt(CAN1, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN1, ((CAN1->IIDR) - 1)); /* Clear Interrupt Pending */
    } else if(CAN1->WU_STATUS == 1) {
        printf("Wake up\n");

        CAN1->WU_STATUS = 0;    /* Write '0' to clear */
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
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    FMC->FATCON |= 0x50;
    
    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Enable module clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk
                  | CLK_APBCLK_CAN0_EN_Msk;
    //| CLK_APBCLK_CAN1_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set PD multi-function pins for CANTX0, CANRX0 */
    SYS->GPB_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP |= SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;

    /* Set PA multi-function pins for CANTX1, CANRX1 */
//     SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA10_Msk | SYS_GPA_MFP_PA11_Msk);
//     SYS->GPA_MFP = SYS_GPA_MFP_PA10_CAN1_TXD | SYS_GPA_MFP_PA11_CAN1_RXD;
//     SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA10_Msk | SYS_ALT_MFP_PA11_Msk);
//     SYS->ALT_MFP |= SYS_ALT_MFP_PA10_CAN1_TXD | SYS_ALT_MFP_PA11_CAN1_RXD;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*----------------------------------------------------------------------------*/
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void Note_Configure()
{
    printf("\n\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  About CAN sample code configure                                         |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN         |\n");
    printf("|   Before execute it, please check description as below                   |\n");
    printf("|                                                                          |\n");
    printf("|   1.CAN_TX and CAN_RX should be connected to your CAN transceiver        |\n");
    printf("|   2.Using two module board and connect to the same CAN BUS               |\n");
    printf("|   3.Check the terminal resistor of bus is connected                      |\n");
    printf("|   4.Using UART0 as print message port                                    |\n");
    printf("|                                                                          |\n");
    printf("|  |--------|       |-----------|   CANBUS  |-----------|       |--------| |\n");
    printf("|  |        |------>|           |<--------->|           |<------|        | |\n");
    printf("|  |        |CAN_TX |    CAN    |   CAN_H   |   CAN     |CAN_TX |        | |\n");
    printf("|  | NUC2XX |       |Transceiver|           |Transceiver|       | NUC2XX | |\n");
    printf("|  |        |<------|           |<--------->|           |------>|        | |\n");
    printf("|  |        |CAN_RX |           |   CAN_L   |           |CAN_RX |        | |\n");
    printf("|  |--------|       |-----------|           |-----------|       |--------| |\n");
    printf("|  |                                                            |          |\n");
    printf("|  |                                                            |          |\n");
    printf("|  V                                                            V          |\n");
    printf("| UART0                                                         UART0      |\n");
    printf("|(print message)                                          (print message)  |\n");
    printf("+--------------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Test Menu                                                                 */
/*----------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("\n");
    printf("+------------------------------------------------------------------ +\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                                                                   |\n");
    printf("|      Set Mask Filter                                              |\n");
    printf("|                                                                   |\n");
    printf("+-------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Set target baud-rate and Basic mode                                       */
/*----------------------------------------------------------------------------*/
void CAN_Init(CAN_T *tCAN, uint32_t u32kbps)
{
    uint8_t u8Tseg1, u8Tseg2;
    uint32_t u32Brp;
    uint32_t u32Value;

    /* Set the CAN to enter initialization mode and enable access bit timing register */
    tCAN->CON |= CAN_CON_INIT_Msk;
    tCAN->CON |= CAN_CON_CCE_Msk;

    SystemCoreClockUpdate();

    u32Value = (SystemCoreClock / 1000) / u32kbps;
    u32Brp = u32Value;
    u8Tseg1 = 3;        /* Time Segment After sample Point */
    u8Tseg2 = 2;        /* Time Segment before the sample Point */

    /* Determine the value of Tseg1, Tseg2 and BRP */
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

    u32Brp  = u32Brp / (u8Tseg1 + u8Tseg2 + 3) - 1;
    
    /* Set the TSeg2, TSeg1, SJW and BRP for Bit Timing Register */
    u32Value = ((uint32_t)u8Tseg2 << CAN_BTIME_TSEG2_Pos) | ((uint32_t)u8Tseg1 << CAN_BTIME_TSEG1_Pos) |
               (u32Brp & CAN_BTIME_BRP_Msk) | (tCAN->BTIME & CAN_BTIME_SJW_Msk);
    tCAN->BTIME = u32Value;

    /* Set the BRPE for Baud Rate Prescaler Extension Register */
    tCAN->BRPE = (u32Brp >> 6) & 0x0F;

    /* Set the CAN to leave initialization mode */
    tCAN->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
    while(tCAN->CON & CAN_CON_INIT_Msk); /* Check INIT bit is released */

}

/*----------------------------------------------------------------------------*/
/*  Get current bit rate                                                      */
/*----------------------------------------------------------------------------*/
uint32_t GetCANBitRate(CAN_T  *tCAN)
{
    uint8_t u8Tseg1, u8Tseg2;
    uint32_t u32Bpr;

    u8Tseg1 = (tCAN->BTIME & CAN_BTIME_TSEG1_Msk) >> CAN_BTIME_TSEG1_Pos;
    u8Tseg2 = (tCAN->BTIME & CAN_BTIME_TSEG2_Msk) >> CAN_BTIME_TSEG2_Pos;
    u32Bpr  = (tCAN->BTIME & CAN_BTIME_BRP_Msk) | (tCAN->BRPE << 6);

    return (SystemCoreClock / (u32Bpr + 1) / (u8Tseg1 + u8Tseg2 + 3));
}

/*----------------------------------------------------------------------------*/
/*  Set the CAN speed                                                         */
/*----------------------------------------------------------------------------*/
void SelectCANSpeed(CAN_T  *tCAN)
{
    uint32_t unItem;

    printf("Please select CAN speed you desired\n");
    printf("[0] 1000Kbps\n");
    printf("[1]  500Kbps\n");
    printf("[2]  250Kbps\n");
    printf("[3]  125Kbps\n");
    printf("[4]  100Kbps\n");
    printf("[5]   50Kbps\n");

    unItem = GetChar();
    printf("%c\n", unItem);
    if(unItem == '1')
        CAN_Init(tCAN,  500);//Set target baud-rate and Basic mode
    else if(unItem == '2')
        CAN_Init(tCAN,  250);
    else if(unItem == '3')
        CAN_Init(tCAN,  125);
    else if(unItem == '4')
        CAN_Init(tCAN,  100);
    else if(unItem == '5')
        CAN_Init(tCAN,   50);
    else
        CAN_Init(tCAN, 1000);

    printf("Real baud-rate value(bps): %d\n", GetCANBitRate(tCAN));

}

/*---------------------------------------------------------------------------------------------------------*/
/* Disable CAN                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_STOP(void)
{
    /* Disable CAN0 Clock and Reset it */
    SYS->IPRSTC2 |= SYS_IPRSTC2_CAN0_RST_Msk;   /* CAN0 controller reset */
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_CAN0_RST_Msk;  /* CAN0 controller normal operation */
    CLK->APBCLK &= ~CLK_APBCLK_CAN0_EN_Msk;     /* CAN0 clock disabled */

    /* Disable CAN1 Clock and Reset it */
    SYS->IPRSTC2 |= SYS_IPRSTC2_CAN1_RST_Msk;   /* CAN1 controller reset */
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_CAN1_RST_Msk;  /* CAN1 controller normal operation */
    CLK->APBCLK &= ~CLK_APBCLK_CAN1_EN_Msk;     /* CAN1 clock disabled */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Check if SmartCard slot is presented                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t CAN_GetFreeIF(CAN_T *tCAN)
{
    /* Check Read/write action has finished */
    if((tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk) == 0)
        return 0;
    else if((tCAN->IF[1].CREQ  & CAN_IF_CREQ_BUSY_Msk) == 0)
        return 1;
    else
        return 2;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Configures Mask as the message object                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t CAN_SetMsgObjMask(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMASK_T* MaskMsg)
{
    /* Set the Message Buffer Register */
    uint8_t u8MsgIfNum = 0;

    /* Check Free Interface for configure */
    if((u8MsgIfNum = CAN_GetFreeIF(tCAN)) == 2) {
        return FALSE;
    }

    if(MaskMsg->u8IdType == CAN_STD_ID) {
        /* Set the Mask Standard ID(11-bit) for IFn Mask Register is used for acceptance filtering*/
        tCAN->IF[u8MsgIfNum].MASK1 =  0;
        tCAN->IF[u8MsgIfNum].MASK2 = ((MaskMsg->u32Id & 0x7FF) << 2) ;
    } else {
        /* Set the Mask Extended ID(29-bit) for IFn Mask Register is used for acceptance filtering*/
        tCAN->IF[u8MsgIfNum].MASK1 = (MaskMsg->u32Id) & 0xFFFF;
        tCAN->IF[u8MsgIfNum].MASK2 = ((MaskMsg->u32Id) & 0x1FFF0000) >> 16 ;
    }

    if(MaskMsg->u8Xtd)
        tCAN->IF[u8MsgIfNum].MASK2 |= CAN_IF_MASK2_MXTD_Msk;            /* The extended identifier bit (IDE) is used for acceptance filtering */
    else
        tCAN->IF[u8MsgIfNum].MASK2 &= (~CAN_IF_MASK2_MXTD_Msk);  /* The extended identifier bit (IDE) has no effect on the acceptance filtering */

    if(MaskMsg->u8Dir)
        tCAN->IF[u8MsgIfNum].MASK2 |= CAN_IF_MASK2_MDIR_Msk;     /* The message direction bit (Dir) is used for acceptance filtering */
    else
        tCAN->IF[u8MsgIfNum].MASK2 &= (~CAN_IF_MASK2_MDIR_Msk);  /* The message direction bit (Dir) has no effect on the acceptance filtering */

    tCAN->IF[u8MsgIfNum].MCON |= CAN_IF_MCON_UMASK_Msk;                 /* Use Mask (Msk28-0, MXtd, and MDir) for acceptance filtering */

    /* Update the contents needed for transmission*/
    tCAN->IF[u8MsgIfNum].CMASK = CAN_IF_CMASK_WRRD_Msk      /* Transfer data from the selected Message Buffer Registers to the Message Object addressed */
                                 | CAN_IF_CMASK_MASK_Msk;     /* Transfer Identifier Mask + MDir + MXtd to Message Object  */

    /* Set the Message Object in the Message RAM is selected for data transfer */
    tCAN->IF[u8MsgIfNum].CREQ  = 1 + u8MsgObj;

    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Set Rx message object                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t CAN_SetRxMsgObj(CAN_T  *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint8_t u8singleOrFifoLast)
{
    uint8_t u8MsgIfNum = 0;

    if((u8MsgIfNum = CAN_GetFreeIF(tCAN)) == 2) {                       /* Check Free Interface for configure */
        return FALSE;
    }
    /* Command Setting */
    tCAN->IF[u8MsgIfNum].CMASK = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_MASK_Msk | CAN_IF_CMASK_ARB_Msk |
                                 CAN_IF_CMASK_CONTROL_Msk | CAN_IF_CMASK_DATAA_Msk | CAN_IF_CMASK_DATAB_Msk;

    if(u8idType == CAN_STD_ID) {  /* According STD/EXT ID format,Configure Mask and Arbitration register */
        tCAN->IF[u8MsgIfNum].ARB1 = 0;
        tCAN->IF[u8MsgIfNum].ARB2 = CAN_IF_ARB2_MSGVAL_Msk | (u32id & 0x7FF) << 2;
    } else {
        tCAN->IF[u8MsgIfNum].ARB1 = u32id & 0xFFFF;
        tCAN->IF[u8MsgIfNum].ARB2 = CAN_IF_ARB2_MSGVAL_Msk | CAN_IF_ARB2_XTD_Msk | (u32id & 0x1FFF0000) >> 16;
    }

    tCAN->IF[u8MsgIfNum].MCON |= CAN_IF_MCON_UMASK_Msk | CAN_IF_MCON_RXIE_Msk;
    if(u8singleOrFifoLast)
        tCAN->IF[u8MsgIfNum].MCON |= CAN_IF_MCON_EOB_Msk;
    else
        tCAN->IF[u8MsgIfNum].MCON &= (~CAN_IF_MCON_EOB_Msk);

    tCAN->IF[u8MsgIfNum].DAT_A1  = 0;
    tCAN->IF[u8MsgIfNum].DAT_A2  = 0;
    tCAN->IF[u8MsgIfNum].DAT_B1  = 0;
    tCAN->IF[u8MsgIfNum].DAT_B2  = 0;

    tCAN->IF[u8MsgIfNum].CREQ = 1 + u8MsgObj;

    return TRUE;
}

/*----------------------------------------------------------------------------*/
/*  Set the Mask Message Function                                                                       */
/*----------------------------------------------------------------------------*/
void Test_SetMaskFilter(CAN_T *tCAN)
{
    /* Declare a CAN message structures */
    STR_CANMASK_T MaskMsg;

    /* Enable CAN interrupt */
    tCAN->CON |= CAN_CON_IE_Msk;

    /* Enable corresponding NVIC of CAN */
    NVIC_EnableIRQ(CAN0_IRQn);
    NVIC_EnableIRQ(CAN1_IRQn);

    /* Set b'0' means don't care*/
    /* Set Message Object No.0 mask ID */
    MaskMsg.u8Xtd    = 1;               /* 1: 29-bit Extended Identifier or 0:11-bit Standard Identifier */
    MaskMsg.u8Dir    = 1;               /* 1:Direction is transmit or 0:Direction is receive*/
    MaskMsg.u8IdType = 0;       /* 1: 29-bit Extended Identifier or 0:11-bit Standard Identifier */
    MaskMsg.u32Id    = 0x707;       /* Set the Message Identifier  */

    /* Configures Mask as the message object */
    CAN_SetMsgObjMask(tCAN, MSG(0), &MaskMsg);

    /* Set Rx message object */
    CAN_SetRxMsgObj(tCAN, MSG(0), CAN_STD_ID, 0x7FF , TRUE);

    printf("Set Receive Message Object Content\n");
    printf("===================================\n");
    printf("Message Object: No.0 \n");
    printf("Identifier Type: Standard\n");
    printf("Message Direction: Transmit\n");
    printf("Message Identifier: 0x707\n\n\n");
    printf("Set Mask Content\n");
    printf("===================================\n");
    printf("Compare Xtd: %s\n", MaskMsg.u8Xtd ? "YES" : "NO");
    printf("Compare Dir: %s\n", MaskMsg.u8Dir ? "YES" : "NO");
    printf("Compare MaskID:%s 0x%x\n\n\n", MaskMsg.u8IdType ? "EXT" : "STD", MaskMsg.u32Id);
    printf("If there is a message-ID 0x700~0x70F,\nONLY 0x707/0x70F can pass acceptance filter.\n");

    printf("Waiting Message\n");

    GetChar();

    /* Disable CAN interrupt */
    tCAN->CON &= ~(CAN_CON_IE_Msk);

    /* Disable corresponding NVIC of CAN */
    NVIC_DisableIRQ(CAN0_IRQn);
    NVIC_DisableIRQ(CAN1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    CAN_T *tCAN;
    tCAN = (CAN_T *) CAN0;
    //tCAN = (CAN_T *) CAN1;

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
     PE->PMD = (GPIO_PMD_OUTPUT << 2*2) | (GPIO_PMD_OUTPUT << 3*2);
     PE2 = 0;
     PE3 = 0;

    /* Some description about how to create test environment */
    Note_Configure();

    /* Configuring the Bit Timing */
    SelectCANSpeed(tCAN);

    /* Test Menu */
    TestItem();

    printf("Transmit a message by normal mode\n\n");
    printf("Please confirm receiver is ready.\n");
    printf("Press any key to continue ...\n\n");
    GetChar();

    /* Set the Mask Message */
    Test_SetMaskFilter(tCAN);

    /* Disable CAN */
    CAN_STOP();

    while(1);

}



