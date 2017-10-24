///YHTANG
#include <stdio.h>
#include "NUC230_240.h"
#include "DMX512.h"

uint8_t u8GetDmx512ResetSignal;
uint8_t u8DMX512ReceiveDone;
uint16_t u16ReceivingChannelIndex;
uint16_t u16ReceivedTotalChannelNum;
uint16_t u16ReceiveBufferPointer;
uint8_t u16ReceiveBufferArea;

uint8_t u8ReceivedData;
uint8_t u8WaitTimer1Flag = 0;
extern uint32_t u32SysTimerDMX512SendCounter;
extern uint8_t u8DMX512TransData[] ;
extern uint8_t  u8DMX512RevBuffer[];
extern uint32_t u32SysTimerCounter;
#define DMX512_ERROR_NONE  0
#define DMX512_ERROR_DATA_EXCEED_512CHANNEL 1
#define DMX512_ERROR_MTBP_EXCEED_1S  2
#define DMX512_ERROR_MTBF_EXCEED_1S  3
#define DMX512_ERROR_DATA_ERROR 4
uint8_t u8DMX512ReceiveErrorCode;
uint32_t u32SysTimerDMX512MTBF = 0;

void DMX512_UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    //DMX512_UART
    /* Configure UART0 and set UART0 Baud-rate */
    UART_Open(UART0, 250000);
}


void TMR1_IRQHandler(void)
{
    u8WaitTimer1Flag = 1;
    // clear Timer0 interrupt flag
    TIMER_ClearIntFlag(TIMER1);
}


/****************************************************************************************/
void RestartDMX512Receive(void)
{
    u8DMX512ReceiveDone = 0;                //=1,receive a package
    u16ReceivedTotalChannelNum = 0;
    u8GetDmx512ResetSignal = 0;             //=1,wait DMX reset signal
    u16ReceivingChannelIndex = 0;           //DMX Channel Index
    u16ReceiveBufferArea = 0;           		//DMX buffer area select, 0: first 513 1:second 513
}



void DMX512_SendInit(void)
{

    //DISABLE RLS,RDA ,TRA INT
    UART_DisableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RLS_IEN_Msk));
    //Enable Timer1 clock and select Timer1 clock source
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HXT, 0);

    //Enable Timer1 interrupt

    NVIC_EnableIRQ(TMR1_IRQn);

}


void DMX512_TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Us)
{
    uint32_t u32Clk = TIMER_GetModuleClock(timer);
    uint32_t u32Cmpr = 0;

    u32Cmpr = u32Us * (u32Clk / 1000000);
    timer->TCSR = u32Mode;
    timer->TCMPR = u32Cmpr;
}



void DMX512_SendData(void)
{
    uint32_t i = 0;
    uint32_t u32Clk = TIMER_GetModuleClock(TIMER1);
    uint32_t u32Cmpr = 0;
    while(UART0->FSR & UART_FSR_TX_FULL_Msk);
    //Initial Timer1 to periodic mode with 88us
#ifdef SUPPORT_DEBUGINFO
    GPIO_FOR_SIGNAL_OSCILLOSCOPE_LOW ;
#endif
// Must take attention  to  below code, because MAB=8us, code fetching time must be considered!
//  DMX512_TIMER_Open(TIMER1, TIMER_ONESHOT_MODE, 88);
    TIMER1->TCSR = TIMER_ONESHOT_MODE;
    u32Cmpr = 88 * (u32Clk / 1000000);
    TIMER1->TCMPR = u32Cmpr;
    UART0->LCR |= UART_LCR_BCB_Msk; //Force TX output logic 0;
    TIMER1->TCSR |= TIMER_TCSR_CEN_Msk;
    while((TIMER1->TISR & TIMER_TISR_TIF_Msk) == 0);
    TIMER1->TISR = TIMER_TISR_TIF_Msk;
    UART0->LCR &= ~UART_LCR_BCB_Msk; // Stop TX output logic 0;
    TIMER1->TCSR &= ~TIMER_TCSR_CEN_Msk;

    u32Cmpr = 36; // 2*(u32Clk/1000000);  //think programmer has know   u32Clk=12M*
    TIMER1->TCMPR = u32Cmpr;
    TIMER1->TCSR |= TIMER_TCSR_CEN_Msk;
    while((TIMER1->TISR & TIMER_TISR_TIF_Msk) == 0);
    UART0->DATA = u8DMX512TransData[0];
    //output STARTCODE+ 512 CHANNEL DATA,
    for(i = 1; i < (DMX512_CHANNEL_NUM + 1); i++)
    {
        while(UART0->FSR & UART_FSR_TX_FULL_Msk);
        UART0->DATA = u8DMX512TransData[i];
        if(i == 1)
        {
#ifdef SUPPORT_DEBUGINFO
            GPIO_FOR_SIGNAL_OSCILLOSCOPE_HIGH ;
#endif
        }
    }
    TIMER1->TISR = TIMER_TISR_TIF_Msk;
    TIMER1->TCSR &= ~TIMER_TCSR_CEN_Msk;


}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void DMX_512_Receive_Handle(void)
{
    uint32_t u32IntSts = UART0->ISR;
    if(u32IntSts & UART_ISR_RLS_INT_Msk)
    {
        if(UART0->FSR & UART_FSR_BIF_Msk)             //break interrupt
        {
            u8GetDmx512ResetSignal = 1;         //get break signal
            u8DMX512ReceiveDone = 1;           //use break signal as one packet received
            u16ReceivedTotalChannelNum = u16ReceivingChannelIndex;   //save received data number
            u16ReceivingChannelIndex = 0;
            u16ReceiveBufferArea ^= 0x01;       //DMX buffer area select, 0: first 513 1:second 513
#ifdef SUPPORT_DEBUGINFO
            if(u16ReceiveBufferArea)
            {
                GPIO_FOR_SIGNAL_OSCILLOSCOPE_LOW ;
            }
            else
            {
                GPIO_FOR_SIGNAL_OSCILLOSCOPE_HIGH ;
            }
#endif
        }
        else if(UART0->FSR & UART_FSR_FEF_Msk)   //frame error  ,received data is error
        {
            u8GetDmx512ResetSignal = 0;
            u8DMX512ReceiveDone = 1;
            u8DMX512ReceiveErrorCode = DMX512_ERROR_DATA_ERROR ;
            u16ReceivedTotalChannelNum = u16ReceivingChannelIndex;
            u16ReceiveBufferArea ^= 0x01;       //DMX buffer area select, 0: first 513 1:second 513
        }
        UART0->FSR |= UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_PEF_Msk; //clear RLS_IF =0
        u8ReceivedData = UART_READ(UART0);      //clear RDA interrupt flag
    }
    else if(u32IntSts & UART_ISR_RDA_INT_Msk)          //received data valid
    {
        u8ReceivedData = UART_READ(UART0);      //receive data
        if(u8GetDmx512ResetSignal)
        {
            if(u16ReceivingChannelIndex > DMX512_CHANNEL_NUM)
            {
                u8DMX512ReceiveDone = 0;                
                u8DMX512ReceiveErrorCode = DMX512_ERROR_DATA_EXCEED_512CHANNEL ;
                u16ReceivedTotalChannelNum = 0;         
                u8GetDmx512ResetSignal = 0;             
                u16ReceivingChannelIndex = 0;           
            }
            else
            {
                if(u16ReceiveBufferArea)              	//DMX buffer area select, 0: first 513 1:second 513
                {
                    u8DMX512RevBuffer[u16ReceivingChannelIndex + 513] = u8ReceivedData;
                }
                else
                {
                    u8DMX512RevBuffer[u16ReceivingChannelIndex] = u8ReceivedData;
                }
                u16ReceivingChannelIndex++;
            }
            u32SysTimerDMX512MTBF = u32SysTimerCounter;
        }
    }
    else                                //other interrupt, No use
    {
        u8DMX512ReceiveDone = 0;                
        u16ReceivedTotalChannelNum = 0;         
        u8GetDmx512ResetSignal = 0;             
        u16ReceivingChannelIndex = 0;          
    }
}

void UART02_IRQHandler(void)
{
    DMX_512_Receive_Handle();
}

void DMX512_ReceiveInit(void)
{

    /* Enable Interrupt and install the call back function */
    //set RX FIFO INT TRIGGER LEVEL
    UART0->FCR &= ~(UART_FCR_RFITL_Msk | UART_FCR_RTS_TRI_LEV_Msk);
    UART0->FCR |= ((0x0ul << UART_FCR_RFITL_Pos));
    UART_DisableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RLS_IEN_Msk));
    UART_EnableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_RLS_IEN_Msk));
}
