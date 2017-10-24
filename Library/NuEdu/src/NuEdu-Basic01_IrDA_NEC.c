#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_IrDA_NEC.h"
void IrDA_Code_Exe(uint8_t* IR_CODE1);

#define     Percent             0.04        // 4 % 
#define     MaxValue            0xFFFF

// Leader code range
#define     IR_LDC_MAX          (13460 * (1 + Percent))
#define     IR_LDC_MIN          (13460 * (1 - Percent))
// Repeater code range
#define     IR_RPC_MAX          (11280 * (1 + Percent))
#define     IR_RPC_MIN          (11280 * (1 - Percent))
// Bit = 1 range
#define     IR_BIT_1_MAX        (2236 * (1 + Percent))
#define     IR_BIT_1_MIN        (2236 * (1 - Percent))
// Bit = 0 range
#define     IR_BIT_0_MAX        (1120 * (1 + Percent))
#define     IR_BIT_0_MIN        (1120 * (1 - Percent))

volatile    uint8_t             IR_State = 0;       // IR State
volatile    uint8_t             IR_LDC_Ready = 0;   // LeaDer Code is Ready
volatile    uint8_t             IR_CTC_Ready = 0;   // CusTomer Code is Ready
volatile    uint8_t             IR_CTC0 = 0;        // Received CusTomer Code 0
volatile    uint8_t             IR_CTC1 = 0;        // Received CusTomer Code 1
volatile    uint8_t             IR_DAC = 0;         // Received DAta Code
volatile    uint8_t             IR_DAB = 0;         // Received DAta Bar code
volatile    uint8_t             IR_cnt = 0;
uint8_t             IR_CODE[4]  =   {0x00, 0x00, 0x00, 0x00};

void IrDa_NEC_Rx(uint32_t u32Time)
{
    if(IR_State == 0)
    {
        IR_LDC_Ready = 0;           // Clear LeaDer Code Ready
        IR_CTC_Ready = 0;           // Clear CusTomer Code Ready
        IR_State++;
    }
    // Leader or Repeater code
    else if(IR_State == 1)
    {
        // Leader code
        if((u32Time >= IR_LDC_MIN) && (u32Time <= IR_LDC_MAX))
        {
            IR_LDC_Ready = 1;       // Set LeaDer Code Ready
            IR_State++;
        }
        else
        {
            IR_State = 1;
            IR_LDC_Ready = 0;           // Clear LeaDer Code Ready
            IR_CTC_Ready = 0;           // Clear CusTomer Code Ready
        }
    }
    // Customer code 0
    else if((IR_State >= 2 && IR_State < 10) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_CTC0 = IR_CTC0 >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_CTC0 &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX)) // 2.25ms = 1
            IR_CTC0 |= 0x80;
        else
            IR_State = 0;
    }
    // Customer code 1
    else if((IR_State >= 10 && IR_State < 18) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_CTC1 = IR_CTC1 >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_CTC1 &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX)) // 2.25ms = 1
            IR_CTC1 |= 0x80;
        else
            IR_State = 0;
    }
    // Data
    else if((IR_State >= 18 && IR_State < 26) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_DAC = IR_DAC >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_DAC &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_DAC |= 0x80;
        else
            IR_State = 0;

    }
    // Data bar
    else if((IR_State >= 26 && IR_State < 34) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_DAB = IR_DAB >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))      // 1.12ms = 0
            IR_DAB &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_DAB |= 0x80;
        else
            IR_State = 0;

        if(IR_State == 34)
        {
            if((IR_DAC ^ IR_DAB) == 0xff)
            {
                IR_LDC_Ready = 0;   // Clear LeaDer Code Ready
                IR_CODE[0] = IR_CTC0;
                IR_CODE[1] = IR_CTC1;
                IR_CODE[2] = IR_DAC;
                IR_CODE[3] = IR_DAB;
                IR_cnt++;
                IrDA_Code_Exe(IR_CODE);
                printf("IR_cnt=%d, CTC0=%02x, CTC1=%02x, DAC=%02x, DAB=%02x\n", IR_cnt, IR_CTC0, IR_CTC1, IR_DAC, IR_DAB);
            }
            IR_State = 0;
        }
    }
}

void PWMB_IRQHandler(void)
{
    uint32_t TDR1_tmp;
    TDR1_tmp = MaxValue - PWM_GET_CAPTURE_FALLING_DATA(PWMB, 3);
    //printf("%d\n\r",TDR1_tmp);
    PWM_ClearCaptureIntFlag(PWMB, 3, PWM_CAPTURE_INT_FALLING_LATCH);
    IrDa_NEC_Rx(TDR1_tmp);
}

#define     NEC_LDC_MARK        16      // 16 x 560us = 8960us =   9ms
#define     NEC_LDC_SPACE       8       //  8 x 560us = 4480us = 4.5ms
#define     NEC_BIT_MARK        1       // 560us
#define     NEC_ONE_SPACE       3       //  3 x 560us = 1680us = 1690us
#define     NEC_ZERO_SPACE      1       // 560us
#define     NEC_BYTES           4


//*******************************
// Mark
//
// Pulse = 1/3 duty @38KHz frequency
//*******************************
void Mark(uint8_t N)
{
    /* Switch to PWM output waveform */
    PWM_EnableOutput(PWMB, 0X04);
    CLK_SysTickDelay(560 * N);
    PWM_DisableOutput(PWMB, 0X04);
}
void SPACE(uint8_t N)
{
    CLK_SysTickDelay(560 * N);
}
//*******************************
// Send NEC IR Format
//
// PC15 (PWM1_CH3)
//*******************************
void SendNEC(uint8_t* data)
{
    uint8_t nbyte;
    uint8_t nbit;

    /* Send out Leader code */
    Mark(NEC_LDC_MARK);
    SPACE(NEC_LDC_SPACE);

    /* Send out Customer code and Data code */
    for(nbyte = 0; nbyte < NEC_BYTES; nbyte++)
    {
        for(nbit = 0; nbit < 8; nbit++)
        {
            Mark(NEC_BIT_MARK);
            if(data[nbyte] & (1 << nbit))       // LSB first
                SPACE(NEC_ONE_SPACE);
            else
                SPACE(NEC_ZERO_SPACE);
        }
    }

    /* Send out Stop bit */
    Mark(NEC_BIT_MARK);

}


void IrDA_NEC_TxRx_Init(void)
{
    //Initial PWM7 PWM6 Function Pin
    SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE1_Msk) | SYS_GPE_MFP_PE1_PWM7;
    SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE0_Msk) | SYS_GPE_MFP_PE0_PWM6;
    GPIO_SetMode(PA, BIT15, GPIO_PMD_INPUT);
    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM67_MODULE);


    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM67_MODULE, CLK_CLKSEL2_PWM67_S_HXT, 0);

    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM47_RST);

    /* Enable Backward Compatible: write 1 to clear CFLRI0~3 and CRLRI0~3 */
    PWMB->PBCR = 1;
    PWMB->PCR |= PWM_PCR_CH3MOD_Msk | PWM_PCR_CH2MOD_Msk;
    PWM_SET_DIVIDER(PWMB, 3, PWM_CLK_DIV_4);                            //Set PWMB_CH3 Clock Source Divider
    PWM_SET_PRESCALER(PWMB, 3, 2);                                      //Set PWM1_CH3 and PWM1_CH3 Prescaler
    PWMB->CNR3 = MaxValue;

    PWM_SET_DIVIDER(PWMB, 2, PWM_CLK_DIV_1);                           //Set PWMB_CH2 Clock Source Divider
    PWM_SET_CNR(PWMB, 2, ((120000000 / 3 / 38000 + 5) / 10));
    PWM_SET_CMR(PWMB, 2, ((120000000 / 3 / 38000 + 5) / 10) / 2);
    //Start Capture
    PWM_EnableCapture(PWMB, 0x8);

    PWM_EnableCaptureInt(PWMB, 3, PWM_CAPTURE_INT_FALLING_LATCH);
    NVIC_EnableIRQ(PWMB_IRQn);
    PWM_Start(PWMB, 0x08);

    PWM_Start(PWMB, 0X04);
}

void IrDA_NEC_Rx_Init(void)
{
    //Initial PWM7 Function Pin
    SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE1_Msk) | SYS_GPE_MFP_PE1_PWM7;

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM67_MODULE);


    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM67_MODULE, CLK_CLKSEL2_PWM67_S_HXT, 0);

    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM47_RST);

    /* Enable Backward Compatible: write 1 to clear CFLRI0~3 and CRLRI0~3 */
    PWMB->PBCR = 1;
    PWMB->PCR |= PWM_PCR_CH3MOD_Msk;
    PWM_SET_DIVIDER(PWMB, 3, PWM_CLK_DIV_4);                            //Set PWM1_CH3 Clock Source Divider
    PWM_SET_PRESCALER(PWMB, 3, 2);                                      //Set PWM1_CH2 and PWM1_CH3 Prescaler
    PWMB->CNR3 = MaxValue;

    //Start Capture
    PWM_EnableCapture(PWMB, 0x8);

    PWM_EnableCaptureInt(PWMB, 3, PWM_CAPTURE_INT_FALLING_LATCH);
    NVIC_EnableIRQ(PWMB_IRQn);
    PWM_Start(PWMB, 0x8);
}



void IrDA_NEC_Tx_Init(void)
{
    //Initial PWM7 Function Pin
    SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE0_Msk) | SYS_GPE_MFP_PE0_PWM6;
    GPIO_SetMode(PA, BIT15, GPIO_PMD_INPUT);
    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM67_MODULE);

    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM47_RST);


    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM67_MODULE, CLK_CLKSEL2_PWM67_S_HXT, 0);
    PWMB->PBCR = 1;
    PWMB->PCR |= PWM_PCR_CH2MOD_Msk;
    PWM_SET_DIVIDER(PWMB, 2, PWM_CLK_DIV_1);                           //Set PWM1_CH2 Clock Source Divider

    PWM_SET_CNR(PWMB, 2, ((120000000 / 3 / 38000 + 5) / 10));
    PWM_SET_CMR(PWMB, 2, ((120000000 / 3 / 38000 + 5) / 10) / 2);

    PWM_Start(PWMB, 0X04);
}

