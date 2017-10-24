#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_Timer_Input_Capture.h"

void Initial_Timer_Port(void)
{


    /* Set PB multi-function pins for UART0 RXD, UART0 TXD; TMR0 ~ TMR3 external counter pins; TMR0, TMR2 and TMR3 external capture pins */
    SYS->GPB_MFP |= SYS_GPB_MFP_PB10_TM2 | SYS_GPB_MFP_PB3_TM3_EXT;

    /* Set ALT MPF settings for TMR0 ~ TMR3 external counter and capture functions */
    SYS->ALT_MFP |= SYS_ALT_MFP_PB10_TM2 | SYS_ALT_MFP_PB3_TM3_EXT;

    /* Set ALT MPF1 settings for TMR3 external capture */
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PB3_TM3_EXT;
}

void Initial_Timer2_Toggle(void)
{

    CLK->APBCLK |= CLK_APBCLK_TMR2_EN_Msk;

    CLK->CLKSEL1 |= CLK_CLKSEL1_TMR2_S_HXT;

    TIMER_Open(TIMER2, TIMER_TOGGLE_MODE, 1000);
    TIMER_Start(TIMER2);
}


void Initial_Timer2_Count(void)
{
    CLK->APBCLK |= CLK_APBCLK_TMR2_EN_Msk;

    CLK->CLKSEL1 |= CLK_CLKSEL1_TMR2_S_HXT;

    /* Initial Timer1 default setting */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);

    /* Configure Timer1 setting for external counter input function */
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_FALLING_EDGE);
    TIMER_Start(TIMER2);

}

void initial_Timer3_Capture(void)
{
    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_TMR3_EN_Msk;

    CLK->CLKSEL1 |= CLK_CLKSEL1_TMR3_S_HXT;


    /* Initial Timer2 default setting */
    TIMER_Open(TIMER3, TIMER_CONTINUOUS_MODE, 1);

    /* Configure Timer2 setting for external counter input and capture function */
    TIMER_SET_PRESCALE_VALUE(TIMER3, 0);
    TIMER_SET_CMP_VALUE(TIMER3, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER3, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableCapture(TIMER3, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_FALLING_EDGE);
    //TIMER_EnableCaptureInt(TIMER3);
    TIMER_Start(TIMER3);
}

unsigned int Get_Timer3_Capture(void)
{
    unsigned int g_au32TMRINTCount[2];
    while(TIMER_GetCaptureIntFlag(TIMER3) == 0);
    g_au32TMRINTCount[0] = TIMER_GetCaptureData(TIMER3);
    /* Clear Timer2 capture interrupt flag */
    TIMER_ClearCaptureIntFlag(TIMER3);

    while(TIMER_GetCaptureIntFlag(TIMER3) == 0);
    g_au32TMRINTCount[1] = TIMER_GetCaptureData(TIMER3);
    /* Clear Timer2 capture interrupt flag */
    TIMER_ClearCaptureIntFlag(TIMER3);
    return (g_au32TMRINTCount[1] - g_au32TMRINTCount[0]);
}

