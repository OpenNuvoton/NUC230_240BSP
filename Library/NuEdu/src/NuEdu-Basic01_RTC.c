#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_RTC.h"

void RTC_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12 MHz XTAL, 32 kHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL32K_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL32K_STB_Msk);

    /* Enable peripheral clock */
    CLK->APBCLK |= CLK_APBCLK_RTC_EN_Msk;

    /* lock protected registers */
    SYS_LockReg();
}

void RTC_SET(S_RTC_TIME_DATA_T sWriteRTC)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    RTC_Open(&sWriteRTC);
}
