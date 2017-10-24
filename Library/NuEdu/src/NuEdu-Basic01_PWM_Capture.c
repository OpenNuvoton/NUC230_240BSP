#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_PWM_Capture.h"


void Open_PWM6_OUT(uint32_t Enable, uint32_t PWM_Frequency, uint32_t PWM_Duty)
{

    //SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE0_Msk) | SYS_GPE_MFP_PE0_PWM6;
    SYS->GPE_MFP |= SYS_GPE_MFP_PE0_PWM6;
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PE0_PWM6;

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM67_MODULE);


    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM67_MODULE, CLK_CLKSEL2_PWM67_S_HXT, 0);

    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM47_RST);


    /* set PWMA channel 2 output configuration */
    PWM_ConfigOutputChannel(PWMB, PWM_CH2, PWM_Frequency, PWM_Duty);

    // Start
    PWM_Start(PWMB, 0x4);
    if(Enable == 1)
        /* Enable PWM Output path for PWMB channel 3 */
        PWM_EnableOutput(PWMB, 0x4);
    else
        PWM_DisableOutput(PWMB, 0x4);

}

void Open_PWM3_Capture(void)
{
    SYS->GPA_MFP = (SYS->GPA_MFP & ~SYS_GPA_MFP_PA15_Msk) | SYS_GPA_MFP_PA15_PWM3;
    SYS->ALT_MFP = (SYS->ALT_MFP & ~SYS_ALT_MFP_PA15_Msk) | SYS_ALT_MFP_PA15_PWM3;
    SYS->ALT_MFP1 = (SYS->ALT_MFP1 & ~SYS_ALT_MFP1_PA15_Msk) | SYS_ALT_MFP1_PA15_PWM3;


    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM23_MODULE);


    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM23_S_HXT, 0);

    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM03_RST);

    /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
       Capture clock source frequency = __HXT = 12000000 in the sample code.
       (CNR+1) = Capture clock source frequency/prescaler/clock source divider/minimum input frequency
               = 12000000/2/1/250 = 24000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescaler value.)
       CNR = 0xFFFF
       (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
    */

    /* set PWMB channel 2 capture configuration */
    PWM_ConfigCaptureChannel(PWMA, PWM_CH3, 166, 0);

    /* Enable Backward Compatible: write 1 to clear CFLRI0~3 and CRLRI0~3 */
    PWMA->PBCR = 1;

    /* Enable capture falling edge interrupt for PWMB channel 2 */
    PWM_EnableCaptureInt(PWMA, PWM_CH3, PWM_CAPTURE_INT_FALLING_LATCH);

    /* Enable Timer for PWMA channel 3  */
    PWM_Start(PWMA, 0x8);

    /* Enable Capture Function for PWMA channel 3 */
    PWM_EnableCapture(PWMA, 0x8);
}




void Open_PWM7_Capture(void)
{
    //Initial PWM7 Function Pin
    SYS->GPE_MFP = (SYS->GPE_MFP & ~SYS_GPE_MFP_PE1_Msk) | SYS_GPE_MFP_PE1_PWM7;

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM67_MODULE);


    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM67_MODULE, CLK_CLKSEL2_PWM67_S_HXT, 0);

    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM47_RST);

    /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
       Capture clock source frequency = __HXT = 12000000 in the sample code.
       (CNR+1) = Capture clock source frequency/prescaler/clock source divider/minimum input frequency
               = 12000000/2/1/250 = 24000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescaler value.)
       CNR = 0xFFFF
       (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
    */

    /* set PWMB channel 2 capture configuration */
    PWM_ConfigCaptureChannel(PWMB, PWM_CH3, 166, 0);

    /* Enable Backward Compatible: write 1 to clear CFLRI0~3 and CRLRI0~3 */
    PWMB->PBCR = 1;

    /* Enable capture falling edge interrupt for PWMB channel 2 */
    PWM_EnableCaptureInt(PWMB, PWM_CH3, PWM_CAPTURE_INT_FALLING_LATCH);

    /* Enable Timer for PWMA channel 3  */
    PWM_Start(PWMB, 0x8);

    /* Enable Capture Function for PWMA channel 3 */
    PWM_EnableCapture(PWMB, 0x8);
}


void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid;

    /* Clear Capture Falling Indicator (Time A) */
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    /* Wait for Capture Falling Indicator  */
    while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

    /* Clear Capture Falling Indicator (Time B)*/
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

        /* Clear Capture Falling and Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_FALLING_DATA(PWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

        /* Clear Capture Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_RISING_DATA(PWM, u32Ch);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeroid = u32Count[1] - u32Count[2];

    u16LowPeroid = 0x10000 - u32Count[1];

    u16TotalPeroid = 0x10000 - u32Count[2];

    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid);
}

void Get_PWM3_Capture_Data(void)
{
    /* Wait until PWMA channel 3 Timer start to count */
    while(PWMA->PDR3 == 0);

    /* Capture the Input Waveform Data */
    CalPeriodTime(PWMA, PWM_CH3);
}


void Get_PWM7_Capture_Data(void)
{
    /* Wait until PWMB channel 3 Timer start to count */
    while(PWMB->PDR3 == 0);

    /* Capture the Input Waveform Data */
    CalPeriodTime(PWMB, PWM_CH3);
}
