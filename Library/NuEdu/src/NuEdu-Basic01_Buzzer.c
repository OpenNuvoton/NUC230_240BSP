#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_Buzzer.h"

void Open_Buzzer(void)
{
    SYS->GPE_MFP |= SYS_GPE_MFP_PE5_PWM5;
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PE5_PWM5;
}

void Write_Buzzer(unsigned Enable, unsigned  int Frequency, unsigned int Duty)
{

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM45_MODULE);


    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM45_MODULE, CLK_CLKSEL2_PWM45_S_HXT, 0);

    /* Reset PWMA channel4~channel7 */
    SYS_ResetModule(PWM47_RST);


    /* set PWMA channel 0 output configuration */
    PWM_ConfigOutputChannel(PWMB, PWM_CH1, Frequency, Duty);

    // Start
    PWM_Start(PWMB, 0x2);
    if(Enable == 1)
        /* Enable PWM Output path for PWMA channel 0 */
        PWM_EnableOutput(PWMB, 0x2);
    else
        PWM_DisableOutput(PWMB, 0x2);
}



