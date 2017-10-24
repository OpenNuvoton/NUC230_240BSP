#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_PWMDAC.h"


void Write_PWMDAC(unsigned char Enable, unsigned char ch0_dut)
{
    CLK_EnableModuleClock(PWM45_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM45_MODULE, CLK_CLKSEL2_PWM45_S_HXT, 0);


    /* Reset PWMB channel4~channel7 */
    SYS_ResetModule(PWM47_RST);


    /* set PWMB channel 0 output configuration */
    PWM_ConfigOutputChannel(PWMB, PWM_CH0, 1000, ch0_dut);


    // Start PWM COUNT
    PWM_Start(PWMB, 0x01);

    if(Enable == 0)
        /* Enable PWM Output path for PWMB channel 0 */
        PWM_DisableOutput(PWMB, 0x01);
    else
        /* Disable PWM Output path for PWMB channel 0 */
        PWM_EnableOutput(PWMB, 0x01);

}


void Initial_PWM_DAC(void)
{

    GPIO_SetMode(PA, 0x2, GPIO_PMD_INPUT); //avoid to PWM dac out
    SYS->GPB_MFP |= SYS_GPB_MFP_PB11_PWM4;
    SYS->ALT_MFP |= SYS_ALT_MFP_PB11_PWM4;
}

