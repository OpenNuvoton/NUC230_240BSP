#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_RGBLED.h"

void Initial_PWM_LED(void)
{

    /* Set PA12~PA14 multi-function pins for PWMA Channel0~2  */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA12_Msk | SYS_GPA_MFP_PA13_Msk | SYS_GPA_MFP_PA14_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA12_PWM0 | SYS_GPA_MFP_PA13_PWM1 | SYS_GPA_MFP_PA14_PWM2;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA12_Msk | SYS_ALT_MFP_PA13_Msk | SYS_ALT_MFP_PA14_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PA12_PWM0 | SYS_ALT_MFP_PA13_PWM1 | SYS_ALT_MFP_PA14_PWM2;
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PA12_Msk | SYS_ALT_MFP1_PA13_Msk | SYS_ALT_MFP1_PA14_Msk);
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PA12_PWM0 | SYS_ALT_MFP1_PA13_PWM1 | SYS_ALT_MFP1_PA14_PWM2;
}

void PWM_LED(unsigned char ch, unsigned int ch0_fre, unsigned int ch0_dut, unsigned int ch1_fre, unsigned int ch1_dut, unsigned int ch2_fre, unsigned int ch2_dut)
{

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM01_MODULE);
    CLK_EnableModuleClock(PWM23_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HXT, 0);
    CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM23_S_HXT, 0);

    /* Reset PWMA channel0~channel3 */
    SYS_ResetModule(PWM03_RST);


    /* set PWMA channel 0 output configuration */
    PWM_ConfigOutputChannel(PWMA, PWM_CH0, ch0_fre, ch0_dut);
    PWM_ConfigOutputChannel(PWMA, PWM_CH1, ch1_fre, ch1_dut);
    PWM_ConfigOutputChannel(PWMA, PWM_CH2, ch2_fre, ch2_dut);

    /* Enable PWM Output path for PWMA channel 0 */
    PWM_EnableOutput(PWMA, ch);


    // Start
    PWM_Start(PWMA, ch);

}

