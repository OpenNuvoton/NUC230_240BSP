#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_ADC_Knob.h"


void Open_ADC_Knob(void)
{
    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);
    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

    /* Disable the GPA7 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PA, 0x01);

    /* Configure the GPA7 ADC analog input pins */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA0_Msk) ;
    SYS->GPA_MFP |= SYS_GPA_MFP_PA0_ADC0 ;

    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PA0_Msk);
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PA0_ADC0;

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 7 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 0x1 << 0);

    /* Power on ADC module */
    ADC_POWER_ON(ADC);



}



uint32_t Get_ADC_Knob(void)
{
    uint32_t ADC_Raw_Data;
    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    //Start ADC Conversion
    ADC_START_CONV(ADC);

    while(ADC_GET_INT_FLAG(ADC, ADC_ADF_INT) != ADC_ADF_INT);
    ADC_Raw_Data = ADC_GET_CONVERSION_DATA(ADC, 0);

    return ADC_Raw_Data;
}
