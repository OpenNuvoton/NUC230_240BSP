#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_ACMP.h"

void Open_ACMP(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC.6 multi-function pin for ACMP0 positive input pin */
    SYS->GPC_MFP |= SYS_GPC_MFP_PC6_ACMP0_P | SYS_GPC_MFP_PC7_ACMP0_N;
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PC6_ACMP0_P | SYS_ALT_MFP1_PC7_ACMP0_N;


    /* Disable digital input path of analog pin ACMP0_P and ACMP0_N to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PC, (1ul << 6));
    GPIO_DISABLE_DIGITAL_PATH(PC, (1ul << 7));

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP_MODULE);

    /* Configure ACMP0. Enable ACMP0 and select internal reference voltage as negative input. */
    ACMP_Open(ACMP, 0, ACMP_CR_VNEG_PIN, ACMP_CR_HYSTERESIS_DISABLE);
}



uint32_t Get_ACMP(void)
{
    uint32_t ACMP0_Output_Level;

    if(ACMP->CMPSR & ACMP_CMPSR_CO0_Msk)
        ACMP0_Output_Level = 1;
    else
        ACMP0_Output_Level = 0;

    ACMP->CMPSR |= ACMP_CMPSR_CMPF0_Msk;        //Clear ACMP0 Flags

    return ACMP0_Output_Level;
}
