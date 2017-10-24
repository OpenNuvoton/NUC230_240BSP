#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_ClkOut.h"


void Open_CLK_OUT(void)
{
    //Initial CLKO Function Pin
    SYS->GPB_MFP = (SYS->GPB_MFP & ~SYS_GPB_MFP_PB8_Msk) | SYS_GPB_MFP_PB8_CLKO;
    SYS->ALT_MFP = (SYS->ALT_MFP & ~SYS_ALT_MFP_PB8_Msk) | SYS_ALT_MFP_PB8_CLKO;

    /* Output selected clock to CKO, CKO Clock = HCLK / 2^(7 + 1) */
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HXT, 7, 0);
}
