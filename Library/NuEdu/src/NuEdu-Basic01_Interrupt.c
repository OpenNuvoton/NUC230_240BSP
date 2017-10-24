#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_Interrupt.h"

void Initial_EINT0_GPIO(void)
{
    /* Set PB.14 as Input */
    GPIO_SetMode(PB, BIT14, GPIO_PMD_INPUT);

    /* Set PB.14 multi-function pins for EINT0 */
    SYS->GPB_MFP |= SYS_GPB_MFP_PB14_INT0;
}

void Open_EINT0(void)
{
    Initial_EINT0_GPIO();

    /* Enable interrupt by falling edge trigger */
    GPIO_EnableInt(PB, 14, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);

}

void EINT0_IRQHandler(void)
{
    /* For PB.14, clear the INT flag */
    GPIO_CLR_INT_FLAG(PB, BIT14);

}

