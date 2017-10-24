#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_Button.h"
#define KEY1 PC13
#define KEY2 PE8
#define KEY3 PC15
#define KEY4 PC14
void Initial_Key_Input(void)
{
    GPIO_SetMode(PC, BIT13, GPIO_PMD_INPUT);
    GPIO_SetMode(PE, BIT8,  GPIO_PMD_INPUT);
    GPIO_SetMode(PC, BIT14, GPIO_PMD_INPUT);
    GPIO_SetMode(PC, BIT15, GPIO_PMD_INPUT);
}

unsigned char Get_Key_Input(void)
{
    unsigned char temp = 0;
    if(KEY1 == 0)
        temp |= 0x1;


    if(KEY2 == 0)
        temp |= 0x2;


    if(KEY3 == 0)
        temp |= 0x4;


    if(KEY4 == 0)
        temp |= 0x8;

    return   temp;
}
