#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_7_Segment.h"
//porting define
#define SEG_A_ON    PA2=0
#define SEG_B_ON    PA3=0
#define SEG_C_ON    PA4=0
#define SEG_D_ON    PA5=0
#define SEG_E_ON    PA6=0
#define SEG_F_ON    PA7=0
#define SEG_G_ON    PC4=0
#define SEG_H_ON    PB9=0
#define SEG_CONTROL1_ON    PC5=1
#define SEG_CONTROL2_ON    PE6=1


#define SEG_A_OFF    PA2=1
#define SEG_B_OFF    PA3=1
#define SEG_C_OFF    PA4=1
#define SEG_D_OFF    PA5=1
#define SEG_E_OFF    PA6=1
#define SEG_F_OFF    PA7=1
#define SEG_G_OFF    PC4=1
#define SEG_H_OFF    PB9=1
#define SEG_CONTROL1_OFF    PC5=0
#define SEG_CONTROL2_OFF    PE6=0

void Initial_SEG_GPIO(void)
{
    //io initial output mode
    GPIO_SetMode(PA, BIT2, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT3, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT4, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT5, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT6, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT7, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT4, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PB, BIT9, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT5, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PE, BIT6, GPIO_PMD_OUTPUT);
}


void Open_Seven_Segment(void)
{
    Initial_SEG_GPIO();
    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
}



void Show_Seven_Segment(unsigned char no, unsigned char number)
{
    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
    SEG_CONTROL1_OFF;
    SEG_CONTROL2_OFF;
    switch(no)
    {
        //show 0
        case 0:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_D_ON;
            SEG_E_ON;
            SEG_F_ON;

            break;

        //show 1
        case 1:
            SEG_B_ON;
            SEG_C_ON;
            break;

        //show 2
        case 2:
            SEG_A_ON;
            SEG_B_ON;
            SEG_G_ON;
            SEG_E_ON;
            SEG_D_ON;
            break;

        //show 3
        case 3:
            SEG_A_ON;
            SEG_B_ON;
            SEG_G_ON;
            SEG_C_ON;
            SEG_D_ON;
            break;

        //show 4
        case 4:
            SEG_F_ON;
            SEG_B_ON;
            SEG_G_ON;
            SEG_C_ON;
            break;

        //show 5
        case 5:
            SEG_A_ON;
            SEG_F_ON;
            SEG_G_ON;
            SEG_C_ON;
            SEG_D_ON;
            break;

        //show 6
        case 6:
            SEG_A_ON;
            SEG_F_ON;
            SEG_E_ON;
            SEG_G_ON;
            SEG_C_ON;
            SEG_D_ON;
            break;

        //show 7
        case 7:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_F_ON;
            break;

        //show 8
        case 8:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_D_ON;
            SEG_E_ON;
            SEG_F_ON;
            SEG_G_ON;
            break;

        //show 9
        case 9:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_F_ON;
            SEG_G_ON;
            break;
    }

    switch(number)
    {
        case 1:
            SEG_CONTROL1_ON;
            break;

        //show 1
        case 2:
            SEG_CONTROL2_ON;
            break;
    }
}


