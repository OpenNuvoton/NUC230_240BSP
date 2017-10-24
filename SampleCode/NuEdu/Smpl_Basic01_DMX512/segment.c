#include <stdio.h>
#include "NUC230_240.h"
#include "Segment.h"
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
    //IO initial output mode
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

void Turnoff_Seven_Segment(unsigned char number)
{
    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
    switch(number)
    {
        case 1:
            SEG_CONTROL1_OFF;
            break;

        //show 1
        case 2:
            SEG_CONTROL2_OFF;
            break;
        default:
            break;
    }
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


        case SEG_CHAR_H:
//          SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
//          SEG_D_ON;
            SEG_E_ON;
            SEG_F_ON;
            SEG_G_ON;
//          SEG_H_ON;
            break;
        case SEG_CHAR_S:
            SEG_A_ON;
//          SEG_B_ON;
            SEG_C_ON;
            SEG_D_ON;
//          SEG_E_ON;
            SEG_F_ON;
            SEG_G_ON;
//          SEG_H_ON;
            break;
        default:
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







#define _LED_Bar_Count      8

#define _LED1               PB4
#define _LED2               PB5
#define _LED3               PD14
#define _LED4               PD15
#define _LED5               PE7
#define _LED6               PE15
#define _LED7               PE14
#define _LED8               PD5


void Write_LED_Bar(uint32_t Number)
{
    uint32_t i;
    volatile uint32_t *ptrLED[_LED_Bar_Count] = {&_LED1, &_LED2, &_LED3, &_LED4, &_LED5, &_LED6, &_LED7, &_LED8};

    for(i = 0; i < _LED_Bar_Count; i++)
    {
        if(Number > i)
            *ptrLED[i] = 0; //LED ON
        else
            *ptrLED[i] = 1; //LED OFF
    }
}

void Initial_LED(void)
{
    GPIO_SetMode(PB, BIT4, GPIO_PMD_OUTPUT); //LED1
    GPIO_SetMode(PB, BIT5, GPIO_PMD_OUTPUT); //LED2
    GPIO_SetMode(PD, BIT14, GPIO_PMD_OUTPUT); //LED3
    GPIO_SetMode(PD, BIT15, GPIO_PMD_OUTPUT); //LED4
    GPIO_SetMode(PE, BIT7, GPIO_PMD_OUTPUT); //LED5
    GPIO_SetMode(PE, BIT15, GPIO_PMD_OUTPUT); //LED6
    GPIO_SetMode(PE, BIT14, GPIO_PMD_OUTPUT); //LED7
    GPIO_SetMode(PD, BIT5, GPIO_PMD_OUTPUT); //LED8
}
//unsigned int tempprev=0;
void LED_On(unsigned int temp)
{
//  if(tempprev==temp)
//  {
//      return;
//  }
//  printf("\n  0x%lx",temp);
    if((temp & 1) != 1)
        _LED1 = 1;
    else
        _LED1 = 0;

    temp = temp >> 1;

    if((temp & 1) != 1)
        _LED2 = 1;
    else
        _LED2 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED3 = 1;
    else
        _LED3 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED4 = 1;
    else
        _LED4 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED5 = 1;
    else
        _LED5 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED6 = 1;
    else
        _LED6 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED7 = 1;
    else
        _LED7 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED8 = 1;
    else
        _LED8 = 0;

}
