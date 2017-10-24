#include <stdio.h>
#include "NUC230_240.h"
#include "NuEdu-Basic01_LED.h"

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

void LED_On(unsigned int temp)
{
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
