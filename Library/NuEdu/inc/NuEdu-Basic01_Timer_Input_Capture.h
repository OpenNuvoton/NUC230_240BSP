#ifndef __NuEdu_Basic01_Timer_Input_Capture_H__
#define __NuEdu_Basic01_Timer_Input_Capture_H__
extern void Initial_Timer2_Toggle(void);
extern void Initial_Timer_Port(void);
extern void Initial_Timer2_Count(void);
extern unsigned int Get_Timer3_Capture(void);
extern void initial_Timer3_Capture(void);
#define Get_Timer2_Count()  TIMER_GetCounter(TIMER2)
#endif
