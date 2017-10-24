#ifndef __NuEdu_Basic01_PWM_Capture_H__
#define __NuEdu_Basic01_PWM_Capture_H__



extern void Open_PWM6_OUT(uint32_t Enable, uint32_t PWM_Frequency, uint32_t PWM_Duty);

extern void Open_PWM3_Capture(void);
extern void Open_PWM7_Capture(void);
extern void Get_PWM3_Capture_Data(void);
extern void Get_PWM7_Capture_Data(void);


#endif
