#ifndef __NuEdu_Basic01_7_Segment_H__
#define __NuEdu_Basic01_7_Segment_H__

#define SEG_CHAR_A  0X10
#define SEG_CHAR_B  0X11
#define SEG_CHAR_C  0X12
#define SEG_CHAR_D  0X13
#define SEG_CHAR_E  0X14
#define SEG_CHAR_F  0X15
#define SEG_CHAR_G  0X16
#define SEG_CHAR_H  0X17
#define SEG_CHAR_L  0X18
#define SEG_CHAR_M  0X19
#define SEG_CHAR_N  0X1A
#define SEG_CHAR_O  0X1B
#define SEG_CHAR_P  0X1C
#define SEG_CHAR_Q  0X1D
#define SEG_CHAR_R  0X1E
#define SEG_CHAR_S  0X1F
#define SEG_CHAR_T  0X20
#define SEG_CHAR_U  0X21
#define SEG_CHAR_V  0X22
#define SEG_CHAR_W  0X23
#define SEG_CHAR_X  0X24
#define SEG_CHAR_Y  0X25
#define SEG_CHAR_Z  0X26
#define SEG_CHAR_NULL 0XFF

extern void Show_Seven_Segment(unsigned char no, unsigned char number);
extern void Open_Seven_Segment(void);
extern void Write_LED_Bar(uint32_t Number);
extern void Initial_LED(void);
extern void LED_On(unsigned int temp);

#endif
