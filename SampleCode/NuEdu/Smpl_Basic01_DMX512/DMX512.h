#ifndef __DMX512_H__

#define __DMX512_H__

#define DMX512_CHANNEL_NUM   512
#define DMX512_START_CODE    0

extern uint8_t u8GetDmx512ResetSignal;
extern uint8_t u8DMX512ReceiveDone;
extern uint16_t u16ReceivingChannelIndex;
extern uint16_t u16ReceivedTotalChannelNum;
extern uint16_t u16ReceiveBufferPointer;
extern uint8_t u16ReceiveBufferArea;

extern uint8_t u8ReceivedData;
extern uint8_t u8WaitTimer1Flag;
extern uint32_t u32SysTimerDMX512MTBF;

extern void DMX512_ReceiveInit(void);
extern void DMX512_SendData(void);
extern void DMX512_SendInit(void);
extern void RestartDMX512Receive(void);

#define SUPPORT_DEBUGINFO
#ifdef  SUPPORT_DEBUGINFO
#define GPIO_FOR_SIGNAL_OSCILLOSCOPE_INITIAL GPIO_SetMode(PC, 11, GPIO_PMD_OUTPUT); //SPI1_MOSI
#define GPIO_FOR_SIGNAL_OSCILLOSCOPE_LOW    PC11=0   //SPI1_MOSI
#define GPIO_FOR_SIGNAL_OSCILLOSCOPE_HIGH    PC11=1   //SPI1_MOSI
#endif
#endif
