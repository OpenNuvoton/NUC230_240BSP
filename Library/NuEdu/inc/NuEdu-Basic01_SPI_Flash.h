#ifndef __NuEdu_Basic01_SPI_FLASH_H__
#define __NuEdu_Basic01_SPI_FLASH_H__

extern void Open_SPI_Flash(void);
extern unsigned int SpiFlash_ReadJedecID(void);
extern void SpiFlash_ChipErase(void);
extern unsigned int SpiFlash_ReadStatusReg1(void);
extern unsigned int SpiFlash_ReadStatusReg2(void);
extern void SpiFlash_WaitReady(void);
extern void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount);
extern void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount);
#endif
