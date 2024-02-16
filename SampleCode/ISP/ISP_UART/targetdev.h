/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x33
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NUC230_240.h"
#include "uart_transfer.h"
#include "ISP_USER.h"

/* rename for uart_transfer.c */
#define UART_N					UART0
#define UART_N_IRQHandler		UART02_IRQHandler
#define UART_N_IRQn				UART02_IRQn

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
