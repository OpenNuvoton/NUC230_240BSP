/**************************************************************************//**
 * @file     sc.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/05/12 1:48p $
 * @brief    NUC230_240 series Smartcard(SC) driver source file
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NUC230_240.h"

// Below are variables used locally by SC driver and does not want to parse by doxygen unless HIDDEN_SYMBOLS is defined
/// @cond HIDDEN_SYMBOLS
static uint32_t u32CardStateIgnore[SC_INTERFACE_NUM] = {0, 0, 0};

/// @endcond HIDDEN_SYMBOLS

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SC_Driver SC Driver
  @{
*/


/** @addtogroup SC_EXPORTED_FUNCTIONS SC Exported Functions
  @{
*/

/**
  * @brief This function smartcard is insert of specified smartcard module.
  * @param[in] sc Base address of smartcard module.
  * @retval TRUE Card insert.
  * @retval FALSE Card remove.
  * @details This function is used to check if specified smart card slot is presented.
  */
uint32_t SC_IsCardInserted(SC_T *sc)
{
    // put conditions into two variable to remove IAR compilation warning
    uint32_t cond1 = ((sc->PINCSR & SC_PINCSR_CD_PIN_ST_Msk) >> SC_PINCSR_CD_PIN_ST_Pos);
    uint32_t cond2 = ((sc->PINCSR & SC_PINCSR_CD_LEV_Msk) >> SC_PINCSR_CD_LEV_Pos);

    if(sc == SC0 && u32CardStateIgnore[0] == 1)
        return TRUE;
    else if(sc == SC1 && u32CardStateIgnore[1] == 1)
        return TRUE;
    else if(sc == SC2 && u32CardStateIgnore[2] == 1)
        return TRUE;
    else if(cond1 != cond2)
        return FALSE;
    else
        return TRUE;
}

/**
  * @brief Reset the Tx/Rx FIFO.
  * @param[in] sc Base address of smartcard module.
  * @return None
  * @details This function reset both transmit and receive FIFO of specified smartcard module.
  */
void SC_ClearFIFO(SC_T *sc)
{
    sc->ALTCTL |= (SC_ALTCTL_TX_RST_Msk | SC_ALTCTL_RX_RST_Msk);
}

/**
  * @brief This function disable specified smartcard module.
  * @param[in] sc Base address of smartcard module.
  * @return None
  * @details SC will force all transition to IDLE state.
  */
void SC_Close(SC_T *sc)
{
    uint32_t u32TimeOutCount;

    sc->IER = 0;

    u32TimeOutCount = SC_TIMEOUT;
    while((sc->PINCSR & SC_PINCSR_SYNC_Msk) == SC_PINCSR_SYNC_Msk)
    {
        if(--u32TimeOutCount == 0UL) break;
    }
    sc->PINCSR = 0;
    sc->ALTCTL = 0;

    u32TimeOutCount = SC_TIMEOUT;
    while((sc->CTL & SC_CTL_SYNC_Msk) == SC_CTL_SYNC_Msk)
    {
        if(--u32TimeOutCount == 0UL) break;
    }
    sc->CTL = 0;
}

/**
  * @brief This function initialized smartcard module.
  * @param[in] sc Base address of smartcard module.
  * @param[in] u32CD Card detect polarity, select the CD pin state which indicates card insert. Could be:
  *                 -\ref SC_PIN_STATE_HIGH.
  *                 -\ref SC_PIN_STATE_LOW.
  *                 -\ref SC_PIN_STATE_IGNORE, no card detect pin, always assumes card present.
  * @param[in] u32PWR Power on polarity, select the PWR pin state which could set smartcard VCC to high level. Could be:
  *                 -\ref SC_PIN_STATE_HIGH.
  *                 -\ref SC_PIN_STATE_LOW.
  * @return None
  * @details Initialization process configures smartcard and enables engine clock.
  */
void SC_Open(SC_T *sc, uint32_t u32CD, uint32_t u32PWR)
{
    uint32_t u32Reg = 0UL, u32Intf, u32TimeOutCount;

    if(sc == SC0)
        u32Intf = 0;
    else if(sc == SC1)
        u32Intf = 1;
    else
        u32Intf = 2;

    if(u32CD != SC_PIN_STATE_IGNORE)
    {
        u32Reg = u32CD ? 0 : SC_PINCSR_CD_LEV_Msk;
        u32CardStateIgnore[u32Intf] = 0;
    }
    else
    {
        u32CardStateIgnore[u32Intf] = 1;
    }
    u32TimeOutCount = SC_TIMEOUT;
    while((sc->PINCSR & SC_PINCSR_SYNC_Msk) == SC_PINCSR_SYNC_Msk)
    {
        if(--u32TimeOutCount == 0UL) break;
    }
    u32Reg |= u32PWR ? 0 : SC_PINCSR_POW_INV_Msk;
    sc->PINCSR = u32Reg;

    u32TimeOutCount = SC_TIMEOUT;
    while((sc->CTL & SC_CTL_SYNC_Msk) == SC_CTL_SYNC_Msk)
    {
        if(--u32TimeOutCount == 0UL) break;
    }
    sc->CTL = SC_CTL_SC_CEN_Msk;
}

/**
  * @brief This function reset specified smartcard module to its default state for activate smartcard.
  * @param[in] sc Base address of smartcard module.
  * @return None
  * @details Reset the Tx/Rx FIFO & clock & initail default parameter.
  */
void SC_ResetReader(SC_T *sc)
{
    uint32_t u32Intf, u32TimeOutCount;

    if(sc == SC0)
        u32Intf = 0;
    else if(sc == SC1)
        u32Intf = 1;
    else
        u32Intf = 2;

    // Reset FIFO
    sc->ALTCTL |= (SC_ALTCTL_TX_RST_Msk | SC_ALTCTL_RX_RST_Msk);
    // Set Rx trigger level to 1 character, longest card detect debounce period, disable error retry (EMV ATR does not use error retry)
    u32TimeOutCount = SC_TIMEOUT;
    while((sc->CTL & SC_CTL_SYNC_Msk) == SC_CTL_SYNC_Msk)
    {
        if(--u32TimeOutCount == 0) break;
    }
    sc->CTL &= ~(SC_CTL_RX_FTRI_LEV_Msk | SC_CTL_CD_DEB_SEL_Msk | SC_CTL_TX_ERETRY_Msk | SC_CTL_RX_ERETRY_Msk);
    u32TimeOutCount = SC_TIMEOUT;
    while((sc->CTL & SC_CTL_SYNC_Msk) == SC_CTL_SYNC_Msk)
    {
        if(--u32TimeOutCount == 0) break;
    }
    // Enable auto convention, and all three smartcard internal timers
    sc->CTL |= SC_CTL_AUTO_CON_EN_Msk | SC_CTL_TMR_SEL_Msk;
    // Disable Rx timeout
    sc->RFTMR = 0;
    // 372 clocks per ETU by default
    sc->ETUCR = 371;
    // Enable auto de-activation while card removal
    sc->PINCSR |= SC_PINCSR_ADAC_CD_EN_Msk;

    /* Enable necessary interrupt for smartcard operation */
    if(u32CardStateIgnore[u32Intf]) // Do not enable card detect interrupt if card present state ignore
        sc->IER = (SC_IER_RDA_IE_Msk |
                   SC_IER_TERR_IE_Msk |
                   SC_IER_TMR0_IE_Msk |
                   SC_IER_TMR1_IE_Msk |
                   SC_IER_TMR2_IE_Msk |
                   SC_IER_BGT_IE_Msk |
                   SC_IER_ACON_ERR_IE_Msk);
    else
        sc->IER = (SC_IER_RDA_IE_Msk |
                   SC_IER_TERR_IE_Msk |
                   SC_IER_TMR0_IE_Msk |
                   SC_IER_TMR1_IE_Msk |
                   SC_IER_TMR2_IE_Msk |
                   SC_IER_BGT_IE_Msk |
                   SC_IER_CD_IE_Msk |
                   SC_IER_ACON_ERR_IE_Msk);

    return;
}

/**
  * @brief Set Block Guard Time.
  * @param[in] sc Base address of smartcard module.
  * @param[in] u32BGT Block guard time using ETU as unit, valid range are between 1 ~ 32.
  * @return None
  * @details This function block guard time (BGT) of specified smartcard module.
  */
void SC_SetBlockGuardTime(SC_T *sc, uint32_t u32BGT)
{
    sc->CTL = (sc->CTL & ~SC_CTL_BGT_Msk) | ((u32BGT - 1) << SC_CTL_BGT_Pos);
}

/**
  * @brief Set character guard time.
  * @param[in] sc Base address of smartcard module.
  * @param[in] u32CGT Character guard time using ETU as unit, valid range are between 11 ~ 267.
  * @return None
  * @details This function character guard time (CGT) of specified smartcard module.
  * @note Before using this API, user should set the correct stop bit length first.
  */
void SC_SetCharGuardTime(SC_T *sc, uint32_t u32CGT)
{
    /* CGT is "START bit" + "8-bits" + "Parity bit" + "STOP bit(s)" + "EGT counts" */
    u32CGT -= sc->CTL & SC_CTL_SLEN_Msk ? 11 : 12;
    sc->EGTR = u32CGT;
}

/**
  * @brief Stop all Timer counting.
  * @param[in] sc Base address of smartcard module.
  * @return None
  * @details This function stop all smartcard timer of specified smartcard module.
  * @note This function stop the timers within smartcard module, \b not timer module.
  */
void SC_StopAllTimer(SC_T *sc)
{
    sc->ALTCTL &= ~(SC_ALTCTL_TMR0_SEN_Msk | SC_ALTCTL_TMR1_SEN_Msk | SC_ALTCTL_TMR2_SEN_Msk);
}

/**
  * @brief This function configure and start a smartcard timer of specified smartcard module.
  * @param[in] sc Base address of smartcard module.
  * @param[in] u32TimerNum Timer(s) to start. Valid values are 0, 1, 2.
  * @param[in] u32Mode Timer operating mode, valid values are:
  *             - \ref SC_TMR_MODE_0
  *             - \ref SC_TMR_MODE_1
  *             - \ref SC_TMR_MODE_2
  *             - \ref SC_TMR_MODE_3
  *             - \ref SC_TMR_MODE_4
  *             - \ref SC_TMR_MODE_5
  *             - \ref SC_TMR_MODE_6
  *             - \ref SC_TMR_MODE_7
  *             - \ref SC_TMR_MODE_8
  *             - \ref SC_TMR_MODE_F
  * @param[in] u32ETUCount Timer timeout duration, ETU based. For timer 0, valid  range are between 1~0x1000000ETUs.
  *                        For timer 1 and timer 2, valid range are between 1 ~ 0x100 ETUs.
  * @return None
  * @details Enable Timer starting, counter will count when condition match.
  * @note This function start the timer within smartcard module, \b not timer module.
  * @note Depend on the timer operating mode, timer may not start counting immediately.
  */
void SC_StartTimer(SC_T *sc, uint32_t u32TimerNum, uint32_t u32Mode, uint32_t u32ETUCount)
{
    uint32_t reg = u32Mode | (SC_TMR0_CNT_Msk & (u32ETUCount - 1));

    if(u32TimerNum == 0)
    {
        sc->TMR0 = reg;
        sc->ALTCTL |= SC_ALTCTL_TMR0_SEN_Msk;
    }
    else if(u32TimerNum == 1)
    {
        sc->TMR1 = reg;
        sc->ALTCTL |= SC_ALTCTL_TMR1_SEN_Msk;
    }
    else       // timer 2
    {
        sc->TMR2 = reg;
        sc->ALTCTL |= SC_ALTCTL_TMR2_SEN_Msk;
    }
}

/**
  * @brief Stop Timer counting.
  * @param[in] sc Base address of smartcard module.
  * @param[in] u32TimerNum Timer(s) to stop. Valid values are 0, 1, 2.
  * @return None
  * @details This function stop a smartcard timer of specified smartcard module.
  * @note This function stop the timer within smartcard module, \b not timer module.
  */
void SC_StopTimer(SC_T *sc, uint32_t u32TimerNum)
{
    if(u32TimerNum == 0)
        sc->ALTCTL &= ~SC_ALTCTL_TMR0_SEN_Msk;
    else if(u32TimerNum == 1)
        sc->ALTCTL &= ~SC_ALTCTL_TMR1_SEN_Msk;
    else    // timer 2
        sc->ALTCTL &= ~SC_ALTCTL_TMR2_SEN_Msk;
}

/**
  * @brief      Get smartcard clock frequency
  *
  * @param[in]  sc      The pointer of smartcard module.
  *
  * @return     Smartcard frequency in kHZ
  *
  * @details    This function is used to get specified smartcard module clock frequency in kHz.
  */
uint32_t SC_GetInterfaceClock(SC_T *sc)
{
    uint32_t num, u32ClkSrc, freq;

    /* Get smartcard module clock source and divider */
    if(sc == SC0)
    {
        num = 0UL;
    }
    else if(sc == SC1)
    {
        num = 1UL;
    }
    else if(sc == SC2)
    {
        num = 2UL;
    }
    else
    {
        num = 0UL;
    }

    if(num == 0)
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC0_S_Msk) >> CLK_CLKSEL3_SC0_S_Pos;
    else if(num == 1)
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC1_S_Msk) >> CLK_CLKSEL3_SC1_S_Pos;
    else // SC2
        u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC2_S_Msk) >> CLK_CLKSEL3_SC2_S_Pos;

    if(u32ClkSrc == 0)
        freq = __HXT;
    else if(u32ClkSrc == 1)
        freq = CLK_GetPLLClockFreq();
    else if(u32ClkSrc == 2)
        freq = SystemCoreClock;
    else
        freq = __HIRC;

    if(num == 0)
        freq /= ((CLK->CLKDIV1 & CLK_CLKDIV1_SC0_N_Msk) + 1);
    else if(num == 1)
        freq /= (((CLK->CLKDIV1 & CLK_CLKDIV1_SC1_N_Msk) >> CLK_CLKDIV1_SC1_N_Pos) + 1);
    else // SC2
        freq /= (((CLK->CLKDIV1 & CLK_CLKDIV1_SC2_N_Msk) >> CLK_CLKDIV1_SC2_N_Pos) + 1);

    return (freq / 1000);
}

/*@}*/ /* end of group SC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
