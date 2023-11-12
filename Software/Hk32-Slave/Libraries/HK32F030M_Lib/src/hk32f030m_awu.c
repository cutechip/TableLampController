/**
  ******************************************************************************
  * @file    hk32f030m_awu.c
  * @author  Rakan.z
  * @version V1.0  
  * @brief   This file contains all the functions for the AWU peripheral.  
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hk32f030m_awu.h"
/* Public functions ----------------------------------------------------------*/

/**
  * @addtogroup AWU_Public_Functions
  * @{
  */

/**
  * @brief  Deinitializes the AWU peripheral registers to their default reset
  * values.
  * @param  None
  * @retval None
  */
void AWU_DeInit(void)
{
    AWU->CR = AWU_CR_RESET_VALUE;
    AWU->SR = AWU_SR_RESET_VALUE;
}

/**
  * @brief  config the AWU clock
  * @param   eAWU_CLK :  
  *  AWU_CLK_LSI128
    AWU_CLK_HSE
  * @retval None
  * @par Required preconditions:
  * The LS RC calibration must be performed before calling this function.
  */
void AWU_CLKConfig(AWU_CLK_TYPE eAWU_CLK)
{
    uint32_t temp = 0;
    /* Check parameter */
    assert_param(IS_AWU_CLK(eAWU_CLK));

    temp =  AWU->CR;
    /*clear Bit AWU_CKSEL*/
    temp &= 0xFFFFFFFE;
    /* config AWU timer clk*/
    temp |= eAWU_CLK;
    /*set the register*/
    AWU->CR |= temp;
}

/**
  * @brief  loade the AWU timer counter,This load value will be automatically loaded into the 22-bit timer inside the awu
  *  when the mcu enters stop mode and start timing.
  * @param   TimerCounter : the AWU timer counter
  * @note When awu_rlr [22:1] is '0' or is '1' , the loading behavior will not occur and awu will not start working .
  *       when awu_wbusy =1 ,the write operation on the awu-rlr register will be invalid.
  * @return ErrorStatus: the AWU result 
  *       SUCCESS:AWU timer start success
  *       ERROR£ºAWU timer start error
  */
ErrorStatus AWU_TimerCounterAndStart(uint32_t TimerCounter)
{
    uint32_t temp = 0;
    uint32_t TimeoutCnt = 0;
    while (TimeoutCnt ++ <= 0x0fff)
    {
      //  AWU_APB bus is idle    
      if ((AWU->CR & 0x80000000) == 0x00000000)
      {
        temp = AWU->CR;
        temp &= 0xFF800001; 
        temp |= ( TimerCounter << 1);
        AWU->CR |= temp;
        return SUCCESS;
      }
      else
      {
        /* when awu_wbusy =1 ,the write operation on the awu-rlr register will be invalid.*/
      }
    }
 
    return ERROR;
      
}

/**
  * @brief  Returns status of the AWU peripheral flag.
  * @param  None
  * @retval FlagStatus : Status of the AWU flag.
  * This parameter can be any of the @ref FlagStatus enumeration.
  */
FlagStatus AWU_GetFlagStatus(void)
{
    return((FlagStatus)(((uint8_t)(AWU->SR & AWU_SR_BUSY) == (uint8_t)0x00) ? RESET : SET));
}


/**
  * @}
  */
  
