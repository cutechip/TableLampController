/**
  ******************************************************************************
  * @file    hk32f030m_dbgmcu.c
  * @author  Felix.z
  * @version V1.0  
  * @brief   API file of DBGMCU module
  * @changelist  
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hk32f030m_dbgmcu.h"


#define IDCODE_DEVID_MASK    ((uint32_t)0x00000FFF)


/**
  * @brief  Returns the device revision identifier.
  * @param  None
  * @retval Device revision identifier
  */
uint32_t DBGMCU_GetREVID(void)
{
   return(DBGMCU->IDCODE >> 16);
}

/**
  * @brief  Returns the device identifier.
  * @param  None
  * @retval Device identifier
  */
uint32_t DBGMCU_GetDEVID(void)
{
   return(DBGMCU->IDCODE & IDCODE_DEVID_MASK);
}


/**
  * @brief  Configures low power mode behavior when the MCU is in Debug mode.
  * @param  DBGMCU_Periph: specifies the low power mode.
  *          This parameter can be any combination of the following values:
  *             @arg DBGMCU_STOP: Keep debugger connection during STOP mode
  * @param  NewState: new state of the specified low power mode in Debug mode.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DBGMCU_PERIPH(DBGMCU_Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    DBGMCU->CR |= DBGMCU_Periph;
  }
  else
  {
    DBGMCU->CR &= ~DBGMCU_Periph;
  }
}


/**
  * @brief  Configures APB1 peripheral behavior when the MCU is in Debug mode.
  * @param  DBGMCU_Periph: specifies the APB1 peripheral.
  *          This parameter can be any combination of the following values:
  *             @arg DBGMCU_TIM1_STOP: TIM1 counter stopped when Core is halted 
  *             @arg DBGMCU_TIM2_STOP: TIM2 counter stopped when Core is halted
  *             @arg DBGMCU_TIM6_STOP: TIM6 counter stopped when Core is halted 
  *             @arg DBGMCU_WWDG_STOP: Debug WWDG stopped when Core is halted
  *             @arg DBGMCU_IWDG_STOP: Debug IWDG stopped when Core is halted
  *             @arg DBGMCU_I2C1_SMBUS_TIMEOUT: I2C1 SMBUS timeout mode stopped 
  *                                             when Core is halted            
  * @param  NewState: new state of the specified APB1 peripheral in Debug mode.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DBGMCU_APB1PERIPH(DBGMCU_Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    DBGMCU->APB1FZ |= DBGMCU_Periph;
  }
  else
  {
    DBGMCU->APB1FZ &= ~DBGMCU_Periph;
  }
}
  
