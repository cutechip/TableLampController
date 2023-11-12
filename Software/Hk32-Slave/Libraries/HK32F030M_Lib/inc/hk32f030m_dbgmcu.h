/**
  ******************************************************************************
  * @file    hk32f030m_dbgmcu.h
  * @author  Felix.z
  * @version V1.0  
  * @brief   API file of DBGMCU module
  * @changelist  
  *
  ******************************************************************************
  */

#ifndef __HK32F030M_DBGMCU_H
#define __HK32F030M_DBGMCU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hk32f030m.h"
 
/** @defgroup DBGMCU_Exported_Constants
  * @{
  */

#define DBGMCU_STOP                  DBGMCU_CR_DBG_STOP 
#define IS_DBGMCU_PERIPH(PERIPH) ((((PERIPH) & 0xFFFFFFF9) == 0x00) && ((PERIPH) != 0x00))

#define DBGMCU_TIM1_STOP             DBGMCU_APB1_FZ_DBG_TIM1_STOP  
#define DBGMCU_TIM2_STOP             DBGMCU_APB1_FZ_DBG_TIM2_STOP
#define DBGMCU_TIM6_STOP             DBGMCU_APB1_FZ_DBG_TIM6_STOP 
#define DBGMCU_WWDG_STOP             DBGMCU_APB1_FZ_DBG_WWDG_STOP
#define DBGMCU_IWDG_STOP             DBGMCU_APB1_FZ_DBG_IWDG_STOP
#define DBGMCU_I2C1_SMBUS_TIMEOUT    DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT
#define IS_DBGMCU_APB1PERIPH(PERIPH) ((((PERIPH) & 0xFDDFE2CC) == 0x00) && ((PERIPH) != 0x00))


/**
  * @}
  */ 

/* Device and Revision ID management functions ********************************/ 
uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);

/* Peripherals Configuration functions ****************************************/ 
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif /* __HK32F030M_DBGMCU_H */
 
