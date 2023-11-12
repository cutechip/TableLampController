/**
  ******************************************************************************
  * @file    hk32f030m_pwr.h
  * @author  Rakan.z
  * @version V1.0  
  * @brief   Header file of PWR module
  * @changelist  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HK32F030M_PWR_H
#define __HK32F030M_PWR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hk32f030m.h"

/* ------------------ PWR registers bit mask ------------------------ */

/* CR register bit mask */
#define CR_DS_MASK               ((uint32_t)0xFFFFFFFE)


/** @defgroup Regulator_state_is_STOP_mode 
  * @{
  */

#define PWR_Regulator_LowPower    ((uint32_t)0x00000001)
#define IS_PWR_REGULATOR(REGULATOR) ((REGULATOR) == PWR_Regulator_LowPower)

/** @defgroup PWR_mode_entry 
  * @{
  */

#define PWR_Entry_WFI         ((uint8_t)0x01)
#define PWR_Entry_WFE         ((uint8_t)0x02)
#define IS_PWR_ENTRY(ENTRY) (((ENTRY) == PWR_Entry_WFI) || ((ENTRY) == PWR_Entry_WFE))
 
/** @defgroup PWR_LDO_VREF
  * @{
  */
#define ADC_VREF_0D8              ((uint16_t)0x0008)  
#define ADC_VREF_LDO              ((uint16_t)0x000C)  
#define VTEST_SET_MASK            ((uint16_t)0x3FFF)
#define IS_PWR_VTEST_SET(VTEST_SET) (((VTEST_SET) == ADC_VREF_0D8) || ((VTEST_SET) == ADC_VREF_LDO))




void PWR_DeInit(void);

void PWR_EnterSleepMode(uint8_t PWR_Entry);
void PWR_EnterDeepSleepMode(uint8_t PWR_Entry);
void PWR_EnterStopMode(uint32_t PWR_Regulator, uint8_t PWR_Entry);

void PWR_SetLDO_RefVolToADC(uint16_t Vref_Set);


#ifdef __cplusplus
}
#endif

#endif

