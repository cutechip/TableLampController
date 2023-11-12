/**
  ******************************************************************************
  * @file    hk32f030m_syscfg.h
  * @author  Rakan.z
  * @version V1.0  
  * @brief   API file of PWR module
  * @changelist  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HK32F030M_SYSCFG_H
#define __HK32F030M_SYSCFG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hk32f030m.h"


/** @addtogroup SYSCFG
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup SYSCFG_EXTI_Port_Sources 
  * @{
  */ 
#define EXTI_PortSourceGPIOA       ((uint8_t)0x00)
#define EXTI_PortSourceGPIOB       ((uint8_t)0x01)
#define EXTI_PortSourceGPIOC       ((uint8_t)0x02)
#define EXTI_PortSourceGPIOD       ((uint8_t)0x03)

#define IS_EXTI_PORT_SOURCE(PORTSOURCE) (((PORTSOURCE) == EXTI_PortSourceGPIOA) || \
                                         ((PORTSOURCE) == EXTI_PortSourceGPIOB) || \
                                         ((PORTSOURCE) == EXTI_PortSourceGPIOC) || \
                                         ((PORTSOURCE) == EXTI_PortSourceGPIOD) )
                                         
/**
  * @}
  */ 


/** @defgroup SYSCFG_EXTI_Pin_Sources 
  * @{
  */ 
#define EXTI_PinSource0            ((uint8_t)0x00)
#define EXTI_PinSource1            ((uint8_t)0x01)
#define EXTI_PinSource2            ((uint8_t)0x02)
#define EXTI_PinSource3            ((uint8_t)0x03)
#define EXTI_PinSource4            ((uint8_t)0x04)
#define EXTI_PinSource5            ((uint8_t)0x05)
#define EXTI_PinSource6            ((uint8_t)0x06)
#define EXTI_PinSource7            ((uint8_t)0x07)
#define EXTI_PinSource8            ((uint8_t)0x08)
#define EXTI_PinSource9            ((uint8_t)0x09)
#define EXTI_PinSource10           ((uint8_t)0x0A)
#define EXTI_PinSource11           ((uint8_t)0x0B)
#define EXTI_PinSource12           ((uint8_t)0x0C)
#define EXTI_PinSource13           ((uint8_t)0x0D)
#define EXTI_PinSource14           ((uint8_t)0x0E)
#define EXTI_PinSource15           ((uint8_t)0x0F)
#define IS_EXTI_PIN_SOURCE(PINSOURCE) (((PINSOURCE) == EXTI_PinSource0)  || \
                                       ((PINSOURCE) == EXTI_PinSource1)  || \
                                       ((PINSOURCE) == EXTI_PinSource2)  || \
                                       ((PINSOURCE) == EXTI_PinSource3)  || \
                                       ((PINSOURCE) == EXTI_PinSource4)  || \
                                       ((PINSOURCE) == EXTI_PinSource5)  || \
                                       ((PINSOURCE) == EXTI_PinSource6)  || \
                                       ((PINSOURCE) == EXTI_PinSource7)  || \
                                       ((PINSOURCE) == EXTI_PinSource8)  || \
                                       ((PINSOURCE) == EXTI_PinSource9)  || \
                                       ((PINSOURCE) == EXTI_PinSource10) || \
                                       ((PINSOURCE) == EXTI_PinSource11) || \
                                       ((PINSOURCE) == EXTI_PinSource12) || \
                                       ((PINSOURCE) == EXTI_PinSource13) || \
                                       ((PINSOURCE) == EXTI_PinSource14) || \
                                       ((PINSOURCE) == EXTI_PinSource15))

/**
 * SYSCFG memoryremap
  * @}
  */ 
#define SYSCFG_MemoryRemap_Flash         ((uint8_t)0x00)
#define SYSCFG_MemoryRemap_SRAM           ((uint8_t)0x03)

#define IS_SYSCFG_MEMORY_REMAP_CONFING(REMAP) (((REMAP) == SYSCFG_MemoryRemap_Flash) || ((REMAP) == SYSCFG_MemoryRemap_SRAM))
/**
 * SYSCFG Cortex-M0  LOCKUP_LOCK
  * @}
*/ 

#define SYSCFG_Lockup_TIM1Break_OFF       ((uint8_t)0x00)
#define SYSCFG_Lockup_TIM1Break_ON        ((uint8_t)0x01)

#define IS_SYSCFG_LOCKUP_TIM1BREAK_ONOFF(ONOFF) (((ONOFF) == SYSCFG_Lockup_TIM1Break_OFF)       || \
                                               ((ONOFF) == SYSCFG_Lockup_TIM1Break_ON)   )
           
/*
*SYSYCFG CFGR1 registers mask
*/
#define MEM_REMAP_MASK                    ((uint32_t)0xFFFFFFC)
#define MEM_LOCKUP_OUT_MASK               ((uint32_t)0x7FFFFFF)                      

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 
 
void       SYSCFG_DeInit(void);
void       SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
void       SYSCFG_MemoryRemapConfig(uint8_t SYSCFG_MemoryRemap);
void       SYSCFG_Lockup_Tim1BreakConfig(uint8_t Lockup_lockOnOff);

#ifdef __cplusplus
}
#endif

#endif /*__HK32F030M_SYSCFG_H */

/**
  * @}
  */ 

