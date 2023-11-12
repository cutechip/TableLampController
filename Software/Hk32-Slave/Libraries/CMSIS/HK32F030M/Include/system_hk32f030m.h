/**
	******************************************************************************
	* @file    system_hk32f030m.h
	* @author  Rakan.Z/laura.C
	* @version V1.0  
	* @brief   API file of system clk config
	* @changelist  
	******************************************************************************
*/ 


#ifndef __SYSTEM_HK32F030M_H
#define __SYSTEM_HK32F030M_H

#ifdef __cplusplus
 extern "C" {
#endif 




/** hk32f030m_System_Exported_types  */

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */
extern const uint16_t AHBPrescTable[16];   /*!< AHB prescalers table values */
extern const uint8_t APBPrescTable[8];    /*!< APB prescalers table values */


extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_HK32F030M_H */


