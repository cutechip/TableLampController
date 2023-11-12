/**
  ******************************************************************************
  * @file    hk32f030m_conf_Template.h
  * @brief   hk32f030m configuration file of backup.
  ******************************************************************************
  * @attention 
  * Users can refer to this file for custom configuration 
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HK32F030M_CONF_H
#define __HK32F030M_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC  module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).  
  */
 

#define EXTCLK_VALUE	((uint32_t)32000000) /*!< Value of the Internal oscillator in Hz*/



/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC  module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL). 
  */

#define HSI_VALUE    ((uint32_t)32000000) /*!< Value of the Internal oscillator in Hz*/


/**
  * @brief In the following line adjust the Internal High Speed oscillator (HSI) Startup 
  *        Timeout value 
  */

 #define STARTUP_TIMEOUT   ((uint32_t)0xFFFF) /*!< Time out for start up */
 

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */

 #define LSI_VALUE  ((uint32_t)114000)    
  /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations*/





/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file 
  */

 #include "hk32f030m_rcc.h"

 #include "hk32f030m_crc.h"

 #include "hk32f030m_exti.h"

 #include "hk32f030m_flash.h"

 #include "hk32f030m_gpio.h"

 #include "hk32f030m_misc.h"

 #include "hk32f030m_adc.h"

 #include "hk32f030m_syscfg.h"

 #include "hk32f030m_def.h"

 #include "hk32f030m_i2c.h"

 #include "hk32f030m_iwdg.h"

 #include "hk32f030m_pwr.h"

 #include "hk32f030m_spi.h"

 #include "hk32f030m_tim.h"

 #include "hk32f030m_usart.h"

 #include "hk32f030m_iwdg.h"

 #include "hk32f030m_wwdg.h"

 #include "hk32f030m_awu.h"
 
 #include "hk32f030m_beep.h"
/* Exported macro ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *         drivers code
  */
//#define USE_FULL_ASSERT   (1U) 

#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed. 
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((char *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(char* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */    
    
#ifdef __cplusplus
}
#endif

#endif /* __HK32F030M_CONF_H */

/************************ (C) COPYRIGHT MKMcircoChuip *****END OF FILE****/
