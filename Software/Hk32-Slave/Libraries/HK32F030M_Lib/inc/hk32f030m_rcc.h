/**
  ******************************************************************************
  * @file    hk32f030m_rcc.h
  * @author  laura.C
  * @version V1.0  
  * @brief   API file of RCC module
  * @changelist  
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HK32F030M_RCC_H
#define __HK32F030M_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hk32f030m.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK_Frequency;
  uint32_t ADCCLK_Frequency;
  uint32_t I2C1CLK_Frequency;
  uint32_t USART1CLK_Frequency;
}RCC_ClocksTypeDef;

/* Exported constants --------------------------------------------------------*/


 
/** RCC_System_Clock_Source  */

#define RCC_SYSCLKSource_HSI             RCC_CFGR_SW_HSI
#define RCC_SYSCLKSource_EXTCLK          RCC_CFGR_SW_EXTCLK
#define RCC_SYSCLKSource_LSI             RCC_CFGR_SW_LSI


#define IS_RCC_SYSCLK_SOURCE(SOURCE) (((SOURCE) == RCC_SYSCLKSource_HSI)   || \
                                      ((SOURCE) == RCC_SYSCLKSource_EXTCLK)   || \
                                      ((SOURCE) == RCC_SYSCLKSource_LSI))

/**  RCC_AHB_Clock_Source  */

#define RCC_SYSCLK_Div1                  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_Div2                  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_Div4                  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_Div6                  RCC_CFGR_HPRE_DIV6
#define RCC_SYSCLK_Div8                  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_Div16                 RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_Div64                 RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_Div128                RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_Div256                RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_Div512                RCC_CFGR_HPRE_DIV512
#define IS_RCC_HCLK(HCLK) (((HCLK) == RCC_SYSCLK_Div1) || ((HCLK) == RCC_SYSCLK_Div2) || \
                           ((HCLK) == RCC_SYSCLK_Div4) || ((HCLK) == RCC_SYSCLK_Div8) || \
                           ((HCLK) == RCC_SYSCLK_Div16) || ((HCLK) == RCC_SYSCLK_Div64) || \
                           ((HCLK) == RCC_SYSCLK_Div128) || ((HCLK) == RCC_SYSCLK_Div256) || \
                           ((HCLK) == RCC_SYSCLK_Div128) ||((HCLK) == RCC_SYSCLK_Div6))


/** RCC_APB_Clock_Source  */

#define RCC_HCLK_Div1                    RCC_CFGR_PPRE_DIV1
#define RCC_HCLK_Div2                    RCC_CFGR_PPRE_DIV2
#define RCC_HCLK_Div4                    RCC_CFGR_PPRE_DIV4
#define RCC_HCLK_Div8                    RCC_CFGR_PPRE_DIV8
#define RCC_HCLK_Div16                   RCC_CFGR_PPRE_DIV16
#define IS_RCC_PCLK(PCLK) (((PCLK) == RCC_HCLK_Div1) || ((PCLK) == RCC_HCLK_Div2) || \
                           ((PCLK) == RCC_HCLK_Div4) || ((PCLK) == RCC_HCLK_Div8) || \
                           ((PCLK) == RCC_HCLK_Div16))
                           
/**  RCC_ADC_clock_source  */
#define RCC_ADCCLK_HSI32M_Div1             ((uint32_t)0x00000000)  
#define RCC_ADCCLK_HSI32M_Div1_5           ((uint32_t)0x04000000)  
#define RCC_ADCCLK_HSI32M_Div2             ((uint32_t)0x08000000)  
#define RCC_ADCCLK_HSI32M_Div2_5           ((uint32_t)0x0C000000)  
#define RCC_ADCCLK_HSI32M_Div3             ((uint32_t)0x10000000)  
#define RCC_ADCCLK_HSI32M_Div3_5           ((uint32_t)0x14000000)  
#define RCC_ADCCLK_HSI32M_Div4             ((uint32_t)0x18000000)  
#define RCC_ADCCLK_HSI32M_Div4_5           ((uint32_t)0x1C000000)  
#define RCC_ADCCLK_HSI32M_Div5             ((uint32_t)0x20000000)  
#define RCC_ADCCLK_HSI32M_Div5_5           ((uint32_t)0x24000000)  
#define RCC_ADCCLK_HSI32M_Div6             ((uint32_t)0x28000000)  
#define RCC_ADCCLK_HSI32M_Div6_5           ((uint32_t)0x2C000000)  
#define RCC_ADCCLK_HSI32M_Div7             ((uint32_t)0x30000000)  
#define RCC_ADCCLK_HSI32M_Div7_5           ((uint32_t)0x34000000)  
#define RCC_ADCCLK_HSI32M_Div8             ((uint32_t)0x38000000)  
#define RCC_ADCCLK_HSI32M_Div8_5           ((uint32_t)0x3C000000)  
#define RCC_ADCCLK_HSI32M_Div9             ((uint32_t)0x40000000)  
#define RCC_ADCCLK_HSI32M_Div9_5           ((uint32_t)0x44000000)  
#define RCC_ADCCLK_HSI32M_Div10            ((uint32_t)0x48000000)  
#define RCC_ADCCLK_HSI32M_Div10_5          ((uint32_t)0x4C000000)  
#define RCC_ADCCLK_HSI32M_Div11            ((uint32_t)0x50000000)  
#define RCC_ADCCLK_HSI32M_Div11_5          ((uint32_t)0x54000000)  
#define RCC_ADCCLK_HSI32M_Div12            ((uint32_t)0x58000000)  
#define RCC_ADCCLK_HSI32M_Div12_5          ((uint32_t)0x5C000000)  
#define RCC_ADCCLK_HSI32M_Div13            ((uint32_t)0x60000000)  
#define RCC_ADCCLK_HSI32M_Div13_5          ((uint32_t)0x64000000)  
#define RCC_ADCCLK_HSI32M_Div14            ((uint32_t)0x68000000)  
#define RCC_ADCCLK_HSI32M_Div14_5          ((uint32_t)0x6C000000)  
#define RCC_ADCCLK_HSI32M_Div15            ((uint32_t)0x70000000)  
#define RCC_ADCCLK_HSI32M_Div15_5          ((uint32_t)0x74000000)  
#define RCC_ADCCLK_HSI32M_Div16            ((uint32_t)0x78000000)  
#define RCC_ADCCLK_HSI32M_Div16_5          ((uint32_t)0x7C000000)  

#define RCC_CFGR4_ADCHSIPRE				        ((uint32_t)0x7C000000) 

#define RCC_ADCCLK_PCLK_DIV2              ADC_CFGR2_CKMODE_0
#define RCC_ADCCLK_PCLK_DIV4          	  ADC_CFGR2_CKMODE_1

/**  RCC_I2C_clock_source */

#define RCC_I2C1CLK_HSI                   ((uint32_t)0x00000000)
#define RCC_I2C1CLK_SYSCLK                RCC_CFGR3_I2C1SW

#define IS_RCC_I2CCLK(I2CCLK) (((I2CCLK) == RCC_I2C1CLK_HSI) || ((I2CCLK) == RCC_I2C1CLK_SYSCLK))


/**  RCC_USART_clock_source   */
#define RCC_USART1CLK_PCLK                  ((uint32_t)0x00000000)
#define RCC_USART1CLK_SYSCLK                ((uint32_t)0x00000001)
#define RCC_USART1CLK_HSI                   ((uint32_t)0x00000003)

#define IS_RCC_USARTCLK(USARTCLK) (((USARTCLK) == RCC_USART1CLK_PCLK)   || \
                                   ((USARTCLK) == RCC_USART1CLK_SYSCLK) || \
                                   ((USARTCLK) == RCC_USART1CLK_HSI))
         
/**  RCC_Interrupt_Source   */
#define RCC_IT_LSIRDY                    ((uint8_t)0x01)
#define RCC_IT_HSIRDY                    ((uint8_t)0x04)
#define RCC_IT_EXTRDY                    ((uint8_t)0x08)
#define RCC_IT_CSS                       ((uint8_t)0x80)

#define IS_RCC_GET_IT(IT) (((IT) == RCC_IT_LSIRDY) || ((IT) == RCC_IT_HSIRDY) || \
                           ((IT) == RCC_IT_CSS)    || ((IT) == RCC_IT_EXTRDY))

#define IS_RCC_CLEAR_IT(IT) ((IT) != 0x00)

  
/**  RCC_AHB_Peripherals   */
#define RCC_AHBPeriph_GPIOA               RCC_AHBENR_GPIOAEN
#define RCC_AHBPeriph_GPIOB               RCC_AHBENR_GPIOBEN
#define RCC_AHBPeriph_GPIOC               RCC_AHBENR_GPIOCEN
#define RCC_AHBPeriph_GPIOD               RCC_AHBENR_GPIODEN
#define RCC_AHBPeriph_CRC                 RCC_AHBENR_CRCEN
#define RCC_AHBPeriph_FLITF               RCC_AHBENR_FLITFEN
#define RCC_AHBPeriph_SRAM                RCC_AHBENR_SRAMEN


#define IS_RCC_AHB_PERIPH(PERIPH) ((((PERIPH) & 0xFFE1FFAB) == 0x00) && ((PERIPH) != 0x00))
#define IS_RCC_AHB_RST_PERIPH(PERIPH) ((((PERIPH) & 0xFFE1FFAB) == 0x00) && ((PERIPH) != 0x00))



/**  RCC_APB2_Peripherals   */

#define RCC_APB2Periph_SYSCFG            RCC_APB2ENR_SYSCFGEN
#define RCC_APB2Periph_ADC               RCC_APB2ENR_ADCEN
#define RCC_APB2Periph_TIM1              RCC_APB2ENR_TIM1EN
#define RCC_APB2Periph_SPI1              RCC_APB2ENR_SPI1EN
#define RCC_APB2Periph_USART1            RCC_APB2ENR_USART1EN
#define RCC_APB2Periph_DBGMCU            RCC_APB2ENR_DBGMCUEN

#define IS_RCC_APB2_PERIPH(PERIPH) ((((PERIPH) & 0xFFB8A51E) == 0x00) && ((PERIPH) != 0x00))


/**  RCC_APB1_Peripherals   */
#define RCC_APB1Periph_TIM2              RCC_APB1ENR_TIM2EN
#define RCC_APB1Periph_TIM6              RCC_APB1ENR_TIM6EN
#define RCC_APB1Periph_WWDG              RCC_APB1ENR_WWDGEN
#define RCC_APB1Periph_AWU               RCC_APB1ENR_AWUEN
#define RCC_APB1Periph_I2C1              RCC_APB1ENR_I2C1EN
#define RCC_APB1Periph_PWR               RCC_APB1ENR_PWREN
#define RCC_APB1Periph_BEEPER            RCC_APB1ENR_BEEPEREN    
#define RCC_APB1Periph_IOMUX             RCC_APB1ENR_IOMUXEN     

#define IS_RCC_APB1_PERIPH(PERIPH) ((((PERIPH) & 0x8FDEF7EE) == 0x00) && ((PERIPH) != 0x00))

/** RCC_MCO_Clock_Source  */

#define RCC_MCOSource_NoClock            ((uint8_t)0x00)
#define RCC_MCOSource_LSI                ((uint8_t)0x02)
#define RCC_MCOSource_SYSCLK             ((uint8_t)0x04)
#define RCC_MCOSource_HSI                ((uint8_t)0x05)


#define IS_RCC_MCO_SOURCE(SOURCE) (((SOURCE) == RCC_MCOSource_NoClock) || ((SOURCE) == RCC_MCOSource_SYSCLK)  ||\
									((SOURCE) == RCC_MCOSource_HSI)    || ((SOURCE) == RCC_MCOSource_LSI))


/**  RCC_MCOPrescaler  */

#define RCC_MCOPrescaler_1            ((uint32_t)0x00000000)
#define RCC_MCOPrescaler_2            ((uint32_t)0x10000000)
#define RCC_MCOPrescaler_4            ((uint32_t)0x20000000)
#define RCC_MCOPrescaler_8            ((uint32_t)0x30000000)
#define RCC_MCOPrescaler_16           ((uint32_t)0x40000000)
#define RCC_MCOPrescaler_32           ((uint32_t)0x50000000)
#define RCC_MCOPrescaler_64           ((uint32_t)0x60000000)
#define RCC_MCOPrescaler_128          ((uint32_t)0x70000000)

#define IS_RCC_MCO_PRESCALER(PRESCALER) (((PRESCALER) == RCC_MCOPrescaler_1)  || \
                                         ((PRESCALER) == RCC_MCOPrescaler_2)  || \
                                         ((PRESCALER) == RCC_MCOPrescaler_4)  || \
                                         ((PRESCALER) == RCC_MCOPrescaler_8)  || \
                                         ((PRESCALER) == RCC_MCOPrescaler_16) || \
                                         ((PRESCALER) == RCC_MCOPrescaler_32) || \
                                         ((PRESCALER) == RCC_MCOPrescaler_64) || \
                                         ((PRESCALER) == RCC_MCOPrescaler_128))

/** @defgroup RCC_Flag   */

#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_EXTCLKDY                ((uint8_t)0x31)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)


 #define IS_RCC_FLAG(FLAG) (((FLAG) == RCC_FLAG_HSIRDY) || ((FLAG) == RCC_FLAG_EXTCLKDY) || \
                            ((FLAG) == RCC_FLAG_LSIRDY) || ((FLAG) == RCC_FLAG_PORRST) || \
                            ((FLAG) == RCC_FLAG_PINRST) || ((FLAG) == RCC_FLAG_SFTRST) || \
                            ((FLAG) == RCC_FLAG_IWDGRST)|| ((FLAG) == RCC_FLAG_PINRST) || \
                            ((FLAG) == RCC_FLAG_WWDGRST)|| ((FLAG) == RCC_FLAG_LPWRRST))

#define IS_RCC_HSI_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x1F)


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Function used to set the RCC clock configuration to the default reset state */
void RCC_DeInit(void);

/* Internal clocks, CSS and MCO configuration functions *********/
ErrorStatus RCC_WaitForStartUp(uint8_t RCC_FLAG);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_LSICmd(FunctionalState NewState);
void RCC_EXTCmd(FunctionalState NewState, uint32_t EXTCKL_SEL);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCOSource,uint32_t RCC_MCOPrescaler);


/* System, AHB and APB busses clocks configuration functions ******************/
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLKConfig(uint32_t RCC_HCLK);
void RCC_ADCCLKConfig(uint32_t RCC_ADCCLK); 
void RCC_I2CCLKConfig(uint32_t RCC_I2CCLK);
void RCC_USARTCLKConfig(uint32_t RCC_USARTCLK);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

/* Peripheral clocks configuration functions **********************************/
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

/* Interrupts and flags management functions **********************************/
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);

#ifdef __cplusplus
}
#endif

#endif /* __HK32F030M_RCC_H */


