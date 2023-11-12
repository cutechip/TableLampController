/**
  ******************************************************************************
  * @file    hk32f030m_gpio.h 
  * @version V1.0.1
  * @date    2019-08-15
  * author     Rakan.Z
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HK32F030M_GPIO_H
#define __HK32F030M_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hk32f030m.h"

/** @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/

#define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIOA) || \
                                    ((PERIPH) == GPIOB) || \
                                    ((PERIPH) == GPIOC) || \
                                    ((PERIPH) == GPIOD) )

#define IS_GPIO_LIST_PERIPH(PERIPH) (((PERIPH) == GPIOA) || \
                                     ((PERIPH) == GPIOB))

/** @defgroup Configuration_Mode_enumeration 
  * @{
  */
typedef enum
{
  GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode              */
  GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode             */
  GPIO_Mode_AF   = 0x02, /*!< GPIO Alternate function Mode */
  GPIO_Mode_AN   = 0x03  /*!< GPIO Analog In/Out Mode      */
}GPIOMode_TypeDef;

#define IS_GPIO_MODE(MODE) (((MODE) == GPIO_Mode_IN)|| ((MODE) == GPIO_Mode_OUT) || \
                            ((MODE) == GPIO_Mode_AF)|| ((MODE) == GPIO_Mode_AN))
/**
  * @}
  */

/** @defgroup Output_type_enumeration
  * @{
  */
typedef enum
{
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;

#define IS_GPIO_OTYPE(OTYPE) (((OTYPE) == GPIO_OType_PP) || ((OTYPE) == GPIO_OType_OD))

/**
  * @}
  */

/** @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */
typedef enum
{
  GPIO_Speed_Level_1  = 0x00, /*!< I/O output speed: Low 2 MHz */
  GPIO_Speed_Level_2  = 0x01, /*!< I/O output speed: Medium 10 MHz */
}GPIOSpeed_TypeDef;

#define IS_GPIO_SPEED(SPEED) (((SPEED) == GPIO_Speed_Level_1) || ((SPEED) == GPIO_Speed_Level_2))
/**
  * @}
  */

/** @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;

#define IS_GPIO_PUPD(PUPD) (((PUPD) == GPIO_PuPd_NOPULL) || ((PUPD) == GPIO_PuPd_UP) || \
                            ((PUPD) == GPIO_PuPd_DOWN))
/**
  * @}
  */

/** @defgroup Bit_SET_and_Bit_RESET_enumeration
  * @{
  */
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;

#define IS_GPIO_BIT_ACTION(ACTION) (((ACTION) == Bit_RESET) || ((ACTION) == Bit_SET))
/**
  * @}
  */

/* @brief  Configuration Schmit */
typedef enum
{ 
	GPIO_Schmit_Disable = 0x0,
  GPIO_Schmit_Enable 	= 0x1,
}GPIOSchmit_TypeDef;
/**
  * @brief  GPIO Init structure definition  
  */
typedef struct
{
  uint32_t GPIO_Pin;              /*!< Specifies the GPIO pins to be configured.
                                       This parameter can be any value of @ref GPIO_pins_define */
                                       
  GPIOMode_TypeDef GPIO_Mode;     /*!< Specifies the operating mode for the selected pins.
                                       This parameter can be a value of @ref GPIOMode_TypeDef   */

  GPIOSpeed_TypeDef GPIO_Speed;   /*!< Specifies the speed for the selected pins.
                                       This parameter can be a value of @ref GPIOSpeed_TypeDef  */

  GPIOOType_TypeDef GPIO_OType;   /*!< Specifies the operating output type for the selected pins.
                                       This parameter can be a value of @ref GPIOOType_TypeDef  */

  GPIOPuPd_TypeDef GPIO_PuPd;     /*!< Specifies the operating Pull-up/Pull down for the selected pins.
                                       This parameter can be a value of @ref GPIOPuPd_TypeDef   */

  GPIOSchmit_TypeDef  GPIO_Schmit; /*!<GPIO Schmitt>*/                              
    
}GPIO_InitTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup GPIO_Exported_Constants
  * @{
  */

/** @defgroup GPIO_pins_define 
  * @{
  */
#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected    */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected    */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected    */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected    */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected    */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected    */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected    */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected    */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected    */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected    */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected   */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected   */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected   */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected   */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected   */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected   */
#define GPIO_Pin_All               ((uint16_t)0xFFFF)  /*!< All pins selected */

#define IS_GPIO_PIN(PIN) ((PIN) != (uint16_t)0x00)

#define IS_GET_GPIO_PIN(PIN) (((PIN) == GPIO_Pin_0) || \
                              ((PIN) == GPIO_Pin_1) || \
                              ((PIN) == GPIO_Pin_2) || \
                              ((PIN) == GPIO_Pin_3) || \
                              ((PIN) == GPIO_Pin_4) || \
                              ((PIN) == GPIO_Pin_5) || \
                              ((PIN) == GPIO_Pin_6) || \
                              ((PIN) == GPIO_Pin_7) || \
                              ((PIN) == GPIO_Pin_8) || \
                              ((PIN) == GPIO_Pin_9) || \
                              ((PIN) == GPIO_Pin_10) || \
                              ((PIN) == GPIO_Pin_11) || \
                              ((PIN) == GPIO_Pin_12) || \
                              ((PIN) == GPIO_Pin_13) || \
                              ((PIN) == GPIO_Pin_14) || \
                              ((PIN) == GPIO_Pin_15))

/**
  * @}
  */

/** @defgroup GPIO_Pin_sources 
  * @{
  */
#define GPIO_PinSource0            ((uint8_t)0x00)
#define GPIO_PinSource1            ((uint8_t)0x01)
#define GPIO_PinSource2            ((uint8_t)0x02)
#define GPIO_PinSource3            ((uint8_t)0x03)
#define GPIO_PinSource4            ((uint8_t)0x04)
#define GPIO_PinSource5            ((uint8_t)0x05)
#define GPIO_PinSource6            ((uint8_t)0x06)
#define GPIO_PinSource7            ((uint8_t)0x07)
#define GPIO_PinSource8            ((uint8_t)0x08)
#define GPIO_PinSource9            ((uint8_t)0x09)
#define GPIO_PinSource10           ((uint8_t)0x0A)
#define GPIO_PinSource11           ((uint8_t)0x0B)
#define GPIO_PinSource12           ((uint8_t)0x0C)
#define GPIO_PinSource13           ((uint8_t)0x0D)
#define GPIO_PinSource14           ((uint8_t)0x0E)
#define GPIO_PinSource15           ((uint8_t)0x0F)

#define IS_GPIO_PIN_SOURCE(PINSOURCE) (((PINSOURCE) == GPIO_PinSource0) || \
                                       ((PINSOURCE) == GPIO_PinSource1) || \
                                       ((PINSOURCE) == GPIO_PinSource2) || \
                                       ((PINSOURCE) == GPIO_PinSource3) || \
                                       ((PINSOURCE) == GPIO_PinSource4) || \
                                       ((PINSOURCE) == GPIO_PinSource5) || \
                                       ((PINSOURCE) == GPIO_PinSource6) || \
                                       ((PINSOURCE) == GPIO_PinSource7) || \
                                       ((PINSOURCE) == GPIO_PinSource8) || \
                                       ((PINSOURCE) == GPIO_PinSource9) || \
                                       ((PINSOURCE) == GPIO_PinSource10) || \
                                       ((PINSOURCE) == GPIO_PinSource11) || \
                                       ((PINSOURCE) == GPIO_PinSource12) || \
                                       ((PINSOURCE) == GPIO_PinSource13) || \
                                       ((PINSOURCE) == GPIO_PinSource14) || \
                                       ((PINSOURCE) == GPIO_PinSource15))
/**
  * @}
  */

/** @defgroup GPIO_Alternate_function_selection_define 
  * @{
  */

/** 
  * @brief  AF 0 selection
  */
 
#define GPIO_AF_0            ((uint8_t)0x00) /* (I2C1_SWD)I2C1_SMBA, I2C1_SCL, SWCLK_I2C1_SDA, I2C1_SDA, SWDIO*/
/** 
  * @brief  AF 1 selection
  */
#define GPIO_AF_1            ((uint8_t)0x01) /* (USART1)USART1_TX, USART1_RX, USART1_CK */
/** 
  * @brief  AF 2 selection
  */
#define GPIO_AF_2            ((uint8_t)0x02) /* (SPI)SPI1_SCK, SPI1_NSS, SPI1_MISO, SPI1_NSS, SPI1_MOSI*/
/** 
  * @brief  AF 3 selection
  */
#define GPIO_AF_3            ((uint8_t)0x03) /* (TIM1)TIM1_BKIN, TIM1_CH1N, TIM1_CH2N, TIM1_CH3N,TIM1_CH3_CH1N,TIM1_CH4_CH2N, TIM1_ETR,TIM1_CH1,TIM1_CH2,TIM1_CH4,TIM1_ETR */

/** 
  * @brief  AF 4 selection
  */
#define GPIO_AF_4            ((uint8_t)0x04) /* (TIM2)TIM2_CH3, TIM2_ETR, TIM2_CH4, TIM2_CH2, TIM2_CH1*/

/** 
  * @brief  AF 5 selection
  */
#define GPIO_AF_5            ((uint8_t)0x05) /* (RCC)RCC_MCO */

/** 
  * @brief  AF 6 selection
  */
#define GPIO_AF_6            ((uint8_t)0x06) /*(BEEPER)BEEP */
/** 
  * @brief  AF 7 selection
  */
#define GPIO_AF_7            ((uint8_t)0x07) /*(ADC1)ADC1_ETR */

#define IS_GPIO_AF(AF)   (((AF) == GPIO_AF_0) || ((AF) == GPIO_AF_1) || \
                          ((AF) == GPIO_AF_2) || ((AF) == GPIO_AF_3) || \
                          ((AF) == GPIO_AF_4) || ((AF) == GPIO_AF_5) || \
                          ((AF) == GPIO_AF_6) || ((AF) == GPIO_AF_7)  )



/** 
  * @brief  IOMUX PIN selection
  */
#define GPIOMUX_AF3_TIM1CH3       ((uint8_t)0x01) /* PC3_AF3_TIM1CH3  */   //	 PC3 AS AF3
#define GPIOMUX_AF3_TIM1CH1N      ((uint8_t)0x06) /* PC3_AF3_TIM1CH1N  */  //  PC3 AS AF3
#define GPIOMUX_AF3_TIM1CH4       ((uint8_t)0x02) /* PC4_AF3_TIM1CH4  */   //  PC4 AS AF3
#define GPIOMUX_AF3_TIM1CH2N      ((uint8_t)0x05) /* PC4_AF3_TIM1CH2N  */  //  PC4 AS AF3
#define GPIOMUX_AF0_SWCLK         ((uint8_t)0x04) /* PB5_AF0_SWCLK    */   //  PB5 AS AF0
#define GPIOMUX_AF0_I2C_SDA       ((uint8_t)0x03) /* PB5_AF0_I2C_SDA   */  //  PB5 AS AF0

#define GPIO_IOMUX_AF(IOMUX_AF)    (((IOMUX_AF) == GPIOMUX_AF_3_TIM1CH3) || ((IOMUX_AF) == GPIOMUX_AF_3_TIM1CH1N) || \
																	 ((IOMUX_AF) == GPIOMUX_AF_3_TIM1CH4) || ((IOMUX_AF) == GPIOMUX_AF_3_TIM1CH2N)|| \
																	 ((IOMUX_AF) == GPIOMUX_AF_0_SWCLK)|| ((IOMUX_AF) == GPIOMUX_AF_0_I2C_SDA) )
																	 
/**
  * @}
  */

/** @defgroup GPIO_Speed_Legacy 
  * @{
  */

#define GPIO_Speed_2MHz  GPIO_Speed_Level_1   /*!< I/O output speed: Low 2 MHz  */
#define GPIO_Speed_10MHz GPIO_Speed_Level_2   /*!< I/O output speed: Medium 10 MHz */
  
/**
  * @}
  */

/** @defgroup GPIO_IOMUX
  * @{
  */
#define IOMUX_PC3_TIM1CH3           0x00000001          
#define IOMUX_PC3_TIM1CH1N          0xFFFFFFFE
#define IOMUX_PC4_TIM1CH4           0x00000002
#define IOMUX_PC4_TIM1CH2N          0xFFFFFFFD
#define IOMUX_PB5_SWCLK             0x00000004
#define IOMUX_PB5_I2C_SDA           0xFFFFFFFB


 /** @defgroup GPIO_IOMUX_function_selection_define 
  * @{
  */
typedef enum
{
  // SO8N PIN
  IOMUX_PIN1, 
  //IOMUX_PIN4,  only HK32F0301Mxx 
  IOMUX_PIN5,
  IOMUX_PIN6,
  IOMUX_PIN7,
  IOMUX_PIN8,
  // TSSOP16 PIN
  IOMUX_PIN9,
  IOMUX_PIN12,
  IOMUX_PIN15, 
  // TSSOP20/UFQFN20 PIN 
  IOMUX_PIN2,
  IOMUX_PIN11

}IOMUX_PIN;

/// list of IOMUX_FuncPin 

#define IOMUX_PD6_SEL_PD6            ((uint32_t)0x00000000)
#define IOMUX_PD6_SEL_PA1            ((uint32_t)0x00000080)
#define IOMUX_PD6_SEL_PD4            ((uint32_t)0x00000100)
#define IOMUX_PD6_SEL_PA2            ((uint32_t)0x00000180)
#define IOMUX_PD6_SEL_MASK           ((uint32_t)0xFFFFFE7F)

#define IOMUX_PB5_SEL_PB5            ((uint32_t)0x00000000)
#define IOMUX_PB5_SEL_PA3            ((uint32_t)0x00000002)
#define IOMUX_PB5_SEL_PD2            ((uint32_t)0x00000004)
#define IOMUX_PB5_SEL_MASK           ((uint32_t)0xFFFFFFF9)

#define IOMUX_NRST_SEL_NRST           ((uint32_t)0x00000002)
#define IOMUX_NRST_SEL_PA0            ((uint32_t)0x00000000)
#define IOMUX_NRST_SEL_PB4            ((uint32_t)0x00000001)
#define IOMUX_NRST_SEL_MASK           ((uint32_t)0xFFFFFFFE)

#define IOMUX_PC4_SEL_PC4            ((uint32_t)0x00000000)
#define IOMUX_PC4_SEL_PC5            ((uint32_t)0x00000008)
#define IOMUX_PC4_SEL_PC3            ((uint32_t)0x00000010)
#define IOMUX_PC4_SEL_PC7            ((uint32_t)0x00000018)
#define IOMUX_PC4_SEL_MASK           ((uint32_t)0xFFFFFFE7)

#define IOMUX_PD5_SEL_PD5            ((uint32_t)0x00000000)
#define IOMUX_PD5_SEL_PD3            ((uint32_t)0x00000020)
#define IOMUX_PD5_SEL_PD1            ((uint32_t)0x00000040)
#define IOMUX_PD5_SEL_PC6            ((uint32_t)0x00000060)
#define IOMUX_PD5_SEL_MASK           ((uint32_t)0xFFFFFF9F)

#define NRST_PINKEY                 (uint32_t)(0x00005AE1)

#define IS_IOMUX_PIN(IOMUX_PIN)       (((IOMUX_PIN) == IOMUX_PIN1) ||((IOMUX_PIN) == IOMUX_PIN2) ||\
                                        ((IOMUX_PIN) == IOMUX_PIN5) || ((IOMUX_PIN) == IOMUX_PIN6) ||\
                                         ((IOMUX_PIN) == IOMUX_PIN7) || ((IOMUX_PIN) == IOMUX_PIN8) ||\
                                         ((IOMUX_PIN) == IOMUX_PIN9) || ((IOMUX_PIN) == IOMUX_PIN11)||\
                                          ((IOMUX_PIN) == IOMUX_PIN12) || ((IOMUX_PIN) == IOMUX_PIN15) )


#define IS_IOMUX_PINFNC(IOMUX_PINFNC) (((IOMUX_PINFNC) == IOMUX_PD6_SEL_PD6) || ((IOMUX_PINFNC) == IOMUX_PD6_SEL_PA1) || \
                                          ((IOMUX_PINFNC) == IOMUX_PD6_SEL_PD4) || ((IOMUX_PINFNC) == IOMUX_PD6_SEL_PA2)   || \
                                          ((IOMUX_PINFNC) == IOMUX_PD6_SEL_MASK) || ((IOMUX_PINFNC) == IOMUX_PB5_SEL_PB5)  || \
                                          ((IOMUX_PINFNC) == IOMUX_PB5_SEL_PA3) || ((IOMUX_PINFNC) == IOMUX_PB5_SEL_PD2)   || \
                                          ((IOMUX_PINFNC) == IOMUX_PB5_SEL_MASK) || ((IOMUX_PINFNC) == IOMUX_NRST_SEL_NRST)  || \
                                          ((IOMUX_PINFNC) == IOMUX_NRST_SEL_PB4) || ((IOMUX_PINFNC) == IOMUX_NRST_SEL_MASK)  || \
                                          ((IOMUX_PINFNC) == IOMUX_PC4_SEL_PC4) || ((IOMUX_PINFNC) == IOMUX_PC4_SEL_PC5)   || \
                                           ((IOMUX_PINFNC) == IOMUX_PC4_SEL_PC3) || ((IOMUX_PINFNC) == IOMUX_PC4_SEL_PC7)  || \
                                          ((IOMUX_PINFNC) == IOMUX_PC4_SEL_MASK) || ((IOMUX_PINFNC) == IOMUX_PD5_SEL_PD5)  || \
                                          ((IOMUX_PINFNC) == IOMUX_PD5_SEL_PD3) || ((IOMUX_PINFNC) == IOMUX_PD5_SEL_PD1)   || \
                                          ((IOMUX_PINFNC) == IOMUX_PD5_SEL_PC6) || ((IOMUX_PINFNC) == IOMUX_PD5_SEL_MASK)  || \
                                          ((IOMUX_PINFNC) == NRST_PINKEY) || ((IOMUX_PINFNC) == IOMUX_NRST_SEL_PA0))
/**
  * @}
  */
 
typedef enum
{
  TIM2_CN1_EXTERNAL = 0,
  TIM2_CN1_HSIDIV,
  TIM2_CN1_LSI_128,
  TIM2_CN1_EXTERNAL_MAX
}TIM2_SOURCE;

#define IS_TIM2_SOURCE(TIM2_SOURCE)       (((TIM2_SOURCE) == TIM2_CN1_EXTERNAL) || ((TIM2_SOURCE) == TIM2_CN1_HSIDIV) || \
                                          ((TIM2_SOURCE) == TIM2_CN1_LSI_128) || ((TIM2_SOURCE) == TIM2_CN1_EXTERNAL_MAX))




/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Function used to set the GPIO configuration to the default reset state *****/
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

/* Initialization and Configuration functions *********************************/
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* GPIO Read and Write functions **********************************************/
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_Toggle(GPIO_TypeDef* GPIOx , uint16_t GPIO_Pin);

/* GPIO Alternate functions configuration functions ***************************/
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);
/* GPIO IOMUX*/
void GPIO_IOMUX_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t IOMUX_AF);
void GPIO_IOMUX_ChangePin(IOMUX_PIN eIOMUX_Pinx, uint32_t eIOMUX_FuncPin);

void GPIO_IOMUX_SetTIM2CN1_Source(TIM2_SOURCE TIM2CN1Source);
#ifdef __cplusplus
}
#endif

#endif /* __HK32F030M_GPIO_H */
/**
  * @}
  */

/**
  * @}
  */
 
