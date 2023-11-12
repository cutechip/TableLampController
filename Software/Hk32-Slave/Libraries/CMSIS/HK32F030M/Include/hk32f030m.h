/*
 * HKMicroChip Limited (HKMicroChip) is supplying this software for use with Cortex-M0!
 *
 * @file     hk32f030m.h
 * @brief    CMSIS HeaderFile
 * @version  1.0
 * @date     01. March 2020
 * @Author   Rakan.Z
 */



/** @addtogroup HKMicroChip Ltd.
  * @{
  */


/** @addtogroup HK32F030M
  * @{
  */


#ifndef HK32F030M_H
#define HK32F030M_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined (HK32F030MJ4M6) && !defined (HK32F030MD4P6) && !defined (HK32F030MF4P6) && !defined (HK32F030MF4U6) 
  #error "Please select first the target HK32F030M device used in your application"
#endif

/** @addtogroup Configuration_of_CMSIS
  * @{
  */



/* =========================================================================================================================== */
/* ================                                Interrupt Number Definition                                ================ */
/* =========================================================================================================================== */
typedef enum
{
/******  Cortex-M0 Processor Exceptions Numbers **************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                        */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                                */
  SVC_IRQn                    = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                                  */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                                  */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                              */

/******  HK32F030M specific Interrupt Numbers ******************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                       */  
  EXTI11_IRQn           	    = 2,      /* EXTI Line 11 interrupt(AWU_WKP)                                   */ 
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                          */
  RCC_IRQn                    = 4,      /*!< RCC global Interrupt                                            */
  EXTI0_IRQn          		    = 5,      /*!< EXTI Line 0 Interrupt                                           */
  EXTI1_IRQn            	    = 6,      /*!< EXTI Line 1 Interrupt                                           */
  EXTI2_IRQn            	    = 7,      /*!< EXTI Line 2 Interrupt                                           */
  EXTI3_IRQn           		    = 8,      /*!*!< EXTI Line 3 Interrupt                                         */
  EXTI4_IRQn           		    = 9,      /*!*!< EXTI Line 4 Interrupt                                         */
  EXTI5_IRQn           		    = 10,     /*!*!< EXTI Line 5 Interrupt                                         */
  TIM1_BRK_IRQn         	    = 11,     /*!< TIM1 break interrupt                                            */
  ADC1_IRQn                   = 12,     /*!< ADC1 Interrupt(combined with EXTI line 8)                       */
  TIM1_UP_TRG_COM_IRQn        = 13,     /*!< TIM1  Update, Trigger and Commutation Interrupt                 */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                                  */
  TIM2_IRQn                   = 15,     /*!< TIM2 global interrupt                                           */                                        
  TIM6_IRQn                   = 17,     /*!< TIM6 global Interrupt                                           */
  EXTI6_IRQn                  = 21,     /*!*!< EXTI Line 6 Interrupt                                         */
  EXTI7_IRQn                  = 22,     /*!*!< EXTI Line 7 Interrupt                                         */
  I2C1_IRQn                   = 23,     /*!< I2C1 Event Interrupt                                            */
  SPI1_IRQn                   = 25,     /*!< SPI1 global Interrupt                                           */
  USART1_IRQn                 = 27,     /*!< USART1 global Interrupt                                         */
} IRQn_Type;


/* =========================================================================================================================== */
/* ================                           Processor and Core Peripheral Section                           ================ */
/* =========================================================================================================================== */

/* ===========================  Configuration of the ARM Cortex-M0 Processor and Core Peripherals  =========================== */
#define __CM0_REV                 0x0000U       /*!< CM0 Core Revision                                                         */
#define __NVIC_PRIO_BITS               3        /*!< Number of Bits used for Priority Levels                                   */
#define __Vendor_SysTickConfig         0        /*!< Set to 1 if different SysTick Config is used                              */
#define __MPU_PRESENT                  0        /*!< MPU present                                                               */
#define __FPU_PRESENT                  0        /*!< FPU present                                                               */


/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm0.h"                           /*!< ARM Cortex-M0 processor and core peripherals                              */
#include "system_hk32f030m.h"                   /*!< hk32f030m System                                                          */

#ifndef __IM                                    /*!< Fallback for older CMSIS versions                                         */
  #define __IM   __I
#endif
#ifndef __OM                                    /*!< Fallback for older CMSIS versions                                         */
  #define __OM   __O
#endif
#ifndef __IOM                                   /*!< Fallback for older CMSIS versions                                         */
  #define __IOM  __IO
#endif


/* ========================================  Start of section using anonymous unions  ======================================== */
#if defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
  #pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
  #pragma clang diagnostic ignored "-Wnested-anon-types"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/* =========================================================================================================================== */
/* ================                            Device Specific Peripheral Section                             ================ */
/* =========================================================================================================================== */


/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */
typedef struct
{
  __IO uint32_t ISR;          /*!< ADC interrupt and status register,             Address offset: 0x00 */
  __IO uint32_t IER;          /*!< ADC interrupt enable register,                 Address offset: 0x04 */
  __IO uint32_t CR;           /*!< ADC control register,                          Address offset: 0x08 */
  __IO uint32_t CFGR1;        /*!< ADC configuration register 1,                  Address offset: 0x0C */
  __IO uint32_t CFGR2;        /*!< ADC configuration register 2,                  Address offset: 0x10 */
  __IO uint32_t SMPR;         /*!< ADC sampling time register,                    Address offset: 0x14 */
       uint32_t RESERVED1;    /*!< Reserved,                                                      0x18 */
       uint32_t RESERVED2;    /*!< Reserved,                                                      0x1C */
  __IO uint32_t TR;           /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
       uint32_t RESERVED3;    /*!< Reserved,                                                      0x24 */
  __IO uint32_t CHSELR;       /*!< ADC group regular sequencer register,          Address offset: 0x28 */
       uint32_t RESERVED4[5]; /*!< Reserved,                                                      0x2C */
  __IO uint32_t DR;           /*!< ADC group regular data register,               Address offset: 0x40 */
	     uint32_t RESERVED5[177];  /*!< Reserved,                             Address offset:0x44 - 0x304*/
	__IO uint32_t CCR;          /*!< ADC common configuration register              Address offset: 0x308 */
	     uint32_t RESERVED6[57];/*!< Reserved,                                      Address offset: 0x30C */
	__IO uint32_t CR2;          /*!< ADC control register                           Address offset: 0x3f0 */
} ADC_TypeDef;

/** 
  * @brief CRC calculation unit
  */

typedef struct
{
  __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint8_t  IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
  uint8_t       RESERVED0;   /*!< Reserved,                                                    0x05 */
  uint16_t      RESERVED1;   /*!< Reserved,                                                    0x06 */
  __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */ 
  uint32_t      RESERVED2;   /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t RESERVED3;   /*!< Reserved,                                                    0x14 */
} CRC_TypeDef;

/** 
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;       /*!< MCU device ID code,                          Address offset: 0x00 */
  __IO uint32_t CR;           /*!< Debug MCU configuration register,            Address offset: 0x04 */
  __IO uint32_t APB1FZ;       /*!< Debug MCU APB1 freeze register,              Address offset: 0x08 */ 
}DBGMCU_TypeDef;


/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;          /*!<EXTI Interrupt mask register,                 Address offset: 0x00 */
  __IO uint32_t EMR;          /*!<EXTI Event mask register,                     Address offset: 0x04 */
  __IO uint32_t RTSR;         /*!<EXTI Rising trigger selection register ,      Address offset: 0x08 */
  __IO uint32_t FTSR;         /*!<EXTI Falling trigger selection register,      Address offset: 0x0C */
  __IO uint32_t SWIER;        /*!<EXTI Software interrupt event register,       Address offset: 0x10 */
  __IO uint32_t PR;           /*!<EXTI Pending register,                        Address offset: 0x14 */
} EXTI_TypeDef;

/** 
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t ACR;          /*!<FLASH access control register,                 Address offset: 0x00 */
  __IO uint32_t KEYR;         /*!<FLASH key register,                            Address offset: 0x04 */
  __IO uint32_t OPTKEYR;      /*!<FLASH OPT key register,                        Address offset: 0x08 */
  __IO uint32_t SR;           /*!<FLASH status register,                         Address offset: 0x0C */
  __IO uint32_t CR;           /*!<FLASH control register,                        Address offset: 0x10 */
  __IO uint32_t AR;           /*!<FLASH address register,                        Address offset: 0x14 */
  __IO uint32_t RESERVED;     /*!< Reserved,                                                     0x18 */
  __IO uint32_t OBR;          /*!<FLASH option bytes register,                   Address offset: 0x1C */
  __IO uint32_t WRPR;         /*!<FLASH option bytes register,                   Address offset: 0x20 */
  __IO uint32_t RESERVED1[19];    /*!< Reserved,					                 */
  __IO uint32_t ECR;         /*!<EEPROM control register,                   	 Address offset: 0x70 */  
  __IO uint32_t INT_VEC_OFFSET;  /*							                   	 Address offset: 0x74 */   
} FLASH_TypeDef;

/** 
  * @brief Option Bytes Registers
  */
typedef struct
{
  __IO uint16_t RDP;          /*!< FLASH option byte Read protection,             Address offset: 0x00 */
  __IO uint16_t USER;         /*!< FLASH option byte user options,                Address offset: 0x02 */
  __IO uint16_t DATA0;        /*!< User data byte 0 (stored in FLASH_OBR[23:16]), Address offset: 0x04 */
  __IO uint16_t DATA1;        /*!< User data byte 1 (stored in FLASH_OBR[31:24]), Address offset: 0x06 */
  __IO uint16_t WRP0;         /*!< FLASH option byte write protection 0,          Address offset: 0x08 */
  __IO uint16_t WRP1;         /*!< FLASH option byte write protection 1,          Address offset: 0x0A */
  __IO uint16_t WRP2;         /*!< FLASH option byte write protection 2,          Address offset: 0x0C */
  __IO uint16_t WRP3;         /*!< FLASH option byte write protection 3,          Address offset: 0x0E */  
  __IO uint16_t IWDG_RL_IV;   /*!< IWDG_RLR init value          				Address offset: 0x10 */
  __IO uint16_t IWDG_INI_KEY; /*!< IWDG_RL_IV is valid,          				Address offset: 0x12 */
  __IO uint16_t LSI_LP_CTL;   /*!< LSI ,          								Address offset: 0x14 */
  __IO uint16_t DBG_CLK_CTL;  /*!< DBG clk,          							Address offset: 0x16 */    
} OB_TypeDef;

/** 
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;        /*!< GPIO port mode register,                     Address offset: 0x00      */
  __IO uint32_t OTYPER;       /*!< GPIO port output type register,              Address offset: 0x04      */
  __IO uint32_t OSPEEDR;      /*!< GPIO port output speed register,             Address offset: 0x08      */
  __IO uint32_t PUPDR;        /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C      */
  __IO uint32_t IDR;          /*!< GPIO port input data register,               Address offset: 0x10      */
  __IO uint32_t ODR;          /*!< GPIO port output data register,              Address offset: 0x14      */
  __IO uint32_t BSRR;         /*!< GPIO port bit set/reset register,            Address offset: 0x1A      */
  __IO uint32_t LCKR;         /*!< GPIO port configuration lock register,       Address offset: 0x1C      */
  __IO uint32_t AFR[2];       /*!< GPIO alternate function low register,        Address offset: 0x20-0x24 */
  __IO uint32_t BRR;          /*!< GPIO bit reset register,                     Address offset: 0x28      */
  __IO uint32_t RESERVED;      /*!< Reserved                          					Address offset: 0x2C      */
  __IO uint32_t IOSR;          /*!< GPIO Schmitt  disable register,             Address offset: 0x30      */
} GPIO_TypeDef;

/** 
  * @brief General Purpose I/O MUX
  */

typedef struct
{
  __IO uint32_t PIN_FUNC_SEL;         /*!< GPIO IOMUX Pin Function select register,     Address offset: 0x00      */
  __IO uint32_t PKG_PIN_SEL;         /*!< GPIO IOMUX Pin Function select register,     Address offset: 0x04      */
  __IO uint32_t NRST_PIN_KEY;         /*!< GPIO IOMUX Pin Function select register,     Address offset: 0x08      */
  __IO uint32_t NRST_PA0_SEL;         /*!< GPIO IOMUX Pin Function select register,     Address offset: 0x0C      */
  __IO uint32_t TIM2_CH0_IN_SEL;      /*!< GPIO IOMUX Pin Function select register,     Address offset: 0x10      */
  } GPIOMUX_TypeDef;

/** 
  * @brief SysTem Configuration
  */

typedef struct
{
  __IO uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                           Address offset: 0x00 */
       uint32_t RESERVED;    /*!< Reserved,                                                                  0x04 */
  __IO uint32_t EXTICR[2];   /*!< SYSCFG external interrupt configuration register,     Address offset: 0x0C-0x08 */
} SYSCFG_TypeDef;

/** 
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;          /*!< I2C Control register 1,                      Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< I2C Control register 2,                      Address offset: 0x04 */
  __IO uint32_t OAR1;     /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;     /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;  /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;      /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;      /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;     /*!< I2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;     /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;     /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/** 
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;

/** 
  * @brief Power Control
  */
/*
* BY Rakan.zhang
*/
typedef struct
{
  __IO uint32_t CR;   			    /*!< PWR power control register,                          Address offset: 0x00 */
  __IO uint32_t RESERVED[25];  /* reserved    											                      padding 0x04-0x64    */
  __IO uint32_t VREF_SEL;   	/*!< PWR power LDO voltage reference configer register,     				    Address offset: 0x68 */
} PWR_TypeDef;

/** 
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                   Address offset: 0x00 */
  __IO uint32_t CFGR;       /*!< RCC clock configuration register,                            Address offset: 0x04 */
  __IO uint32_t CIR;        /*!< RCC clock interrupt register,                                Address offset: 0x08 */
  __IO uint32_t APB2RSTR;   /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
  __IO uint32_t APB1RSTR;   /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHBENR;     /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
  __IO uint32_t APB2ENR;    /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
  __IO uint32_t APB1ENR;    /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
  __IO uint32_t RESERVED0;       											/*                Address offset: 0x20 */
  __IO uint32_t CSR;        /*!< RCC clock control & status register,                         Address offset: 0x24 */
  __IO uint32_t AHBRSTR;    /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
  __IO uint32_t RESERVED1;     																/*Address offset: 0x2C */
  __IO uint32_t CFGR3;      /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
  __IO uint32_t CR2;        /*!< RCC clock control register 2,                                Address offset: 0x34 */
       uint32_t RESERVED[42]; /* reserved    											    padding 0x38-0xdc    */
  __IO uint32_t RCC_CSS;               /*!< (@ 0x000000E0) HSE control register            	 Address offset: 0xe0    */
       uint32_t RESERVED2;																  /* Address offset: 0xe4    */
  __IO uint32_t CFGR4;                   /*!< (@ 0x000000E8) Clock configuration register 4  Address offset: 0xe8   */
} RCC_TypeDef;


/** 
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< SPI Control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI Control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI Status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI Rx CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI Tx CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
	__IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

/** 
  * @brief TIM
  */
typedef struct
{
  __IO uint32_t CR1;          /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;         /*!< TIM slave Mode Control register,     Address offset: 0x08 */
  __IO uint32_t DIER;         /*!< TIM interrupt enable register,       Address offset: 0x0C */
  __IO uint32_t SR;           /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;          /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;        /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;        /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;         /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;          /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;          /*!< TIM prescaler register,              Address offset: 0x28 */
  __IO uint32_t ARR;          /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;             /*!< TIM  repetition counter register, Address offset: 0x30 */
  __IO uint32_t CCR1;         /*!< TIM capture/compare register 1,      Address offset: 0x34 */    
  __IO uint32_t CCR2;         /*!< TIM capture/compare register 2,      Address offset: 0x38 */    
  __IO uint32_t CCR3;         /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;         /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;            /*!< TIM break and dead-time register, Address offset: 0x44 */
} TIM_TypeDef;

/** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
 
typedef struct
{
  __IO uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */ 
  __IO uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */ 
  __IO uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint32_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  __IO uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */  
  __IO uint32_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
  __IO uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint32_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
  __IO uint32_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
} USART_TypeDef;

/** 
  * @brief Window WATCHDOG
  */
typedef struct
{
  __IO uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

/** 
  * @brief STOPMODE AUTO WAKEUP
  */
typedef struct
{
  __IO uint32_t CR;   /*!< AWU Control register,       Address offset: 0x00 */
  __IO uint32_t SR;   /*!< AWU Status register,        Address offset: 0x04 */
} AWU_TypeDef;


typedef struct
{
	__IO uint32_t CFGR;
	__IO uint32_t CR;
}BEEP_TypeDef;
		


 /** 
  * @}
  *  */
  /* End of  Peripheral_registers_structures */


  
/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            ((uint32_t)0x08000000U)              /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END       ((uint32_t)0x0800FFFFU)              /*!< FLASH END address of bank1 */
#define SRAM_BASE             ((uint32_t)0x20000000U)              /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000U)              /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map */
#define APBPERIPH_BASE        PERIPH_BASE
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)
  
#define SRAM_BB_BASE          ((uint32_t)0x22000000U)             /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000U)             /*!< Peripheral base address in the bit-band region */


/*!< APB peripherals */
#define TIM2_BASE             (APBPERIPH_BASE + 0x00000000)  
#define TIM6_BASE             (APBPERIPH_BASE + 0x00001000)
#define WWDG_BASE             (APBPERIPH_BASE + 0x00002C00)
#define IWDG_BASE             (APBPERIPH_BASE + 0x00003000)
#define I2C1_BASE             (APBPERIPH_BASE + 0x00005400)
#define PWR_BASE              (APBPERIPH_BASE + 0x00007000)
#define AWU_BASE              (APBPERIPH_BASE + 0x00007800)
#define BEEPER_BASE           (APBPERIPH_BASE + 0x00007C00)
#define SYSCFG_BASE           (APBPERIPH_BASE + 0x00010000)
#define EXTI_BASE             (APBPERIPH_BASE + 0x00010400)
#define ADC1_BASE             (APBPERIPH_BASE + 0x00012400)
#define TIM1_BASE             (APBPERIPH_BASE + 0x00012C00)
#define SPI1_BASE             (APBPERIPH_BASE + 0x00013000)
#define USART1_BASE           (APBPERIPH_BASE + 0x00013800)
#define DBGMCU_BASE           (APBPERIPH_BASE + 0x00015800)
#define GPIOMUX_BASE          (APBPERIPH_BASE + 0x00017C00)
/*!< AHB peripherals */
 
#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000) /*!< FLASH registers base address */
#define OB_BASE               ((uint32_t)0x1FFFF800U)       /*!< FLASH Option Bytes base address */
#define UID_BASE              ((uint32_t)0x1FFFF838U)       /*!< Unique device ID register base address */
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400)

 /** 
  * @}
  *  */
  /* End of  Peripheral_memory_map */

  
/** @addtogroup Peripheral_declaration
  * @{
  */  
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define ADC1                 ((ADC_TypeDef *) ADC1_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE) 
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOMUX             ((GPIOMUX_TypeDef *)GPIOMUX_BASE)
#define AWU                 ((AWU_TypeDef *)AWU_BASE)
#define BEEP                ((BEEP_TypeDef *)BEEPER_BASE)
 /** 
  * @}
  *  */
  /* End of  Peripheral_declaration */



/* =========================================  End of section using anonymous unions  ========================================= */
#if defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#endif

/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                      Analog to Digital Converter (ADC)                     */
/*                                                                            */
/******************************************************************************/

/* Note: No specific macro feature on this device */

/********************  Bits definition for ADC_ISR register  ******************/
#define ADC_ISR_ADRDY_Pos         (0U)                                         
#define ADC_ISR_ADRDY_Msk         (0x1U << ADC_ISR_ADRDY_Pos)                  /*!< 0x00000001 */
#define ADC_ISR_ADRDY             ADC_ISR_ADRDY_Msk                            /*!< ADC ready flag */
#define ADC_ISR_EOSMP_Pos         (1U)                                         
#define ADC_ISR_EOSMP_Msk         (0x1U << ADC_ISR_EOSMP_Pos)                  /*!< 0x00000002 */
#define ADC_ISR_EOSMP             ADC_ISR_EOSMP_Msk                            /*!< ADC group regular end of sampling flag */
#define ADC_ISR_EOC_Pos           (2U)                                         
#define ADC_ISR_EOC_Msk           (0x1U << ADC_ISR_EOC_Pos)                    /*!< 0x00000004 */
#define ADC_ISR_EOC               ADC_ISR_EOC_Msk                              /*!< ADC group regular end of unitary conversion flag */
#define ADC_ISR_EOS_Pos           (3U)                                         
#define ADC_ISR_EOS_Msk           (0x1U << ADC_ISR_EOS_Pos)                    /*!< 0x00000008 */
#define ADC_ISR_EOS               ADC_ISR_EOS_Msk                              /*!< ADC group regular end of sequence conversions flag */
#define ADC_ISR_OVR_Pos           (4U)                                         
#define ADC_ISR_OVR_Msk           (0x1U << ADC_ISR_OVR_Pos)                    /*!< 0x00000010 */
#define ADC_ISR_OVR               ADC_ISR_OVR_Msk                              /*!< ADC group regular overrun flag */
#define ADC_ISR_AWD1_Pos          (7U)                                         
#define ADC_ISR_AWD1_Msk          (0x1U << ADC_ISR_AWD1_Pos)                   /*!< 0x00000080 */
#define ADC_ISR_AWD1              ADC_ISR_AWD1_Msk                             /*!< ADC analog watchdog 1 flag */

/* Legacy defines */
#define ADC_ISR_AWD             (ADC_ISR_AWD1)
#define ADC_ISR_EOSEQ           (ADC_ISR_EOS)

/********************  Bits definition for ADC_IER register  ******************/
#define ADC_IER_ADRDYIE_Pos       (0U)                                         
#define ADC_IER_ADRDYIE_Msk       (0x1U << ADC_IER_ADRDYIE_Pos)                /*!< 0x00000001 */
#define ADC_IER_ADRDYIE           ADC_IER_ADRDYIE_Msk                          /*!< ADC ready interrupt */
#define ADC_IER_EOSMPIE_Pos       (1U)                                         
#define ADC_IER_EOSMPIE_Msk       (0x1U << ADC_IER_EOSMPIE_Pos)                /*!< 0x00000002 */
#define ADC_IER_EOSMPIE           ADC_IER_EOSMPIE_Msk                          /*!< ADC group regular end of sampling interrupt */
#define ADC_IER_EOCIE_Pos         (2U)                                         
#define ADC_IER_EOCIE_Msk         (0x1U << ADC_IER_EOCIE_Pos)                  /*!< 0x00000004 */
#define ADC_IER_EOCIE             ADC_IER_EOCIE_Msk                            /*!< ADC group regular end of unitary conversion interrupt */
#define ADC_IER_EOSIE_Pos         (3U)                                         
#define ADC_IER_EOSIE_Msk         (0x1U << ADC_IER_EOSIE_Pos)                  /*!< 0x00000008 */
#define ADC_IER_EOSIE             ADC_IER_EOSIE_Msk                            /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_IER_OVRIE_Pos         (4U)                                         
#define ADC_IER_OVRIE_Msk         (0x1U << ADC_IER_OVRIE_Pos)                  /*!< 0x00000010 */
#define ADC_IER_OVRIE             ADC_IER_OVRIE_Msk                            /*!< ADC group regular overrun interrupt */
#define ADC_IER_AWD1IE_Pos        (7U)                                         
#define ADC_IER_AWD1IE_Msk        (0x1U << ADC_IER_AWD1IE_Pos)                 /*!< 0x00000080 */
#define ADC_IER_AWD1IE            ADC_IER_AWD1IE_Msk                           /*!< ADC analog watchdog 1 interrupt */

/* Legacy defines */
#define ADC_IER_AWDIE           (ADC_IER_AWD1IE)
#define ADC_IER_EOSEQIE         (ADC_IER_EOSIE)

/********************  Bits definition for ADC_CR register  *******************/
#define ADC_CR_ADEN_Pos           (0U)                                         
#define ADC_CR_ADEN_Msk           (0x1U << ADC_CR_ADEN_Pos)                    /*!< 0x00000001 */
#define ADC_CR_ADEN               ADC_CR_ADEN_Msk                              /*!< ADC enable */
#define ADC_CR_ADDIS_Pos          (1U)                                         
#define ADC_CR_ADDIS_Msk          (0x1U << ADC_CR_ADDIS_Pos)                   /*!< 0x00000002 */
#define ADC_CR_ADDIS              ADC_CR_ADDIS_Msk                             /*!< ADC disable */
#define ADC_CR_ADSTART_Pos        (2U)                                         
#define ADC_CR_ADSTART_Msk        (0x1U << ADC_CR_ADSTART_Pos)                 /*!< 0x00000004 */
#define ADC_CR_ADSTART            ADC_CR_ADSTART_Msk                           /*!< ADC group regular conversion start */
#define ADC_CR_ADSTP_Pos          (4U)                                         
#define ADC_CR_ADSTP_Msk          (0x1U << ADC_CR_ADSTP_Pos)                   /*!< 0x00000010 */
#define ADC_CR_ADSTP              ADC_CR_ADSTP_Msk                             /*!< ADC group regular conversion stop */
#define ADC_CR_ADCAL_Pos          (31U)                                        
#define ADC_CR_ADCAL_Msk          (0x1U << ADC_CR_ADCAL_Pos)                   /*!< 0x80000000 */
#define ADC_CR_ADCAL              ADC_CR_ADCAL_Msk                             /*!< ADC calibration */

/*******************  Bits definition for ADC_CFGR1 register  *****************/
#define ADC_CFGR1_SCANDIR_Pos     (2U)                                         
#define ADC_CFGR1_SCANDIR_Msk     (0x1U << ADC_CFGR1_SCANDIR_Pos)              /*!< 0x00000004 */
#define ADC_CFGR1_SCANDIR         ADC_CFGR1_SCANDIR_Msk                        /*!< ADC group regular sequencer scan direction */

#define ADC_CFGR1_ALIGN_Pos       (5U)                                         
#define ADC_CFGR1_ALIGN_Msk       (0x1U << ADC_CFGR1_ALIGN_Pos)                /*!< 0x00000020 */
#define ADC_CFGR1_ALIGN           ADC_CFGR1_ALIGN_Msk                          /*!< ADC data alignement */

#define ADC_CFGR1_EXTSEL_Pos      (6U)                                         
#define ADC_CFGR1_EXTSEL_Msk      (0x7U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x000001C0 */
#define ADC_CFGR1_EXTSEL          ADC_CFGR1_EXTSEL_Msk                         /*!< ADC group regular external trigger source */
#define ADC_CFGR1_EXTSEL_0        (0x1U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000040 */
#define ADC_CFGR1_EXTSEL_1        (0x2U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000080 */
#define ADC_CFGR1_EXTSEL_2        (0x4U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000100 */
//#define ADC_CFGR1_EXTSEL_3        (0x4U << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000100 */

#define ADC_CFGR1_EXTEN_Pos       (10U)                                        
#define ADC_CFGR1_EXTEN_Msk       (0x3U << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000C00 */
#define ADC_CFGR1_EXTEN           ADC_CFGR1_EXTEN_Msk                          /*!< ADC group regular external trigger polarity */
#define ADC_CFGR1_EXTEN_0         (0x1U << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000400 */
#define ADC_CFGR1_EXTEN_1         (0x2U << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000800 */

#define ADC_CFGR1_OVRMOD_Pos      (12U)                                        
#define ADC_CFGR1_OVRMOD_Msk      (0x1U << ADC_CFGR1_OVRMOD_Pos)               /*!< 0x00001000 */
#define ADC_CFGR1_OVRMOD          ADC_CFGR1_OVRMOD_Msk                         /*!< ADC group regular overrun configuration */
#define ADC_CFGR1_CONT_Pos        (13U)                                        
#define ADC_CFGR1_CONT_Msk        (0x1U << ADC_CFGR1_CONT_Pos)                 /*!< 0x00002000 */
#define ADC_CFGR1_CONT            ADC_CFGR1_CONT_Msk                           /*!< ADC group regular continuous conversion mode */
#define ADC_CFGR1_WAIT_Pos        (14U)                                        
#define ADC_CFGR1_WAIT_Msk        (0x1U << ADC_CFGR1_WAIT_Pos)                 /*!< 0x00004000 */
#define ADC_CFGR1_WAIT            ADC_CFGR1_WAIT_Msk                           /*!< ADC low power auto wait */
#define ADC_CFGR1_AUTOFF_Pos      (15U)                                        
#define ADC_CFGR1_AUTOFF_Msk      (0x1U << ADC_CFGR1_AUTOFF_Pos)               /*!< 0x00008000 */
#define ADC_CFGR1_AUTOFF          ADC_CFGR1_AUTOFF_Msk                         /*!< ADC low power auto power off */
#define ADC_CFGR1_DISCEN_Pos      (16U)                                        
#define ADC_CFGR1_DISCEN_Msk      (0x1U << ADC_CFGR1_DISCEN_Pos)               /*!< 0x00010000 */
#define ADC_CFGR1_DISCEN          ADC_CFGR1_DISCEN_Msk                         /*!< ADC group regular sequencer discontinuous mode */

#define ADC_CFGR1_AWD1SGL_Pos     (22U)                                        
#define ADC_CFGR1_AWD1SGL_Msk     (0x1U << ADC_CFGR1_AWD1SGL_Pos)              /*!< 0x00400000 */
#define ADC_CFGR1_AWD1SGL         ADC_CFGR1_AWD1SGL_Msk                        /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
#define ADC_CFGR1_AWD1EN_Pos      (23U)                                        
#define ADC_CFGR1_AWD1EN_Msk      (0x1U << ADC_CFGR1_AWD1EN_Pos)               /*!< 0x00800000 */
#define ADC_CFGR1_AWD1EN          ADC_CFGR1_AWD1EN_Msk                         /*!< ADC analog watchdog 1 enable on scope ADC group regular */

#define ADC_CFGR1_AWD1CH_Pos      (26U)                                        
#define ADC_CFGR1_AWD1CH_Msk      (0x7U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x7C000000 */
#define ADC_CFGR1_AWD1CH          ADC_CFGR1_AWD1CH_Msk                         /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CFGR1_AWD1CH_0        (0x01U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x04000000 */
#define ADC_CFGR1_AWD1CH_1        (0x02U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x08000000 */
#define ADC_CFGR1_AWD1CH_2        (0x04U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x10000000 */
#define ADC_CFGR1_AWD1CH_3        (0x08U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x20000000 */
#define ADC_CFGR1_AWD1CH_4        (0x10U << ADC_CFGR1_AWD1CH_Pos)              /*!< 0x40000000 */

/* Legacy defines */
#define ADC_CFGR1_AUTDLY        (ADC_CFGR1_WAIT)
#define ADC_CFGR1_AWDSGL        (ADC_CFGR1_AWD1SGL)
#define ADC_CFGR1_AWDEN         (ADC_CFGR1_AWD1EN)
#define ADC_CFGR1_AWDCH         (ADC_CFGR1_AWD1CH)
#define ADC_CFGR1_AWDCH_0       (ADC_CFGR1_AWD1CH_0)
#define ADC_CFGR1_AWDCH_1       (ADC_CFGR1_AWD1CH_1)
#define ADC_CFGR1_AWDCH_2       (ADC_CFGR1_AWD1CH_2)
#define ADC_CFGR1_AWDCH_3       (ADC_CFGR1_AWD1CH_3)
#define ADC_CFGR1_AWDCH_4       (ADC_CFGR1_AWD1CH_4)

/*******************  Bits definition for ADC_CFGR2 register  *****************/
#define ADC_CFGR2_CKMODE_Pos      (30U)                                        
#define ADC_CFGR2_CKMODE_Msk      (0x3U << ADC_CFGR2_CKMODE_Pos)               /*!< 0xC0000000 */
#define ADC_CFGR2_CKMODE          ADC_CFGR2_CKMODE_Msk                         /*!< ADC clock source and prescaler (prescaler only for clock source synchronous) */
#define ADC_CFGR2_CKMODE_1        (0x2U << ADC_CFGR2_CKMODE_Pos)               /*!< 0x80000000 */
#define ADC_CFGR2_CKMODE_0        (0x1U << ADC_CFGR2_CKMODE_Pos)               /*!< 0x40000000 */

/* Legacy defines */
#define  ADC_CFGR2_JITOFFDIV4   (ADC_CFGR2_CKMODE_1)   /*!< ADC clocked by PCLK div4 */
#define  ADC_CFGR2_JITOFFDIV2   (ADC_CFGR2_CKMODE_0)   /*!< ADC clocked by PCLK div2 */

/******************  Bit definition for ADC_SMPR register  ********************/
#define ADC_SMPR_SMP_Pos          (0U)                                         
#define ADC_SMPR_SMP_Msk          (0x7U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000007 */
#define ADC_SMPR_SMP              ADC_SMPR_SMP_Msk                             /*!< ADC group of channels sampling time 2 */
#define ADC_SMPR_SMP_0            (0x1U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000001 */
#define ADC_SMPR_SMP_1            (0x2U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000002 */
#define ADC_SMPR_SMP_2            (0x4U << ADC_SMPR_SMP_Pos)                   /*!< 0x00000004 */

/* Legacy defines */
#define  ADC_SMPR1_SMPR         (ADC_SMPR_SMP)         /*!< SMP[2:0] bits (Sampling time selection) */
#define  ADC_SMPR1_SMPR_0       (ADC_SMPR_SMP_0)       /*!< bit 0 */
#define  ADC_SMPR1_SMPR_1       (ADC_SMPR_SMP_1)       /*!< bit 1 */
#define  ADC_SMPR1_SMPR_2       (ADC_SMPR_SMP_2)       /*!< bit 2 */

/*******************  Bit definition for ADC_TR register  ********************/
#define ADC_TR1_LT1_Pos           (0U)                                         
#define ADC_TR1_LT1_Msk           (0xFFFU << ADC_TR1_LT1_Pos)                  /*!< 0x00000FFF */
#define ADC_TR1_LT1               ADC_TR1_LT1_Msk                              /*!< ADC analog watchdog 1 threshold low */
#define ADC_TR1_LT1_0             (0x001U << ADC_TR1_LT1_Pos)                  /*!< 0x00000001 */
#define ADC_TR1_LT1_1             (0x002U << ADC_TR1_LT1_Pos)                  /*!< 0x00000002 */
#define ADC_TR1_LT1_2             (0x004U << ADC_TR1_LT1_Pos)                  /*!< 0x00000004 */
#define ADC_TR1_LT1_3             (0x008U << ADC_TR1_LT1_Pos)                  /*!< 0x00000008 */
#define ADC_TR1_LT1_4             (0x010U << ADC_TR1_LT1_Pos)                  /*!< 0x00000010 */
#define ADC_TR1_LT1_5             (0x020U << ADC_TR1_LT1_Pos)                  /*!< 0x00000020 */
#define ADC_TR1_LT1_6             (0x040U << ADC_TR1_LT1_Pos)                  /*!< 0x00000040 */
#define ADC_TR1_LT1_7             (0x080U << ADC_TR1_LT1_Pos)                  /*!< 0x00000080 */
#define ADC_TR1_LT1_8             (0x100U << ADC_TR1_LT1_Pos)                  /*!< 0x00000100 */
#define ADC_TR1_LT1_9             (0x200U << ADC_TR1_LT1_Pos)                  /*!< 0x00000200 */
#define ADC_TR1_LT1_10            (0x400U << ADC_TR1_LT1_Pos)                  /*!< 0x00000400 */
#define ADC_TR1_LT1_11            (0x800U << ADC_TR1_LT1_Pos)                  /*!< 0x00000800 */

#define ADC_TR1_HT1_Pos           (16U)                                        
#define ADC_TR1_HT1_Msk           (0xFFFU << ADC_TR1_HT1_Pos)                  /*!< 0x0FFF0000 */
#define ADC_TR1_HT1               ADC_TR1_HT1_Msk                              /*!< ADC Analog watchdog 1 threshold high */
#define ADC_TR1_HT1_0             (0x001U << ADC_TR1_HT1_Pos)                  /*!< 0x00010000 */
#define ADC_TR1_HT1_1             (0x002U << ADC_TR1_HT1_Pos)                  /*!< 0x00020000 */
#define ADC_TR1_HT1_2             (0x004U << ADC_TR1_HT1_Pos)                  /*!< 0x00040000 */
#define ADC_TR1_HT1_3             (0x008U << ADC_TR1_HT1_Pos)                  /*!< 0x00080000 */
#define ADC_TR1_HT1_4             (0x010U << ADC_TR1_HT1_Pos)                  /*!< 0x00100000 */
#define ADC_TR1_HT1_5             (0x020U << ADC_TR1_HT1_Pos)                  /*!< 0x00200000 */
#define ADC_TR1_HT1_6             (0x040U << ADC_TR1_HT1_Pos)                  /*!< 0x00400000 */
#define ADC_TR1_HT1_7             (0x080U << ADC_TR1_HT1_Pos)                  /*!< 0x00800000 */
#define ADC_TR1_HT1_8             (0x100U << ADC_TR1_HT1_Pos)                  /*!< 0x01000000 */
#define ADC_TR1_HT1_9             (0x200U << ADC_TR1_HT1_Pos)                  /*!< 0x02000000 */
#define ADC_TR1_HT1_10            (0x400U << ADC_TR1_HT1_Pos)                  /*!< 0x04000000 */
#define ADC_TR1_HT1_11            (0x800U << ADC_TR1_HT1_Pos)                  /*!< 0x08000000 */

/* Legacy defines */
#define  ADC_TR_HT              (ADC_TR1_HT1)
#define  ADC_TR_LT              (ADC_TR1_LT1)
#define  ADC_HTR_HT             (ADC_TR1_HT1)
#define  ADC_LTR_LT             (ADC_TR1_LT1)

/******************  Bit definition for ADC_CHSELR register  ******************/
#define ADC_CHSELR_CHSEL_Pos      (0U)                                         
#define ADC_CHSELR_CHSEL_Msk      (0x7FFFFU << ADC_CHSELR_CHSEL_Pos)           /*!< 0x0007FFFF */
#define ADC_CHSELR_CHSEL          ADC_CHSELR_CHSEL_Msk                         /*!< ADC group regular sequencer channels, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL18_Pos    (18U)                                        
#define ADC_CHSELR_CHSEL18_Msk    (0x1U << ADC_CHSELR_CHSEL18_Pos)             /*!< 0x00040000 */
#define ADC_CHSELR_CHSEL18        ADC_CHSELR_CHSEL18_Msk                       /*!< ADC group regular sequencer channel 18, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL17_Pos    (17U)                                        
#define ADC_CHSELR_CHSEL17_Msk    (0x1U << ADC_CHSELR_CHSEL17_Pos)             /*!< 0x00020000 */
#define ADC_CHSELR_CHSEL17        ADC_CHSELR_CHSEL17_Msk                       /*!< ADC group regular sequencer channel 17, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL16_Pos    (16U)                                        
#define ADC_CHSELR_CHSEL16_Msk    (0x1U << ADC_CHSELR_CHSEL16_Pos)             /*!< 0x00010000 */
#define ADC_CHSELR_CHSEL16        ADC_CHSELR_CHSEL16_Msk                       /*!< ADC group regular sequencer channel 16, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL15_Pos    (15U)                                        
#define ADC_CHSELR_CHSEL15_Msk    (0x1U << ADC_CHSELR_CHSEL15_Pos)             /*!< 0x00008000 */
#define ADC_CHSELR_CHSEL15        ADC_CHSELR_CHSEL15_Msk                       /*!< ADC group regular sequencer channel 15, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL14_Pos    (14U)                                        
#define ADC_CHSELR_CHSEL14_Msk    (0x1U << ADC_CHSELR_CHSEL14_Pos)             /*!< 0x00004000 */
#define ADC_CHSELR_CHSEL14        ADC_CHSELR_CHSEL14_Msk                       /*!< ADC group regular sequencer channel 14, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL13_Pos    (13U)                                        
#define ADC_CHSELR_CHSEL13_Msk    (0x1U << ADC_CHSELR_CHSEL13_Pos)             /*!< 0x00002000 */
#define ADC_CHSELR_CHSEL13        ADC_CHSELR_CHSEL13_Msk                       /*!< ADC group regular sequencer channel 13, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL12_Pos    (12U)                                        
#define ADC_CHSELR_CHSEL12_Msk    (0x1U << ADC_CHSELR_CHSEL12_Pos)             /*!< 0x00001000 */
#define ADC_CHSELR_CHSEL12        ADC_CHSELR_CHSEL12_Msk                       /*!< ADC group regular sequencer channel 12, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL11_Pos    (11U)                                        
#define ADC_CHSELR_CHSEL11_Msk    (0x1U << ADC_CHSELR_CHSEL11_Pos)             /*!< 0x00000800 */
#define ADC_CHSELR_CHSEL11        ADC_CHSELR_CHSEL11_Msk                       /*!< ADC group regular sequencer channel 11, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL10_Pos    (10U)                                        
#define ADC_CHSELR_CHSEL10_Msk    (0x1U << ADC_CHSELR_CHSEL10_Pos)             /*!< 0x00000400 */
#define ADC_CHSELR_CHSEL10        ADC_CHSELR_CHSEL10_Msk                       /*!< ADC group regular sequencer channel 10, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL9_Pos     (9U)                                         
#define ADC_CHSELR_CHSEL9_Msk     (0x1U << ADC_CHSELR_CHSEL9_Pos)              /*!< 0x00000200 */
#define ADC_CHSELR_CHSEL9         ADC_CHSELR_CHSEL9_Msk                        /*!< ADC group regular sequencer channel 9, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL8_Pos     (8U)                                         
#define ADC_CHSELR_CHSEL8_Msk     (0x1U << ADC_CHSELR_CHSEL8_Pos)              /*!< 0x00000100 */
#define ADC_CHSELR_CHSEL8         ADC_CHSELR_CHSEL8_Msk                        /*!< ADC group regular sequencer channel 8, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL7_Pos     (7U)                                         
#define ADC_CHSELR_CHSEL7_Msk     (0x1U << ADC_CHSELR_CHSEL7_Pos)              /*!< 0x00000080 */
#define ADC_CHSELR_CHSEL7         ADC_CHSELR_CHSEL7_Msk                        /*!< ADC group regular sequencer channel 7, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL6_Pos     (6U)                                         
#define ADC_CHSELR_CHSEL6_Msk     (0x1U << ADC_CHSELR_CHSEL6_Pos)              /*!< 0x00000040 */
#define ADC_CHSELR_CHSEL6         ADC_CHSELR_CHSEL6_Msk                        /*!< ADC group regular sequencer channel 6, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL5_Pos     (5U)                                         
#define ADC_CHSELR_CHSEL5_Msk     (0x1U << ADC_CHSELR_CHSEL5_Pos)              /*!< 0x00000020 */
#define ADC_CHSELR_CHSEL5         ADC_CHSELR_CHSEL5_Msk                        /*!< ADC group regular sequencer channel 5, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL4_Pos     (4U)                                         
#define ADC_CHSELR_CHSEL4_Msk     (0x1U << ADC_CHSELR_CHSEL4_Pos)              /*!< 0x00000010 */
#define ADC_CHSELR_CHSEL4         ADC_CHSELR_CHSEL4_Msk                        /*!< ADC group regular sequencer channel 4, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL3_Pos     (3U)                                         
#define ADC_CHSELR_CHSEL3_Msk     (0x1U << ADC_CHSELR_CHSEL3_Pos)              /*!< 0x00000008 */
#define ADC_CHSELR_CHSEL3         ADC_CHSELR_CHSEL3_Msk                        /*!< ADC group regular sequencer channel 3, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL2_Pos     (2U)                                         
#define ADC_CHSELR_CHSEL2_Msk     (0x1U << ADC_CHSELR_CHSEL2_Pos)              /*!< 0x00000004 */
#define ADC_CHSELR_CHSEL2         ADC_CHSELR_CHSEL2_Msk                        /*!< ADC group regular sequencer channel 2, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL1_Pos     (1U)                                         
#define ADC_CHSELR_CHSEL1_Msk     (0x1U << ADC_CHSELR_CHSEL1_Pos)              /*!< 0x00000002 */
#define ADC_CHSELR_CHSEL1         ADC_CHSELR_CHSEL1_Msk                        /*!< ADC group regular sequencer channel 1, available when ADC_CFGR1_CHSELRMOD is reset */
#define ADC_CHSELR_CHSEL0_Pos     (0U)                                         
#define ADC_CHSELR_CHSEL0_Msk     (0x1U << ADC_CHSELR_CHSEL0_Pos)              /*!< 0x00000001 */
#define ADC_CHSELR_CHSEL0         ADC_CHSELR_CHSEL0_Msk                        /*!< ADC group regular sequencer channel 0, available when ADC_CFGR1_CHSELRMOD is reset */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos           (0U)                                         
#define ADC_DR_DATA_Msk           (0xFFFFU << ADC_DR_DATA_Pos)                 /*!< 0x0000FFFF */
#define ADC_DR_DATA               ADC_DR_DATA_Msk                              /*!< ADC group regular conversion data */
#define ADC_DR_DATA_0             (0x0001U << ADC_DR_DATA_Pos)                 /*!< 0x00000001 */
#define ADC_DR_DATA_1             (0x0002U << ADC_DR_DATA_Pos)                 /*!< 0x00000002 */
#define ADC_DR_DATA_2             (0x0004U << ADC_DR_DATA_Pos)                 /*!< 0x00000004 */
#define ADC_DR_DATA_3             (0x0008U << ADC_DR_DATA_Pos)                 /*!< 0x00000008 */
#define ADC_DR_DATA_4             (0x0010U << ADC_DR_DATA_Pos)                 /*!< 0x00000010 */
#define ADC_DR_DATA_5             (0x0020U << ADC_DR_DATA_Pos)                 /*!< 0x00000020 */
#define ADC_DR_DATA_6             (0x0040U << ADC_DR_DATA_Pos)                 /*!< 0x00000040 */
#define ADC_DR_DATA_7             (0x0080U << ADC_DR_DATA_Pos)                 /*!< 0x00000080 */
#define ADC_DR_DATA_8             (0x0100U << ADC_DR_DATA_Pos)                 /*!< 0x00000100 */
#define ADC_DR_DATA_9             (0x0200U << ADC_DR_DATA_Pos)                 /*!< 0x00000200 */
#define ADC_DR_DATA_10            (0x0400U << ADC_DR_DATA_Pos)                 /*!< 0x00000400 */
#define ADC_DR_DATA_11            (0x0800U << ADC_DR_DATA_Pos)                 /*!< 0x00000800 */
#define ADC_DR_DATA_12            (0x1000U << ADC_DR_DATA_Pos)                 /*!< 0x00001000 */
#define ADC_DR_DATA_13            (0x2000U << ADC_DR_DATA_Pos)                 /*!< 0x00002000 */
#define ADC_DR_DATA_14            (0x4000U << ADC_DR_DATA_Pos)                 /*!< 0x00004000 */
#define ADC_DR_DATA_15            (0x8000U << ADC_DR_DATA_Pos)                 /*!< 0x00008000 */

/*************************  ADC Common registers  *****************************/
/*******************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_VREFEN_Pos        (22U)                                        
#define ADC_CCR_VREFEN_Msk        (0x1U << ADC_CCR_VREFEN_Pos)                 /*!< 0x00400000 */
#define ADC_CCR_VREFEN            ADC_CCR_VREFEN_Msk                           /*!< ADC internal path to VrefInt enable */

/*************************  ADC CR2 registers  *****************************/
/*******************  Bit definition for ADC_CR2 register  ********************/
#define ADC_CR2_GCMP_Pos           (0)
#define ADC_CR2_GCMP               (0x00000001ul)
#define ADC_CR2_SDIF_Pos           (1)
#define ADC_CR2_SDIF               (0x00000001ul << ADC_CR2_SDIF_Pos)
#define ADC_CR2_WAKE_EN_Pos        (31)
#define ADC_CR2_WAKE_EN            (0x00000001ul << ADC_CR2_WAKE_EN_Pos)

/******************************************************************************/
/*                                                                            */
/*                       CRC calculation unit (CRC)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define CRC_DR_DR_Pos            (0U)                                          
#define CRC_DR_DR_Msk            (0xFFFFFFFFU << CRC_DR_DR_Pos)                /*!< 0xFFFFFFFF */
#define CRC_DR_DR                CRC_DR_DR_Msk                                 /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define CRC_IDR_IDR              ((uint8_t)0xFFU)                              /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define CRC_CR_RESET_Pos         (0U)                                          
#define CRC_CR_RESET_Msk         (0x1U << CRC_CR_RESET_Pos)                    /*!< 0x00000001 */
#define CRC_CR_RESET             CRC_CR_RESET_Msk                              /*!< RESET the CRC computation unit bit */
#define CRC_CR_REV_IN_Pos        (5U)                                          
#define CRC_CR_REV_IN_Msk        (0x3U << CRC_CR_REV_IN_Pos)                   /*!< 0x00000060 */
#define CRC_CR_REV_IN            CRC_CR_REV_IN_Msk                             /*!< REV_IN Reverse Input Data bits */
#define CRC_CR_REV_IN_0          (0x1U << CRC_CR_REV_IN_Pos)                   /*!< 0x00000020 */
#define CRC_CR_REV_IN_1          (0x2U << CRC_CR_REV_IN_Pos)                   /*!< 0x00000040 */
#define CRC_CR_REV_OUT_Pos       (7U)                                          
#define CRC_CR_REV_OUT_Msk       (0x1U << CRC_CR_REV_OUT_Pos)                  /*!< 0x00000080 */
#define CRC_CR_REV_OUT           CRC_CR_REV_OUT_Msk                            /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
#define CRC_INIT_INIT_Pos        (0U)                                          
#define CRC_INIT_INIT_Msk        (0xFFFFFFFFU << CRC_INIT_INIT_Pos)            /*!< 0xFFFFFFFF */
#define CRC_INIT_INIT            CRC_INIT_INIT_Msk                             /*!< Initial CRC value bits */

/******************************************************************************/
/*                                                                            */
/*                           Debug MCU (DBGMCU)                               */
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for DBGMCU_IDCODE register  *****************/
#define DBGMCU_IDCODE_DEV_ID_Pos                     (0U)                      
#define DBGMCU_IDCODE_DEV_ID_Msk                     (0xFFFU << DBGMCU_IDCODE_DEV_ID_Pos) /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID                         DBGMCU_IDCODE_DEV_ID_Msk  /*!< Device Identifier */

#define DBGMCU_IDCODE_REV_ID_Pos                     (16U)                     
#define DBGMCU_IDCODE_REV_ID_Msk                     (0xFFFFU << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID                         DBGMCU_IDCODE_REV_ID_Msk  /*!< REV_ID[15:0] bits (Revision Identifier) */
#define DBGMCU_IDCODE_REV_ID_0                       (0x0001U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00010000 */
#define DBGMCU_IDCODE_REV_ID_1                       (0x0002U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00020000 */
#define DBGMCU_IDCODE_REV_ID_2                       (0x0004U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00040000 */
#define DBGMCU_IDCODE_REV_ID_3                       (0x0008U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00080000 */
#define DBGMCU_IDCODE_REV_ID_4                       (0x0010U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00100000 */
#define DBGMCU_IDCODE_REV_ID_5                       (0x0020U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00200000 */
#define DBGMCU_IDCODE_REV_ID_6                       (0x0040U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00400000 */
#define DBGMCU_IDCODE_REV_ID_7                       (0x0080U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00800000 */
#define DBGMCU_IDCODE_REV_ID_8                       (0x0100U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x01000000 */
#define DBGMCU_IDCODE_REV_ID_9                       (0x0200U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x02000000 */
#define DBGMCU_IDCODE_REV_ID_10                      (0x0400U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x04000000 */
#define DBGMCU_IDCODE_REV_ID_11                      (0x0800U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x08000000 */
#define DBGMCU_IDCODE_REV_ID_12                      (0x1000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x10000000 */
#define DBGMCU_IDCODE_REV_ID_13                      (0x2000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x20000000 */
#define DBGMCU_IDCODE_REV_ID_14                      (0x4000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x40000000 */
#define DBGMCU_IDCODE_REV_ID_15                      (0x8000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x80000000 */

/******************  Bit definition for DBGMCU_CR register  *******************/
#define DBGMCU_CR_DBG_STOP_Pos                       (1U)                      
#define DBGMCU_CR_DBG_STOP_Msk                       (0x1U << DBGMCU_CR_DBG_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP                           DBGMCU_CR_DBG_STOP_Msk    /*!< Debug Stop Mode */ 

/******************  Bit definition for DBGMCU_APB1_FZ register  **************/

#define DBGMCU_APB1_FZ_DBG_TIM1_STOP_Pos             (0U)                      
#define DBGMCU_APB1_FZ_DBG_TIM1_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM1_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB1_FZ_DBG_TIM1_STOP                 DBGMCU_APB1_FZ_DBG_TIM1_STOP_Msk /*!< TIM2 counter stopped when core is halted */

#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos             (1U)                      
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP                 DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk /*!< TIM3 counter stopped when core is halted */
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos             (4U)                      
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos) /*!< 0x00000010 */
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP                 DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk /*!< TIM6 counter stopped when core is halted */
 
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos             (11U)                     
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP                 DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk /*!< Debug Window Watchdog stopped when Core is halted */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos             (12U)                     
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP                 DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk /*!< Debug Independent Watchdog stopped when Core is halted */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos    (21U)                     
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk    (0x1U << DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) /*!< 0x00200000 */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk /*!< I2C1 SMBUS timeout mode stopped when Core is halted */

 

/******************************************************************************/
/*                                                                            */
/*                  Nested Vectored Interrupt Controller                      */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for NVIC_ISER register  *******************/
#define  NVIC_ISER_SETENA                    ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set enable bits */
#define  NVIC_ISER_SETENA_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISER_SETENA_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISER_SETENA_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISER_SETENA_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISER_SETENA_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISER_SETENA_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISER_SETENA_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISER_SETENA_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISER_SETENA_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISER_SETENA_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISER_SETENA_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISER_SETENA_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISER_SETENA_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISER_SETENA_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISER_SETENA_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISER_SETENA_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISER_SETENA_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISER_SETENA_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISER_SETENA_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISER_SETENA_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISER_SETENA_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISER_SETENA_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISER_SETENA_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISER_SETENA_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISER_SETENA_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISER_SETENA_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISER_SETENA_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISER_SETENA_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISER_SETENA_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISER_SETENA_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISER_SETENA_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISER_SETENA_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICER register  *******************/
#define  NVIC_ICER_CLRENA                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-enable bits */
#define  NVIC_ICER_CLRENA_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICER_CLRENA_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICER_CLRENA_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICER_CLRENA_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICER_CLRENA_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICER_CLRENA_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICER_CLRENA_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICER_CLRENA_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICER_CLRENA_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICER_CLRENA_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICER_CLRENA_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICER_CLRENA_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICER_CLRENA_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICER_CLRENA_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICER_CLRENA_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICER_CLRENA_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICER_CLRENA_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICER_CLRENA_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICER_CLRENA_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICER_CLRENA_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICER_CLRENA_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICER_CLRENA_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICER_CLRENA_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICER_CLRENA_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICER_CLRENA_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICER_CLRENA_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICER_CLRENA_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICER_CLRENA_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICER_CLRENA_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICER_CLRENA_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICER_CLRENA_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICER_CLRENA_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ISPR register  *******************/
#define  NVIC_ISPR_SETPEND                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set-pending bits */
#define  NVIC_ISPR_SETPEND_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISPR_SETPEND_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISPR_SETPEND_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISPR_SETPEND_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISPR_SETPEND_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISPR_SETPEND_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISPR_SETPEND_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISPR_SETPEND_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISPR_SETPEND_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISPR_SETPEND_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISPR_SETPEND_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISPR_SETPEND_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISPR_SETPEND_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISPR_SETPEND_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISPR_SETPEND_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISPR_SETPEND_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISPR_SETPEND_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISPR_SETPEND_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISPR_SETPEND_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISPR_SETPEND_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISPR_SETPEND_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISPR_SETPEND_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISPR_SETPEND_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISPR_SETPEND_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISPR_SETPEND_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISPR_SETPEND_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISPR_SETPEND_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISPR_SETPEND_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISPR_SETPEND_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISPR_SETPEND_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISPR_SETPEND_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISPR_SETPEND_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICPR register  *******************/
#define  NVIC_ICPR_CLRPEND                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-pending bits */
#define  NVIC_ICPR_CLRPEND_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICPR_CLRPEND_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICPR_CLRPEND_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICPR_CLRPEND_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICPR_CLRPEND_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICPR_CLRPEND_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICPR_CLRPEND_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICPR_CLRPEND_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICPR_CLRPEND_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICPR_CLRPEND_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICPR_CLRPEND_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICPR_CLRPEND_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICPR_CLRPEND_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICPR_CLRPEND_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICPR_CLRPEND_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICPR_CLRPEND_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICPR_CLRPEND_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICPR_CLRPEND_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICPR_CLRPEND_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICPR_CLRPEND_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICPR_CLRPEND_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICPR_CLRPEND_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICPR_CLRPEND_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICPR_CLRPEND_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICPR_CLRPEND_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICPR_CLRPEND_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICPR_CLRPEND_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICPR_CLRPEND_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICPR_CLRPEND_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICPR_CLRPEND_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICPR_CLRPEND_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICPR_CLRPEND_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_IABR register  *******************/
#define  NVIC_IABR_ACTIVE                    ((uint32_t)0xFFFFFFFF)        /*!< Interrupt active flags */
#define  NVIC_IABR_ACTIVE_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_IABR_ACTIVE_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_IABR_ACTIVE_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_IABR_ACTIVE_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_IABR_ACTIVE_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_IABR_ACTIVE_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_IABR_ACTIVE_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_IABR_ACTIVE_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_IABR_ACTIVE_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_IABR_ACTIVE_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_IABR_ACTIVE_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_IABR_ACTIVE_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_IABR_ACTIVE_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_IABR_ACTIVE_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_IABR_ACTIVE_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_IABR_ACTIVE_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_IABR_ACTIVE_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_IABR_ACTIVE_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_IABR_ACTIVE_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_IABR_ACTIVE_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_IABR_ACTIVE_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_IABR_ACTIVE_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_IABR_ACTIVE_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_IABR_ACTIVE_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_IABR_ACTIVE_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_IABR_ACTIVE_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_IABR_ACTIVE_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_IABR_ACTIVE_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_IABR_ACTIVE_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_IABR_ACTIVE_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_IABR_ACTIVE_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_IABR_ACTIVE_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_PRI0 register  *******************/
#define  NVIC_IPR0_PRI_0                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 0 */
#define  NVIC_IPR0_PRI_1                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 1 */
#define  NVIC_IPR0_PRI_2                     ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 2 */
#define  NVIC_IPR0_PRI_3                     ((uint32_t)0xFF000000)        /*!< Priority of interrupt 3 */

/******************  Bit definition for NVIC_PRI1 register  *******************/
#define  NVIC_IPR1_PRI_4                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 4 */
#define  NVIC_IPR1_PRI_5                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 5 */
#define  NVIC_IPR1_PRI_6                     ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 6 */
#define  NVIC_IPR1_PRI_7                     ((uint32_t)0xFF000000)        /*!< Priority of interrupt 7 */

/******************  Bit definition for NVIC_PRI2 register  *******************/
#define  NVIC_IPR2_PRI_8                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 8 */
#define  NVIC_IPR2_PRI_9                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 9 */
#define  NVIC_IPR2_PRI_10                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 10 */
#define  NVIC_IPR2_PRI_11                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 11 */

/******************  Bit definition for NVIC_PRI3 register  *******************/
#define  NVIC_IPR3_PRI_12                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 12 */
#define  NVIC_IPR3_PRI_13                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 13 */
#define  NVIC_IPR3_PRI_14                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 14 */
#define  NVIC_IPR3_PRI_15                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 15 */

/******************  Bit definition for NVIC_PRI4 register  *******************/
#define  NVIC_IPR4_PRI_16                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 16 */
#define  NVIC_IPR4_PRI_17                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 17 */
#define  NVIC_IPR4_PRI_18                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 18 */
#define  NVIC_IPR4_PRI_19                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 19 */

/******************  Bit definition for NVIC_PRI5 register  *******************/
#define  NVIC_IPR5_PRI_20                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 20 */
#define  NVIC_IPR5_PRI_21                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 21 */
#define  NVIC_IPR5_PRI_22                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 22 */
#define  NVIC_IPR5_PRI_23                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 23 */

/******************  Bit definition for NVIC_PRI6 register  *******************/
#define  NVIC_IPR6_PRI_24                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 24 */
#define  NVIC_IPR6_PRI_25                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 25 */
#define  NVIC_IPR6_PRI_26                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 26 */
#define  NVIC_IPR6_PRI_27                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 27 */

/******************  Bit definition for NVIC_PRI7 register  *******************/
#define  NVIC_IPR7_PRI_28                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 28 */
#define  NVIC_IPR7_PRI_29                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 29 */
#define  NVIC_IPR7_PRI_30                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 30 */
#define  NVIC_IPR7_PRI_31                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 31 */

/******************  Bit definition for SCB_CPUID register  *******************/
#define  SCB_CPUID_REVISION                  ((uint32_t)0x0000000F)        /*!< Implementation defined revision number */
#define  SCB_CPUID_PARTNO                    ((uint32_t)0x0000FFF0)        /*!< Number of processor within family */
#define  SCB_CPUID_Constant                  ((uint32_t)0x000F0000)        /*!< Reads as 0x0F */
#define  SCB_CPUID_VARIANT                   ((uint32_t)0x00F00000)        /*!< Implementation defined variant number */
#define  SCB_CPUID_IMPLEMENTER               ((uint32_t)0xFF000000)        /*!< Implementer code. ARM is 0x41 */

/*******************  Bit definition for SCB_ICSR register  *******************/
#define  SCB_ICSR_VECTACTIVE                 ((uint32_t)0x000001FF)        /*!< Active ISR number field */
#define  SCB_ICSR_RETTOBASE                  ((uint32_t)0x00000800)        /*!< All active exceptions minus the IPSR_current_exception yields the empty set */
#define  SCB_ICSR_VECTPENDING                ((uint32_t)0x003FF000)        /*!< Pending ISR number field */
#define  SCB_ICSR_ISRPENDING                 ((uint32_t)0x00400000)        /*!< Interrupt pending flag */
#define  SCB_ICSR_ISRPREEMPT                 ((uint32_t)0x00800000)        /*!< It indicates that a pending interrupt becomes active in the next running cycle */
#define  SCB_ICSR_PENDSTCLR                  ((uint32_t)0x02000000)        /*!< Clear pending SysTick bit */
#define  SCB_ICSR_PENDSTSET                  ((uint32_t)0x04000000)        /*!< Set pending SysTick bit */
#define  SCB_ICSR_PENDSVCLR                  ((uint32_t)0x08000000)        /*!< Clear pending pendSV bit */
#define  SCB_ICSR_PENDSVSET                  ((uint32_t)0x10000000)        /*!< Set pending pendSV bit */
#define  SCB_ICSR_NMIPENDSET                 ((uint32_t)0x80000000)        /*!< Set pending NMI bit */

/*******************  Bit definition for SCB_VTOR register  *******************/
#define  SCB_VTOR_TBLOFF                     ((uint32_t)0x1FFFFF80)        /*!< Vector table base offset field */
#define  SCB_VTOR_TBLBASE                    ((uint32_t)0x20000000)        /*!< Table base in code(0) or RAM(1) */

/*!<*****************  Bit definition for SCB_AIRCR register  *******************/
#define  SCB_AIRCR_VECTRESET                 ((uint32_t)0x00000001)        /*!< System Reset bit */
#define  SCB_AIRCR_VECTCLRACTIVE             ((uint32_t)0x00000002)        /*!< Clear active vector bit */
#define  SCB_AIRCR_SYSRESETREQ               ((uint32_t)0x00000004)        /*!< Requests chip control logic to generate a reset */

#define  SCB_AIRCR_PRIGROUP                  ((uint32_t)0x00000700)        /*!< PRIGROUP[2:0] bits (Priority group) */
#define  SCB_AIRCR_PRIGROUP_0                ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  SCB_AIRCR_PRIGROUP_1                ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  SCB_AIRCR_PRIGROUP_2                ((uint32_t)0x00000400)        /*!< Bit 2  */

/* prority group configuration */
#define  SCB_AIRCR_PRIGROUP0                 ((uint32_t)0x00000000)        /*!< Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority) */
#define  SCB_AIRCR_PRIGROUP1                 ((uint32_t)0x00000100)        /*!< Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP2                 ((uint32_t)0x00000200)        /*!< Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP3                 ((uint32_t)0x00000300)        /*!< Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP4                 ((uint32_t)0x00000400)        /*!< Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP5                 ((uint32_t)0x00000500)        /*!< Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP6                 ((uint32_t)0x00000600)        /*!< Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP7                 ((uint32_t)0x00000700)        /*!< Priority group=7 (no pre-emption priority, 8 bits of subpriority) */

#define  SCB_AIRCR_ENDIANESS                 ((uint32_t)0x00008000)        /*!< Data endianness bit */
#define  SCB_AIRCR_VECTKEY                   ((uint32_t)0xFFFF0000)        /*!< Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT) */

/*******************  Bit definition for SCB_SCR register  ********************/
#define  SCB_SCR_SLEEPONEXIT                 ((uint8_t)0x02)               /*!< Sleep on exit bit */
#define  SCB_SCR_SLEEPDEEP                   ((uint8_t)0x04)               /*!< Sleep deep bit */
#define  SCB_SCR_SEVONPEND                   ((uint8_t)0x10)               /*!< Wake up from WFE */

/********************  Bit definition for SCB_CCR register  *******************/
#define  SCB_CCR_NONBASETHRDENA              ((uint16_t)0x0001)            /*!< Thread mode can be entered from any level in Handler mode by controlled return value */
#define  SCB_CCR_USERSETMPEND                ((uint16_t)0x0002)            /*!< Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception */
#define  SCB_CCR_UNALIGN_TRP                 ((uint16_t)0x0008)            /*!< Trap for unaligned access */
#define  SCB_CCR_DIV_0_TRP                   ((uint16_t)0x0010)            /*!< Trap on Divide by 0 */
#define  SCB_CCR_BFHFNMIGN                   ((uint16_t)0x0100)            /*!< Handlers running at priority -1 and -2 */
#define  SCB_CCR_STKALIGN                    ((uint16_t)0x0200)            /*!< On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned */

/*******************  Bit definition for SCB_SHPR register ********************/
#define  SCB_SHPR_PRI_N                      ((uint32_t)0x000000FF)        /*!< Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor */
#define  SCB_SHPR_PRI_N1                     ((uint32_t)0x0000FF00)        /*!< Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved */
#define  SCB_SHPR_PRI_N2                     ((uint32_t)0x00FF0000)        /*!< Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV */
#define  SCB_SHPR_PRI_N3                     ((uint32_t)0xFF000000)        /*!< Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick */

/******************  Bit definition for SCB_SHCSR register  *******************/
#define  SCB_SHCSR_MEMFAULTACT               ((uint32_t)0x00000001)        /*!< MemManage is active */
#define  SCB_SHCSR_BUSFAULTACT               ((uint32_t)0x00000002)        /*!< BusFault is active */
#define  SCB_SHCSR_USGFAULTACT               ((uint32_t)0x00000008)        /*!< UsageFault is active */
#define  SCB_SHCSR_SVCALLACT                 ((uint32_t)0x00000080)        /*!< SVCall is active */
#define  SCB_SHCSR_MONITORACT                ((uint32_t)0x00000100)        /*!< Monitor is active */
#define  SCB_SHCSR_PENDSVACT                 ((uint32_t)0x00000400)        /*!< PendSV is active */
#define  SCB_SHCSR_SYSTICKACT                ((uint32_t)0x00000800)        /*!< SysTick is active */
#define  SCB_SHCSR_USGFAULTPENDED            ((uint32_t)0x00001000)        /*!< Usage Fault is pended */
#define  SCB_SHCSR_MEMFAULTPENDED            ((uint32_t)0x00002000)        /*!< MemManage is pended */
#define  SCB_SHCSR_BUSFAULTPENDED            ((uint32_t)0x00004000)        /*!< Bus Fault is pended */
#define  SCB_SHCSR_SVCALLPENDED              ((uint32_t)0x00008000)        /*!< SVCall is pended */
#define  SCB_SHCSR_MEMFAULTENA               ((uint32_t)0x00010000)        /*!< MemManage enable */
#define  SCB_SHCSR_BUSFAULTENA               ((uint32_t)0x00020000)        /*!< Bus Fault enable */
#define  SCB_SHCSR_USGFAULTENA               ((uint32_t)0x00040000)        /*!< UsageFault enable */

/*******************  Bit definition for SCB_CFSR register  *******************/
/*!< MFSR */
#define  SCB_CFSR_IACCVIOL                   ((uint32_t)0x00000001)        /*!< Instruction access violation */
#define  SCB_CFSR_DACCVIOL                   ((uint32_t)0x00000002)        /*!< Data access violation */
#define  SCB_CFSR_MUNSTKERR                  ((uint32_t)0x00000008)        /*!< Unstacking error */
#define  SCB_CFSR_MSTKERR                    ((uint32_t)0x00000010)        /*!< Stacking error */
#define  SCB_CFSR_MMARVALID                  ((uint32_t)0x00000080)        /*!< Memory Manage Address Register address valid flag */
/*!< BFSR */
#define  SCB_CFSR_IBUSERR                    ((uint32_t)0x00000100)        /*!< Instruction bus error flag */
#define  SCB_CFSR_PRECISERR                  ((uint32_t)0x00000200)        /*!< Precise data bus error */
#define  SCB_CFSR_IMPRECISERR                ((uint32_t)0x00000400)        /*!< Imprecise data bus error */
#define  SCB_CFSR_UNSTKERR                   ((uint32_t)0x00000800)        /*!< Unstacking error */
#define  SCB_CFSR_STKERR                     ((uint32_t)0x00001000)        /*!< Stacking error */
#define  SCB_CFSR_BFARVALID                  ((uint32_t)0x00008000)        /*!< Bus Fault Address Register address valid flag */
/*!< UFSR */
#define  SCB_CFSR_UNDEFINSTR                 ((uint32_t)0x00010000)        /*!< The processor attempt to execute an undefined instruction */
#define  SCB_CFSR_INVSTATE                   ((uint32_t)0x00020000)        /*!< Invalid combination of EPSR and instruction */
#define  SCB_CFSR_INVPC                      ((uint32_t)0x00040000)        /*!< Attempt to load EXC_RETURN into pc illegally */
#define  SCB_CFSR_NOCP                       ((uint32_t)0x00080000)        /*!< Attempt to use a coprocessor instruction */
#define  SCB_CFSR_UNALIGNED                  ((uint32_t)0x01000000)        /*!< Fault occurs when there is an attempt to make an unaligned memory access */
#define  SCB_CFSR_DIVBYZERO                  ((uint32_t)0x02000000)        /*!< Fault occurs when SDIV or DIV instruction is used with a divisor of 0 */

/*******************  Bit definition for SCB_HFSR register  *******************/
#define  SCB_HFSR_VECTTBL                    ((uint32_t)0x00000002)        /*!< Fault occurs because of vector table read on exception processing */
#define  SCB_HFSR_FORCED                     ((uint32_t)0x40000000)        /*!< Hard Fault activated when a configurable Fault was received and cannot activate */
#define  SCB_HFSR_DEBUGEVT                   ((uint32_t)0x80000000)        /*!< Fault related to debug */

/*******************  Bit definition for SCB_DFSR register  *******************/
#define  SCB_DFSR_HALTED                     ((uint8_t)0x01)               /*!< Halt request flag */
#define  SCB_DFSR_BKPT                       ((uint8_t)0x02)               /*!< BKPT flag */
#define  SCB_DFSR_DWTTRAP                    ((uint8_t)0x04)               /*!< Data Watchpoint and Trace (DWT) flag */
#define  SCB_DFSR_VCATCH                     ((uint8_t)0x08)               /*!< Vector catch flag */
#define  SCB_DFSR_EXTERNAL                   ((uint8_t)0x10)               /*!< External debug request flag */

/*******************  Bit definition for SCB_MMFAR register  ******************/
#define  SCB_MMFAR_ADDRESS                   ((uint32_t)0xFFFFFFFF)        /*!< Mem Manage fault address field */

/*******************  Bit definition for SCB_BFAR register  *******************/
#define  SCB_BFAR_ADDRESS                    ((uint32_t)0xFFFFFFFF)        /*!< Bus fault address field */

/*******************  Bit definition for SCB_afsr register  *******************/
#define  SCB_AFSR_IMPDEF                     ((uint32_t)0xFFFFFFFF)        /*!< Implementation defined */

/******************************************************************************/
/*                                                                            */
/*                 External Interrupt/Event Controller (EXTI)                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)                                         
#define EXTI_IMR_MR0_Msk          (0x1U << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR0              EXTI_IMR_MR0_Msk                             /*!< Interrupt Mask on line 0  */
#define EXTI_IMR_MR1_Pos          (1U)                                         
#define EXTI_IMR_MR1_Msk          (0x1U << EXTI_IMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_IMR_MR1              EXTI_IMR_MR1_Msk                             /*!< Interrupt Mask on line 1  */
#define EXTI_IMR_MR2_Pos          (2U)                                         
#define EXTI_IMR_MR2_Msk          (0x1U << EXTI_IMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_IMR_MR2              EXTI_IMR_MR2_Msk                             /*!< Interrupt Mask on line 2  */
#define EXTI_IMR_MR3_Pos          (3U)                                         
#define EXTI_IMR_MR3_Msk          (0x1U << EXTI_IMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_IMR_MR3              EXTI_IMR_MR3_Msk                             /*!< Interrupt Mask on line 3  */
#define EXTI_IMR_MR4_Pos          (4U)                                         
#define EXTI_IMR_MR4_Msk          (0x1U << EXTI_IMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_IMR_MR4              EXTI_IMR_MR4_Msk                             /*!< Interrupt Mask on line 4  */
#define EXTI_IMR_MR5_Pos          (5U)                                         
#define EXTI_IMR_MR5_Msk          (0x1U << EXTI_IMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_IMR_MR5              EXTI_IMR_MR5_Msk                             /*!< Interrupt Mask on line 5  */
#define EXTI_IMR_MR6_Pos          (6U)                                         
#define EXTI_IMR_MR6_Msk          (0x1U << EXTI_IMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_IMR_MR6              EXTI_IMR_MR6_Msk                             /*!< Interrupt Mask on line 6  */
#define EXTI_IMR_MR7_Pos          (7U)                                         
#define EXTI_IMR_MR7_Msk          (0x1U << EXTI_IMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_IMR_MR7              EXTI_IMR_MR7_Msk                             /*!< Interrupt Mask on line 7  */
#define EXTI_IMR_MR8_Pos          (8U)                                         
#define EXTI_IMR_MR8_Msk          (0x1U << EXTI_IMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_IMR_MR8              EXTI_IMR_MR8_Msk                             /*!< Interrupt Mask on line 8  */
#define EXTI_IMR_MR9_Pos          (9U)                                         
#define EXTI_IMR_MR9_Msk          (0x1U << EXTI_IMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_IMR_MR9              EXTI_IMR_MR9_Msk                             /*!< Interrupt Mask on line 9  */
#define EXTI_IMR_MR10_Pos         (10U)                                        
#define EXTI_IMR_MR10_Msk         (0x1U << EXTI_IMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_IMR_MR10             EXTI_IMR_MR10_Msk                            /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos         (11U)                                        
#define EXTI_IMR_MR11_Msk         (0x1U << EXTI_IMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_IMR_MR11             EXTI_IMR_MR11_Msk                            /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos         (12U)                                        
#define EXTI_IMR_MR12_Msk         (0x1U << EXTI_IMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_IMR_MR12             EXTI_IMR_MR12_Msk                            /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos         (13U)                                        
#define EXTI_IMR_MR13_Msk         (0x1U << EXTI_IMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_IMR_MR13             EXTI_IMR_MR13_Msk                            /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos         (14U)                                        
#define EXTI_IMR_MR14_Msk         (0x1U << EXTI_IMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_IMR_MR14             EXTI_IMR_MR14_Msk                            /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos         (15U)                                        
#define EXTI_IMR_MR15_Msk         (0x1U << EXTI_IMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_IMR_MR15             EXTI_IMR_MR15_Msk                            /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR17_Pos         (17U)                                        
#define EXTI_IMR_MR17_Msk         (0x1U << EXTI_IMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_IMR_MR17             EXTI_IMR_MR17_Msk                            /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR18_Pos         (18U)                                        
#define EXTI_IMR_MR18_Msk         (0x1U << EXTI_IMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_IMR_MR18             EXTI_IMR_MR18_Msk                            /*!< Interrupt Mask on line 18 */
#define EXTI_IMR_MR19_Pos         (19U)                                        
#define EXTI_IMR_MR19_Msk         (0x1U << EXTI_IMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_IMR_MR19             EXTI_IMR_MR19_Msk                            /*!< Interrupt Mask on line 19 */
#define EXTI_IMR_MR23_Pos         (23U)                                        
#define EXTI_IMR_MR23_Msk         (0x1U << EXTI_IMR_MR23_Pos)                  /*!< 0x00800000 */
#define EXTI_IMR_MR23             EXTI_IMR_MR23_Msk                            /*!< Interrupt Mask on line 23 */

/* References Defines */
#define  EXTI_IMR_IM0 EXTI_IMR_MR0
#define  EXTI_IMR_IM1 EXTI_IMR_MR1
#define  EXTI_IMR_IM2 EXTI_IMR_MR2
#define  EXTI_IMR_IM3 EXTI_IMR_MR3
#define  EXTI_IMR_IM4 EXTI_IMR_MR4
#define  EXTI_IMR_IM5 EXTI_IMR_MR5
#define  EXTI_IMR_IM6 EXTI_IMR_MR6
#define  EXTI_IMR_IM7 EXTI_IMR_MR7
#define  EXTI_IMR_IM8 EXTI_IMR_MR8
#define  EXTI_IMR_IM9 EXTI_IMR_MR9
#define  EXTI_IMR_IM10 EXTI_IMR_MR10
#define  EXTI_IMR_IM11 EXTI_IMR_MR11
#define  EXTI_IMR_IM12 EXTI_IMR_MR12
#define  EXTI_IMR_IM13 EXTI_IMR_MR13
#define  EXTI_IMR_IM14 EXTI_IMR_MR14
#define  EXTI_IMR_IM15 EXTI_IMR_MR15
#define  EXTI_IMR_IM17 EXTI_IMR_MR17
#define  EXTI_IMR_IM18 EXTI_IMR_MR18
#define  EXTI_IMR_IM19 EXTI_IMR_MR19
#define  EXTI_IMR_IM23 EXTI_IMR_MR23

#define EXTI_IMR_IM_Pos           (0U)                                         
#define EXTI_IMR_IM_Msk           (0x8EFFFFU << EXTI_IMR_IM_Pos)               /*!< 0x008EFFFF */
#define EXTI_IMR_IM               EXTI_IMR_IM_Msk                              /*!< Interrupt Mask All */


/******************  Bit definition for EXTI_EMR register  ********************/
#define EXTI_EMR_MR0_Pos          (0U)                                         
#define EXTI_EMR_MR0_Msk          (0x1U << EXTI_EMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_EMR_MR0              EXTI_EMR_MR0_Msk                             /*!< Event Mask on line 0  */
#define EXTI_EMR_MR1_Pos          (1U)                                         
#define EXTI_EMR_MR1_Msk          (0x1U << EXTI_EMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_EMR_MR1              EXTI_EMR_MR1_Msk                             /*!< Event Mask on line 1  */
#define EXTI_EMR_MR2_Pos          (2U)                                         
#define EXTI_EMR_MR2_Msk          (0x1U << EXTI_EMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_EMR_MR2              EXTI_EMR_MR2_Msk                             /*!< Event Mask on line 2  */
#define EXTI_EMR_MR3_Pos          (3U)                                         
#define EXTI_EMR_MR3_Msk          (0x1U << EXTI_EMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_EMR_MR3              EXTI_EMR_MR3_Msk                             /*!< Event Mask on line 3  */
#define EXTI_EMR_MR4_Pos          (4U)                                         
#define EXTI_EMR_MR4_Msk          (0x1U << EXTI_EMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_EMR_MR4              EXTI_EMR_MR4_Msk                             /*!< Event Mask on line 4  */
#define EXTI_EMR_MR5_Pos          (5U)                                         
#define EXTI_EMR_MR5_Msk          (0x1U << EXTI_EMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_EMR_MR5              EXTI_EMR_MR5_Msk                             /*!< Event Mask on line 5  */
#define EXTI_EMR_MR6_Pos          (6U)                                         
#define EXTI_EMR_MR6_Msk          (0x1U << EXTI_EMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_EMR_MR6              EXTI_EMR_MR6_Msk                             /*!< Event Mask on line 6  */
#define EXTI_EMR_MR7_Pos          (7U)                                         
#define EXTI_EMR_MR7_Msk          (0x1U << EXTI_EMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_EMR_MR7              EXTI_EMR_MR7_Msk                             /*!< Event Mask on line 7  */
#define EXTI_EMR_MR8_Pos          (8U)                                         
#define EXTI_EMR_MR8_Msk          (0x1U << EXTI_EMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_EMR_MR8              EXTI_EMR_MR8_Msk                             /*!< Event Mask on line 8  */
#define EXTI_EMR_MR9_Pos          (9U)                                         
#define EXTI_EMR_MR9_Msk          (0x1U << EXTI_EMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_EMR_MR9              EXTI_EMR_MR9_Msk                             /*!< Event Mask on line 9  */
#define EXTI_EMR_MR10_Pos         (10U)                                        
#define EXTI_EMR_MR10_Msk         (0x1U << EXTI_EMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_EMR_MR10             EXTI_EMR_MR10_Msk                            /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos         (11U)                                        
#define EXTI_EMR_MR11_Msk         (0x1U << EXTI_EMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_EMR_MR11             EXTI_EMR_MR11_Msk                            /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos         (12U)                                        
#define EXTI_EMR_MR12_Msk         (0x1U << EXTI_EMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_EMR_MR12             EXTI_EMR_MR12_Msk                            /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos         (13U)                                        
#define EXTI_EMR_MR13_Msk         (0x1U << EXTI_EMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_EMR_MR13             EXTI_EMR_MR13_Msk                            /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos         (14U)                                        
#define EXTI_EMR_MR14_Msk         (0x1U << EXTI_EMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_EMR_MR14             EXTI_EMR_MR14_Msk                            /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos         (15U)                                        
#define EXTI_EMR_MR15_Msk         (0x1U << EXTI_EMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_EMR_MR15             EXTI_EMR_MR15_Msk                            /*!< Event Mask on line 15 */
#define EXTI_EMR_MR17_Pos         (17U)                                        
#define EXTI_EMR_MR17_Msk         (0x1U << EXTI_EMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_EMR_MR17             EXTI_EMR_MR17_Msk                            /*!< Event Mask on line 17 */
#define EXTI_EMR_MR18_Pos         (18U)                                        
#define EXTI_EMR_MR18_Msk         (0x1U << EXTI_EMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_EMR_MR18             EXTI_EMR_MR18_Msk                            /*!< Event Mask on line 18 */
#define EXTI_EMR_MR19_Pos         (19U)                                        
#define EXTI_EMR_MR19_Msk         (0x1U << EXTI_EMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_EMR_MR19             EXTI_EMR_MR19_Msk                            /*!< Event Mask on line 19 */
#define EXTI_EMR_MR23_Pos         (23U)                                        
#define EXTI_EMR_MR23_Msk         (0x1U << EXTI_EMR_MR23_Pos)                  /*!< 0x00800000 */
#define EXTI_EMR_MR23             EXTI_EMR_MR23_Msk                            /*!< Event Mask on line 23 */

/* References Defines */
#define  EXTI_EMR_EM0 EXTI_EMR_MR0
#define  EXTI_EMR_EM1 EXTI_EMR_MR1
#define  EXTI_EMR_EM2 EXTI_EMR_MR2
#define  EXTI_EMR_EM3 EXTI_EMR_MR3
#define  EXTI_EMR_EM4 EXTI_EMR_MR4
#define  EXTI_EMR_EM5 EXTI_EMR_MR5
#define  EXTI_EMR_EM6 EXTI_EMR_MR6
#define  EXTI_EMR_EM7 EXTI_EMR_MR7
#define  EXTI_EMR_EM8 EXTI_EMR_MR8
#define  EXTI_EMR_EM9 EXTI_EMR_MR9
#define  EXTI_EMR_EM10 EXTI_EMR_MR10
#define  EXTI_EMR_EM11 EXTI_EMR_MR11
#define  EXTI_EMR_EM12 EXTI_EMR_MR12
#define  EXTI_EMR_EM13 EXTI_EMR_MR13
#define  EXTI_EMR_EM14 EXTI_EMR_MR14
#define  EXTI_EMR_EM15 EXTI_EMR_MR15
#define  EXTI_EMR_EM17 EXTI_EMR_MR17
#define  EXTI_EMR_EM18 EXTI_EMR_MR18
#define  EXTI_EMR_EM19 EXTI_EMR_MR19
#define  EXTI_EMR_EM23 EXTI_EMR_MR23

/*******************  Bit definition for EXTI_RTSR register  ******************/
#define EXTI_RTSR_TR0_Pos         (0U)                                         
#define EXTI_RTSR_TR0_Msk         (0x1U << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR0             EXTI_RTSR_TR0_Msk                            /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos         (1U)                                         
#define EXTI_RTSR_TR1_Msk         (0x1U << EXTI_RTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_RTSR_TR1             EXTI_RTSR_TR1_Msk                            /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos         (2U)                                         
#define EXTI_RTSR_TR2_Msk         (0x1U << EXTI_RTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_RTSR_TR2             EXTI_RTSR_TR2_Msk                            /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos         (3U)                                         
#define EXTI_RTSR_TR3_Msk         (0x1U << EXTI_RTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_RTSR_TR3             EXTI_RTSR_TR3_Msk                            /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos         (4U)                                         
#define EXTI_RTSR_TR4_Msk         (0x1U << EXTI_RTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_RTSR_TR4             EXTI_RTSR_TR4_Msk                            /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos         (5U)                                         
#define EXTI_RTSR_TR5_Msk         (0x1U << EXTI_RTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_RTSR_TR5             EXTI_RTSR_TR5_Msk                            /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos         (6U)                                         
#define EXTI_RTSR_TR6_Msk         (0x1U << EXTI_RTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_RTSR_TR6             EXTI_RTSR_TR6_Msk                            /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos         (7U)                                         
#define EXTI_RTSR_TR7_Msk         (0x1U << EXTI_RTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_RTSR_TR7             EXTI_RTSR_TR7_Msk                            /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos         (8U)                                         
#define EXTI_RTSR_TR8_Msk         (0x1U << EXTI_RTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_RTSR_TR8             EXTI_RTSR_TR8_Msk                            /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos         (9U)                                         
#define EXTI_RTSR_TR9_Msk         (0x1U << EXTI_RTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_RTSR_TR9             EXTI_RTSR_TR9_Msk                            /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos        (10U)                                        
#define EXTI_RTSR_TR10_Msk        (0x1U << EXTI_RTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_RTSR_TR10            EXTI_RTSR_TR10_Msk                           /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos        (11U)                                        
#define EXTI_RTSR_TR11_Msk        (0x1U << EXTI_RTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_RTSR_TR11            EXTI_RTSR_TR11_Msk                           /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos        (12U)                                        
#define EXTI_RTSR_TR12_Msk        (0x1U << EXTI_RTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_RTSR_TR12            EXTI_RTSR_TR12_Msk                           /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos        (13U)                                        
#define EXTI_RTSR_TR13_Msk        (0x1U << EXTI_RTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_RTSR_TR13            EXTI_RTSR_TR13_Msk                           /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos        (14U)                                        
#define EXTI_RTSR_TR14_Msk        (0x1U << EXTI_RTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_RTSR_TR14            EXTI_RTSR_TR14_Msk                           /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos        (15U)                                        
#define EXTI_RTSR_TR15_Msk        (0x1U << EXTI_RTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_RTSR_TR15            EXTI_RTSR_TR15_Msk                           /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos        (16U)                                        
#define EXTI_RTSR_TR16_Msk        (0x1U << EXTI_RTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_RTSR_TR16            EXTI_RTSR_TR16_Msk                           /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos        (17U)                                        
#define EXTI_RTSR_TR17_Msk        (0x1U << EXTI_RTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_RTSR_TR17            EXTI_RTSR_TR17_Msk                           /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR19_Pos        (19U)                                        
#define EXTI_RTSR_TR19_Msk        (0x1U << EXTI_RTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_RTSR_TR19            EXTI_RTSR_TR19_Msk                           /*!< Rising trigger event configuration bit of line 19 */

/* References Defines */
#define EXTI_RTSR_RT0 EXTI_RTSR_TR0
#define EXTI_RTSR_RT1 EXTI_RTSR_TR1
#define EXTI_RTSR_RT2 EXTI_RTSR_TR2
#define EXTI_RTSR_RT3 EXTI_RTSR_TR3
#define EXTI_RTSR_RT4 EXTI_RTSR_TR4
#define EXTI_RTSR_RT5 EXTI_RTSR_TR5
#define EXTI_RTSR_RT6 EXTI_RTSR_TR6
#define EXTI_RTSR_RT7 EXTI_RTSR_TR7
#define EXTI_RTSR_RT8 EXTI_RTSR_TR8
#define EXTI_RTSR_RT9 EXTI_RTSR_TR9
#define EXTI_RTSR_RT10 EXTI_RTSR_TR10
#define EXTI_RTSR_RT11 EXTI_RTSR_TR11
#define EXTI_RTSR_RT12 EXTI_RTSR_TR12
#define EXTI_RTSR_RT13 EXTI_RTSR_TR13
#define EXTI_RTSR_RT14 EXTI_RTSR_TR14
#define EXTI_RTSR_RT15 EXTI_RTSR_TR15
#define EXTI_RTSR_RT16 EXTI_RTSR_TR16
#define EXTI_RTSR_RT17 EXTI_RTSR_TR17
#define EXTI_RTSR_RT19 EXTI_RTSR_TR19

/*******************  Bit definition for EXTI_FTSR register *******************/
#define EXTI_FTSR_TR0_Pos         (0U)                                         
#define EXTI_FTSR_TR0_Msk         (0x1U << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR0             EXTI_FTSR_TR0_Msk                            /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos         (1U)                                         
#define EXTI_FTSR_TR1_Msk         (0x1U << EXTI_FTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_FTSR_TR1             EXTI_FTSR_TR1_Msk                            /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos         (2U)                                         
#define EXTI_FTSR_TR2_Msk         (0x1U << EXTI_FTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_FTSR_TR2             EXTI_FTSR_TR2_Msk                            /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos         (3U)                                         
#define EXTI_FTSR_TR3_Msk         (0x1U << EXTI_FTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_FTSR_TR3             EXTI_FTSR_TR3_Msk                            /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos         (4U)                                         
#define EXTI_FTSR_TR4_Msk         (0x1U << EXTI_FTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_FTSR_TR4             EXTI_FTSR_TR4_Msk                            /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos         (5U)                                         
#define EXTI_FTSR_TR5_Msk         (0x1U << EXTI_FTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_FTSR_TR5             EXTI_FTSR_TR5_Msk                            /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos         (6U)                                         
#define EXTI_FTSR_TR6_Msk         (0x1U << EXTI_FTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_FTSR_TR6             EXTI_FTSR_TR6_Msk                            /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos         (7U)                                         
#define EXTI_FTSR_TR7_Msk         (0x1U << EXTI_FTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_FTSR_TR7             EXTI_FTSR_TR7_Msk                            /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos         (8U)                                         
#define EXTI_FTSR_TR8_Msk         (0x1U << EXTI_FTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_FTSR_TR8             EXTI_FTSR_TR8_Msk                            /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos         (9U)                                         
#define EXTI_FTSR_TR9_Msk         (0x1U << EXTI_FTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_FTSR_TR9             EXTI_FTSR_TR9_Msk                            /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos        (10U)                                        
#define EXTI_FTSR_TR10_Msk        (0x1U << EXTI_FTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_FTSR_TR10            EXTI_FTSR_TR10_Msk                           /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos        (11U)                                        
#define EXTI_FTSR_TR11_Msk        (0x1U << EXTI_FTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_FTSR_TR11            EXTI_FTSR_TR11_Msk                           /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos        (12U)                                        
#define EXTI_FTSR_TR12_Msk        (0x1U << EXTI_FTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_FTSR_TR12            EXTI_FTSR_TR12_Msk                           /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos        (13U)                                        
#define EXTI_FTSR_TR13_Msk        (0x1U << EXTI_FTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_FTSR_TR13            EXTI_FTSR_TR13_Msk                           /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos        (14U)                                        
#define EXTI_FTSR_TR14_Msk        (0x1U << EXTI_FTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_FTSR_TR14            EXTI_FTSR_TR14_Msk                           /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos        (15U)                                        
#define EXTI_FTSR_TR15_Msk        (0x1U << EXTI_FTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_FTSR_TR15            EXTI_FTSR_TR15_Msk                           /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos        (16U)                                        
#define EXTI_FTSR_TR16_Msk        (0x1U << EXTI_FTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_FTSR_TR16            EXTI_FTSR_TR16_Msk                           /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos        (17U)                                        
#define EXTI_FTSR_TR17_Msk        (0x1U << EXTI_FTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_FTSR_TR17            EXTI_FTSR_TR17_Msk                           /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR19_Pos        (19U)                                        
#define EXTI_FTSR_TR19_Msk        (0x1U << EXTI_FTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_FTSR_TR19            EXTI_FTSR_TR19_Msk                           /*!< Falling trigger event configuration bit of line 19 */

/* References Defines */
#define EXTI_FTSR_FT0 EXTI_FTSR_TR0
#define EXTI_FTSR_FT1 EXTI_FTSR_TR1
#define EXTI_FTSR_FT2 EXTI_FTSR_TR2
#define EXTI_FTSR_FT3 EXTI_FTSR_TR3
#define EXTI_FTSR_FT4 EXTI_FTSR_TR4
#define EXTI_FTSR_FT5 EXTI_FTSR_TR5
#define EXTI_FTSR_FT6 EXTI_FTSR_TR6
#define EXTI_FTSR_FT7 EXTI_FTSR_TR7
#define EXTI_FTSR_FT8 EXTI_FTSR_TR8
#define EXTI_FTSR_FT9 EXTI_FTSR_TR9
#define EXTI_FTSR_FT10 EXTI_FTSR_TR10
#define EXTI_FTSR_FT11 EXTI_FTSR_TR11
#define EXTI_FTSR_FT12 EXTI_FTSR_TR12
#define EXTI_FTSR_FT13 EXTI_FTSR_TR13
#define EXTI_FTSR_FT14 EXTI_FTSR_TR14
#define EXTI_FTSR_FT15 EXTI_FTSR_TR15
#define EXTI_FTSR_FT16 EXTI_FTSR_TR16
#define EXTI_FTSR_FT17 EXTI_FTSR_TR17
#define EXTI_FTSR_FT19 EXTI_FTSR_TR19

/******************* Bit definition for EXTI_SWIER register *******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)                                         
#define EXTI_SWIER_SWIER0_Msk     (0x1U << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0         EXTI_SWIER_SWIER0_Msk                        /*!< Software Interrupt on line 0  */
#define EXTI_SWIER_SWIER1_Pos     (1U)                                         
#define EXTI_SWIER_SWIER1_Msk     (0x1U << EXTI_SWIER_SWIER1_Pos)              /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1         EXTI_SWIER_SWIER1_Msk                        /*!< Software Interrupt on line 1  */
#define EXTI_SWIER_SWIER2_Pos     (2U)                                         
#define EXTI_SWIER_SWIER2_Msk     (0x1U << EXTI_SWIER_SWIER2_Pos)              /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2         EXTI_SWIER_SWIER2_Msk                        /*!< Software Interrupt on line 2  */
#define EXTI_SWIER_SWIER3_Pos     (3U)                                         
#define EXTI_SWIER_SWIER3_Msk     (0x1U << EXTI_SWIER_SWIER3_Pos)              /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3         EXTI_SWIER_SWIER3_Msk                        /*!< Software Interrupt on line 3  */
#define EXTI_SWIER_SWIER4_Pos     (4U)                                         
#define EXTI_SWIER_SWIER4_Msk     (0x1U << EXTI_SWIER_SWIER4_Pos)              /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4         EXTI_SWIER_SWIER4_Msk                        /*!< Software Interrupt on line 4  */
#define EXTI_SWIER_SWIER5_Pos     (5U)                                         
#define EXTI_SWIER_SWIER5_Msk     (0x1U << EXTI_SWIER_SWIER5_Pos)              /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5         EXTI_SWIER_SWIER5_Msk                        /*!< Software Interrupt on line 5  */
#define EXTI_SWIER_SWIER6_Pos     (6U)                                         
#define EXTI_SWIER_SWIER6_Msk     (0x1U << EXTI_SWIER_SWIER6_Pos)              /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6         EXTI_SWIER_SWIER6_Msk                        /*!< Software Interrupt on line 6  */
#define EXTI_SWIER_SWIER7_Pos     (7U)                                         
#define EXTI_SWIER_SWIER7_Msk     (0x1U << EXTI_SWIER_SWIER7_Pos)              /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7         EXTI_SWIER_SWIER7_Msk                        /*!< Software Interrupt on line 7  */
#define EXTI_SWIER_SWIER8_Pos     (8U)                                         
#define EXTI_SWIER_SWIER8_Msk     (0x1U << EXTI_SWIER_SWIER8_Pos)              /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8         EXTI_SWIER_SWIER8_Msk                        /*!< Software Interrupt on line 8  */
#define EXTI_SWIER_SWIER9_Pos     (9U)                                         
#define EXTI_SWIER_SWIER9_Msk     (0x1U << EXTI_SWIER_SWIER9_Pos)              /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9         EXTI_SWIER_SWIER9_Msk                        /*!< Software Interrupt on line 9  */
#define EXTI_SWIER_SWIER10_Pos    (10U)                                        
#define EXTI_SWIER_SWIER10_Msk    (0x1U << EXTI_SWIER_SWIER10_Pos)             /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10        EXTI_SWIER_SWIER10_Msk                       /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos    (11U)                                        
#define EXTI_SWIER_SWIER11_Msk    (0x1U << EXTI_SWIER_SWIER11_Pos)             /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11        EXTI_SWIER_SWIER11_Msk                       /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos    (12U)                                        
#define EXTI_SWIER_SWIER12_Msk    (0x1U << EXTI_SWIER_SWIER12_Pos)             /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12        EXTI_SWIER_SWIER12_Msk                       /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos    (13U)                                        
#define EXTI_SWIER_SWIER13_Msk    (0x1U << EXTI_SWIER_SWIER13_Pos)             /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13        EXTI_SWIER_SWIER13_Msk                       /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos    (14U)                                        
#define EXTI_SWIER_SWIER14_Msk    (0x1U << EXTI_SWIER_SWIER14_Pos)             /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14        EXTI_SWIER_SWIER14_Msk                       /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos    (15U)                                        
#define EXTI_SWIER_SWIER15_Msk    (0x1U << EXTI_SWIER_SWIER15_Pos)             /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15        EXTI_SWIER_SWIER15_Msk                       /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos    (16U)                                        
#define EXTI_SWIER_SWIER16_Msk    (0x1U << EXTI_SWIER_SWIER16_Pos)             /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16        EXTI_SWIER_SWIER16_Msk                       /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos    (17U)                                        
#define EXTI_SWIER_SWIER17_Msk    (0x1U << EXTI_SWIER_SWIER17_Pos)             /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17        EXTI_SWIER_SWIER17_Msk                       /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER19_Pos    (19U)                                        
#define EXTI_SWIER_SWIER19_Msk    (0x1U << EXTI_SWIER_SWIER19_Pos)             /*!< 0x00080000 */
#define EXTI_SWIER_SWIER19        EXTI_SWIER_SWIER19_Msk                       /*!< Software Interrupt on line 19 */

/* References Defines */
#define EXTI_SWIER_SWI0 EXTI_SWIER_SWIER0
#define EXTI_SWIER_SWI1 EXTI_SWIER_SWIER1
#define EXTI_SWIER_SWI2 EXTI_SWIER_SWIER2
#define EXTI_SWIER_SWI3 EXTI_SWIER_SWIER3
#define EXTI_SWIER_SWI4 EXTI_SWIER_SWIER4
#define EXTI_SWIER_SWI5 EXTI_SWIER_SWIER5
#define EXTI_SWIER_SWI6 EXTI_SWIER_SWIER6
#define EXTI_SWIER_SWI7 EXTI_SWIER_SWIER7
#define EXTI_SWIER_SWI8 EXTI_SWIER_SWIER8
#define EXTI_SWIER_SWI9 EXTI_SWIER_SWIER9
#define EXTI_SWIER_SWI10 EXTI_SWIER_SWIER10
#define EXTI_SWIER_SWI11 EXTI_SWIER_SWIER11
#define EXTI_SWIER_SWI12 EXTI_SWIER_SWIER12
#define EXTI_SWIER_SWI13 EXTI_SWIER_SWIER13
#define EXTI_SWIER_SWI14 EXTI_SWIER_SWIER14
#define EXTI_SWIER_SWI15 EXTI_SWIER_SWIER15
#define EXTI_SWIER_SWI16 EXTI_SWIER_SWIER16
#define EXTI_SWIER_SWI17 EXTI_SWIER_SWIER17
#define EXTI_SWIER_SWI19 EXTI_SWIER_SWIER19

/******************  Bit definition for EXTI_PR register  *********************/
#define EXTI_PR_PR0_Pos           (0U)                                         
#define EXTI_PR_PR0_Msk           (0x1U << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR0               EXTI_PR_PR0_Msk                              /*!< Pending bit 0  */
#define EXTI_PR_PR1_Pos           (1U)                                         
#define EXTI_PR_PR1_Msk           (0x1U << EXTI_PR_PR1_Pos)                    /*!< 0x00000002 */
#define EXTI_PR_PR1               EXTI_PR_PR1_Msk                              /*!< Pending bit 1  */
#define EXTI_PR_PR2_Pos           (2U)                                         
#define EXTI_PR_PR2_Msk           (0x1U << EXTI_PR_PR2_Pos)                    /*!< 0x00000004 */
#define EXTI_PR_PR2               EXTI_PR_PR2_Msk                              /*!< Pending bit 2  */
#define EXTI_PR_PR3_Pos           (3U)                                         
#define EXTI_PR_PR3_Msk           (0x1U << EXTI_PR_PR3_Pos)                    /*!< 0x00000008 */
#define EXTI_PR_PR3               EXTI_PR_PR3_Msk                              /*!< Pending bit 3  */
#define EXTI_PR_PR4_Pos           (4U)                                         
#define EXTI_PR_PR4_Msk           (0x1U << EXTI_PR_PR4_Pos)                    /*!< 0x00000010 */
#define EXTI_PR_PR4               EXTI_PR_PR4_Msk                              /*!< Pending bit 4  */
#define EXTI_PR_PR5_Pos           (5U)                                         
#define EXTI_PR_PR5_Msk           (0x1U << EXTI_PR_PR5_Pos)                    /*!< 0x00000020 */
#define EXTI_PR_PR5               EXTI_PR_PR5_Msk                              /*!< Pending bit 5  */
#define EXTI_PR_PR6_Pos           (6U)                                         
#define EXTI_PR_PR6_Msk           (0x1U << EXTI_PR_PR6_Pos)                    /*!< 0x00000040 */
#define EXTI_PR_PR6               EXTI_PR_PR6_Msk                              /*!< Pending bit 6  */
#define EXTI_PR_PR7_Pos           (7U)                                         
#define EXTI_PR_PR7_Msk           (0x1U << EXTI_PR_PR7_Pos)                    /*!< 0x00000080 */
#define EXTI_PR_PR7               EXTI_PR_PR7_Msk                              /*!< Pending bit 7  */
#define EXTI_PR_PR8_Pos           (8U)                                         
#define EXTI_PR_PR8_Msk           (0x1U << EXTI_PR_PR8_Pos)                    /*!< 0x00000100 */
#define EXTI_PR_PR8               EXTI_PR_PR8_Msk                              /*!< Pending bit 8  */
#define EXTI_PR_PR9_Pos           (9U)                                         
#define EXTI_PR_PR9_Msk           (0x1U << EXTI_PR_PR9_Pos)                    /*!< 0x00000200 */
#define EXTI_PR_PR9               EXTI_PR_PR9_Msk                              /*!< Pending bit 9  */
#define EXTI_PR_PR10_Pos          (10U)                                        
#define EXTI_PR_PR10_Msk          (0x1U << EXTI_PR_PR10_Pos)                   /*!< 0x00000400 */
#define EXTI_PR_PR10              EXTI_PR_PR10_Msk                             /*!< Pending bit 10 */
#define EXTI_PR_PR11_Pos          (11U)                                        
#define EXTI_PR_PR11_Msk          (0x1U << EXTI_PR_PR11_Pos)                   /*!< 0x00000800 */
#define EXTI_PR_PR11              EXTI_PR_PR11_Msk                             /*!< Pending bit 11 */
#define EXTI_PR_PR12_Pos          (12U)                                        
#define EXTI_PR_PR12_Msk          (0x1U << EXTI_PR_PR12_Pos)                   /*!< 0x00001000 */
#define EXTI_PR_PR12              EXTI_PR_PR12_Msk                             /*!< Pending bit 12 */
#define EXTI_PR_PR13_Pos          (13U)                                        
#define EXTI_PR_PR13_Msk          (0x1U << EXTI_PR_PR13_Pos)                   /*!< 0x00002000 */
#define EXTI_PR_PR13              EXTI_PR_PR13_Msk                             /*!< Pending bit 13 */
#define EXTI_PR_PR14_Pos          (14U)                                        
#define EXTI_PR_PR14_Msk          (0x1U << EXTI_PR_PR14_Pos)                   /*!< 0x00004000 */
#define EXTI_PR_PR14              EXTI_PR_PR14_Msk                             /*!< Pending bit 14 */
#define EXTI_PR_PR15_Pos          (15U)                                        
#define EXTI_PR_PR15_Msk          (0x1U << EXTI_PR_PR15_Pos)                   /*!< 0x00008000 */
#define EXTI_PR_PR15              EXTI_PR_PR15_Msk                             /*!< Pending bit 15 */
#define EXTI_PR_PR16_Pos          (16U)                                        
#define EXTI_PR_PR16_Msk          (0x1U << EXTI_PR_PR16_Pos)                   /*!< 0x00010000 */
#define EXTI_PR_PR16              EXTI_PR_PR16_Msk                             /*!< Pending bit 16 */
#define EXTI_PR_PR17_Pos          (17U)                                        
#define EXTI_PR_PR17_Msk          (0x1U << EXTI_PR_PR17_Pos)                   /*!< 0x00020000 */
#define EXTI_PR_PR17              EXTI_PR_PR17_Msk                             /*!< Pending bit 17 */
#define EXTI_PR_PR19_Pos          (19U)                                        
#define EXTI_PR_PR19_Msk          (0x1U << EXTI_PR_PR19_Pos)                   /*!< 0x00080000 */
#define EXTI_PR_PR19              EXTI_PR_PR19_Msk                             /*!< Pending bit 19 */

/* References Defines */
#define EXTI_PR_PIF0 EXTI_PR_PR0
#define EXTI_PR_PIF1 EXTI_PR_PR1
#define EXTI_PR_PIF2 EXTI_PR_PR2
#define EXTI_PR_PIF3 EXTI_PR_PR3
#define EXTI_PR_PIF4 EXTI_PR_PR4
#define EXTI_PR_PIF5 EXTI_PR_PR5
#define EXTI_PR_PIF6 EXTI_PR_PR6
#define EXTI_PR_PIF7 EXTI_PR_PR7
#define EXTI_PR_PIF8 EXTI_PR_PR8
#define EXTI_PR_PIF9 EXTI_PR_PR9
#define EXTI_PR_PIF10 EXTI_PR_PR10
#define EXTI_PR_PIF11 EXTI_PR_PR11
#define EXTI_PR_PIF12 EXTI_PR_PR12
#define EXTI_PR_PIF13 EXTI_PR_PR13
#define EXTI_PR_PIF14 EXTI_PR_PR14
#define EXTI_PR_PIF15 EXTI_PR_PR15
#define EXTI_PR_PIF16 EXTI_PR_PR16
#define EXTI_PR_PIF17 EXTI_PR_PR17
#define EXTI_PR_PIF19 EXTI_PR_PR19

/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/



/*******************  Bit definition for FLASH_ACR register  ******************/
#define  FLASH_ACR_LATENCY                   ((uint32_t)0x00000007)        /*!< LATENCY bit (Latency) */



/******************  Bit definition for FLASH_KEYR register  ******************/
#define  FLASH_KEYR_FKEYR                    ((uint32_t)0xFFFFFFFF)        /*!< FPEC Key */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define  FLASH_OPTKEYR_OPTKEYR               ((uint32_t)0xFFFFFFFF)        /*!< Option Byte Key */

/******************  FLASH Keys  **********************************************/
#define FLASH_FKEY1                          ((uint32_t)0x45670123)        /*!< Flash program erase key1 */
#define FLASH_FKEY2                          ((uint32_t)0xCDEF89AB)        /*!< Flash program erase key2: used with FLASH_PEKEY1
                                                                                to unlock the write access to the FPEC. */
                                                               
#define FLASH_OPTKEY1                        ((uint32_t)0x45670123)        /*!< Flash option key1 */
#define FLASH_OPTKEY2                        ((uint32_t)0xCDEF89AB)        /*!< Flash option key2: used with FLASH_OPTKEY1 to
                                                                                unlock the write access to the option byte block */

/******************  Bit definition for FLASH_SR register  *******************/
#define  FLASH_SR_BSY                        ((uint32_t)0x00000001)        /*!< Busy */
#define  FLASH_SR_WRPRTERR                   ((uint32_t)0x00000010)        /*!< Write Protection Error */
#define  FLASH_SR_EOP                        ((uint32_t)0x00000020)        /*!< End of operation */
#define  FLASH_SR_WRPERR                     FLASH_SR_WRPRTERR             /*!< Legacy of Write Protection Error */

/*******************  Bit definition for FLASH_CR register  *******************/
#define  FLASH_CR_PG                         ((uint32_t)0x00000001)        /*!< Programming */
#define  FLASH_CR_PER                        ((uint32_t)0x00000002)        /*!< Page Erase */
#define  FLASH_CR_MER                        ((uint32_t)0x00000004)        /*!< Mass Erase */
#define  FLASH_CR_OPTPG                      ((uint32_t)0x00000010)        /*!< OB half word Programming */
#define  FLASH_CR_OPTER                      ((uint32_t)0x00000020)        /*!< OB byte Erase */
#define  FLASH_CR_STRT                       ((uint32_t)0x00000040)        /*!< Start */
#define  FLASH_CR_LOCK                       ((uint32_t)0x00000080)        /*!< Lock */
#define  FLASH_CR_OPTWRE                     ((uint32_t)0x00000200)        /*!< Option Bytes Write Enable */
#define  FLASH_CR_ERRIE                      ((uint32_t)0x00000400)        /*!< Error Interrupt Enable */
#define  FLASH_CR_EOPIE                      ((uint32_t)0x00001000)        /*!< End of operation interrupt enable */

/*******************  Bit definition for FLASH_AR register  *******************/
#define  FLASH_AR_FAR                        ((uint32_t)0xFFFFFFFF)        /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define  FLASH_OBR_OPTERR                    ((uint32_t)0x00000001)        /*!< Option Byte Error */
#define  FLASH_OBR_RDPRT1                    ((uint32_t)0x00000002)        /*!< Read protection Level bit 1 */
#define  FLASH_OBR_RDPRT2                    ((uint32_t)0x00000004)        /*!< Read protection Level bit 2 */
 
#define  FLASH_OBR_IWDG_SW                   ((uint32_t)0x00000100)        /*!< IWDG SW */
#define  FLASH_OBR_nRST_STOP                 ((uint32_t)0x00000200)        /*!< nRST_STOP */ 
#define  FLASH_OBR_VDDA_MONITOR              ((uint32_t)0x00002000)        /*!< VDDA power supply supervisor */
#define  FLASH_OBR_DATA0                     ((uint32_t)0x00FF0000)        /*!< DATA0 */
#define  FLASH_OBR_DATA1                     ((uint32_t)0xFF000000)        /*!< DATA0 */
 
/* Old OBR_VDDA bit definition, maintained for legacy purpose */
#define FLASH_OBR_VDDA_ANALOG                FLASH_OBR_VDDA_MONITOR

/******************  Bit definition for FLASH_WRPR register  ******************/
#define  FLASH_WRPR_WRP                      ((uint32_t)0xFFFFFFFF)        /*!< Write Protect */

/******************  Bit definition for FLASH_ECR register  ******************/
#define  FLASH_ECR_BPG                      ((uint32_t)0x00000001)        /*!< byte program  */
#define  FLASH_ECR_EEPROM_ER                ((uint32_t)0x00000002)        /*!< EEPROM byte erase  */
#define  FLASH_ECR_EEPROM_BPG               ((uint32_t)0x00000004)        /*!< EEPROM byte program  */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for OB_RDP register  **********************/
#define OB_RDP_RDP_Pos       (0U)                                              
#define OB_RDP_RDP_Msk       (0xFFU << OB_RDP_RDP_Pos)                         /*!< 0x000000FF */
#define OB_RDP_RDP           OB_RDP_RDP_Msk                                    /*!< Read protection option byte */
#define OB_RDP_nRDP_Pos      (8U)                                              
#define OB_RDP_nRDP_Msk      (0xFFU << OB_RDP_nRDP_Pos)                        /*!< 0x0000FF00 */
#define OB_RDP_nRDP          OB_RDP_nRDP_Msk                                   /*!< Read protection complemented option byte */

/******************  Bit definition for OB_USER register  *********************/
#define OB_USER_USER_Pos     (16U)                                             
#define OB_USER_USER_Msk     (0xFFU << OB_USER_USER_Pos)                       /*!< 0x00FF0000 */
#define OB_USER_USER         OB_USER_USER_Msk                                  /*!< User option byte */
#define OB_USER_nUSER_Pos    (24U)                                             
#define OB_USER_nUSER_Msk    (0xFFU << OB_USER_nUSER_Pos)                      /*!< 0xFF000000 */
#define OB_USER_nUSER        OB_USER_nUSER_Msk                                 /*!< User complemented option byte */

/******************  Bit definition for OB_WRP0 register  *********************/
#define OB_WRP0_WRP0_Pos     (0U)                                              
#define OB_WRP0_WRP0_Msk     (0xFFU << OB_WRP0_WRP0_Pos)                       /*!< 0x000000FF */
#define OB_WRP0_WRP0         OB_WRP0_WRP0_Msk                                  /*!< Flash memory write protection option bytes */
#define OB_WRP0_nWRP0_Pos    (8U)                                              
#define OB_WRP0_nWRP0_Msk    (0xFFU << OB_WRP0_nWRP0_Pos)                      /*!< 0x0000FF00 */
#define OB_WRP0_nWRP0        OB_WRP0_nWRP0_Msk                                 /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for OB_WRP1 register  *********************/
#define OB_WRP1_WRP1_Pos     (16U)                                             
#define OB_WRP1_WRP1_Msk     (0xFFU << OB_WRP1_WRP1_Pos)                       /*!< 0x00FF0000 */
#define OB_WRP1_WRP1         OB_WRP1_WRP1_Msk                                  /*!< Flash memory write protection option bytes */
#define OB_WRP1_nWRP1_Pos    (24U)                                             
#define OB_WRP1_nWRP1_Msk    (0xFFU << OB_WRP1_nWRP1_Pos)                      /*!< 0xFF000000 */
#define OB_WRP1_nWRP1        OB_WRP1_nWRP1_Msk                                 /*!< Flash memory write protection complemented option bytes */

/******************************************************************************/
/*                                                                            */
/*                       General Purpose IOs (GPIO)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos           (0U)                                   
#define GPIO_MODER_MODER0_Msk           (0x3U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000003 */
#define GPIO_MODER_MODER0               GPIO_MODER_MODER0_Msk                  
#define GPIO_MODER_MODER0_0             (0x1U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000001 */
#define GPIO_MODER_MODER0_1             (0x2U << GPIO_MODER_MODER0_Pos)        /*!< 0x00000002 */
#define GPIO_MODER_MODER1_Pos           (2U)                                   
#define GPIO_MODER_MODER1_Msk           (0x3U << GPIO_MODER_MODER1_Pos)        /*!< 0x0000000C */
#define GPIO_MODER_MODER1               GPIO_MODER_MODER1_Msk                  
#define GPIO_MODER_MODER1_0             (0x1U << GPIO_MODER_MODER1_Pos)        /*!< 0x00000004 */
#define GPIO_MODER_MODER1_1             (0x2U << GPIO_MODER_MODER1_Pos)        /*!< 0x00000008 */
#define GPIO_MODER_MODER2_Pos           (4U)                                   
#define GPIO_MODER_MODER2_Msk           (0x3U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000030 */
#define GPIO_MODER_MODER2               GPIO_MODER_MODER2_Msk                  
#define GPIO_MODER_MODER2_0             (0x1U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000010 */
#define GPIO_MODER_MODER2_1             (0x2U << GPIO_MODER_MODER2_Pos)        /*!< 0x00000020 */
#define GPIO_MODER_MODER3_Pos           (6U)                                   
#define GPIO_MODER_MODER3_Msk           (0x3U << GPIO_MODER_MODER3_Pos)        /*!< 0x000000C0 */
#define GPIO_MODER_MODER3               GPIO_MODER_MODER3_Msk                  
#define GPIO_MODER_MODER3_0             (0x1U << GPIO_MODER_MODER3_Pos)        /*!< 0x00000040 */
#define GPIO_MODER_MODER3_1             (0x2U << GPIO_MODER_MODER3_Pos)        /*!< 0x00000080 */
#define GPIO_MODER_MODER4_Pos           (8U)                                   
#define GPIO_MODER_MODER4_Msk           (0x3U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000300 */
#define GPIO_MODER_MODER4               GPIO_MODER_MODER4_Msk                  
#define GPIO_MODER_MODER4_0             (0x1U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000100 */
#define GPIO_MODER_MODER4_1             (0x2U << GPIO_MODER_MODER4_Pos)        /*!< 0x00000200 */
#define GPIO_MODER_MODER5_Pos           (10U)                                  
#define GPIO_MODER_MODER5_Msk           (0x3U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000C00 */
#define GPIO_MODER_MODER5               GPIO_MODER_MODER5_Msk                  
#define GPIO_MODER_MODER5_0             (0x1U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000400 */
#define GPIO_MODER_MODER5_1             (0x2U << GPIO_MODER_MODER5_Pos)        /*!< 0x00000800 */
#define GPIO_MODER_MODER6_Pos           (12U)                                  
#define GPIO_MODER_MODER6_Msk           (0x3U << GPIO_MODER_MODER6_Pos)        /*!< 0x00003000 */
#define GPIO_MODER_MODER6               GPIO_MODER_MODER6_Msk                  
#define GPIO_MODER_MODER6_0             (0x1U << GPIO_MODER_MODER6_Pos)        /*!< 0x00001000 */
#define GPIO_MODER_MODER6_1             (0x2U << GPIO_MODER_MODER6_Pos)        /*!< 0x00002000 */
#define GPIO_MODER_MODER7_Pos           (14U)                                  
#define GPIO_MODER_MODER7_Msk           (0x3U << GPIO_MODER_MODER7_Pos)        /*!< 0x0000C000 */
#define GPIO_MODER_MODER7               GPIO_MODER_MODER7_Msk                  
#define GPIO_MODER_MODER7_0             (0x1U << GPIO_MODER_MODER7_Pos)        /*!< 0x00004000 */
#define GPIO_MODER_MODER7_1             (0x2U << GPIO_MODER_MODER7_Pos)        /*!< 0x00008000 */
#define GPIO_MODER_MODER8_Pos           (16U)                                  
#define GPIO_MODER_MODER8_Msk           (0x3U << GPIO_MODER_MODER8_Pos)        /*!< 0x00030000 */
#define GPIO_MODER_MODER8               GPIO_MODER_MODER8_Msk                  
#define GPIO_MODER_MODER8_0             (0x1U << GPIO_MODER_MODER8_Pos)        /*!< 0x00010000 */
#define GPIO_MODER_MODER8_1             (0x2U << GPIO_MODER_MODER8_Pos)        /*!< 0x00020000 */
#define GPIO_MODER_MODER9_Pos           (18U)                                  
#define GPIO_MODER_MODER9_Msk           (0x3U << GPIO_MODER_MODER9_Pos)        /*!< 0x000C0000 */
#define GPIO_MODER_MODER9               GPIO_MODER_MODER9_Msk                  
#define GPIO_MODER_MODER9_0             (0x1U << GPIO_MODER_MODER9_Pos)        /*!< 0x00040000 */
#define GPIO_MODER_MODER9_1             (0x2U << GPIO_MODER_MODER9_Pos)        /*!< 0x00080000 */
#define GPIO_MODER_MODER10_Pos          (20U)                                  
#define GPIO_MODER_MODER10_Msk          (0x3U << GPIO_MODER_MODER10_Pos)       /*!< 0x00300000 */
#define GPIO_MODER_MODER10              GPIO_MODER_MODER10_Msk                 
#define GPIO_MODER_MODER10_0            (0x1U << GPIO_MODER_MODER10_Pos)       /*!< 0x00100000 */
#define GPIO_MODER_MODER10_1            (0x2U << GPIO_MODER_MODER10_Pos)       /*!< 0x00200000 */
#define GPIO_MODER_MODER11_Pos          (22U)                                  
#define GPIO_MODER_MODER11_Msk          (0x3U << GPIO_MODER_MODER11_Pos)       /*!< 0x00C00000 */
#define GPIO_MODER_MODER11              GPIO_MODER_MODER11_Msk                 
#define GPIO_MODER_MODER11_0            (0x1U << GPIO_MODER_MODER11_Pos)       /*!< 0x00400000 */
#define GPIO_MODER_MODER11_1            (0x2U << GPIO_MODER_MODER11_Pos)       /*!< 0x00800000 */
#define GPIO_MODER_MODER12_Pos          (24U)                                  
#define GPIO_MODER_MODER12_Msk          (0x3U << GPIO_MODER_MODER12_Pos)       /*!< 0x03000000 */
#define GPIO_MODER_MODER12              GPIO_MODER_MODER12_Msk                 
#define GPIO_MODER_MODER12_0            (0x1U << GPIO_MODER_MODER12_Pos)       /*!< 0x01000000 */
#define GPIO_MODER_MODER12_1            (0x2U << GPIO_MODER_MODER12_Pos)       /*!< 0x02000000 */
#define GPIO_MODER_MODER13_Pos          (26U)                                  
#define GPIO_MODER_MODER13_Msk          (0x3U << GPIO_MODER_MODER13_Pos)       /*!< 0x0C000000 */
#define GPIO_MODER_MODER13              GPIO_MODER_MODER13_Msk                 
#define GPIO_MODER_MODER13_0            (0x1U << GPIO_MODER_MODER13_Pos)       /*!< 0x04000000 */
#define GPIO_MODER_MODER13_1            (0x2U << GPIO_MODER_MODER13_Pos)       /*!< 0x08000000 */
#define GPIO_MODER_MODER14_Pos          (28U)                                  
#define GPIO_MODER_MODER14_Msk          (0x3U << GPIO_MODER_MODER14_Pos)       /*!< 0x30000000 */
#define GPIO_MODER_MODER14              GPIO_MODER_MODER14_Msk                 
#define GPIO_MODER_MODER14_0            (0x1U << GPIO_MODER_MODER14_Pos)       /*!< 0x10000000 */
#define GPIO_MODER_MODER14_1            (0x2U << GPIO_MODER_MODER14_Pos)       /*!< 0x20000000 */
#define GPIO_MODER_MODER15_Pos          (30U)                                  
#define GPIO_MODER_MODER15_Msk          (0x3U << GPIO_MODER_MODER15_Pos)       /*!< 0xC0000000 */
#define GPIO_MODER_MODER15              GPIO_MODER_MODER15_Msk                 
#define GPIO_MODER_MODER15_0            (0x1U << GPIO_MODER_MODER15_Pos)       /*!< 0x40000000 */
#define GPIO_MODER_MODER15_1            (0x2U << GPIO_MODER_MODER15_Pos)       /*!< 0x80000000 */

/******************  Bit definition for GPIO_OTYPER register  *****************/
#define GPIO_OTYPER_OT_0                (0x00000001U)                          
#define GPIO_OTYPER_OT_1                (0x00000002U)                          
#define GPIO_OTYPER_OT_2                (0x00000004U)                          
#define GPIO_OTYPER_OT_3                (0x00000008U)                          
#define GPIO_OTYPER_OT_4                (0x00000010U)                          
#define GPIO_OTYPER_OT_5                (0x00000020U)                          
#define GPIO_OTYPER_OT_6                (0x00000040U)                          
#define GPIO_OTYPER_OT_7                (0x00000080U)                          
#define GPIO_OTYPER_OT_8                (0x00000100U)                          
#define GPIO_OTYPER_OT_9                (0x00000200U)                          
#define GPIO_OTYPER_OT_10               (0x00000400U)                          
#define GPIO_OTYPER_OT_11               (0x00000800U)                          
#define GPIO_OTYPER_OT_12               (0x00001000U)                          
#define GPIO_OTYPER_OT_13               (0x00002000U)                          
#define GPIO_OTYPER_OT_14               (0x00004000U)                          
#define GPIO_OTYPER_OT_15               (0x00008000U)                          

/****************  Bit definition for GPIO_OSPEEDR register  ******************/
#define GPIO_OSPEEDR_OSPEEDR0_Pos       (0U)                                   
#define GPIO_OSPEEDR_OSPEEDR0_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEEDR0           GPIO_OSPEEDR_OSPEEDR0_Msk              
#define GPIO_OSPEEDR_OSPEEDR0_0         (0x1U << GPIO_OSPEEDR_OSPEEDR0_Pos)    /*!< 0x00000001 */

#define GPIO_OSPEEDR_OSPEEDR1_Pos       (2U)                                   
#define GPIO_OSPEEDR_OSPEEDR1_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR1_Pos)    /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEEDR1           GPIO_OSPEEDR_OSPEEDR1_Msk              
#define GPIO_OSPEEDR_OSPEEDR1_0         (0x1U << GPIO_OSPEEDR_OSPEEDR1_Pos)    /*!< 0x00000004 */

#define GPIO_OSPEEDR_OSPEEDR2_Pos       (4U)                                   
#define GPIO_OSPEEDR_OSPEEDR2_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR2_Pos)    /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEEDR2           GPIO_OSPEEDR_OSPEEDR2_Msk              
#define GPIO_OSPEEDR_OSPEEDR2_0         (0x1U << GPIO_OSPEEDR_OSPEEDR2_Pos)    /*!< 0x00000010 */

#define GPIO_OSPEEDR_OSPEEDR3_Pos       (6U)                                   
#define GPIO_OSPEEDR_OSPEEDR3_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR3_Pos)    /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEEDR3           GPIO_OSPEEDR_OSPEEDR3_Msk              
#define GPIO_OSPEEDR_OSPEEDR3_0         (0x1U << GPIO_OSPEEDR_OSPEEDR3_Pos)    /*!< 0x00000040 */

#define GPIO_OSPEEDR_OSPEEDR4_Pos       (8U)                                   
#define GPIO_OSPEEDR_OSPEEDR4_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR4_Pos)    /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEEDR4           GPIO_OSPEEDR_OSPEEDR4_Msk              
#define GPIO_OSPEEDR_OSPEEDR4_0         (0x1U << GPIO_OSPEEDR_OSPEEDR4_Pos)    /*!< 0x00000100 */

#define GPIO_OSPEEDR_OSPEEDR5_Pos       (10U)                                  
#define GPIO_OSPEEDR_OSPEEDR5_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR5_Pos)    /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEEDR5           GPIO_OSPEEDR_OSPEEDR5_Msk              
#define GPIO_OSPEEDR_OSPEEDR5_0         (0x1U << GPIO_OSPEEDR_OSPEEDR5_Pos)    /*!< 0x00000400 */

#define GPIO_OSPEEDR_OSPEEDR6_Pos       (12U)                                  
#define GPIO_OSPEEDR_OSPEEDR6_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR6_Pos)    /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEEDR6           GPIO_OSPEEDR_OSPEEDR6_Msk              
#define GPIO_OSPEEDR_OSPEEDR6_0         (0x1U << GPIO_OSPEEDR_OSPEEDR6_Pos)    /*!< 0x00001000 */

#define GPIO_OSPEEDR_OSPEEDR7_Pos       (14U)                                  
#define GPIO_OSPEEDR_OSPEEDR7_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR7_Pos)    /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEEDR7           GPIO_OSPEEDR_OSPEEDR7_Msk              
#define GPIO_OSPEEDR_OSPEEDR7_0         (0x1U << GPIO_OSPEEDR_OSPEEDR7_Pos)    /*!< 0x00004000 */

#define GPIO_OSPEEDR_OSPEEDR8_Pos       (16U)                                  
#define GPIO_OSPEEDR_OSPEEDR8_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR8_Pos)    /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEEDR8           GPIO_OSPEEDR_OSPEEDR8_Msk              
#define GPIO_OSPEEDR_OSPEEDR8_0         (0x1U << GPIO_OSPEEDR_OSPEEDR8_Pos)    /*!< 0x00010000 */

#define GPIO_OSPEEDR_OSPEEDR9_Pos       (18U)                                  
#define GPIO_OSPEEDR_OSPEEDR9_Msk       (0x3U << GPIO_OSPEEDR_OSPEEDR9_Pos)    /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEEDR9           GPIO_OSPEEDR_OSPEEDR9_Msk              
#define GPIO_OSPEEDR_OSPEEDR9_0         (0x1U << GPIO_OSPEEDR_OSPEEDR9_Pos)    /*!< 0x00040000 */

#define GPIO_OSPEEDR_OSPEEDR10_Pos      (20U)                                  
#define GPIO_OSPEEDR_OSPEEDR10_Msk      (0x3U << GPIO_OSPEEDR_OSPEEDR10_Pos)   /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEEDR10          GPIO_OSPEEDR_OSPEEDR10_Msk             
#define GPIO_OSPEEDR_OSPEEDR10_0        (0x1U << GPIO_OSPEEDR_OSPEEDR10_Pos)   /*!< 0x00100000 */

#define GPIO_OSPEEDR_OSPEEDR11_Pos      (22U)                                  
#define GPIO_OSPEEDR_OSPEEDR11_Msk      (0x3U << GPIO_OSPEEDR_OSPEEDR11_Pos)   /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEEDR11          GPIO_OSPEEDR_OSPEEDR11_Msk             
#define GPIO_OSPEEDR_OSPEEDR11_0        (0x1U << GPIO_OSPEEDR_OSPEEDR11_Pos)   /*!< 0x00400000 */

#define GPIO_OSPEEDR_OSPEEDR12_Pos      (24U)                                  
#define GPIO_OSPEEDR_OSPEEDR12_Msk      (0x3U << GPIO_OSPEEDR_OSPEEDR12_Pos)   /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEEDR12          GPIO_OSPEEDR_OSPEEDR12_Msk             
#define GPIO_OSPEEDR_OSPEEDR12_0        (0x1U << GPIO_OSPEEDR_OSPEEDR12_Pos)   /*!< 0x01000000 */

#define GPIO_OSPEEDR_OSPEEDR13_Pos      (26U)                                  
#define GPIO_OSPEEDR_OSPEEDR13_Msk      (0x3U << GPIO_OSPEEDR_OSPEEDR13_Pos)   /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEEDR13          GPIO_OSPEEDR_OSPEEDR13_Msk             
#define GPIO_OSPEEDR_OSPEEDR13_0        (0x1U << GPIO_OSPEEDR_OSPEEDR13_Pos)   /*!< 0x04000000 */

#define GPIO_OSPEEDR_OSPEEDR14_Pos      (28U)                                  
#define GPIO_OSPEEDR_OSPEEDR14_Msk      (0x3U << GPIO_OSPEEDR_OSPEEDR14_Pos)   /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEEDR14          GPIO_OSPEEDR_OSPEEDR14_Msk             
#define GPIO_OSPEEDR_OSPEEDR14_0        (0x1U << GPIO_OSPEEDR_OSPEEDR14_Pos)   /*!< 0x10000000 */

#define GPIO_OSPEEDR_OSPEEDR15_Pos      (30U)                                  
#define GPIO_OSPEEDR_OSPEEDR15_Msk      (0x3U << GPIO_OSPEEDR_OSPEEDR15_Pos)   /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEEDR15          GPIO_OSPEEDR_OSPEEDR15_Msk             
#define GPIO_OSPEEDR_OSPEEDR15_0        (0x1U << GPIO_OSPEEDR_OSPEEDR15_Pos)   /*!< 0x40000000 */


/*******************  Bit definition for GPIO_PUPDR register ******************/
#define GPIO_PUPDR_PUPDR0_Pos           (0U)                                   
#define GPIO_PUPDR_PUPDR0_Msk           (0x3U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPDR0               GPIO_PUPDR_PUPDR0_Msk                  
#define GPIO_PUPDR_PUPDR0_0             (0x1U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000001 */
#define GPIO_PUPDR_PUPDR0_1             (0x2U << GPIO_PUPDR_PUPDR0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPDR1_Pos           (2U)                                   
#define GPIO_PUPDR_PUPDR1_Msk           (0x3U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x0000000C */
#define GPIO_PUPDR_PUPDR1               GPIO_PUPDR_PUPDR1_Msk                  
#define GPIO_PUPDR_PUPDR1_0             (0x1U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000004 */
#define GPIO_PUPDR_PUPDR1_1             (0x2U << GPIO_PUPDR_PUPDR1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPDR2_Pos           (4U)                                   
#define GPIO_PUPDR_PUPDR2_Msk           (0x3U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000030 */
#define GPIO_PUPDR_PUPDR2               GPIO_PUPDR_PUPDR2_Msk                  
#define GPIO_PUPDR_PUPDR2_0             (0x1U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000010 */
#define GPIO_PUPDR_PUPDR2_1             (0x2U << GPIO_PUPDR_PUPDR2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPDR3_Pos           (6U)                                   
#define GPIO_PUPDR_PUPDR3_Msk           (0x3U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPDR3               GPIO_PUPDR_PUPDR3_Msk                  
#define GPIO_PUPDR_PUPDR3_0             (0x1U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000040 */
#define GPIO_PUPDR_PUPDR3_1             (0x2U << GPIO_PUPDR_PUPDR3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPDR4_Pos           (8U)                                   
#define GPIO_PUPDR_PUPDR4_Msk           (0x3U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000300 */
#define GPIO_PUPDR_PUPDR4               GPIO_PUPDR_PUPDR4_Msk                  
#define GPIO_PUPDR_PUPDR4_0             (0x1U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000100 */
#define GPIO_PUPDR_PUPDR4_1             (0x2U << GPIO_PUPDR_PUPDR4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPDR5_Pos           (10U)                                  
#define GPIO_PUPDR_PUPDR5_Msk           (0x3U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPDR5               GPIO_PUPDR_PUPDR5_Msk                  
#define GPIO_PUPDR_PUPDR5_0             (0x1U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000400 */
#define GPIO_PUPDR_PUPDR5_1             (0x2U << GPIO_PUPDR_PUPDR5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPDR6_Pos           (12U)                                  
#define GPIO_PUPDR_PUPDR6_Msk           (0x3U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00003000 */
#define GPIO_PUPDR_PUPDR6               GPIO_PUPDR_PUPDR6_Msk                  
#define GPIO_PUPDR_PUPDR6_0             (0x1U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00001000 */
#define GPIO_PUPDR_PUPDR6_1             (0x2U << GPIO_PUPDR_PUPDR6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPDR7_Pos           (14U)                                  
#define GPIO_PUPDR_PUPDR7_Msk           (0x3U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPDR7               GPIO_PUPDR_PUPDR7_Msk                  
#define GPIO_PUPDR_PUPDR7_0             (0x1U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00004000 */
#define GPIO_PUPDR_PUPDR7_1             (0x2U << GPIO_PUPDR_PUPDR7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPDR8_Pos           (16U)                                  
#define GPIO_PUPDR_PUPDR8_Msk           (0x3U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00030000 */
#define GPIO_PUPDR_PUPDR8               GPIO_PUPDR_PUPDR8_Msk                  
#define GPIO_PUPDR_PUPDR8_0             (0x1U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00010000 */
#define GPIO_PUPDR_PUPDR8_1             (0x2U << GPIO_PUPDR_PUPDR8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPDR9_Pos           (18U)                                  
#define GPIO_PUPDR_PUPDR9_Msk           (0x3U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPDR9               GPIO_PUPDR_PUPDR9_Msk                  
#define GPIO_PUPDR_PUPDR9_0             (0x1U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00040000 */
#define GPIO_PUPDR_PUPDR9_1             (0x2U << GPIO_PUPDR_PUPDR9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPDR10_Pos          (20U)                                  
#define GPIO_PUPDR_PUPDR10_Msk          (0x3U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00300000 */
#define GPIO_PUPDR_PUPDR10              GPIO_PUPDR_PUPDR10_Msk                 
#define GPIO_PUPDR_PUPDR10_0            (0x1U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00100000 */
#define GPIO_PUPDR_PUPDR10_1            (0x2U << GPIO_PUPDR_PUPDR10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPDR11_Pos          (22U)                                  
#define GPIO_PUPDR_PUPDR11_Msk          (0x3U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPDR11              GPIO_PUPDR_PUPDR11_Msk                 
#define GPIO_PUPDR_PUPDR11_0            (0x1U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00400000 */
#define GPIO_PUPDR_PUPDR11_1            (0x2U << GPIO_PUPDR_PUPDR11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPDR12_Pos          (24U)                                  
#define GPIO_PUPDR_PUPDR12_Msk          (0x3U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x03000000 */
#define GPIO_PUPDR_PUPDR12              GPIO_PUPDR_PUPDR12_Msk                 
#define GPIO_PUPDR_PUPDR12_0            (0x1U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x01000000 */
#define GPIO_PUPDR_PUPDR12_1            (0x2U << GPIO_PUPDR_PUPDR12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPDR13_Pos          (26U)                                  
#define GPIO_PUPDR_PUPDR13_Msk          (0x3U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPDR13              GPIO_PUPDR_PUPDR13_Msk                 
#define GPIO_PUPDR_PUPDR13_0            (0x1U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x04000000 */
#define GPIO_PUPDR_PUPDR13_1            (0x2U << GPIO_PUPDR_PUPDR13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPDR14_Pos          (28U)                                  
#define GPIO_PUPDR_PUPDR14_Msk          (0x3U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x30000000 */
#define GPIO_PUPDR_PUPDR14              GPIO_PUPDR_PUPDR14_Msk                 
#define GPIO_PUPDR_PUPDR14_0            (0x1U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x10000000 */
#define GPIO_PUPDR_PUPDR14_1            (0x2U << GPIO_PUPDR_PUPDR14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPDR15_Pos          (30U)                                  
#define GPIO_PUPDR_PUPDR15_Msk          (0x3U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPDR15              GPIO_PUPDR_PUPDR15_Msk                 
#define GPIO_PUPDR_PUPDR15_0            (0x1U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x40000000 */
#define GPIO_PUPDR_PUPDR15_1            (0x2U << GPIO_PUPDR_PUPDR15_Pos)       /*!< 0x80000000 */

/*******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_0                      (0x00000001U)                          
#define GPIO_IDR_1                      (0x00000002U)                          
#define GPIO_IDR_2                      (0x00000004U)                          
#define GPIO_IDR_3                      (0x00000008U)                          
#define GPIO_IDR_4                      (0x00000010U)                          
#define GPIO_IDR_5                      (0x00000020U)                          
#define GPIO_IDR_6                      (0x00000040U)                          
#define GPIO_IDR_7                      (0x00000080U)                          
#define GPIO_IDR_8                      (0x00000100U)                          
#define GPIO_IDR_9                      (0x00000200U)                          
#define GPIO_IDR_10                     (0x00000400U)                          
#define GPIO_IDR_11                     (0x00000800U)                          
#define GPIO_IDR_12                     (0x00001000U)                          
#define GPIO_IDR_13                     (0x00002000U)                          
#define GPIO_IDR_14                     (0x00004000U)                          
#define GPIO_IDR_15                     (0x00008000U)                          

/******************  Bit definition for GPIO_ODR register  ********************/
#define GPIO_ODR_0                      (0x00000001U)                          
#define GPIO_ODR_1                      (0x00000002U)                          
#define GPIO_ODR_2                      (0x00000004U)                          
#define GPIO_ODR_3                      (0x00000008U)                          
#define GPIO_ODR_4                      (0x00000010U)                          
#define GPIO_ODR_5                      (0x00000020U)                          
#define GPIO_ODR_6                      (0x00000040U)                          
#define GPIO_ODR_7                      (0x00000080U)                          
#define GPIO_ODR_8                      (0x00000100U)                          
#define GPIO_ODR_9                      (0x00000200U)                          
#define GPIO_ODR_10                     (0x00000400U)                          
#define GPIO_ODR_11                     (0x00000800U)                          
#define GPIO_ODR_12                     (0x00001000U)                          
#define GPIO_ODR_13                     (0x00002000U)                          
#define GPIO_ODR_14                     (0x00004000U)                          
#define GPIO_ODR_15                     (0x00008000U)                          

/****************** Bit definition for GPIO_BSRR register  ********************/
#define GPIO_BSRR_BS_0                  (0x00000001U)                          
#define GPIO_BSRR_BS_1                  (0x00000002U)                          
#define GPIO_BSRR_BS_2                  (0x00000004U)                          
#define GPIO_BSRR_BS_3                  (0x00000008U)                          
#define GPIO_BSRR_BS_4                  (0x00000010U)                          
#define GPIO_BSRR_BS_5                  (0x00000020U)                          
#define GPIO_BSRR_BS_6                  (0x00000040U)                          
#define GPIO_BSRR_BS_7                  (0x00000080U)                          
#define GPIO_BSRR_BS_8                  (0x00000100U)                          
#define GPIO_BSRR_BS_9                  (0x00000200U)                          
#define GPIO_BSRR_BS_10                 (0x00000400U)                          
#define GPIO_BSRR_BS_11                 (0x00000800U)                          
#define GPIO_BSRR_BS_12                 (0x00001000U)                          
#define GPIO_BSRR_BS_13                 (0x00002000U)                          
#define GPIO_BSRR_BS_14                 (0x00004000U)                          
#define GPIO_BSRR_BS_15                 (0x00008000U)                          
#define GPIO_BSRR_BR_0                  (0x00010000U)                          
#define GPIO_BSRR_BR_1                  (0x00020000U)                          
#define GPIO_BSRR_BR_2                  (0x00040000U)                          
#define GPIO_BSRR_BR_3                  (0x00080000U)                          
#define GPIO_BSRR_BR_4                  (0x00100000U)                          
#define GPIO_BSRR_BR_5                  (0x00200000U)                          
#define GPIO_BSRR_BR_6                  (0x00400000U)                          
#define GPIO_BSRR_BR_7                  (0x00800000U)                          
#define GPIO_BSRR_BR_8                  (0x01000000U)                          
#define GPIO_BSRR_BR_9                  (0x02000000U)                          
#define GPIO_BSRR_BR_10                 (0x04000000U)                          
#define GPIO_BSRR_BR_11                 (0x08000000U)                          
#define GPIO_BSRR_BR_12                 (0x10000000U)                          
#define GPIO_BSRR_BR_13                 (0x20000000U)                          
#define GPIO_BSRR_BR_14                 (0x40000000U)                          
#define GPIO_BSRR_BR_15                 (0x80000000U)                          

/****************** Bit definition for GPIO_LCKR register  ********************/
#define GPIO_LCKR_LCK0_Pos              (0U)                                   
#define GPIO_LCKR_LCK0_Msk              (0x1U << GPIO_LCKR_LCK0_Pos)           /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                  GPIO_LCKR_LCK0_Msk                     
#define GPIO_LCKR_LCK1_Pos              (1U)                                   
#define GPIO_LCKR_LCK1_Msk              (0x1U << GPIO_LCKR_LCK1_Pos)           /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                  GPIO_LCKR_LCK1_Msk                     
#define GPIO_LCKR_LCK2_Pos              (2U)                                   
#define GPIO_LCKR_LCK2_Msk              (0x1U << GPIO_LCKR_LCK2_Pos)           /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                  GPIO_LCKR_LCK2_Msk                     
#define GPIO_LCKR_LCK3_Pos              (3U)                                   
#define GPIO_LCKR_LCK3_Msk              (0x1U << GPIO_LCKR_LCK3_Pos)           /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                  GPIO_LCKR_LCK3_Msk                     
#define GPIO_LCKR_LCK4_Pos              (4U)                                   
#define GPIO_LCKR_LCK4_Msk              (0x1U << GPIO_LCKR_LCK4_Pos)           /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                  GPIO_LCKR_LCK4_Msk                     
#define GPIO_LCKR_LCK5_Pos              (5U)                                   
#define GPIO_LCKR_LCK5_Msk              (0x1U << GPIO_LCKR_LCK5_Pos)           /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                  GPIO_LCKR_LCK5_Msk                     
#define GPIO_LCKR_LCK6_Pos              (6U)                                   
#define GPIO_LCKR_LCK6_Msk              (0x1U << GPIO_LCKR_LCK6_Pos)           /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                  GPIO_LCKR_LCK6_Msk                     
#define GPIO_LCKR_LCK7_Pos              (7U)                                   
#define GPIO_LCKR_LCK7_Msk              (0x1U << GPIO_LCKR_LCK7_Pos)           /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                  GPIO_LCKR_LCK7_Msk                     
#define GPIO_LCKR_LCK8_Pos              (8U)                                   
#define GPIO_LCKR_LCK8_Msk              (0x1U << GPIO_LCKR_LCK8_Pos)           /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                  GPIO_LCKR_LCK8_Msk                     
#define GPIO_LCKR_LCK9_Pos              (9U)                                   
#define GPIO_LCKR_LCK9_Msk              (0x1U << GPIO_LCKR_LCK9_Pos)           /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                  GPIO_LCKR_LCK9_Msk                     
#define GPIO_LCKR_LCK10_Pos             (10U)                                  
#define GPIO_LCKR_LCK10_Msk             (0x1U << GPIO_LCKR_LCK10_Pos)          /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                 GPIO_LCKR_LCK10_Msk                    
#define GPIO_LCKR_LCK11_Pos             (11U)                                  
#define GPIO_LCKR_LCK11_Msk             (0x1U << GPIO_LCKR_LCK11_Pos)          /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                 GPIO_LCKR_LCK11_Msk                    
#define GPIO_LCKR_LCK12_Pos             (12U)                                  
#define GPIO_LCKR_LCK12_Msk             (0x1U << GPIO_LCKR_LCK12_Pos)          /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                 GPIO_LCKR_LCK12_Msk                    
#define GPIO_LCKR_LCK13_Pos             (13U)                                  
#define GPIO_LCKR_LCK13_Msk             (0x1U << GPIO_LCKR_LCK13_Pos)          /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                 GPIO_LCKR_LCK13_Msk                    
#define GPIO_LCKR_LCK14_Pos             (14U)                                  
#define GPIO_LCKR_LCK14_Msk             (0x1U << GPIO_LCKR_LCK14_Pos)          /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                 GPIO_LCKR_LCK14_Msk                    
#define GPIO_LCKR_LCK15_Pos             (15U)                                  
#define GPIO_LCKR_LCK15_Msk             (0x1U << GPIO_LCKR_LCK15_Pos)          /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                 GPIO_LCKR_LCK15_Msk                    
#define GPIO_LCKR_LCKK_Pos              (16U)                                  
#define GPIO_LCKR_LCKK_Msk              (0x1U << GPIO_LCKR_LCKK_Pos)           /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                  GPIO_LCKR_LCKK_Msk                     

/****************** Bit definition for GPIO_AFRL register  ********************/
#define GPIO_AFRL_AFSEL0_Pos            (0U)                                   
#define GPIO_AFRL_AFSEL0_Msk            (0xFU << GPIO_AFRL_AFSEL0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0                GPIO_AFRL_AFSEL0_Msk                    
#define GPIO_AFRL_AFSEL1_Pos            (4U)                                   
#define GPIO_AFRL_AFSEL1_Msk            (0xFU << GPIO_AFRL_AFSEL1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1                GPIO_AFRL_AFSEL1_Msk                    
#define GPIO_AFRL_AFSEL2_Pos            (8U)                                   
#define GPIO_AFRL_AFSEL2_Msk            (0xFU << GPIO_AFRL_AFSEL2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2                GPIO_AFRL_AFSEL2_Msk                    
#define GPIO_AFRL_AFSEL3_Pos            (12U)                                  
#define GPIO_AFRL_AFSEL3_Msk            (0xFU << GPIO_AFRL_AFSEL3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3                GPIO_AFRL_AFSEL3_Msk                    
#define GPIO_AFRL_AFSEL4_Pos            (16U)                                  
#define GPIO_AFRL_AFSEL4_Msk            (0xFU << GPIO_AFRL_AFSEL4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4                GPIO_AFRL_AFSEL4_Msk                    
#define GPIO_AFRL_AFSEL5_Pos            (20U)                                  
#define GPIO_AFRL_AFSEL5_Msk            (0xFU << GPIO_AFRL_AFSEL5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5                GPIO_AFRL_AFSEL5_Msk                    
#define GPIO_AFRL_AFSEL6_Pos            (24U)                                  
#define GPIO_AFRL_AFSEL6_Msk            (0xFU << GPIO_AFRL_AFSEL6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6                GPIO_AFRL_AFSEL6_Msk                    
#define GPIO_AFRL_AFSEL7_Pos            (28U)                                  
#define GPIO_AFRL_AFSEL7_Msk            (0xFU << GPIO_AFRL_AFSEL7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7                GPIO_AFRL_AFSEL7_Msk  

/* Legacy aliases */                  
#define GPIO_AFRL_AFRL0_Pos             GPIO_AFRL_AFSEL0_Pos                                  
#define GPIO_AFRL_AFRL0_Msk             GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFRL0                 GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL1_Pos             GPIO_AFRL_AFSEL1_Pos
#define GPIO_AFRL_AFRL1_Msk             GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFRL1                 GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL2_Pos             GPIO_AFRL_AFSEL2_Pos
#define GPIO_AFRL_AFRL2_Msk             GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFRL2                 GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL3_Pos             GPIO_AFRL_AFSEL3_Pos
#define GPIO_AFRL_AFRL3_Msk             GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFRL3                 GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL4_Pos             GPIO_AFRL_AFSEL4_Pos
#define GPIO_AFRL_AFRL4_Msk             GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFRL4                 GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL5_Pos             GPIO_AFRL_AFSEL5_Pos
#define GPIO_AFRL_AFRL5_Msk             GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFRL5                 GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL6_Pos             GPIO_AFRL_AFSEL6_Pos
#define GPIO_AFRL_AFRL6_Msk             GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFRL6                 GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL7_Pos             GPIO_AFRL_AFSEL7_Pos
#define GPIO_AFRL_AFRL7_Msk             GPIO_AFRL_AFSEL7_Msk
#define GPIO_AFRL_AFRL7                 GPIO_AFRL_AFSEL7
 
/****************** Bit definition for GPIO_AFRH register  ********************/
#define GPIO_AFRH_AFSEL8_Pos            (0U)                                   
#define GPIO_AFRH_AFSEL8_Msk            (0xFU << GPIO_AFRH_AFSEL8_Pos)         /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8                GPIO_AFRH_AFSEL8_Msk                    
#define GPIO_AFRH_AFSEL9_Pos            (4U)                                   
#define GPIO_AFRH_AFSEL9_Msk            (0xFU << GPIO_AFRH_AFSEL9_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9                GPIO_AFRH_AFSEL9_Msk                    
#define GPIO_AFRH_AFSEL10_Pos           (8U)                                   
#define GPIO_AFRH_AFSEL10_Msk           (0xFU << GPIO_AFRH_AFSEL10_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10               GPIO_AFRH_AFSEL10_Msk                    
#define GPIO_AFRH_AFSEL11_Pos           (12U)                                  
#define GPIO_AFRH_AFSEL11_Msk           (0xFU << GPIO_AFRH_AFSEL11_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11               GPIO_AFRH_AFSEL11_Msk                    
#define GPIO_AFRH_AFSEL12_Pos           (16U)                                  
#define GPIO_AFRH_AFSEL12_Msk           (0xFU << GPIO_AFRH_AFSEL12_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12               GPIO_AFRH_AFSEL12_Msk                    
#define GPIO_AFRH_AFSEL13_Pos           (20U)                                  
#define GPIO_AFRH_AFSEL13_Msk           (0xFU << GPIO_AFRH_AFSEL13_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13               GPIO_AFRH_AFSEL13_Msk                    
#define GPIO_AFRH_AFSEL14_Pos           (24U)                                  
#define GPIO_AFRH_AFSEL14_Msk           (0xFU << GPIO_AFRH_AFSEL14_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14               GPIO_AFRH_AFSEL14_Msk                    
#define GPIO_AFRH_AFSEL15_Pos           (28U)                                  
#define GPIO_AFRH_AFSEL15_Msk           (0xFU << GPIO_AFRH_AFSEL15_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15               GPIO_AFRH_AFSEL15_Msk                    

/* Legacy aliases */                  
#define GPIO_AFRH_AFRH0_Pos             GPIO_AFRH_AFSEL8_Pos
#define GPIO_AFRH_AFRH0_Msk             GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFRH0                 GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH1_Pos             GPIO_AFRH_AFSEL9_Pos
#define GPIO_AFRH_AFRH1_Msk             GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFRH1                 GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH2_Pos             GPIO_AFRH_AFSEL10_Pos
#define GPIO_AFRH_AFRH2_Msk             GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFRH2                 GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH3_Pos             GPIO_AFRH_AFSEL11_Pos
#define GPIO_AFRH_AFRH3_Msk             GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFRH3                 GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH4_Pos             GPIO_AFRH_AFSEL12_Pos
#define GPIO_AFRH_AFRH4_Msk             GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFRH4                 GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH5_Pos             GPIO_AFRH_AFSEL13_Pos
#define GPIO_AFRH_AFRH5_Msk             GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFRH5                 GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH6_Pos             GPIO_AFRH_AFSEL14_Pos
#define GPIO_AFRH_AFRH6_Msk             GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFRH6                 GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH7_Pos             GPIO_AFRH_AFSEL15_Pos
#define GPIO_AFRH_AFRH7_Msk             GPIO_AFRH_AFSEL15_Msk
#define GPIO_AFRH_AFRH7                 GPIO_AFRH_AFSEL15

/****************** Bit definition for GPIO_BRR register  *********************/
#define GPIO_BRR_BR_0                   (0x00000001U)                          
#define GPIO_BRR_BR_1                   (0x00000002U)                          
#define GPIO_BRR_BR_2                   (0x00000004U)                          
#define GPIO_BRR_BR_3                   (0x00000008U)                          
#define GPIO_BRR_BR_4                   (0x00000010U)                          
#define GPIO_BRR_BR_5                   (0x00000020U)                          
#define GPIO_BRR_BR_6                   (0x00000040U)                          
#define GPIO_BRR_BR_7                   (0x00000080U)                          
#define GPIO_BRR_BR_8                   (0x00000100U)                          
#define GPIO_BRR_BR_9                   (0x00000200U)                          
#define GPIO_BRR_BR_10                  (0x00000400U)                          
#define GPIO_BRR_BR_11                  (0x00000800U)                          
#define GPIO_BRR_BR_12                  (0x00001000U)                          
#define GPIO_BRR_BR_13                  (0x00002000U)                          
#define GPIO_BRR_BR_14                  (0x00004000U)                          
#define GPIO_BRR_BR_15                  (0x00008000U)                          

/******************************************************************************/
/*                                                                            */
/*                   Inter-integrated Circuit Interface (I2C)                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  *******************/
#define I2C_CR1_PE_Pos               (0U)                                      
#define I2C_CR1_PE_Msk               (0x1U << I2C_CR1_PE_Pos)                  /*!< 0x00000001 */
#define I2C_CR1_PE                   I2C_CR1_PE_Msk                            /*!< Peripheral enable */
#define I2C_CR1_TXIE_Pos             (1U)                                      
#define I2C_CR1_TXIE_Msk             (0x1U << I2C_CR1_TXIE_Pos)                /*!< 0x00000002 */
#define I2C_CR1_TXIE                 I2C_CR1_TXIE_Msk                          /*!< TX interrupt enable */
#define I2C_CR1_RXIE_Pos             (2U)                                      
#define I2C_CR1_RXIE_Msk             (0x1U << I2C_CR1_RXIE_Pos)                /*!< 0x00000004 */
#define I2C_CR1_RXIE                 I2C_CR1_RXIE_Msk                          /*!< RX interrupt enable */
#define I2C_CR1_ADDRIE_Pos           (3U)                                      
#define I2C_CR1_ADDRIE_Msk           (0x1U << I2C_CR1_ADDRIE_Pos)              /*!< 0x00000008 */
#define I2C_CR1_ADDRIE               I2C_CR1_ADDRIE_Msk                        /*!< Address match interrupt enable */
#define I2C_CR1_NACKIE_Pos           (4U)                                      
#define I2C_CR1_NACKIE_Msk           (0x1U << I2C_CR1_NACKIE_Pos)              /*!< 0x00000010 */
#define I2C_CR1_NACKIE               I2C_CR1_NACKIE_Msk                        /*!< NACK received interrupt enable */
#define I2C_CR1_STOPIE_Pos           (5U)                                      
#define I2C_CR1_STOPIE_Msk           (0x1U << I2C_CR1_STOPIE_Pos)              /*!< 0x00000020 */
#define I2C_CR1_STOPIE               I2C_CR1_STOPIE_Msk                        /*!< STOP detection interrupt enable */
#define I2C_CR1_TCIE_Pos             (6U)                                      
#define I2C_CR1_TCIE_Msk             (0x1U << I2C_CR1_TCIE_Pos)                /*!< 0x00000040 */
#define I2C_CR1_TCIE                 I2C_CR1_TCIE_Msk                          /*!< Transfer complete interrupt enable */
#define I2C_CR1_ERRIE_Pos            (7U)                                      
#define I2C_CR1_ERRIE_Msk            (0x1U << I2C_CR1_ERRIE_Pos)               /*!< 0x00000080 */
#define I2C_CR1_ERRIE                I2C_CR1_ERRIE_Msk                         /*!< Errors interrupt enable */
#define I2C_CR1_DNF_Pos              (8U)                                      
#define I2C_CR1_DNF_Msk              (0xFU << I2C_CR1_DNF_Pos)                 /*!< 0x00000F00 */
#define I2C_CR1_DNF                  I2C_CR1_DNF_Msk                           /*!< Digital noise filter */
#define I2C_CR1_ANFOFF_Pos           (12U)                                     
#define I2C_CR1_ANFOFF_Msk           (0x1U << I2C_CR1_ANFOFF_Pos)              /*!< 0x00001000 */
#define I2C_CR1_ANFOFF               I2C_CR1_ANFOFF_Msk                        /*!< Analog noise filter OFF */                                    
#define I2C_CR1_SBC_Pos              (16U)                                     
#define I2C_CR1_SBC_Msk              (0x1U << I2C_CR1_SBC_Pos)                 /*!< 0x00010000 */
#define I2C_CR1_SBC                  I2C_CR1_SBC_Msk                           /*!< Slave byte control */
#define I2C_CR1_NOSTRETCH_Pos        (17U)                                     
#define I2C_CR1_NOSTRETCH_Msk        (0x1U << I2C_CR1_NOSTRETCH_Pos)           /*!< 0x00020000 */
#define I2C_CR1_NOSTRETCH            I2C_CR1_NOSTRETCH_Msk                     /*!< Clock stretching disable */
#define I2C_CR1_WUPEN_Pos						 (18U)																		 /*!<>*/
#define I2C_CR1_WUPEN_Msk            (0x1U<<I2C_CR1_WUPEN_Pos)
#define I2C_CR1_WUPEN								 I2C_CR1_WUPEN_Msk
#define I2C_CR1_GCEN_Pos             (19U)                                     
#define I2C_CR1_GCEN_Msk             (0x1U << I2C_CR1_GCEN_Pos)                /*!< 0x00080000 */
#define I2C_CR1_GCEN                 I2C_CR1_GCEN_Msk                          /*!< General call enable */
#define I2C_CR1_SMBHEN_Pos           (20U)                                     
#define I2C_CR1_SMBHEN_Msk           (0x1U << I2C_CR1_SMBHEN_Pos)              /*!< 0x00100000 */
#define I2C_CR1_SMBHEN               I2C_CR1_SMBHEN_Msk                        /*!< SMBus host address enable */
#define I2C_CR1_SMBDEN_Pos           (21U)                                     
#define I2C_CR1_SMBDEN_Msk           (0x1U << I2C_CR1_SMBDEN_Pos)              /*!< 0x00200000 */
#define I2C_CR1_SMBDEN               I2C_CR1_SMBDEN_Msk                        /*!< SMBus device default address enable */
#define I2C_CR1_ALERTEN_Pos          (22U)                                     
#define I2C_CR1_ALERTEN_Msk          (0x1U << I2C_CR1_ALERTEN_Pos)             /*!< 0x00400000 */
#define I2C_CR1_ALERTEN              I2C_CR1_ALERTEN_Msk                       /*!< SMBus alert enable */
#define I2C_CR1_PECEN_Pos            (23U)                                     
#define I2C_CR1_PECEN_Msk            (0x1U << I2C_CR1_PECEN_Pos)               /*!< 0x00800000 */
#define I2C_CR1_PECEN                I2C_CR1_PECEN_Msk                         /*!< PEC enable */

/******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_SADD_Pos             (0U)                                      
#define I2C_CR2_SADD_Msk             (0x3FFU << I2C_CR2_SADD_Pos)              /*!< 0x000003FF */
#define I2C_CR2_SADD                 I2C_CR2_SADD_Msk                          /*!< Slave address (master mode) */
#define I2C_CR2_RD_WRN_Pos           (10U)                                     
#define I2C_CR2_RD_WRN_Msk           (0x1U << I2C_CR2_RD_WRN_Pos)              /*!< 0x00000400 */
#define I2C_CR2_RD_WRN               I2C_CR2_RD_WRN_Msk                        /*!< Transfer direction (master mode) */
#define I2C_CR2_ADD10_Pos            (11U)                                     
#define I2C_CR2_ADD10_Msk            (0x1U << I2C_CR2_ADD10_Pos)               /*!< 0x00000800 */
#define I2C_CR2_ADD10                I2C_CR2_ADD10_Msk                         /*!< 10-bit addressing mode (master mode) */
#define I2C_CR2_HEAD10R_Pos          (12U)                                     
#define I2C_CR2_HEAD10R_Msk          (0x1U << I2C_CR2_HEAD10R_Pos)             /*!< 0x00001000 */
#define I2C_CR2_HEAD10R              I2C_CR2_HEAD10R_Msk                       /*!< 10-bit address header only read direction (master mode) */
#define I2C_CR2_START_Pos            (13U)                                     
#define I2C_CR2_START_Msk            (0x1U << I2C_CR2_START_Pos)               /*!< 0x00002000 */
#define I2C_CR2_START                I2C_CR2_START_Msk                         /*!< START generation */
#define I2C_CR2_STOP_Pos             (14U)                                     
#define I2C_CR2_STOP_Msk             (0x1U << I2C_CR2_STOP_Pos)                /*!< 0x00004000 */
#define I2C_CR2_STOP                 I2C_CR2_STOP_Msk                          /*!< STOP generation (master mode) */
#define I2C_CR2_NACK_Pos             (15U)                                     
#define I2C_CR2_NACK_Msk             (0x1U << I2C_CR2_NACK_Pos)                /*!< 0x00008000 */
#define I2C_CR2_NACK                 I2C_CR2_NACK_Msk                          /*!< NACK generation (slave mode) */
#define I2C_CR2_NBYTES_Pos           (16U)                                     
#define I2C_CR2_NBYTES_Msk           (0xFFU << I2C_CR2_NBYTES_Pos)             /*!< 0x00FF0000 */
#define I2C_CR2_NBYTES               I2C_CR2_NBYTES_Msk                        /*!< Number of bytes */
#define I2C_CR2_RELOAD_Pos           (24U)                                     
#define I2C_CR2_RELOAD_Msk           (0x1U << I2C_CR2_RELOAD_Pos)              /*!< 0x01000000 */
#define I2C_CR2_RELOAD               I2C_CR2_RELOAD_Msk                        /*!< NBYTES reload mode */
#define I2C_CR2_AUTOEND_Pos          (25U)                                     
#define I2C_CR2_AUTOEND_Msk          (0x1U << I2C_CR2_AUTOEND_Pos)             /*!< 0x02000000 */
#define I2C_CR2_AUTOEND              I2C_CR2_AUTOEND_Msk                       /*!< Automatic end mode (master mode) */
#define I2C_CR2_PECBYTE_Pos          (26U)                                     
#define I2C_CR2_PECBYTE_Msk          (0x1U << I2C_CR2_PECBYTE_Pos)             /*!< 0x04000000 */
#define I2C_CR2_PECBYTE              I2C_CR2_PECBYTE_Msk                       /*!< Packet error checking byte */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define I2C_OAR1_OA1_Pos             (0U)                                      
#define I2C_OAR1_OA1_Msk             (0x3FFU << I2C_OAR1_OA1_Pos)              /*!< 0x000003FF */
#define I2C_OAR1_OA1                 I2C_OAR1_OA1_Msk                          /*!< Interface own address 1 */
#define I2C_OAR1_OA1MODE_Pos         (10U)                                     
#define I2C_OAR1_OA1MODE_Msk         (0x1U << I2C_OAR1_OA1MODE_Pos)            /*!< 0x00000400 */
#define I2C_OAR1_OA1MODE             I2C_OAR1_OA1MODE_Msk                      /*!< Own address 1 10-bit mode */
#define I2C_OAR1_OA1EN_Pos           (15U)                                     
#define I2C_OAR1_OA1EN_Msk           (0x1U << I2C_OAR1_OA1EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR1_OA1EN               I2C_OAR1_OA1EN_Msk                        /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define I2C_OAR2_OA2_Pos             (1U)                                      
#define I2C_OAR2_OA2_Msk             (0x7FU << I2C_OAR2_OA2_Pos)               /*!< 0x000000FE */
#define I2C_OAR2_OA2                 I2C_OAR2_OA2_Msk                          /*!< Interface own address 2 */
#define I2C_OAR2_OA2MSK_Pos          (8U)                                      
#define I2C_OAR2_OA2MSK_Msk          (0x7U << I2C_OAR2_OA2MSK_Pos)             /*!< 0x00000700 */
#define I2C_OAR2_OA2MSK              I2C_OAR2_OA2MSK_Msk                       /*!< Own address 2 masks */
#define I2C_OAR2_OA2NOMASK           (0x00000000U)                             /*!< No mask                                        */
#define I2C_OAR2_OA2MASK01_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK01_Msk       (0x1U << I2C_OAR2_OA2MASK01_Pos)          /*!< 0x00000100 */
#define I2C_OAR2_OA2MASK01           I2C_OAR2_OA2MASK01_Msk                    /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define I2C_OAR2_OA2MASK02_Pos       (9U)                                      
#define I2C_OAR2_OA2MASK02_Msk       (0x1U << I2C_OAR2_OA2MASK02_Pos)          /*!< 0x00000200 */
#define I2C_OAR2_OA2MASK02           I2C_OAR2_OA2MASK02_Msk                    /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define I2C_OAR2_OA2MASK03_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK03_Msk       (0x3U << I2C_OAR2_OA2MASK03_Pos)          /*!< 0x00000300 */
#define I2C_OAR2_OA2MASK03           I2C_OAR2_OA2MASK03_Msk                    /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define I2C_OAR2_OA2MASK04_Pos       (10U)                                     
#define I2C_OAR2_OA2MASK04_Msk       (0x1U << I2C_OAR2_OA2MASK04_Pos)          /*!< 0x00000400 */
#define I2C_OAR2_OA2MASK04           I2C_OAR2_OA2MASK04_Msk                    /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define I2C_OAR2_OA2MASK05_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK05_Msk       (0x5U << I2C_OAR2_OA2MASK05_Pos)          /*!< 0x00000500 */
#define I2C_OAR2_OA2MASK05           I2C_OAR2_OA2MASK05_Msk                    /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define I2C_OAR2_OA2MASK06_Pos       (9U)                                      
#define I2C_OAR2_OA2MASK06_Msk       (0x3U << I2C_OAR2_OA2MASK06_Pos)          /*!< 0x00000600 */
#define I2C_OAR2_OA2MASK06           I2C_OAR2_OA2MASK06_Msk                    /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define I2C_OAR2_OA2MASK07_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK07_Msk       (0x7U << I2C_OAR2_OA2MASK07_Pos)          /*!< 0x00000700 */
#define I2C_OAR2_OA2MASK07           I2C_OAR2_OA2MASK07_Msk                    /*!< OA2[7:1] is masked, No comparison is done      */
#define I2C_OAR2_OA2EN_Pos           (15U)                                     
#define I2C_OAR2_OA2EN_Msk           (0x1U << I2C_OAR2_OA2EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR2_OA2EN               I2C_OAR2_OA2EN_Msk                        /*!< Own address 2 enable */

/*******************  Bit definition for I2C_TIMINGR register ****************/
#define I2C_TIMINGR_SCLL_Pos         (0U)                                      
#define I2C_TIMINGR_SCLL_Msk         (0xFFU << I2C_TIMINGR_SCLL_Pos)           /*!< 0x000000FF */
#define I2C_TIMINGR_SCLL             I2C_TIMINGR_SCLL_Msk                      /*!< SCL low period (master mode) */
#define I2C_TIMINGR_SCLH_Pos         (8U)                                      
#define I2C_TIMINGR_SCLH_Msk         (0xFFU << I2C_TIMINGR_SCLH_Pos)           /*!< 0x0000FF00 */
#define I2C_TIMINGR_SCLH             I2C_TIMINGR_SCLH_Msk                      /*!< SCL high period (master mode) */
#define I2C_TIMINGR_SDADEL_Pos       (16U)                                     
#define I2C_TIMINGR_SDADEL_Msk       (0xFU << I2C_TIMINGR_SDADEL_Pos)          /*!< 0x000F0000 */
#define I2C_TIMINGR_SDADEL           I2C_TIMINGR_SDADEL_Msk                    /*!< Data hold time */
#define I2C_TIMINGR_SCLDEL_Pos       (20U)                                     
#define I2C_TIMINGR_SCLDEL_Msk       (0xFU << I2C_TIMINGR_SCLDEL_Pos)          /*!< 0x00F00000 */
#define I2C_TIMINGR_SCLDEL           I2C_TIMINGR_SCLDEL_Msk                    /*!< Data setup time */
#define I2C_TIMINGR_PRESC_Pos        (28U)                                     
#define I2C_TIMINGR_PRESC_Msk        (0xFU << I2C_TIMINGR_PRESC_Pos)           /*!< 0xF0000000 */
#define I2C_TIMINGR_PRESC            I2C_TIMINGR_PRESC_Msk                     /*!< Timings prescaler */

/******************* Bit definition for I2C_TIMEOUTR register ****************/
#define I2C_TIMEOUTR_TIMEOUTA_Pos    (0U)                                      
#define I2C_TIMEOUTR_TIMEOUTA_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTA_Pos)     /*!< 0x00000FFF */
#define I2C_TIMEOUTR_TIMEOUTA        I2C_TIMEOUTR_TIMEOUTA_Msk                 /*!< Bus timeout A */
#define I2C_TIMEOUTR_TIDLE_Pos       (12U)                                     
#define I2C_TIMEOUTR_TIDLE_Msk       (0x1U << I2C_TIMEOUTR_TIDLE_Pos)          /*!< 0x00001000 */
#define I2C_TIMEOUTR_TIDLE           I2C_TIMEOUTR_TIDLE_Msk                    /*!< Idle clock timeout detection */
#define I2C_TIMEOUTR_TIMOUTEN_Pos    (15U)                                     
#define I2C_TIMEOUTR_TIMOUTEN_Msk    (0x1U << I2C_TIMEOUTR_TIMOUTEN_Pos)       /*!< 0x00008000 */
#define I2C_TIMEOUTR_TIMOUTEN        I2C_TIMEOUTR_TIMOUTEN_Msk                 /*!< Clock timeout enable */
#define I2C_TIMEOUTR_TIMEOUTB_Pos    (16U)                                     
#define I2C_TIMEOUTR_TIMEOUTB_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTB_Pos)     /*!< 0x0FFF0000 */
#define I2C_TIMEOUTR_TIMEOUTB        I2C_TIMEOUTR_TIMEOUTB_Msk                 /*!< Bus timeout B*/
#define I2C_TIMEOUTR_TEXTEN_Pos      (31U)                                     
#define I2C_TIMEOUTR_TEXTEN_Msk      (0x1U << I2C_TIMEOUTR_TEXTEN_Pos)         /*!< 0x80000000 */
#define I2C_TIMEOUTR_TEXTEN          I2C_TIMEOUTR_TEXTEN_Msk                   /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  ********************/
#define I2C_ISR_TXE_Pos              (0U)                                      
#define I2C_ISR_TXE_Msk              (0x1U << I2C_ISR_TXE_Pos)                 /*!< 0x00000001 */
#define I2C_ISR_TXE                  I2C_ISR_TXE_Msk                           /*!< Transmit data register empty */
#define I2C_ISR_TXIS_Pos             (1U)                                      
#define I2C_ISR_TXIS_Msk             (0x1U << I2C_ISR_TXIS_Pos)                /*!< 0x00000002 */
#define I2C_ISR_TXIS                 I2C_ISR_TXIS_Msk                          /*!< Transmit interrupt status */
#define I2C_ISR_RXNE_Pos             (2U)                                      
#define I2C_ISR_RXNE_Msk             (0x1U << I2C_ISR_RXNE_Pos)                /*!< 0x00000004 */
#define I2C_ISR_RXNE                 I2C_ISR_RXNE_Msk                          /*!< Receive data register not empty */
#define I2C_ISR_ADDR_Pos             (3U)                                      
#define I2C_ISR_ADDR_Msk             (0x1U << I2C_ISR_ADDR_Pos)                /*!< 0x00000008 */
#define I2C_ISR_ADDR                 I2C_ISR_ADDR_Msk                          /*!< Address matched (slave mode)*/
#define I2C_ISR_NACKF_Pos            (4U)                                      
#define I2C_ISR_NACKF_Msk            (0x1U << I2C_ISR_NACKF_Pos)               /*!< 0x00000010 */
#define I2C_ISR_NACKF                I2C_ISR_NACKF_Msk                         /*!< NACK received flag */
#define I2C_ISR_STOPF_Pos            (5U)                                      
#define I2C_ISR_STOPF_Msk            (0x1U << I2C_ISR_STOPF_Pos)               /*!< 0x00000020 */
#define I2C_ISR_STOPF                I2C_ISR_STOPF_Msk                         /*!< STOP detection flag */
#define I2C_ISR_TC_Pos               (6U)                                      
#define I2C_ISR_TC_Msk               (0x1U << I2C_ISR_TC_Pos)                  /*!< 0x00000040 */
#define I2C_ISR_TC                   I2C_ISR_TC_Msk                            /*!< Transfer complete (master mode) */
#define I2C_ISR_TCR_Pos              (7U)                                      
#define I2C_ISR_TCR_Msk              (0x1U << I2C_ISR_TCR_Pos)                 /*!< 0x00000080 */
#define I2C_ISR_TCR                  I2C_ISR_TCR_Msk                           /*!< Transfer complete reload */
#define I2C_ISR_BERR_Pos             (8U)                                      
#define I2C_ISR_BERR_Msk             (0x1U << I2C_ISR_BERR_Pos)                /*!< 0x00000100 */
#define I2C_ISR_BERR                 I2C_ISR_BERR_Msk                          /*!< Bus error */
#define I2C_ISR_ARLO_Pos             (9U)                                      
#define I2C_ISR_ARLO_Msk             (0x1U << I2C_ISR_ARLO_Pos)                /*!< 0x00000200 */
#define I2C_ISR_ARLO                 I2C_ISR_ARLO_Msk                          /*!< Arbitration lost */
#define I2C_ISR_OVR_Pos              (10U)                                     
#define I2C_ISR_OVR_Msk              (0x1U << I2C_ISR_OVR_Pos)                 /*!< 0x00000400 */
#define I2C_ISR_OVR                  I2C_ISR_OVR_Msk                           /*!< Overrun/Underrun */
#define I2C_ISR_PECERR_Pos           (11U)                                     
#define I2C_ISR_PECERR_Msk           (0x1U << I2C_ISR_PECERR_Pos)              /*!< 0x00000800 */
#define I2C_ISR_PECERR               I2C_ISR_PECERR_Msk                        /*!< PEC error in reception */
#define I2C_ISR_TIMEOUT_Pos          (12U)                                     
#define I2C_ISR_TIMEOUT_Msk          (0x1U << I2C_ISR_TIMEOUT_Pos)             /*!< 0x00001000 */
#define I2C_ISR_TIMEOUT              I2C_ISR_TIMEOUT_Msk                       /*!< Timeout or Tlow detection flag */
#define I2C_ISR_ALERT_Pos            (13U)                                     
#define I2C_ISR_ALERT_Msk            (0x1U << I2C_ISR_ALERT_Pos)               /*!< 0x00002000 */
#define I2C_ISR_ALERT                I2C_ISR_ALERT_Msk                         /*!< SMBus alert */
#define I2C_ISR_BUSY_Pos             (15U)                                     
#define I2C_ISR_BUSY_Msk             (0x1U << I2C_ISR_BUSY_Pos)                /*!< 0x00008000 */
#define I2C_ISR_BUSY                 I2C_ISR_BUSY_Msk                          /*!< Bus busy */
#define I2C_ISR_DIR_Pos              (16U)                                     
#define I2C_ISR_DIR_Msk              (0x1U << I2C_ISR_DIR_Pos)                 /*!< 0x00010000 */
#define I2C_ISR_DIR                  I2C_ISR_DIR_Msk                           /*!< Transfer direction (slave mode) */
#define I2C_ISR_ADDCODE_Pos          (17U)                                     
#define I2C_ISR_ADDCODE_Msk          (0x7FU << I2C_ISR_ADDCODE_Pos)            /*!< 0x00FE0000 */
#define I2C_ISR_ADDCODE              I2C_ISR_ADDCODE_Msk                       /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  ********************/
#define I2C_ICR_ADDRCF_Pos           (3U)                                      
#define I2C_ICR_ADDRCF_Msk           (0x1U << I2C_ICR_ADDRCF_Pos)              /*!< 0x00000008 */
#define I2C_ICR_ADDRCF               I2C_ICR_ADDRCF_Msk                        /*!< Address matched clear flag */
#define I2C_ICR_NACKCF_Pos           (4U)                                      
#define I2C_ICR_NACKCF_Msk           (0x1U << I2C_ICR_NACKCF_Pos)              /*!< 0x00000010 */
#define I2C_ICR_NACKCF               I2C_ICR_NACKCF_Msk                        /*!< NACK clear flag */
#define I2C_ICR_STOPCF_Pos           (5U)                                      
#define I2C_ICR_STOPCF_Msk           (0x1U << I2C_ICR_STOPCF_Pos)              /*!< 0x00000020 */
#define I2C_ICR_STOPCF               I2C_ICR_STOPCF_Msk                        /*!< STOP detection clear flag */
#define I2C_ICR_BERRCF_Pos           (8U)                                      
#define I2C_ICR_BERRCF_Msk           (0x1U << I2C_ICR_BERRCF_Pos)              /*!< 0x00000100 */
#define I2C_ICR_BERRCF               I2C_ICR_BERRCF_Msk                        /*!< Bus error clear flag */
#define I2C_ICR_ARLOCF_Pos           (9U)                                      
#define I2C_ICR_ARLOCF_Msk           (0x1U << I2C_ICR_ARLOCF_Pos)              /*!< 0x00000200 */
#define I2C_ICR_ARLOCF               I2C_ICR_ARLOCF_Msk                        /*!< Arbitration lost clear flag */
#define I2C_ICR_OVRCF_Pos            (10U)                                     
#define I2C_ICR_OVRCF_Msk            (0x1U << I2C_ICR_OVRCF_Pos)               /*!< 0x00000400 */
#define I2C_ICR_OVRCF                I2C_ICR_OVRCF_Msk                         /*!< Overrun/Underrun clear flag */
#define I2C_ICR_PECCF_Pos            (11U)                                     
#define I2C_ICR_PECCF_Msk            (0x1U << I2C_ICR_PECCF_Pos)               /*!< 0x00000800 */
#define I2C_ICR_PECCF                I2C_ICR_PECCF_Msk                         /*!< PAC error clear flag */
#define I2C_ICR_TIMOUTCF_Pos         (12U)                                     
#define I2C_ICR_TIMOUTCF_Msk         (0x1U << I2C_ICR_TIMOUTCF_Pos)            /*!< 0x00001000 */
#define I2C_ICR_TIMOUTCF             I2C_ICR_TIMOUTCF_Msk                      /*!< Timeout clear flag */
#define I2C_ICR_ALERTCF_Pos          (13U)                                     
#define I2C_ICR_ALERTCF_Msk          (0x1U << I2C_ICR_ALERTCF_Pos)             /*!< 0x00002000 */
#define I2C_ICR_ALERTCF              I2C_ICR_ALERTCF_Msk                       /*!< Alert clear flag */

/******************  Bit definition for I2C_PECR register  *******************/
#define I2C_PECR_PEC_Pos             (0U)                                      
#define I2C_PECR_PEC_Msk             (0xFFU << I2C_PECR_PEC_Pos)               /*!< 0x000000FF */
#define I2C_PECR_PEC                 I2C_PECR_PEC_Msk                          /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define I2C_RXDR_RXDATA_Pos          (0U)                                      
#define I2C_RXDR_RXDATA_Msk          (0xFFU << I2C_RXDR_RXDATA_Pos)            /*!< 0x000000FF */
#define I2C_RXDR_RXDATA              I2C_RXDR_RXDATA_Msk                       /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *******************/
#define I2C_TXDR_TXDATA_Pos          (0U)                                      
#define I2C_TXDR_TXDATA_Msk          (0xFFU << I2C_TXDR_TXDATA_Pos)            /*!< 0x000000FF */
#define I2C_TXDR_TXDATA              I2C_TXDR_TXDATA_Msk                       /*!< 8-bit transmit data */

/*****************************************************************************/
/*                                                                           */
/*                        Independent WATCHDOG (IWDG)                        */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for IWDG_KR register  *******************/
#define IWDG_KR_KEY_Pos      (0U)                                              
#define IWDG_KR_KEY_Msk      (0xFFFFU << IWDG_KR_KEY_Pos)                      /*!< 0x0000FFFF */
#define IWDG_KR_KEY          IWDG_KR_KEY_Msk                                   /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  *******************/
#define IWDG_PR_PR_Pos       (0U)                                              
#define IWDG_PR_PR_Msk       (0x7U << IWDG_PR_PR_Pos)                          /*!< 0x00000007 */
#define IWDG_PR_PR           IWDG_PR_PR_Msk                                    /*!< PR[2:0] (Prescaler divider) */
#define IWDG_PR_PR_0         (0x1U << IWDG_PR_PR_Pos)                          /*!< 0x01 */
#define IWDG_PR_PR_1         (0x2U << IWDG_PR_PR_Pos)                          /*!< 0x02 */
#define IWDG_PR_PR_2         (0x4U << IWDG_PR_PR_Pos)                          /*!< 0x04 */

/*******************  Bit definition for IWDG_RLR register  ******************/
#define IWDG_RLR_RL_Pos      (0U)                                              
#define IWDG_RLR_RL_Msk      (0xFFFU << IWDG_RLR_RL_Pos)                       /*!< 0x00000FFF */
#define IWDG_RLR_RL          IWDG_RLR_RL_Msk                                   /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  *******************/
#define IWDG_SR_PVU_Pos      (0U)                                              
#define IWDG_SR_PVU_Msk      (0x1U << IWDG_SR_PVU_Pos)                         /*!< 0x00000001 */
#define IWDG_SR_PVU          IWDG_SR_PVU_Msk                                   /*!< Watchdog prescaler value update */
#define IWDG_SR_RVU_Pos      (1U)                                              
#define IWDG_SR_RVU_Msk      (0x1U << IWDG_SR_RVU_Pos)                         /*!< 0x00000002 */
#define IWDG_SR_RVU          IWDG_SR_RVU_Msk                                   /*!< Watchdog counter reload value update */
#define IWDG_SR_WVU_Pos      (2U)                                              
#define IWDG_SR_WVU_Msk      (0x1U << IWDG_SR_WVU_Pos)                         /*!< 0x00000004 */
#define IWDG_SR_WVU          IWDG_SR_WVU_Msk                                   /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_KR register  *******************/
#define IWDG_WINR_WIN_Pos    (0U)                                              
#define IWDG_WINR_WIN_Msk    (0xFFFU << IWDG_WINR_WIN_Pos)                     /*!< 0x00000FFF */
#define IWDG_WINR_WIN        IWDG_WINR_WIN_Msk                                 /*!< Watchdog counter window value */

/*****************************************************************************/
/*                                                                           */
/*                          Power Control (PWR)                              */
/*                                                                           */
/*****************************************************************************/

/* Note: No specific macro feature on this device */
/*
by Rakan.zhang
*/

/********************  Bit definition for PWR_CR register  *******************/
#define PWR_CR_LPDS_Pos            (0U)                                        
#define PWR_CR_LPDS_Msk            (0x1U << PWR_CR_LPDS_Pos)                   /*!< 0x00000001 */
#define PWR_CR_LPDS                PWR_CR_LPDS_Msk                             /*!< Low-power Deepsleep */
#define PWR_CR_PDDS_Pos            (1U)                                         

/*******************  Bit definition for PWR_VREF_SEL register  *******************/
#define PWR_WUP_LDO_CFG_VTEST_SEL_Pos     (14U)                                        
#define PWR_WUP_LDO_CFG_VTEST_SEL_Msk     (0x3U << PWR_WUP_LDO_CFG_VTEST_SEL_Pos)             /*!< 0x0000C000 */
#define PWR_WUP_LDO_CFG_VTEST_SEL         PWR_WUP_LDO_CFG_VTEST_SEL_Msk                    


/*****************************************************************************/
/*                                                                           */
/*                         Reset and Clock Control                           */
/*                                                                           */
/*****************************************************************************/
/*
*/

/********************  Bit definition for RCC_CR register  *******************/
#define RCC_CR_HSION_Pos                         (0U)                          
#define RCC_CR_HSION_Msk                         (0x1U << RCC_CR_HSION_Pos)    /*!< 0x00000001 */
#define RCC_CR_HSION                             RCC_CR_HSION_Msk              /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY_Pos                        (1U)                          
#define RCC_CR_HSIRDY_Msk                        (0x1U << RCC_CR_HSIRDY_Pos)   /*!< 0x00000002 */
#define RCC_CR_HSIRDY                            RCC_CR_HSIRDY_Msk             /*!< Internal High Speed clock ready flag */

#define RCC_CR_HSITRIM_Pos                       (3U)                          
#define RCC_CR_HSITRIM_Msk                       (0x1FU << RCC_CR_HSITRIM_Pos) /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                           RCC_CR_HSITRIM_Msk            /*!< Internal High Speed clock trimming */
#define RCC_CR_HSITRIM_0                         (0x01U << RCC_CR_HSITRIM_Pos) /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                         (0x02U << RCC_CR_HSITRIM_Pos) /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                         (0x04U << RCC_CR_HSITRIM_Pos) /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                         (0x08U << RCC_CR_HSITRIM_Pos) /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                         (0x10U << RCC_CR_HSITRIM_Pos) /*!< 0x00000080 */

#define RCC_CR_HSICAL_Pos                        (8U)                          
#define RCC_CR_HSICAL_Msk                        (0xFFU << RCC_CR_HSICAL_Pos)  /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                            RCC_CR_HSICAL_Msk             /*!< Internal High Speed clock Calibration */
#define RCC_CR_HSICAL_0                          (0x01U << RCC_CR_HSICAL_Pos)  /*!< 0x00000100 */
#define RCC_CR_HSICAL_1                          (0x02U << RCC_CR_HSICAL_Pos)  /*!< 0x00000200 */
#define RCC_CR_HSICAL_2                          (0x04U << RCC_CR_HSICAL_Pos)  /*!< 0x00000400 */
#define RCC_CR_HSICAL_3                          (0x08U << RCC_CR_HSICAL_Pos)  /*!< 0x00000800 */
#define RCC_CR_HSICAL_4                          (0x10U << RCC_CR_HSICAL_Pos)  /*!< 0x00001000 */
#define RCC_CR_HSICAL_5                          (0x20U << RCC_CR_HSICAL_Pos)  /*!< 0x00002000 */
#define RCC_CR_HSICAL_6                          (0x40U << RCC_CR_HSICAL_Pos)  /*!< 0x00004000 */
#define RCC_CR_HSICAL_7                          (0x80U << RCC_CR_HSICAL_Pos)  /*!< 0x00008000 */

#define RCC_CR_EXTCLKON_Pos                      (16U)                         
#define RCC_CR_EXTCLKON_Msk                      (0x1U << RCC_CR_EXTCLKON_Pos)    /*!< 0x00010000 */
#define RCC_CR_EXTCLKON                          RCC_CR_EXTCLKON_Msk              /*!< EXTCLK enable */
#define RCC_CR_EXTCLKRDY_Pos                     (17U)                         
#define RCC_CR_EXTCLKRDY_Msk                     (0x1U << RCC_CR_EXTCLKRDY_Pos)   /*!< 0x00020000 */
#define RCC_CR_EXTCLKRDY                         RCC_CR_EXTCLKRDY_Msk             /*!< EXTCLK ready flag */
#define RCC_CR_CSSON_Pos                         (19U)                         
#define RCC_CR_CSSON_Msk                         (0x1U << RCC_CR_CSSON_Pos)    /*!< 0x00080000 */
#define RCC_CR_CSSON                             RCC_CR_CSSON_Msk              /*!< Clock Security System enable */

/********************  Bit definition for RCC_CFGR register  *****************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                          (0U)                          
#define RCC_CFGR_SW_Msk                          (0x3U << RCC_CFGR_SW_Pos)     /*!< 0x00000003 */
#define RCC_CFGR_SW                              RCC_CFGR_SW_Msk               /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                            (0x1U << RCC_CFGR_SW_Pos)     /*!< 0x00000001 */
#define RCC_CFGR_SW_1                            (0x2U << RCC_CFGR_SW_Pos)     /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                          (0x00000000U)                 /*!< HSI selected as system clock */
#define RCC_CFGR_SW_EXTCLK                       (0x00000001U)                 /*!< HSE selected as system clock */
#define RCC_CFGR_SW_LSI                          (0x00000003U)                 /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                         (2U)                          
#define RCC_CFGR_SWS_Msk                         (0x3U << RCC_CFGR_SWS_Pos)    /*!< 0x0000000C */
#define RCC_CFGR_SWS                             RCC_CFGR_SWS_Msk              /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                           (0x1U << RCC_CFGR_SWS_Pos)    /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                           (0x2U << RCC_CFGR_SWS_Pos)    /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                         (0x00000000U)                 /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_EXTCLK                      (0x00000004U)                 /*!< EXTCLK used as system clock */
#define RCC_CFGR_SWS_LSI                         (0x0000000CU)                 /*!< LSI used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                        (4U)                          
#define RCC_CFGR_HPRE_Msk                        (0xFU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                            RCC_CFGR_HPRE_Msk             /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                          (0x1U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                          (0x2U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                          (0x4U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                          (0x8U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                       (0x00000000U)                 /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV6                       (0x00000010U)                 /*!< SYSCLK divided by 6 */
#define RCC_CFGR_HPRE_DIV2                       (0x00000080U)                 /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                       (0x00000090U)                 /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                       (0x000000A0U)                 /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                      (0x000000B0U)                 /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                      (0x000000C0U)                 /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                     (0x000000D0U)                 /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                     (0x000000E0U)                 /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                     (0x000000F0U)                 /*!< SYSCLK divided by 512 */

/*!< PPRE configuration */
#define RCC_CFGR_PPRE_Pos                        (8U)                          
#define RCC_CFGR_PPRE_Msk                        (0x7U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000700 */
#define RCC_CFGR_PPRE                            RCC_CFGR_PPRE_Msk             /*!< PRE[2:0] bits (APB prescaler) */
#define RCC_CFGR_PPRE_0                          (0x1U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000100 */
#define RCC_CFGR_PPRE_1                          (0x2U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000200 */
#define RCC_CFGR_PPRE_2                          (0x4U << RCC_CFGR_PPRE_Pos)   /*!< 0x00000400 */

#define RCC_CFGR_PPRE_DIV1                       (0x00000000U)                 /*!< HCLK not divided */
#define RCC_CFGR_PPRE_DIV2_Pos                   (10U)                         
#define RCC_CFGR_PPRE_DIV2_Msk                   (0x1U << RCC_CFGR_PPRE_DIV2_Pos) /*!< 0x00000400 */
#define RCC_CFGR_PPRE_DIV2                       RCC_CFGR_PPRE_DIV2_Msk        /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE_DIV4_Pos                   (8U)                          
#define RCC_CFGR_PPRE_DIV4_Msk                   (0x5U << RCC_CFGR_PPRE_DIV4_Pos) /*!< 0x00000500 */
#define RCC_CFGR_PPRE_DIV4                       RCC_CFGR_PPRE_DIV4_Msk        /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE_DIV8_Pos                   (9U)                          
#define RCC_CFGR_PPRE_DIV8_Msk                   (0x3U << RCC_CFGR_PPRE_DIV8_Pos) /*!< 0x00000600 */
#define RCC_CFGR_PPRE_DIV8                       RCC_CFGR_PPRE_DIV8_Msk        /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE_DIV16_Pos                  (8U)                          
#define RCC_CFGR_PPRE_DIV16_Msk                  (0x7U << RCC_CFGR_PPRE_DIV16_Pos) /*!< 0x00000700 */
#define RCC_CFGR_PPRE_DIV16                      RCC_CFGR_PPRE_DIV16_Msk       /*!< HCLK divided by 16 */


/*!< MCO configuration */
#define RCC_CFGR_MCO_Pos                         (24U)                         
#define RCC_CFGR_MCO_Msk                         (0xFU << RCC_CFGR_MCO_Pos)    /*!< 0x0F000000 */
#define RCC_CFGR_MCO                             RCC_CFGR_MCO_Msk              /*!< MCO[3:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_0                           (0x1U << RCC_CFGR_MCO_Pos)    /*!< 0x01000000 */
#define RCC_CFGR_MCO_1                           (0x2U << RCC_CFGR_MCO_Pos)    /*!< 0x02000000 */
#define RCC_CFGR_MCO_2                           (0x4U << RCC_CFGR_MCO_Pos)    /*!< 0x04000000 */

#define RCC_CFGR_MCO_NOCLOCK                     (0x00000000U)                 /*!< No clock */
#define RCC_CFGR_MCO_LSI                         (0x02000000U)                 /*!< LSI clock selected as MCO source */
#define RCC_CFGR_MCO_SYSCLK                      (0x04000000U)                 /*!< System clock selected as MCO source */
#define RCC_CFGR_MCO_HSI                         (0x05000000U)                 /*!< HSI clock selected as MCO source */

#define  RCC_CFGR_MCO_PRE                    ((uint32_t)0x70000000)        /*!< MCO prescaler*/

/*!<******************  Bit definition for RCC_CIR register  *****************/
#define RCC_CIR_LSIRDYF_Pos                      (0U)                          
#define RCC_CIR_LSIRDYF_Msk                      (0x1U << RCC_CIR_LSIRDYF_Pos) /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                          RCC_CIR_LSIRDYF_Msk           /*!< LSI Ready Interrupt flag */
#define RCC_CIR_HSIRDYF_Pos                      (2U)                          
#define RCC_CIR_HSIRDYF_Msk                      (0x1U << RCC_CIR_HSIRDYF_Pos) /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                          RCC_CIR_HSIRDYF_Msk           /*!< HSI Ready Interrupt flag */
#define RCC_CIR_EXTRDYF_Pos                      (3U)                          
#define RCC_CIR_EXTRDYF_Msk                      (0x1U << RCC_CIR_EXTRDYF_Pos) /*!< 0x00000008 */
#define RCC_CIR_EXTRDYF                          RCC_CIR_EXTRDYF_Msk           /*!< EXT Ready Interrupt flag */
#define RCC_CIR_CSSF_Pos                         (7U)                          
#define RCC_CIR_CSSF_Msk                         (0x1U << RCC_CIR_CSSF_Pos)    /*!< 0x00000080 */
#define RCC_CIR_CSSF                             RCC_CIR_CSSF_Msk              /*!< Clock Security System Interrupt flag */

#define RCC_CIR_LSIRDYIE_Pos                     (8U)                          
#define RCC_CIR_LSIRDYIE_Msk                     (0x1U << RCC_CIR_LSIRDYIE_Pos) /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                         RCC_CIR_LSIRDYIE_Msk          /*!< LSI Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE_Pos                     (10U)                         
#define RCC_CIR_HSIRDYIE_Msk                     (0x1U << RCC_CIR_HSIRDYIE_Pos) /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                         RCC_CIR_HSIRDYIE_Msk          /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_EXTRDYIE_Pos                     (11U)                         
#define RCC_CIR_EXTRDYIE_Msk                     (0x1U << RCC_CIR_EXTRDYIE_Pos) /*!< 0x00000800 */
#define RCC_CIR_EXTRDYIE                         RCC_CIR_EXTRDYIE_Msk          /*!< EXT Ready Interrupt Enable */

#define RCC_CIR_LSIRDYC_Pos                      (16U)                         
#define RCC_CIR_LSIRDYC_Msk                      (0x1U << RCC_CIR_LSIRDYC_Pos) /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                          RCC_CIR_LSIRDYC_Msk           /*!< LSI Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC_Pos                      (18U)                         
#define RCC_CIR_HSIRDYC_Msk                      (0x1U << RCC_CIR_HSIRDYC_Pos) /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                          RCC_CIR_HSIRDYC_Msk           /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_EXTRDYC_Pos                      (19U)                         
#define RCC_CIR_EXTRDYC_Msk                      (0x1U << RCC_CIR_EXTRDYC_Pos) /*!< 0x00080000 */
#define RCC_CIR_EXTRDYC                          RCC_CIR_EXTRDYC_Msk           /*!< EXT Ready Interrupt Clear */
#define RCC_CIR_CSSC_Pos                         (23U)                         
#define RCC_CIR_CSSC_Msk                         (0x1U << RCC_CIR_CSSC_Pos)    /*!< 0x00800000 */
#define RCC_CIR_CSSC                             RCC_CIR_CSSC_Msk              /*!< Clock Security System Interrupt Clear */

/*****************  Bit definition for RCC_APB2RSTR register  ****************/
#define RCC_APB2RSTR_SYSCFGRST_Pos               (0U)                          
#define RCC_APB2RSTR_SYSCFGRST_Msk               (0x1U << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00000001 */
#define RCC_APB2RSTR_SYSCFGRST                   RCC_APB2RSTR_SYSCFGRST_Msk    /*!< SYSCFG reset */
#define RCC_APB2RSTR_ADCRST_Pos                  (9U)                          
#define RCC_APB2RSTR_ADCRST_Msk                  (0x1U << RCC_APB2RSTR_ADCRST_Pos) /*!< 0x00000200 */
#define RCC_APB2RSTR_ADCRST                      RCC_APB2RSTR_ADCRST_Msk       /*!< ADC reset */
#define RCC_APB2RSTR_TIM1RST_Pos                 (11U)                         
#define RCC_APB2RSTR_TIM1RST_Msk                 (0x1U << RCC_APB2RSTR_TIM1RST_Pos) /*!< 0x00000800 */
#define RCC_APB2RSTR_TIM1RST                     RCC_APB2RSTR_TIM1RST_Msk      /*!< TIM1 reset */
#define RCC_APB2RSTR_SPI1RST_Pos                 (12U)                         
#define RCC_APB2RSTR_SPI1RST_Msk                 (0x1U << RCC_APB2RSTR_SPI1RST_Pos) /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST                     RCC_APB2RSTR_SPI1RST_Msk      /*!< SPI1 reset */
#define RCC_APB2RSTR_USART1RST_Pos               (14U)                         
#define RCC_APB2RSTR_USART1RST_Msk               (0x1U << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_USART1RST                   RCC_APB2RSTR_USART1RST_Msk    /*!< USART1 reset */
#define RCC_APB2RSTR_DBGMCURST_Pos               (22U)                         
#define RCC_APB2RSTR_DBGMCURST_Msk               (0x1U << RCC_APB2RSTR_DBGMCURST_Pos) /*!< 0x00400000 */
#define RCC_APB2RSTR_DBGMCURST                   RCC_APB2RSTR_DBGMCURST_Msk    /*!< DBGMCU reset */

/*!< Old ADC1 reset bit definition maintained for legacy purpose */
#define  RCC_APB2RSTR_ADC1RST                RCC_APB2RSTR_ADCRST          

/*****************  Bit definition for RCC_APB1RSTR register  ****************/

#define RCC_APB1RSTR_TIM2RST_Pos                 (0U)                          
#define RCC_APB1RSTR_TIM2RST_Msk                 (0x1U << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST                     RCC_APB1RSTR_TIM2RST_Msk      /*!< Timer 2 reset */


#define RCC_APB1RSTR_TIM6RST_Pos                 (4U)                          
#define RCC_APB1RSTR_TIM6RST_Msk                 (0x1U << RCC_APB1RSTR_TIM6RST_Pos) /*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST                     RCC_APB1RSTR_TIM6RST_Msk      /*!< Timer 6 reset */

#define RCC_APB1RSTR_WWDGRST_Pos                 (11U)                         
#define RCC_APB1RSTR_WWDGRST_Msk                 (0x1U << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST                     RCC_APB1RSTR_WWDGRST_Msk      /*!< Window Watchdog reset */

#define RCC_APB1RSTR_AWURST_Pos               	(16U)                         
#define RCC_APB1RSTR_AWURST_Msk               	(0x1U << RCC_APB1RSTR_AWURST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_AWURSTT                   	RCC_APB1RSTR_AWURST_Msk    /*!< AWU reset */
#define RCC_APB1RSTR_I2C1RST_Pos                 (21U)                         
#define RCC_APB1RSTR_I2C1RST_Msk                 (0x1U << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST                     RCC_APB1RSTR_I2C1RST_Msk      /*!< I2C 1 reset */
#define RCC_APB1RSTR_PWRRST_Pos                  (28U)                         
#define RCC_APB1RSTR_PWRRST_Msk                  (0x1U << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                      RCC_APB1RSTR_PWRRST_Msk       /*!< PWR reset */

#define RCC_APB1RSTR_BEEPERRST_Pos                  (29U)                         
#define RCC_APB1RSTR_BEEPERRST_Msk                  (0x1U << RCC_APB1RSTR_BEEPERRST_Pos) /*!< 0x20000000 */
#define RCC_APB1RSTR_BEEPERRST                      RCC_APB1RSTR_BEEPERRST_Msk       /*!< BEEPER reset */
#define RCC_APB1RSTR_IOMUXRST_Pos                  (30U)                         
#define RCC_APB1RSTR_IOMUXRST_Msk                  (0x1U << RCC_APB1RSTR_IOMUXRST_Pos) /*!< 0x40000000 */
#define RCC_APB1RSTR_IOMUXRST                      RCC_APB1RSTR_IOMUXRST_Msk       /*!< IOMUX reset */

/******************  Bit definition for RCC_AHBENR register  *****************/
#define RCC_AHBENR_SRAMEN_Pos                    (2U)                          
#define RCC_AHBENR_SRAMEN_Msk                    (0x1U << RCC_AHBENR_SRAMEN_Pos) /*!< 0x00000004 */
#define RCC_AHBENR_SRAMEN                        RCC_AHBENR_SRAMEN_Msk         /*!< SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN_Pos                   (4U)                          
#define RCC_AHBENR_FLITFEN_Msk                   (0x1U << RCC_AHBENR_FLITFEN_Pos) /*!< 0x00000010 */
#define RCC_AHBENR_FLITFEN                       RCC_AHBENR_FLITFEN_Msk        /*!< FLITF clock enable */
#define RCC_AHBENR_CRCEN_Pos                     (6U)                          
#define RCC_AHBENR_CRCEN_Msk                     (0x1U << RCC_AHBENR_CRCEN_Pos) /*!< 0x00000040 */
#define RCC_AHBENR_CRCEN                         RCC_AHBENR_CRCEN_Msk          /*!< CRC clock enable */
#define RCC_AHBENR_GPIOAEN_Pos                   (17U)                         
#define RCC_AHBENR_GPIOAEN_Msk                   (0x1U << RCC_AHBENR_GPIOAEN_Pos) /*!< 0x00020000 */
#define RCC_AHBENR_GPIOAEN                       RCC_AHBENR_GPIOAEN_Msk        /*!< GPIOA clock enable */
#define RCC_AHBENR_GPIOBEN_Pos                   (18U)                         
#define RCC_AHBENR_GPIOBEN_Msk                   (0x1U << RCC_AHBENR_GPIOBEN_Pos) /*!< 0x00040000 */
#define RCC_AHBENR_GPIOBEN                       RCC_AHBENR_GPIOBEN_Msk        /*!< GPIOB clock enable */
#define RCC_AHBENR_GPIOCEN_Pos                   (19U)                         
#define RCC_AHBENR_GPIOCEN_Msk                   (0x1U << RCC_AHBENR_GPIOCEN_Pos) /*!< 0x00080000 */
#define RCC_AHBENR_GPIOCEN                       RCC_AHBENR_GPIOCEN_Msk        /*!< GPIOC clock enable */
#define RCC_AHBENR_GPIODEN_Pos                   (20U)                         
#define RCC_AHBENR_GPIODEN_Msk                   (0x1U << RCC_AHBENR_GPIODEN_Pos) /*!< 0x00100000 */
#define RCC_AHBENR_GPIODEN                       RCC_AHBENR_GPIODEN_Msk        /*!< GPIOD clock enable */

/*****************  Bit definition for RCC_APB2ENR register  *****************/
#define RCC_APB2ENR_SYSCFGCOMPEN_Pos             (0U)                          
#define RCC_APB2ENR_SYSCFGCOMPEN_Msk             (0x1U << RCC_APB2ENR_SYSCFGCOMPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2ENR_SYSCFGCOMPEN                 RCC_APB2ENR_SYSCFGCOMPEN_Msk  /*!< SYSCFG and comparator clock enable */
#define RCC_APB2ENR_ADCEN_Pos                    (9U)                          
#define RCC_APB2ENR_ADCEN_Msk                    (0x1U << RCC_APB2ENR_ADCEN_Pos) /*!< 0x00000200 */
#define RCC_APB2ENR_ADCEN                        RCC_APB2ENR_ADCEN_Msk         /*!< ADC1 clock enable */
#define RCC_APB2ENR_TIM1EN_Pos                   (11U)                         
#define RCC_APB2ENR_TIM1EN_Msk                   (0x1U << RCC_APB2ENR_TIM1EN_Pos) /*!< 0x00000800 */
#define RCC_APB2ENR_TIM1EN                       RCC_APB2ENR_TIM1EN_Msk        /*!< TIM1 clock enable */
#define RCC_APB2ENR_SPI1EN_Pos                   (12U)                         
#define RCC_APB2ENR_SPI1EN_Msk                   (0x1U << RCC_APB2ENR_SPI1EN_Pos) /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                       RCC_APB2ENR_SPI1EN_Msk        /*!< SPI1 clock enable */
#define RCC_APB2ENR_USART1EN_Pos                 (14U)                         
#define RCC_APB2ENR_USART1EN_Msk                 (0x1U << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00004000 */
#define RCC_APB2ENR_USART1EN                     RCC_APB2ENR_USART1EN_Msk      /*!< USART1 clock enable */
#define RCC_APB2ENR_DBGMCUEN_Pos                 (22U)                         
#define RCC_APB2ENR_DBGMCUEN_Msk                 (0x1U << RCC_APB2ENR_DBGMCUEN_Pos) /*!< 0x00400000 */
#define RCC_APB2ENR_DBGMCUEN                     RCC_APB2ENR_DBGMCUEN_Msk      /*!< DBGMCU clock enable */

/* Old Bit definition maintained for legacy purpose */
#define  RCC_APB2ENR_SYSCFGEN                RCC_APB2ENR_SYSCFGCOMPEN        /*!< SYSCFG clock enable */

/*****************  Bit definition for RCC_APB1ENR register  *****************/

#define RCC_APB1ENR_TIM2EN_Pos                   (0U)                          
#define RCC_APB1ENR_TIM2EN_Msk                   (0x1U << RCC_APB1ENR_TIM2EN_Pos) /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                       RCC_APB1ENR_TIM2EN_Msk        /*!< Timer 2 clock enable */

#define RCC_APB1ENR_TIM6EN_Pos                   (4U)                          
#define RCC_APB1ENR_TIM6EN_Msk                   (0x1U << RCC_APB1ENR_TIM6EN_Pos) /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN                       RCC_APB1ENR_TIM6EN_Msk        /*!< Timer 6 clock enable */

#define RCC_APB1ENR_WWDGEN_Pos                   (11U)                         
#define RCC_APB1ENR_WWDGEN_Msk                   (0x1U << RCC_APB1ENR_WWDGEN_Pos) /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                       RCC_APB1ENR_WWDGEN_Msk        /*!< Window Watchdog clock enable */
#define RCC_APB1ENR_AWUEN_Pos                 	(16U)                         
#define RCC_APB1ENR_AWUEN_Msk                   (0x1U << RCC_APB1ENR_AWUEN_Pos) /*!< 0x00010000 */
#define RCC_APB1ENR_AWUEN  						RCC_APB1ENR_AWUEN_Msk
#define RCC_APB1ENR_I2C1EN_Pos                   (21U)                         
#define RCC_APB1ENR_I2C1EN_Msk                   (0x1U << RCC_APB1ENR_I2C1EN_Pos) /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                       RCC_APB1ENR_I2C1EN_Msk        /*!< I2C1 clock enable */
#define RCC_APB1ENR_PWREN_Pos                    (28U)                         
#define RCC_APB1ENR_PWREN_Msk                    (0x1U << RCC_APB1ENR_PWREN_Pos) /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                        RCC_APB1ENR_PWREN_Msk         /*!< PWR clock enable */
#define RCC_APB1ENR_BEEPEREN_Pos                 (29U)                         
#define RCC_APB1ENR_BEEPEREN_Msk                 (0x1U << RCC_APB1ENR_BEEPEREN_Pos) /*!< 0x20000000 */
#define RCC_APB1ENR_BEEPEREN                     RCC_APB1ENR_BEEPEREN_Msk         /*!< BEEPER clock enable */
#define RCC_APB1ENR_IOMUXEN_Pos                 (30U)                         
#define RCC_APB1ENR_IOMUXEN_Msk                 (0x1U << RCC_APB1ENR_IOMUXEN_Pos) /*!< 0x40000000 */
#define RCC_APB1ENR_IOMUXEN                     RCC_APB1ENR_IOMUXEN_Msk         /*!< IOMUX clock enable */

/*!< RTC configuration */
#define RCC_BDCR_RTCSEL_NOCLOCK                  (0x00000000U)                 /*!< No clock */
#define RCC_BDCR_RTCSEL_LSE                      (0x00000100U)                 /*!< LSE oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_LSI                      (0x00000200U)                 /*!< LSI oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_HSE                      (0x00000300U)                 /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define RCC_BDCR_RTCEN_Pos                       (15U)                         
#define RCC_BDCR_RTCEN_Msk                       (0x1U << RCC_BDCR_RTCEN_Pos)  /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                           RCC_BDCR_RTCEN_Msk            /*!< RTC clock enable */
#define RCC_BDCR_BDRST_Pos                       (16U)                         
#define RCC_BDCR_BDRST_Msk                       (0x1U << RCC_BDCR_BDRST_Pos)  /*!< 0x00010000 */
#define RCC_BDCR_BDRST                           RCC_BDCR_BDRST_Msk            /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                        (0U)                          
#define RCC_CSR_LSION_Msk                        (0x1U << RCC_CSR_LSION_Pos)   /*!< 0x00000001 */
#define RCC_CSR_LSION                            RCC_CSR_LSION_Msk             /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY_Pos                       (1U)                          
#define RCC_CSR_LSIRDY_Msk                       (0x1U << RCC_CSR_LSIRDY_Pos)  /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                           RCC_CSR_LSIRDY_Msk            /*!< Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF_Pos                         (24U)                         
#define RCC_CSR_RMVF_Msk                         (0x1U << RCC_CSR_RMVF_Pos)    /*!< 0x01000000 */
#define RCC_CSR_RMVF                             RCC_CSR_RMVF_Msk              /*!< Remove reset flag */
#define RCC_CSR_PINRSTF_Pos                      (26U)                         
#define RCC_CSR_PINRSTF_Msk                      (0x1U << RCC_CSR_PINRSTF_Pos) /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                          RCC_CSR_PINRSTF_Msk           /*!< PIN reset flag */
#define RCC_CSR_PORRSTF_Pos                      (27U)                         
#define RCC_CSR_PORRSTF_Msk                      (0x1U << RCC_CSR_PORRSTF_Pos) /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                          RCC_CSR_PORRSTF_Msk           /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF_Pos                      (28U)                         
#define RCC_CSR_SFTRSTF_Msk                      (0x1U << RCC_CSR_SFTRSTF_Pos) /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                          RCC_CSR_SFTRSTF_Msk           /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF_Pos                     (29U)                         
#define RCC_CSR_IWDGRSTF_Msk                     (0x1U << RCC_CSR_IWDGRSTF_Pos) /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                         RCC_CSR_IWDGRSTF_Msk          /*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF_Pos                     (30U)                         
#define RCC_CSR_WWDGRSTF_Msk                     (0x1U << RCC_CSR_WWDGRSTF_Pos) /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                         RCC_CSR_WWDGRSTF_Msk          /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF_Pos                     (31U)                         
#define RCC_CSR_LPWRRSTF_Msk                     (0x1U << RCC_CSR_LPWRRSTF_Pos) /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                         RCC_CSR_LPWRRSTF_Msk          /*!< Low-Power reset flag */


/*******************  Bit definition for RCC_AHBRSTR register  ***************/
#define RCC_AHBRSTR_GPIOARST_Pos                 (17U)                         
#define RCC_AHBRSTR_GPIOARST_Msk                 (0x1U << RCC_AHBRSTR_GPIOARST_Pos) /*!< 0x00020000 */
#define RCC_AHBRSTR_GPIOARST                     RCC_AHBRSTR_GPIOARST_Msk      /*!< GPIOA reset */
#define RCC_AHBRSTR_GPIOBRST_Pos                 (18U)                         
#define RCC_AHBRSTR_GPIOBRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOBRST_Pos) /*!< 0x00040000 */
#define RCC_AHBRSTR_GPIOBRST                     RCC_AHBRSTR_GPIOBRST_Msk      /*!< GPIOB reset */
#define RCC_AHBRSTR_GPIOCRST_Pos                 (19U)                         
#define RCC_AHBRSTR_GPIOCRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOCRST_Pos) /*!< 0x00080000 */
#define RCC_AHBRSTR_GPIOCRST                     RCC_AHBRSTR_GPIOCRST_Msk      /*!< GPIOC reset */
#define RCC_AHBRSTR_GPIODRST_Pos                 (20U)                         
#define RCC_AHBRSTR_GPIODRST_Msk                 (0x1U << RCC_AHBRSTR_GPIODRST_Pos) /*!< 0x00100000 */
#define RCC_AHBRSTR_GPIODRST                     RCC_AHBRSTR_GPIODRST_Msk      /*!< GPIOD reset */
#define RCC_AHBRSTR_CRCRST_Pos                   (6U)                         
#define RCC_AHBRSTR_CRCRST_Msk                   (0x1U << RCC_AHBRSTR_CRCRST_Pos) /*!< 0x00000040 */
#define RCC_AHBRSTR_CRCRST                       RCC_AHBRSTR_CRCRST_Msk      /*!< CRC reset */

/*******************  Bit definition for RCC_CFGR3 register  *****************/
/*!< USART1 Clock source selection */
#define RCC_CFGR3_USART1SW_Pos                   (0U)                          
#define RCC_CFGR3_USART1SW_Msk                   (0x3U << RCC_CFGR3_USART1SW_Pos) /*!< 0x00000003 */
#define RCC_CFGR3_USART1SW                       RCC_CFGR3_USART1SW_Msk        /*!< USART1SW[1:0] bits */
#define RCC_CFGR3_USART1SW_0                     (0x1U << RCC_CFGR3_USART1SW_Pos) /*!< 0x00000001 */
#define RCC_CFGR3_USART1SW_1                     (0x2U << RCC_CFGR3_USART1SW_Pos) /*!< 0x00000002 */

#define RCC_CFGR3_USART1SW_PCLK                  (0x00000000U)                 /*!< PCLK clock used as USART1 clock source */
#define RCC_CFGR3_USART1SW_SYSCLK                (0x00000001U)                 /*!< System clock selected as USART1 clock source */
#define RCC_CFGR3_USART1SW_LSE                   (0x00000002U)                 /*!< LSE oscillator clock used as USART1 clock source */
#define RCC_CFGR3_USART1SW_HSI                   (0x00000003U)                 /*!< HSI oscillator clock used as USART1 clock source */

/*!< I2C1 Clock source selection */
#define RCC_CFGR3_I2C1SW_Pos                     (4U)                          
#define RCC_CFGR3_I2C1SW_Msk                     (0x1U << RCC_CFGR3_I2C1SW_Pos) /*!< 0x00000010 */
#define RCC_CFGR3_I2C1SW                         RCC_CFGR3_I2C1SW_Msk          /*!< I2C1SW bits */ 

#define RCC_CFGR3_I2C1SW_HSI                     (0x00000000U)                 /*!< HSI oscillator clock used as I2C1 clock source */
#define RCC_CFGR3_I2C1SW_SYSCLK_Pos              (4U)                          
#define RCC_CFGR3_I2C1SW_SYSCLK_Msk              (0x1U << RCC_CFGR3_I2C1SW_SYSCLK_Pos) /*!< 0x00000010 */
#define RCC_CFGR3_I2C1SW_SYSCLK                  RCC_CFGR3_I2C1SW_SYSCLK_Msk   /*!< System clock selected as I2C1 clock source */

/* ======================================================  RCC_CSS  ======================================================= */
#define RCC_CSS_THRESHOLD_Pos         (25UL)                    /*!< CSS_THRESHOLD (Bit 25)                                */
#define RCC_CSS_THRESHOLD             (0xfe000000UL)            /*!< CSS_THRESHOLD (Bitfield-Mask: 0x7f)                   */


/* =======================================================  RCC_CFGR4  ======================================================= */
#define RCC_RCC_CFGR4_USARTHSIPRE_Pos                     (0UL)                     /*!< USART(Bit 0)                                           */
#define RCC_RCC_CFGR4_USARTHSIPRE                         (0x1FUL)                   /*!< USART (Bitfield-Mask: 0x1F)                             */

#define RCC_RCC_CFGR4_FLITFCLK_SE_Pos             (9UL)                     /*!< FLITFCLK_SE (Bit 9)                                   */
#define RCC_RCC_CFGR4_FLITFCLK_SE                 (0x600UL)                 /*!< FLITFCLK_SE (Bitfield-Mask: 0x03)                     */
#define RCC_RCC_CFGR4_FLITFCLK_PRE_Pos            (11UL)                    /*!< FLITFCLK_PRE (Bit 11)                                 */
#define RCC_RCC_CFGR4_FLITFCLK_PRE                (0x7800UL)                /*!< FLITFCLK_PRE (Bitfield-Mask: 0x0f)                    */
#define RCC_RCC_CFGR4_I2C1CLK_SEL_Pos             (15UL)                    /*!< I2C1CLK_SEL (Bit 15)                                  */
#define RCC_RCC_CFGR4_I2C1CLK_SEL                 (0x8000UL)                /*!< I2C1CLK_SEL (Bitfield-Mask: 0x01)                     */
#define RCC_RCC_CFGR4_I2CHSIPRE_Pos                 (16UL)                  /*!< I2CHSIPRE (Bit 16)                                      */
#define RCC_RCC_CFGR4_I2CHSIPRE                     (0x1F0000UL)             /*!< I2CHSIPRE (Bitfield-Mask: 0x01F)                         */
#define RCC_RCC_CFGR4_EXTCLK_SEL_Pos              (24UL)                    /*!< EXTCLK_SEL (Bit 24)                                   */
#define RCC_RCC_CFGR4_EXTCLK_SEL                  (0x3000000UL)             /*!< EXTCLK_SEL (Bitfield-Mask: 0x03)                      */
#define RCC_RCC_CFGR4_ADCHSIPRE_Pos             (26UL)                    /*!< ADCHSIPRE (Bit 26)                                  */
#define RCC_RCC_CFGR4_ADCHSIPRE                 (0x7c000000UL)             /*!< ADCHSIPRE (Bitfield-Mask: 0x1F)                     */

#define RCC_CFGR4_EXTCLK_SEL_PA1				(0x0000000UL)
#define RCC_CFGR4_EXTCLK_SEL_PD7				(0x1000000UL)
#define RCC_CFGR4_EXTCLK_SEL_PB5				(0x2000000UL)
#define RCC_CFGR4_EXTCLK_SEL_PC5				(0x3000000UL)

#define RCC_RCC_AHBENR2_DVSQEN_Pos                (0UL)                     /*!< DVSQEN (Bit 0)                                        */
#define RCC_RCC_AHBENR2_DVSQEN                    (0x1UL)                   /*!< DVSQEN (Bitfield-Mask: 0x01)                          */


/*****************************************************************************/
/*                                                                           */
/*                        Serial Peripheral Interface (SPI)                  */
/*                                                                           */
/*****************************************************************************/

/*
 */
/* Note: No specific macro feature on this device */

/*******************  Bit definition for SPI_CR1 register  *******************/
#define SPI_CR1_CPHA_Pos            (0U)                                       
#define SPI_CR1_CPHA_Msk            (0x1U << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!< Clock Phase */
#define SPI_CR1_CPOL_Pos            (1U)                                       
#define SPI_CR1_CPOL_Msk            (0x1U << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!< Clock Polarity */
#define SPI_CR1_MSTR_Pos            (2U)                                       
#define SPI_CR1_MSTR_Msk            (0x1U << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!< Master Selection */
#define SPI_CR1_BR_Pos              (3U)                                       
#define SPI_CR1_BR_Msk              (0x7U << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1U << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2U << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4U << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */
#define SPI_CR1_SPE_Pos             (6U)                                       
#define SPI_CR1_SPE_Msk             (0x1U << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!< SPI Enable */
#define SPI_CR1_LSBFIRST_Pos        (7U)                                       
#define SPI_CR1_LSBFIRST_Msk        (0x1U << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!< Frame Format */
#define SPI_CR1_SSI_Pos             (8U)                                       
#define SPI_CR1_SSI_Msk             (0x1U << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!< Internal slave select */
#define SPI_CR1_SSM_Pos             (9U)                                       
#define SPI_CR1_SSM_Msk             (0x1U << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!< Software slave management */
#define SPI_CR1_RXONLY_Pos          (10U)                                      
#define SPI_CR1_RXONLY_Msk          (0x1U << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!< Receive only */
#define SPI_CR1_CRCL_Pos            (11U)                                      
#define SPI_CR1_CRCL_Msk            (0x1U << SPI_CR1_CRCL_Pos)                 /*!< 0x00000800 */
#define SPI_CR1_CRCL                SPI_CR1_CRCL_Msk                           /*!< CRC Length */
#define SPI_CR1_CRCNEXT_Pos         (12U)                                      
#define SPI_CR1_CRCNEXT_Msk         (0x1U << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!< Transmit CRC next */
#define SPI_CR1_CRCEN_Pos           (13U)                                      
#define SPI_CR1_CRCEN_Msk           (0x1U << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE_Pos          (14U)                                      
#define SPI_CR1_BIDIOE_Msk          (0x1U << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)                                      
#define SPI_CR1_BIDIMODE_Msk        (0x1U << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  *******************/
#define SPI_CR2_SSOE_Pos            (2U)                                       
#define SPI_CR2_SSOE_Msk            (0x1U << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!< SS Output Enable */
#define SPI_CR2_NSSP_Pos            (3U)                                       
#define SPI_CR2_NSSP_Msk            (0x1U << SPI_CR2_NSSP_Pos)                 /*!< 0x00000008 */
#define SPI_CR2_NSSP                SPI_CR2_NSSP_Msk                           /*!< NSS pulse management Enable */
#define SPI_CR2_FRF_Pos             (4U)                                       
#define SPI_CR2_FRF_Msk             (0x1U << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!< Frame Format Enable */
#define SPI_CR2_ERRIE_Pos           (5U)                                       
#define SPI_CR2_ERRIE_Msk           (0x1U << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE_Pos          (6U)                                       
#define SPI_CR2_RXNEIE_Msk          (0x1U << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)                                       
#define SPI_CR2_TXEIE_Msk           (0x1U << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!< Tx buffer Empty Interrupt Enable */
#define SPI_CR2_DS_Pos              (8U)                                       
#define SPI_CR2_DS_Msk              (0xFU << SPI_CR2_DS_Pos)                   /*!< 0x00000F00 */
#define SPI_CR2_DS                  SPI_CR2_DS_Msk                             /*!< DS[3:0] Data Size */
#define SPI_CR2_DS_0                (0x1U << SPI_CR2_DS_Pos)                   /*!< 0x00000100 */
#define SPI_CR2_DS_1                (0x2U << SPI_CR2_DS_Pos)                   /*!< 0x00000200 */
#define SPI_CR2_DS_2                (0x4U << SPI_CR2_DS_Pos)                   /*!< 0x00000400 */
#define SPI_CR2_DS_3                (0x8U << SPI_CR2_DS_Pos)                   /*!< 0x00000800 */
#define SPI_CR2_FRXTH_Pos           (12U)                                      
#define SPI_CR2_FRXTH_Msk           (0x1U << SPI_CR2_FRXTH_Pos)                /*!< 0x00001000 */
#define SPI_CR2_FRXTH               SPI_CR2_FRXTH_Msk                          /*!< FIFO reception Threshold */

/********************  Bit definition for SPI_SR register  *******************/
#define SPI_SR_RXNE_Pos             (0U)                                       
#define SPI_SR_RXNE_Msk             (0x1U << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!< Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)                                       
#define SPI_SR_TXE_Msk              (0x1U << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!< Transmit buffer Empty */
#define SPI_SR_CHSIDE_Pos           (2U)                                       
#define SPI_SR_CHSIDE_Msk           (0x1U << SPI_SR_CHSIDE_Pos)                /*!< 0x00000004 */
#define SPI_SR_CHSIDE               SPI_SR_CHSIDE_Msk                          /*!< Channel side */
#define SPI_SR_UDR_Pos              (3U)                                       
#define SPI_SR_UDR_Msk              (0x1U << SPI_SR_UDR_Pos)                   /*!< 0x00000008 */
#define SPI_SR_UDR                  SPI_SR_UDR_Msk                             /*!< Underrun flag */
#define SPI_SR_CRCERR_Pos           (4U)                                       
#define SPI_SR_CRCERR_Msk           (0x1U << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!< CRC Error flag */
#define SPI_SR_MODF_Pos             (5U)                                       
#define SPI_SR_MODF_Msk             (0x1U << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!< Mode fault */
#define SPI_SR_OVR_Pos              (6U)                                       
#define SPI_SR_OVR_Msk              (0x1U << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!< Overrun flag */
#define SPI_SR_BSY_Pos              (7U)                                       
#define SPI_SR_BSY_Msk              (0x1U << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!< Busy flag */
#define SPI_SR_FRE_Pos              (8U)                                       
#define SPI_SR_FRE_Msk              (0x1U << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!< TI frame format error */
#define SPI_SR_FRLVL_Pos            (9U)                                       
#define SPI_SR_FRLVL_Msk            (0x3U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000600 */
#define SPI_SR_FRLVL                SPI_SR_FRLVL_Msk                           /*!< FIFO Reception Level */
#define SPI_SR_FRLVL_0              (0x1U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000200 */
#define SPI_SR_FRLVL_1              (0x2U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000400 */
#define SPI_SR_FTLVL_Pos            (11U)                                      
#define SPI_SR_FTLVL_Msk            (0x3U << SPI_SR_FTLVL_Pos)                 /*!< 0x00001800 */
#define SPI_SR_FTLVL                SPI_SR_FTLVL_Msk                           /*!< FIFO Transmission Level */
#define SPI_SR_FTLVL_0              (0x1U << SPI_SR_FTLVL_Pos)                 /*!< 0x00000800 */
#define SPI_SR_FTLVL_1              (0x2U << SPI_SR_FTLVL_Pos)                 /*!< 0x00001000 */

/********************  Bit definition for SPI_DR register  *******************/
#define SPI_DR_DR_Pos               (0U)                                       
#define SPI_DR_DR_Msk               (0xFFFFFFFFU << SPI_DR_DR_Pos)             /*!< 0xFFFFFFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  *****************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)                                       
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFFFFFU << SPI_CRCPR_CRCPOLY_Pos)     /*!< 0xFFFFFFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  *****************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)                                       
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFFFFFU << SPI_RXCRCR_RXCRC_Pos)      /*!< 0xFFFFFFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  *****************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)                                       
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFFFFFU << SPI_TXCRCR_TXCRC_Pos)      /*!< 0xFFFFFFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  ****************/
#define SPI_I2SCFGR_CHLEN_Pos	      (0U)
#define SPI_I2SCFGR_CHLEN_Msk       (0x01U <<  SPI_I2SCFGR_CHLEN_Pos)
#define SPI_I2SCFGR_CHLEN           SPI_I2SCFGR_CHLEN_Msk

#define SPI_I2SCFGR_DATLEN_Pos 			(1U)
#define	SPI_I2SCFGR_DATLEN_Msk      (0x03U<< SPI_I2SCFGR_DATLEN_Pos)
#define	SPI_I2SCFGR_DATLEN          SPI_I2SCFGR_DATLEN_Msk
#define	SPI_I2SCFGR_DATLEN_0        (0x1U << SPI_I2SCFGR_DATLEN_Pos)  
#define	SPI_I2SCFGR_DATLEN_1        (0x2U << SPI_I2SCFGR_DATLEN_Pos) 

#define SPI_I2SCFGR_CKPOL_Pos 			(3U)
#define	SPI_I2SCFGR_CKPOL_Msk       (0x01U<< SPI_I2SCFGR_CKPOL_Pos)
#define	SPI_I2SCFGR_CKPOL           SPI_I2SCFGR_CKPOL_Msk

#define SPI_I2SCFGR_I2SSTD_Pos 		  (4U)
#define	SPI_I2SCFGR_I2SSTD_Msk      (0x03U<< SPI_I2SCFGR_I2SSTD_Pos)
#define	SPI_I2SCFGR_I2SSTD          SPI_I2SCFGR_I2SSTD_Msk
#define	SPI_I2SCFGR_I2SSTD_0        (0x1U << SPI_I2SCFGR_I2SSTD_Pos)  
#define	SPI_I2SCFGR_I2SSTD_1        (0x2U << SPI_I2SCFGR_I2SSTD_Pos) 

#define SPI_I2SCFGR_PCMSYNC_Pos 		(7U)
#define	SPI_I2SCFGR_PCMSYNC_Msk     (0x01U<< SPI_I2SCFGR_PCMSYNC_Pos)
#define	SPI_I2SCFGR_PCMSYNC         SPI_I2SCFGR_PCMSYNC_Msk

#define SPI_I2SCFGR_I2SCFG_Pos 			(8U)
#define	SPI_I2SCFGR_I2SCFG_Msk      (0x03U<< SPI_I2SCFGR_I2SCFG_Pos)
#define	SPI_I2SCFGR_I2SCFG          SPI_I2SCFGR_I2SCFG_Msk
#define	SPI_I2SCFGR_I2SCFG_0        (0x1U << SPI_I2SCFGR_I2SCFG_Pos)  
#define	SPI_I2SCFGR_I2SCFG_1        (0x2U << SPI_I2SCFGR_I2SCFG_Pos) 

#define SPI_I2SCFGR_I2SE_Pos 				(10U)
#define	SPI_I2SCFGR_I2SE_Msk        (0x01U<< SPI_I2SCFGR_I2SE_Pos)
#define	SPI_I2SCFGR_I2SE            SPI_I2SCFGR_I2SE_Msk

#define SPI_I2SCFGR_I2SMOD_Pos      (11U)                                      
#define SPI_I2SCFGR_I2SMOD_Msk      (0x01U << SPI_I2SCFGR_I2SMOD_Pos)         /*!< 0x00000800 */
#define SPI_I2SCFGR_I2SMOD          SPI_I2SCFGR_I2SMOD_Msk                     /*!< Keep for compatibility */

/******************  Bit definition for SPI_I2SPR register  ****************/
#define SPI_I2SPR_I2SDIV_Pos         (0U)
#define SPI_I2SPR_I2SDIV_Msk         (0xFFU << SPI_I2SPR_I2SDIV_Pos)
#define SPI_I2SPR_I2SDIV             SPI_I2SPR_I2SDIV_Msk

#define SPI_I2SPR_ODD_Pos           (8U)
#define SPI_I2SPR_ODD_Msk           (0x01U << SPI_I2SPR_ODD_Pos)
#define SPI_I2SPR_ODD               SPI_I2SPR_ODD_Msk

#define SPI_I2SPR_MCKOE_Pos         (9U)
#define SPI_I2SPR_MCKOE_Msk         (0x01U << SPI_I2SPR_MCKOE_Pos)
#define SPI_I2SPR_MCKOE             SPI_I2SPR_MCKOE_Msk


/*****************************************************************************/
/*                                                                           */
/*                       System Configuration (SYSCFG)                       */
/*                                                                           */
/*****************************************************************************/
/*****************  Bit definition for SYSCFG_CFGR1 register  ****************/
#define SYSCFG_CFGR1_MEM_MODE_Pos            (0U)                              
#define SYSCFG_CFGR1_MEM_MODE_Msk            (0x3U << SYSCFG_CFGR1_MEM_MODE_Pos) /*!< 0x00000003 */
#define SYSCFG_CFGR1_MEM_MODE                SYSCFG_CFGR1_MEM_MODE_Msk           /*!< SYSCFG_Memory Remap Config */ 
#define SYSCFG_CFGR1_MEM_MODE_1              (0x2U << SYSCFG_CFGR1_MEM_MODE_Pos) /*!< 0x00000002 */

#define SYSCFG_CFGR1_LOCKUP_LOCK_Pos         (31U)                             
#define SYSCFG_CFGR1_LOCKUP_LOCK_Msk         (0x1U << SYSCFG_CFGR1_LOCKUP_LOCK_Pos) /*!< 0x80000000 */
#define SYSCFG_CFGR1_LOCKUP_LOCK             SYSCFG_CFGR1_LOCKUP_LOCK_Msk      /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM0 with Break Input of TIMER1 */

/*****************  Bit definition for SYSCFG_EXTICR1 register  **************/
#define SYSCFG_EXTICR1_EXTI0_Pos             (0U)                              
#define SYSCFG_EXTICR1_EXTI0_Msk             (0xFU << SYSCFG_EXTICR1_EXTI0_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR1_EXTI0                 SYSCFG_EXTICR1_EXTI0_Msk          /*!< EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1_Pos             (4U)                              
#define SYSCFG_EXTICR1_EXTI1_Msk             (0xFU << SYSCFG_EXTICR1_EXTI1_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR1_EXTI1                 SYSCFG_EXTICR1_EXTI1_Msk          /*!< EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2_Pos             (8U)                              
#define SYSCFG_EXTICR1_EXTI2_Msk             (0xFU << SYSCFG_EXTICR1_EXTI2_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR1_EXTI2                 SYSCFG_EXTICR1_EXTI2_Msk          /*!< EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3_Pos             (12U)                             
#define SYSCFG_EXTICR1_EXTI3_Msk             (0xFU << SYSCFG_EXTICR1_EXTI3_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR1_EXTI3                 SYSCFG_EXTICR1_EXTI3_Msk          /*!< EXTI 3 configuration */

/** 
  * @brief  EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA              (0x00000000U)                     /*!< PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB              (0x00000001U)                     /*!< PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC              (0x00000002U)                     /*!< PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD              (0x00000003U)                     /*!< PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF              (0x00000005U)                     /*!< PF[0] pin */

/** 
  * @brief  EXTI1 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI1_PA              (0x00000000U)                     /*!< PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB              (0x00000010U)                     /*!< PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC              (0x00000020U)                     /*!< PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD              (0x00000030U)                     /*!< PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF              (0x00000050U)                     /*!< PF[1] pin */

/** 
  * @brief  EXTI2 configuration  
  */
#define SYSCFG_EXTICR1_EXTI2_PA              (0x00000000U)                     /*!< PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB              (0x00000100U)                     /*!< PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC              (0x00000200U)                     /*!< PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD              (0x00000300U)                     /*!< PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF              (0x00000500U)                     /*!< PF[2] pin */

/** 
  * @brief  EXTI3 configuration  
  */
#define SYSCFG_EXTICR1_EXTI3_PA              (0x00000000U)                     /*!< PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB              (0x00001000U)                     /*!< PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC              (0x00002000U)                     /*!< PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD              (0x00003000U)                     /*!< PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PF              (0x00005000U)                     /*!< PF[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  **************/
#define SYSCFG_EXTICR2_EXTI4_Pos             (0U)                              
#define SYSCFG_EXTICR2_EXTI4_Msk             (0xFU << SYSCFG_EXTICR2_EXTI4_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR2_EXTI4                 SYSCFG_EXTICR2_EXTI4_Msk          /*!< EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5_Pos             (4U)                              
#define SYSCFG_EXTICR2_EXTI5_Msk             (0xFU << SYSCFG_EXTICR2_EXTI5_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR2_EXTI5                 SYSCFG_EXTICR2_EXTI5_Msk          /*!< EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6_Pos             (8U)                              
#define SYSCFG_EXTICR2_EXTI6_Msk             (0xFU << SYSCFG_EXTICR2_EXTI6_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR2_EXTI6                 SYSCFG_EXTICR2_EXTI6_Msk          /*!< EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7_Pos             (12U)                             
#define SYSCFG_EXTICR2_EXTI7_Msk             (0xFU << SYSCFG_EXTICR2_EXTI7_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR2_EXTI7                 SYSCFG_EXTICR2_EXTI7_Msk          /*!< EXTI 7 configuration */

/** 
  * @brief  EXTI4 configuration  
  */
#define SYSCFG_EXTICR2_EXTI4_PA              (0x00000000U)                     /*!< PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB              (0x00000001U)                     /*!< PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC              (0x00000002U)                     /*!< PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD              (0x00000003U)                     /*!< PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF              (0x00000005U)                     /*!< PF[4] pin */

/** 
  * @brief  EXTI5 configuration  
  */
#define SYSCFG_EXTICR2_EXTI5_PA              (0x00000000U)                     /*!< PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB              (0x00000010U)                     /*!< PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC              (0x00000020U)                     /*!< PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD              (0x00000030U)                     /*!< PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF              (0x00000050U)                     /*!< PF[5] pin */

/** 
  * @brief  EXTI6 configuration  
  */
#define SYSCFG_EXTICR2_EXTI6_PA              (0x00000000U)                     /*!< PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB              (0x00000100U)                     /*!< PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC              (0x00000200U)                     /*!< PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD              (0x00000300U)                     /*!< PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF              (0x00000500U)                     /*!< PF[6] pin */

/** 
  * @brief  EXTI7 configuration  
  */
#define SYSCFG_EXTICR2_EXTI7_PA              (0x00000000U)                     /*!< PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB              (0x00001000U)                     /*!< PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC              (0x00002000U)                     /*!< PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD              (0x00003000U)                     /*!< PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PF              (0x00005000U)                     /*!< PF[7] pin */

/*****************************************************************************/
/*                                                                           */
/*                               Timers (TIM)                                */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for TIM_CR1 register  *******************/
#define TIM_CR1_CEN_Pos           (0U)                                         
#define TIM_CR1_CEN_Msk           (0x1U << TIM_CR1_CEN_Pos)                    /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!<Counter enable */
#define TIM_CR1_UDIS_Pos          (1U)                                         
#define TIM_CR1_UDIS_Msk          (0x1U << TIM_CR1_UDIS_Pos)                   /*!< 0x00000002 */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk                             /*!<Update disable */
#define TIM_CR1_URS_Pos           (2U)                                         
#define TIM_CR1_URS_Msk           (0x1U << TIM_CR1_URS_Pos)                    /*!< 0x00000004 */
#define TIM_CR1_URS               TIM_CR1_URS_Msk                              /*!<Update request source */
#define TIM_CR1_OPM_Pos           (3U)                                         
#define TIM_CR1_OPM_Msk           (0x1U << TIM_CR1_OPM_Pos)                    /*!< 0x00000008 */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk                              /*!<One pulse mode */
#define TIM_CR1_DIR_Pos           (4U)                                         
#define TIM_CR1_DIR_Msk           (0x1U << TIM_CR1_DIR_Pos)                    /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                              /*!<Direction */

#define TIM_CR1_CMS_Pos           (5U)                                         
#define TIM_CR1_CMS_Msk           (0x3U << TIM_CR1_CMS_Pos)                    /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                              /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x1U << TIM_CR1_CMS_Pos)                    /*!< 0x00000020 */
#define TIM_CR1_CMS_1             (0x2U << TIM_CR1_CMS_Pos)                    /*!< 0x00000040 */

#define TIM_CR1_ARPE_Pos          (7U)                                         
#define TIM_CR1_ARPE_Msk          (0x1U << TIM_CR1_ARPE_Pos)                   /*!< 0x00000080 */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk                             /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos           (8U)                                         
#define TIM_CR1_CKD_Msk           (0x3U << TIM_CR1_CKD_Pos)                    /*!< 0x00000300 */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk                              /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0             (0x1U << TIM_CR1_CKD_Pos)                    /*!< 0x00000100 */
#define TIM_CR1_CKD_1             (0x2U << TIM_CR1_CKD_Pos)                    /*!< 0x00000200 */

/*******************  Bit definition for TIM_CR2 register  *******************/
#define TIM_CR2_CCPC_Pos          (0U)                                         
#define TIM_CR2_CCPC_Msk          (0x1U << TIM_CR2_CCPC_Pos)                   /*!< 0x00000001 */
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk                             /*!<Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS_Pos          (2U)                                         
#define TIM_CR2_CCUS_Msk          (0x1U << TIM_CR2_CCUS_Pos)                   /*!< 0x00000004 */
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk                             /*!<Capture/Compare Control Update Selection */

#define TIM_CR2_MMS_Pos           (4U)                                         
#define TIM_CR2_MMS_Msk           (0x7U << TIM_CR2_MMS_Pos)                    /*!< 0x00000070 */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk                              /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0             (0x1U << TIM_CR2_MMS_Pos)                    /*!< 0x00000010 */
#define TIM_CR2_MMS_1             (0x2U << TIM_CR2_MMS_Pos)                    /*!< 0x00000020 */
#define TIM_CR2_MMS_2             (0x4U << TIM_CR2_MMS_Pos)                    /*!< 0x00000040 */

#define TIM_CR2_TI1S_Pos          (7U)                                         
#define TIM_CR2_TI1S_Msk          (0x1U << TIM_CR2_TI1S_Pos)                   /*!< 0x00000080 */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk                             /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos          (8U)                                         
#define TIM_CR2_OIS1_Msk          (0x1U << TIM_CR2_OIS1_Pos)                   /*!< 0x00000100 */
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk                             /*!<Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N_Pos         (9U)                                         
#define TIM_CR2_OIS1N_Msk         (0x1U << TIM_CR2_OIS1N_Pos)                  /*!< 0x00000200 */
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk                            /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos          (10U)                                        
#define TIM_CR2_OIS2_Msk          (0x1U << TIM_CR2_OIS2_Pos)                   /*!< 0x00000400 */
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk                             /*!<Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N_Pos         (11U)                                        
#define TIM_CR2_OIS2N_Msk         (0x1U << TIM_CR2_OIS2N_Pos)                  /*!< 0x00000800 */
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk                            /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos          (12U)                                        
#define TIM_CR2_OIS3_Msk          (0x1U << TIM_CR2_OIS3_Pos)                   /*!< 0x00001000 */
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk                             /*!<Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N_Pos         (13U)                                        
#define TIM_CR2_OIS3N_Msk         (0x1U << TIM_CR2_OIS3N_Pos)                  /*!< 0x00002000 */
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk                            /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos          (14U)                                        
#define TIM_CR2_OIS4_Msk          (0x1U << TIM_CR2_OIS4_Pos)                   /*!< 0x00004000 */
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk                             /*!<Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  ******************/
#define TIM_SMCR_SMS_Pos          (0U)                                         
#define TIM_SMCR_SMS_Msk          (0x7U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000007 */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk                             /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0            (0x1U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000001 */
#define TIM_SMCR_SMS_1            (0x2U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000002 */
#define TIM_SMCR_SMS_2            (0x4U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000004 */

#define TIM_SMCR_OCCS_Pos         (3U)                                         
#define TIM_SMCR_OCCS_Msk         (0x1U << TIM_SMCR_OCCS_Pos)                  /*!< 0x00000008 */
#define TIM_SMCR_OCCS             TIM_SMCR_OCCS_Msk                            /*!< OCREF clear selection */

#define TIM_SMCR_TS_Pos           (4U)                                         
#define TIM_SMCR_TS_Msk           (0x7U << TIM_SMCR_TS_Pos)                    /*!< 0x00000070 */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk                              /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0             (0x1U << TIM_SMCR_TS_Pos)                    /*!< 0x00000010 */
#define TIM_SMCR_TS_1             (0x2U << TIM_SMCR_TS_Pos)                    /*!< 0x00000020 */
#define TIM_SMCR_TS_2             (0x4U << TIM_SMCR_TS_Pos)                    /*!< 0x00000040 */

#define TIM_SMCR_MSM_Pos          (7U)                                         
#define TIM_SMCR_MSM_Msk          (0x1U << TIM_SMCR_MSM_Pos)                   /*!< 0x00000080 */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk                             /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos          (8U)                                         
#define TIM_SMCR_ETF_Msk          (0xFU << TIM_SMCR_ETF_Pos)                   /*!< 0x00000F00 */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk                             /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0            (0x1U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000100 */
#define TIM_SMCR_ETF_1            (0x2U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000200 */
#define TIM_SMCR_ETF_2            (0x4U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000400 */
#define TIM_SMCR_ETF_3            (0x8U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000800 */

#define TIM_SMCR_ETPS_Pos         (12U)                                        
#define TIM_SMCR_ETPS_Msk         (0x3U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00003000 */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk                            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0           (0x1U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00001000 */
#define TIM_SMCR_ETPS_1           (0x2U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00002000 */

#define TIM_SMCR_ECE_Pos          (14U)                                        
#define TIM_SMCR_ECE_Msk          (0x1U << TIM_SMCR_ECE_Pos)                   /*!< 0x00004000 */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk                             /*!<External clock enable */
#define TIM_SMCR_ETP_Pos          (15U)                                        
#define TIM_SMCR_ETP_Msk          (0x1U << TIM_SMCR_ETP_Pos)                   /*!< 0x00008000 */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk                             /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  ******************/
#define TIM_DIER_UIE_Pos          (0U)                                         
#define TIM_DIER_UIE_Msk          (0x1U << TIM_DIER_UIE_Pos)                   /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk                             /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos        (1U)                                         
#define TIM_DIER_CC1IE_Msk        (0x1U << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                           /*!<Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE_Pos        (2U)                                         
#define TIM_DIER_CC2IE_Msk        (0x1U << TIM_DIER_CC2IE_Pos)                 /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk                           /*!<Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE_Pos        (3U)                                         
#define TIM_DIER_CC3IE_Msk        (0x1U << TIM_DIER_CC3IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk                           /*!<Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE_Pos        (4U)                                         
#define TIM_DIER_CC4IE_Msk        (0x1U << TIM_DIER_CC4IE_Pos)                 /*!< 0x00000010 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk                           /*!<Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE_Pos        (5U)                                         
#define TIM_DIER_COMIE_Msk        (0x1U << TIM_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                           /*!<COM interrupt enable */
#define TIM_DIER_TIE_Pos          (6U)                                         
#define TIM_DIER_TIE_Msk          (0x1U << TIM_DIER_TIE_Pos)                   /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                             /*!<Trigger interrupt enable */
#define TIM_DIER_BIE_Pos          (7U)                                         
#define TIM_DIER_BIE_Msk          (0x1U << TIM_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk                             /*!<Break interrupt enable */

/********************  Bit definition for TIM_SR register  *******************/
#define TIM_SR_UIF_Pos            (0U)                                         
#define TIM_SR_UIF_Msk            (0x1U << TIM_SR_UIF_Pos)                     /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                               /*!<Update interrupt Flag */
#define TIM_SR_CC1IF_Pos          (1U)                                         
#define TIM_SR_CC1IF_Msk          (0x1U << TIM_SR_CC1IF_Pos)                   /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk                             /*!<Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF_Pos          (2U)                                         
#define TIM_SR_CC2IF_Msk          (0x1U << TIM_SR_CC2IF_Pos)                   /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk                             /*!<Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF_Pos          (3U)                                         
#define TIM_SR_CC3IF_Msk          (0x1U << TIM_SR_CC3IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk                             /*!<Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF_Pos          (4U)                                         
#define TIM_SR_CC4IF_Msk          (0x1U << TIM_SR_CC4IF_Pos)                   /*!< 0x00000010 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk                             /*!<Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF_Pos          (5U)                                         
#define TIM_SR_COMIF_Msk          (0x1U << TIM_SR_COMIF_Pos)                   /*!< 0x00000020 */
#define TIM_SR_COMIF              TIM_SR_COMIF_Msk                             /*!<COM interrupt Flag */
#define TIM_SR_TIF_Pos            (6U)                                         
#define TIM_SR_TIF_Msk            (0x1U << TIM_SR_TIF_Pos)                     /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk                               /*!<Trigger interrupt Flag */
#define TIM_SR_BIF_Pos            (7U)                                         
#define TIM_SR_BIF_Msk            (0x1U << TIM_SR_BIF_Pos)                     /*!< 0x00000080 */
#define TIM_SR_BIF                TIM_SR_BIF_Msk                               /*!<Break interrupt Flag */
#define TIM_SR_CC1OF_Pos          (9U)                                         
#define TIM_SR_CC1OF_Msk          (0x1U << TIM_SR_CC1OF_Pos)                   /*!< 0x00000200 */
#define TIM_SR_CC1OF              TIM_SR_CC1OF_Msk                             /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos          (10U)                                        
#define TIM_SR_CC2OF_Msk          (0x1U << TIM_SR_CC2OF_Pos)                   /*!< 0x00000400 */
#define TIM_SR_CC2OF              TIM_SR_CC2OF_Msk                             /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos          (11U)                                        
#define TIM_SR_CC3OF_Msk          (0x1U << TIM_SR_CC3OF_Pos)                   /*!< 0x00000800 */
#define TIM_SR_CC3OF              TIM_SR_CC3OF_Msk                             /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos          (12U)                                        
#define TIM_SR_CC4OF_Msk          (0x1U << TIM_SR_CC4OF_Pos)                   /*!< 0x00001000 */
#define TIM_SR_CC4OF              TIM_SR_CC4OF_Msk                             /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  *******************/
#define TIM_EGR_UG_Pos            (0U)                                         
#define TIM_EGR_UG_Msk            (0x1U << TIM_EGR_UG_Pos)                     /*!< 0x00000001 */
#define TIM_EGR_UG                TIM_EGR_UG_Msk                               /*!<Update Generation */
#define TIM_EGR_CC1G_Pos          (1U)                                         
#define TIM_EGR_CC1G_Msk          (0x1U << TIM_EGR_CC1G_Pos)                   /*!< 0x00000002 */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk                             /*!<Capture/Compare 1 Generation */
#define TIM_EGR_CC2G_Pos          (2U)                                         
#define TIM_EGR_CC2G_Msk          (0x1U << TIM_EGR_CC2G_Pos)                   /*!< 0x00000004 */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk                             /*!<Capture/Compare 2 Generation */
#define TIM_EGR_CC3G_Pos          (3U)                                         
#define TIM_EGR_CC3G_Msk          (0x1U << TIM_EGR_CC3G_Pos)                   /*!< 0x00000008 */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk                             /*!<Capture/Compare 3 Generation */
#define TIM_EGR_CC4G_Pos          (4U)                                         
#define TIM_EGR_CC4G_Msk          (0x1U << TIM_EGR_CC4G_Pos)                   /*!< 0x00000010 */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk                             /*!<Capture/Compare 4 Generation */
#define TIM_EGR_COMG_Pos          (5U)                                         
#define TIM_EGR_COMG_Msk          (0x1U << TIM_EGR_COMG_Pos)                   /*!< 0x00000020 */
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk                             /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos            (6U)                                         
#define TIM_EGR_TG_Msk            (0x1U << TIM_EGR_TG_Pos)                     /*!< 0x00000040 */
#define TIM_EGR_TG                TIM_EGR_TG_Msk                               /*!<Trigger Generation */
#define TIM_EGR_BG_Pos            (7U)                                         
#define TIM_EGR_BG_Msk            (0x1U << TIM_EGR_BG_Pos)                     /*!< 0x00000080 */
#define TIM_EGR_BG                TIM_EGR_BG_Msk                               /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  ******************/
#define TIM_CCMR1_CC1S_Pos        (0U)                                         
#define TIM_CCMR1_CC1S_Msk        (0x3U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0          (0x1U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_1          (0x2U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000002 */

#define TIM_CCMR1_OC1FE_Pos       (2U)                                         
#define TIM_CCMR1_OC1FE_Msk       (0x1U << TIM_CCMR1_OC1FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk                          /*!<Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE_Pos       (3U)                                         
#define TIM_CCMR1_OC1PE_Msk       (0x1U << TIM_CCMR1_OC1PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk                          /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos        (4U)                                         
#define TIM_CCMR1_OC1M_Msk        (0x7U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                           /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0          (0x1U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_1          (0x2U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR1_OC1M_2          (0x4U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000040 */

#define TIM_CCMR1_OC1CE_Pos       (7U)                                         
#define TIM_CCMR1_OC1CE_Msk       (0x1U << TIM_CCMR1_OC1CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE           TIM_CCMR1_OC1CE_Msk                          /*!<Output Compare 1Clear Enable */

#define TIM_CCMR1_CC2S_Pos        (8U)                                         
#define TIM_CCMR1_CC2S_Msk        (0x3U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk                           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0          (0x1U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_1          (0x2U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000200 */

#define TIM_CCMR1_OC2FE_Pos       (10U)                                        
#define TIM_CCMR1_OC2FE_Msk       (0x1U << TIM_CCMR1_OC2FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk                          /*!<Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE_Pos       (11U)                                        
#define TIM_CCMR1_OC2PE_Msk       (0x1U << TIM_CCMR1_OC2PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk                          /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos        (12U)                                        
#define TIM_CCMR1_OC2M_Msk        (0x7U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR1_OC2M            TIM_CCMR1_OC2M_Msk                           /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0          (0x1U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_1          (0x2U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_2          (0x4U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00004000 */

#define TIM_CCMR1_OC2CE_Pos       (15U)                                        
#define TIM_CCMR1_OC2CE_Msk       (0x1U << TIM_CCMR1_OC2CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE           TIM_CCMR1_OC2CE_Msk                          /*!<Output Compare 2 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos      (2U)                                         
#define TIM_CCMR1_IC1PSC_Msk      (0x3U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC          TIM_CCMR1_IC1PSC_Msk                         /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0        (0x1U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1        (0x2U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000008 */

#define TIM_CCMR1_IC1F_Pos        (4U)                                         
#define TIM_CCMR1_IC1F_Msk        (0xFU << TIM_CCMR1_IC1F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F            TIM_CCMR1_IC1F_Msk                           /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0          (0x1U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_1          (0x2U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_2          (0x4U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_3          (0x8U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000080 */

#define TIM_CCMR1_IC2PSC_Pos      (10U)                                        
#define TIM_CCMR1_IC2PSC_Msk      (0x3U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC          TIM_CCMR1_IC2PSC_Msk                         /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0        (0x1U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_1        (0x2U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000800 */

#define TIM_CCMR1_IC2F_Pos        (12U)                                        
#define TIM_CCMR1_IC2F_Msk        (0xFU << TIM_CCMR1_IC2F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F            TIM_CCMR1_IC2F_Msk                           /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0          (0x1U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_1          (0x2U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_2          (0x4U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR1_IC2F_3          (0x8U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00008000 */

/******************  Bit definition for TIM_CCMR2 register  ******************/
#define TIM_CCMR2_CC3S_Pos        (0U)                                         
#define TIM_CCMR2_CC3S_Msk        (0x3U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk                           /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0          (0x1U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_1          (0x2U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000002 */

#define TIM_CCMR2_OC3FE_Pos       (2U)                                         
#define TIM_CCMR2_OC3FE_Msk       (0x1U << TIM_CCMR2_OC3FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk                          /*!<Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE_Pos       (3U)                                         
#define TIM_CCMR2_OC3PE_Msk       (0x1U << TIM_CCMR2_OC3PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk                          /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos        (4U)                                         
#define TIM_CCMR2_OC3M_Msk        (0x7U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR2_OC3M            TIM_CCMR2_OC3M_Msk                           /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0          (0x1U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR2_OC3M_1          (0x2U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR2_OC3M_2          (0x4U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000040 */

#define TIM_CCMR2_OC3CE_Pos       (7U)                                         
#define TIM_CCMR2_OC3CE_Msk       (0x1U << TIM_CCMR2_OC3CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE           TIM_CCMR2_OC3CE_Msk                          /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos        (8U)                                         
#define TIM_CCMR2_CC4S_Msk        (0x3U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk                           /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0          (0x1U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_1          (0x2U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000200 */

#define TIM_CCMR2_OC4FE_Pos       (10U)                                        
#define TIM_CCMR2_OC4FE_Msk       (0x1U << TIM_CCMR2_OC4FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk                          /*!<Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE_Pos       (11U)                                        
#define TIM_CCMR2_OC4PE_Msk       (0x1U << TIM_CCMR2_OC4PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk                          /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos        (12U)                                        
#define TIM_CCMR2_OC4M_Msk        (0x7U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR2_OC4M            TIM_CCMR2_OC4M_Msk                           /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0          (0x1U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR2_OC4M_1          (0x2U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR2_OC4M_2          (0x4U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00004000 */

#define TIM_CCMR2_OC4CE_Pos       (15U)                                        
#define TIM_CCMR2_OC4CE_Msk       (0x1U << TIM_CCMR2_OC4CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE           TIM_CCMR2_OC4CE_Msk                          /*!<Output Compare 4 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC_Pos      (2U)                                         
#define TIM_CCMR2_IC3PSC_Msk      (0x3U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC          TIM_CCMR2_IC3PSC_Msk                         /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0        (0x1U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_1        (0x2U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000008 */

#define TIM_CCMR2_IC3F_Pos        (4U)                                         
#define TIM_CCMR2_IC3F_Msk        (0xFU << TIM_CCMR2_IC3F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F            TIM_CCMR2_IC3F_Msk                           /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0          (0x1U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_1          (0x2U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_2          (0x4U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_3          (0x8U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000080 */

#define TIM_CCMR2_IC4PSC_Pos      (10U)                                        
#define TIM_CCMR2_IC4PSC_Msk      (0x3U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC          TIM_CCMR2_IC4PSC_Msk                         /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0        (0x1U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_1        (0x2U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos        (12U)                                        
#define TIM_CCMR2_IC4F_Msk        (0xFU << TIM_CCMR2_IC4F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F            TIM_CCMR2_IC4F_Msk                           /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0          (0x1U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_1          (0x2U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_2          (0x4U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR2_IC4F_3          (0x8U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00008000 */

/*******************  Bit definition for TIM_CCER register  ******************/
#define TIM_CCER_CC1E_Pos         (0U)                                         
#define TIM_CCER_CC1E_Msk         (0x1U << TIM_CCER_CC1E_Pos)                  /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                            /*!<Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos         (1U)                                         
#define TIM_CCER_CC1P_Msk         (0x1U << TIM_CCER_CC1P_Pos)                  /*!< 0x00000002 */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk                            /*!<Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos        (2U)                                         
#define TIM_CCER_CC1NE_Msk        (0x1U << TIM_CCER_CC1NE_Pos)                 /*!< 0x00000004 */
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP_Pos        (3U)                                         
#define TIM_CCER_CC1NP_Msk        (0x1U << TIM_CCER_CC1NP_Pos)                 /*!< 0x00000008 */
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk                           /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos         (4U)                                         
#define TIM_CCER_CC2E_Msk         (0x1U << TIM_CCER_CC2E_Pos)                  /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                            /*!<Capture/Compare 2 output enable */
#define TIM_CCER_CC2P_Pos         (5U)                                         
#define TIM_CCER_CC2P_Msk         (0x1U << TIM_CCER_CC2P_Pos)                  /*!< 0x00000020 */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk                            /*!<Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE_Pos        (6U)                                         
#define TIM_CCER_CC2NE_Msk        (0x1U << TIM_CCER_CC2NE_Pos)                 /*!< 0x00000040 */
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP_Pos        (7U)                                         
#define TIM_CCER_CC2NP_Msk        (0x1U << TIM_CCER_CC2NP_Pos)                 /*!< 0x00000080 */
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk                           /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos         (8U)                                         
#define TIM_CCER_CC3E_Msk         (0x1U << TIM_CCER_CC3E_Pos)                  /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                            /*!<Capture/Compare 3 output enable */
#define TIM_CCER_CC3P_Pos         (9U)                                         
#define TIM_CCER_CC3P_Msk         (0x1U << TIM_CCER_CC3P_Pos)                  /*!< 0x00000200 */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk                            /*!<Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE_Pos        (10U)                                        
#define TIM_CCER_CC3NE_Msk        (0x1U << TIM_CCER_CC3NE_Pos)                 /*!< 0x00000400 */
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP_Pos        (11U)                                        
#define TIM_CCER_CC3NP_Msk        (0x1U << TIM_CCER_CC3NP_Pos)                 /*!< 0x00000800 */
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk                           /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos         (12U)                                        
#define TIM_CCER_CC4E_Msk         (0x1U << TIM_CCER_CC4E_Pos)                  /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                            /*!<Capture/Compare 4 output enable */
#define TIM_CCER_CC4P_Pos         (13U)                                        
#define TIM_CCER_CC4P_Msk         (0x1U << TIM_CCER_CC4P_Pos)                  /*!< 0x00002000 */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk                            /*!<Capture/Compare 4 output Polarity */
#define TIM_CCER_CC4NP_Pos        (15U)                                        
#define TIM_CCER_CC4NP_Msk        (0x1U << TIM_CCER_CC4NP_Pos)                 /*!< 0x00008000 */
#define TIM_CCER_CC4NP            TIM_CCER_CC4NP_Msk                           /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  *******************/
#define TIM_CNT_CNT_Pos           (0U)                                         
#define TIM_CNT_CNT_Msk           (0xFFFFFFFFU << TIM_CNT_CNT_Pos)             /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT               TIM_CNT_CNT_Msk                              /*!<Counter Value */

/*******************  Bit definition for TIM_PSC register  *******************/
#define TIM_PSC_PSC_Pos           (0U)                                         
#define TIM_PSC_PSC_Msk           (0xFFFFU << TIM_PSC_PSC_Pos)                 /*!< 0x0000FFFF */
#define TIM_PSC_PSC               TIM_PSC_PSC_Msk                              /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  *******************/
#define TIM_ARR_ARR_Pos           (0U)                                         
#define TIM_ARR_ARR_Msk           (0xFFFFFFFFU << TIM_ARR_ARR_Pos)             /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR               TIM_ARR_ARR_Msk                              /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  *******************/
#define TIM_RCR_REP_Pos           (0U)                                         
#define TIM_RCR_REP_Msk           (0xFFU << TIM_RCR_REP_Pos)                   /*!< 0x000000FF */
#define TIM_RCR_REP               TIM_RCR_REP_Msk                              /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  ******************/
#define TIM_CCR1_CCR1_Pos         (0U)                                         
#define TIM_CCR1_CCR1_Msk         (0xFFFFU << TIM_CCR1_CCR1_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1             TIM_CCR1_CCR1_Msk                            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  ******************/
#define TIM_CCR2_CCR2_Pos         (0U)                                         
#define TIM_CCR2_CCR2_Msk         (0xFFFFU << TIM_CCR2_CCR2_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2             TIM_CCR2_CCR2_Msk                            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  ******************/
#define TIM_CCR3_CCR3_Pos         (0U)                                         
#define TIM_CCR3_CCR3_Msk         (0xFFFFU << TIM_CCR3_CCR3_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3             TIM_CCR3_CCR3_Msk                            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  ******************/
#define TIM_CCR4_CCR4_Pos         (0U)                                         
#define TIM_CCR4_CCR4_Msk         (0xFFFFU << TIM_CCR4_CCR4_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4             TIM_CCR4_CCR4_Msk                            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  ******************/
#define TIM_BDTR_DTG_Pos          (0U)                                         
#define TIM_BDTR_DTG_Msk          (0xFFU << TIM_BDTR_DTG_Pos)                  /*!< 0x000000FF */
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk                             /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0            (0x01U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000001 */
#define TIM_BDTR_DTG_1            (0x02U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000002 */
#define TIM_BDTR_DTG_2            (0x04U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000004 */
#define TIM_BDTR_DTG_3            (0x08U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000008 */
#define TIM_BDTR_DTG_4            (0x10U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000010 */
#define TIM_BDTR_DTG_5            (0x20U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000020 */
#define TIM_BDTR_DTG_6            (0x40U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000040 */
#define TIM_BDTR_DTG_7            (0x80U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos         (8U)                                         
#define TIM_BDTR_LOCK_Msk         (0x3U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000300 */
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk                            /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0           (0x1U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1           (0x2U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos         (10U)                                        
#define TIM_BDTR_OSSI_Msk         (0x1U << TIM_BDTR_OSSI_Pos)                  /*!< 0x00000400 */
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk                            /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos         (11U)                                        
#define TIM_BDTR_OSSR_Msk         (0x1U << TIM_BDTR_OSSR_Pos)                  /*!< 0x00000800 */
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk                            /*!<Off-State Selection for Run mode */
#define TIM_BDTR_BKE_Pos          (12U)                                        
#define TIM_BDTR_BKE_Msk          (0x1U << TIM_BDTR_BKE_Pos)                   /*!< 0x00001000 */
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk                             /*!<Break enable */
#define TIM_BDTR_BKP_Pos          (13U)                                        
#define TIM_BDTR_BKP_Msk          (0x1U << TIM_BDTR_BKP_Pos)                   /*!< 0x00002000 */
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk                             /*!<Break Polarity */
#define TIM_BDTR_AOE_Pos          (14U)                                        
#define TIM_BDTR_AOE_Msk          (0x1U << TIM_BDTR_AOE_Pos)                   /*!< 0x00004000 */
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk                             /*!<Automatic Output enable */
#define TIM_BDTR_MOE_Pos          (15U)                                        
#define TIM_BDTR_MOE_Msk          (0x1U << TIM_BDTR_MOE_Pos)                   /*!< 0x00008000 */
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk                             /*!<Main Output enable */


/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_UE_Pos              (0U)                                     
#define USART_CR1_UE_Msk              (0x1U << USART_CR1_UE_Pos)               /*!< 0x00000001 */
#define USART_CR1_UE                  USART_CR1_UE_Msk                         /*!< USART Enable */
#define USART_CR1_UESM_Pos						(1U)																		 
#define USART_CR1_UESM_Msk            (0x1U<<USART_CR1_UESM_Pos)
#define USART_CR1_UESM                USART_CR1_UESM_Msk
#define USART_CR1_RE_Pos              (2U)                                     
#define USART_CR1_RE_Msk              (0x1U << USART_CR1_RE_Pos)               /*!< 0x00000004 */
#define USART_CR1_RE                  USART_CR1_RE_Msk                         /*!< Receiver Enable */
#define USART_CR1_TE_Pos              (3U)                                     
#define USART_CR1_TE_Msk              (0x1U << USART_CR1_TE_Pos)               /*!< 0x00000008 */
#define USART_CR1_TE                  USART_CR1_TE_Msk                         /*!< Transmitter Enable */
#define USART_CR1_IDLEIE_Pos          (4U)                                     
#define USART_CR1_IDLEIE_Msk          (0x1U << USART_CR1_IDLEIE_Pos)           /*!< 0x00000010 */
#define USART_CR1_IDLEIE              USART_CR1_IDLEIE_Msk                     /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE_Pos          (5U)                                     
#define USART_CR1_RXNEIE_Msk          (0x1U << USART_CR1_RXNEIE_Pos)           /*!< 0x00000020 */
#define USART_CR1_RXNEIE              USART_CR1_RXNEIE_Msk                     /*!< RXNE Interrupt Enable */
#define USART_CR1_TCIE_Pos            (6U)                                     
#define USART_CR1_TCIE_Msk            (0x1U << USART_CR1_TCIE_Pos)             /*!< 0x00000040 */
#define USART_CR1_TCIE                USART_CR1_TCIE_Msk                       /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_Pos           (7U)                                     
#define USART_CR1_TXEIE_Msk           (0x1U << USART_CR1_TXEIE_Pos)            /*!< 0x00000080 */
#define USART_CR1_TXEIE               USART_CR1_TXEIE_Msk                      /*!< TXE Interrupt Enable */
#define USART_CR1_PEIE_Pos            (8U)                                     
#define USART_CR1_PEIE_Msk            (0x1U << USART_CR1_PEIE_Pos)             /*!< 0x00000100 */
#define USART_CR1_PEIE                USART_CR1_PEIE_Msk                       /*!< PE Interrupt Enable */
#define USART_CR1_PS_Pos              (9U)                                     
#define USART_CR1_PS_Msk              (0x1U << USART_CR1_PS_Pos)               /*!< 0x00000200 */
#define USART_CR1_PS                  USART_CR1_PS_Msk                         /*!< Parity Selection */
#define USART_CR1_PCE_Pos             (10U)                                    
#define USART_CR1_PCE_Msk             (0x1U << USART_CR1_PCE_Pos)              /*!< 0x00000400 */
#define USART_CR1_PCE                 USART_CR1_PCE_Msk                        /*!< Parity Control Enable */
#define USART_CR1_WAKE_Pos            (11U)                                    
#define USART_CR1_WAKE_Msk            (0x1U << USART_CR1_WAKE_Pos)             /*!< 0x00000800 */
#define USART_CR1_WAKE                USART_CR1_WAKE_Msk                       /*!< Receiver Wakeup method */
#define USART_CR1_M_Pos               (12U)                                    
#define USART_CR1_M_Msk               (0x1U << USART_CR1_M_Pos)                /*!< 0x00001000 */
#define USART_CR1_M                   USART_CR1_M_Msk                          /*!< Word Length */
#define USART_CR1_MME_Pos             (13U)                                    
#define USART_CR1_MME_Msk             (0x1U << USART_CR1_MME_Pos)              /*!< 0x00002000 */
#define USART_CR1_MME                 USART_CR1_MME_Msk                        /*!< Mute Mode Enable */
#define USART_CR1_CMIE_Pos            (14U)                                    
#define USART_CR1_CMIE_Msk            (0x1U << USART_CR1_CMIE_Pos)             /*!< 0x00004000 */
#define USART_CR1_CMIE                USART_CR1_CMIE_Msk                       /*!< Character match interrupt enable */
#define USART_CR1_OVER8_Pos           (15U)                                    
#define USART_CR1_OVER8_Msk           (0x1U << USART_CR1_OVER8_Pos)            /*!< 0x00008000 */
#define USART_CR1_OVER8               USART_CR1_OVER8_Msk                      /*!< Oversampling by 8-bit or 16-bit mode */
#define USART_CR1_DEDT_Pos            (16U)                                    
#define USART_CR1_DEDT_Msk            (0x1FU << USART_CR1_DEDT_Pos)            /*!< 0x001F0000 */
#define USART_CR1_DEDT                USART_CR1_DEDT_Msk                       /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define USART_CR1_DEDT_0              (0x01U << USART_CR1_DEDT_Pos)            /*!< 0x00010000 */
#define USART_CR1_DEDT_1              (0x02U << USART_CR1_DEDT_Pos)            /*!< 0x00020000 */
#define USART_CR1_DEDT_2              (0x04U << USART_CR1_DEDT_Pos)            /*!< 0x00040000 */
#define USART_CR1_DEDT_3              (0x08U << USART_CR1_DEDT_Pos)            /*!< 0x00080000 */
#define USART_CR1_DEDT_4              (0x10U << USART_CR1_DEDT_Pos)            /*!< 0x00100000 */
#define USART_CR1_DEAT_Pos            (21U)                                    
#define USART_CR1_DEAT_Msk            (0x1FU << USART_CR1_DEAT_Pos)            /*!< 0x03E00000 */
#define USART_CR1_DEAT                USART_CR1_DEAT_Msk                       /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define USART_CR1_DEAT_0              (0x01U << USART_CR1_DEAT_Pos)            /*!< 0x00200000 */
#define USART_CR1_DEAT_1              (0x02U << USART_CR1_DEAT_Pos)            /*!< 0x00400000 */
#define USART_CR1_DEAT_2              (0x04U << USART_CR1_DEAT_Pos)            /*!< 0x00800000 */
#define USART_CR1_DEAT_3              (0x08U << USART_CR1_DEAT_Pos)            /*!< 0x01000000 */
#define USART_CR1_DEAT_4              (0x10U << USART_CR1_DEAT_Pos)            /*!< 0x02000000 */
#define USART_CR1_RTOIE_Pos           (26U)                                    
#define USART_CR1_RTOIE_Msk           (0x1U << USART_CR1_RTOIE_Pos)            /*!< 0x04000000 */
#define USART_CR1_RTOIE               USART_CR1_RTOIE_Msk                      /*!< Receive Time Out interrupt enable */
#define USART_CR1_EOBIE_Pos           (27U)                                    
#define USART_CR1_EOBIE_Msk           (0x1U << USART_CR1_EOBIE_Pos)            /*!< 0x08000000 */
#define USART_CR1_EOBIE               USART_CR1_EOBIE_Msk                      /*!< End of Block interrupt enable */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADDM7_Pos           (4U)                                     
#define USART_CR2_ADDM7_Msk           (0x1U << USART_CR2_ADDM7_Pos)            /*!< 0x00000010 */
#define USART_CR2_ADDM7               USART_CR2_ADDM7_Msk                      /*!< 7-bit or 4-bit Address Detection */
#define USART_CR2_LBDL_Pos            (5U)
#define USART_CR2_LBDL_Msk            (0x1U<<USART_CR2_LBDL_Pos)               /*!< 0x00000020 */
#define USART_CR2_LBDL                USART_CR2_LBDL_Msk
#define USART_CR2_LBCL_Pos            (8U)                                     
#define USART_CR2_LBCL_Msk            (0x1U << USART_CR2_LBCL_Pos)             /*!< 0x00000100 */
#define USART_CR2_LBCL                USART_CR2_LBCL_Msk                       /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA_Pos            (9U)                                     
#define USART_CR2_CPHA_Msk            (0x1U << USART_CR2_CPHA_Pos)             /*!< 0x00000200 */
#define USART_CR2_CPHA                USART_CR2_CPHA_Msk                       /*!< Clock Phase */
#define USART_CR2_CPOL_Pos            (10U)                                    
#define USART_CR2_CPOL_Msk            (0x1U << USART_CR2_CPOL_Pos)             /*!< 0x00000400 */
#define USART_CR2_CPOL                USART_CR2_CPOL_Msk                       /*!< Clock Polarity */
#define USART_CR2_CLKEN_Pos           (11U)                                    
#define USART_CR2_CLKEN_Msk           (0x1U << USART_CR2_CLKEN_Pos)            /*!< 0x00000800 */
#define USART_CR2_CLKEN               USART_CR2_CLKEN_Msk                      /*!< Clock Enable */
#define USART_CR2_STOP_Pos            (12U)                                    
#define USART_CR2_STOP_Msk            (0x3U << USART_CR2_STOP_Pos)             /*!< 0x00003000 */
#define USART_CR2_STOP                USART_CR2_STOP_Msk                       /*!< STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0              (0x1U << USART_CR2_STOP_Pos)             /*!< 0x00001000 */
#define USART_CR2_STOP_1              (0x2U << USART_CR2_STOP_Pos)             /*!< 0x00002000 */
#define USART_CR2_LINEN_Pos           (14U)
#define USART_CR2_LINEN_Msk           (0x1U<<USART_CR2_LINEN_Pos)
#define USART_CR2_LINEN               USART_CR2_LINEN_Msk
#define USART_CR2_SWAP_Pos            (15U)                                    
#define USART_CR2_SWAP_Msk            (0x1U << USART_CR2_SWAP_Pos)             /*!< 0x00008000 */
#define USART_CR2_SWAP                USART_CR2_SWAP_Msk                       /*!< SWAP TX/RX pins */
#define USART_CR2_RXINV_Pos           (16U)                                    
#define USART_CR2_RXINV_Msk           (0x1U << USART_CR2_RXINV_Pos)            /*!< 0x00010000 */
#define USART_CR2_RXINV               USART_CR2_RXINV_Msk                      /*!< RX pin active level inversion */
#define USART_CR2_TXINV_Pos           (17U)                                    
#define USART_CR2_TXINV_Msk           (0x1U << USART_CR2_TXINV_Pos)            /*!< 0x00020000 */
#define USART_CR2_TXINV               USART_CR2_TXINV_Msk                      /*!< TX pin active level inversion */
#define USART_CR2_DATAINV_Pos         (18U)                                    
#define USART_CR2_DATAINV_Msk         (0x1U << USART_CR2_DATAINV_Pos)          /*!< 0x00040000 */
#define USART_CR2_DATAINV             USART_CR2_DATAINV_Msk                    /*!< Binary data inversion */
#define USART_CR2_MSBFIRST_Pos        (19U)                                    
#define USART_CR2_MSBFIRST_Msk        (0x1U << USART_CR2_MSBFIRST_Pos)         /*!< 0x00080000 */
#define USART_CR2_MSBFIRST            USART_CR2_MSBFIRST_Msk                   /*!< Most Significant Bit First */
#define USART_CR2_ABREN_Pos           (20U)                                    
#define USART_CR2_ABREN_Msk           (0x1U << USART_CR2_ABREN_Pos)            /*!< 0x00100000 */
#define USART_CR2_ABREN               USART_CR2_ABREN_Msk                      /*!< Auto Baud-Rate Enable*/
#define USART_CR2_ABRMODE_Pos         (21U)                                    
#define USART_CR2_ABRMODE_Msk         (0x3U << USART_CR2_ABRMODE_Pos)          /*!< 0x00600000 */
#define USART_CR2_ABRMODE             USART_CR2_ABRMODE_Msk                    /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define USART_CR2_ABRMODE_0           (0x1U << USART_CR2_ABRMODE_Pos)          /*!< 0x00200000 */
#define USART_CR2_ABRMODE_1           (0x2U << USART_CR2_ABRMODE_Pos)          /*!< 0x00400000 */
#define USART_CR2_RTOEN_Pos           (23U)                                    
#define USART_CR2_RTOEN_Msk           (0x1U << USART_CR2_RTOEN_Pos)            /*!< 0x00800000 */
#define USART_CR2_RTOEN               USART_CR2_RTOEN_Msk                      /*!< Receiver Time-Out enable */
#define USART_CR2_ADD_Pos             (24U)                                    
#define USART_CR2_ADD_Msk             (0xFFU << USART_CR2_ADD_Pos)             /*!< 0xFF000000 */
#define USART_CR2_ADD                 USART_CR2_ADD_Msk                        /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos             (0U)                                     
#define USART_CR3_EIE_Msk             (0x1U << USART_CR3_EIE_Pos)              /*!< 0x00000001 */
#define USART_CR3_EIE                 USART_CR3_EIE_Msk                        /*!< Error Interrupt Enable */
#define USART_CR3_IREN_Poss           (1U)
#define USART_CR3_IREN_Msk            (0x1U << USART_CR3_IREN_Poss)
#define USART_CR3_IREN                USART_CR3_IREN_Msk
#define USART_CR3_IRLP_Poss           (2U)
#define USART_CR3_IRLP_Msk            (0x1U << USART_CR3_IRLP_Poss)
#define USART_CR3_IRLP                USART_CR3_IRLP_Msk
#define USART_CR3_HDSEL_Pos           (3U)                                     
#define USART_CR3_HDSEL_Msk           (0x1U << USART_CR3_HDSEL_Pos)            /*!< 0x00000008 */
#define USART_CR3_HDSEL               USART_CR3_HDSEL_Msk                      /*!< Half-Duplex Selection */
#define USART_CR3_NACK_Pos            (4U)
#define USART_CR3_NACK_Msk            (0x1U << USART_CR3_NACK_Pos)
#define USART_CR3_NACK                USART_CR3_NACK_Msk
#define USART_CR3_SCEN_Pos            (5U)
#define USART_CR3_SCEN_Msk            (0x01U << USART_CR3_SCEN_Pos)
#define USART_CR3_SCEN                USART_CR3_SCEN_Msk
#define USART_CR3_RTSE_Pos            (8U)                                     
#define USART_CR3_RTSE_Msk            (0x1U << USART_CR3_RTSE_Pos)             /*!< 0x00000100 */
#define USART_CR3_RTSE                USART_CR3_RTSE_Msk                       /*!< RTS Enable */
#define USART_CR3_CTSE_Pos            (9U)                                     
#define USART_CR3_CTSE_Msk            (0x1U << USART_CR3_CTSE_Pos)             /*!< 0x00000200 */
#define USART_CR3_CTSE                USART_CR3_CTSE_Msk                       /*!< CTS Enable */
#define USART_CR3_CTSIE_Pos           (10U)                                    
#define USART_CR3_CTSIE_Msk           (0x1U << USART_CR3_CTSIE_Pos)            /*!< 0x00000400 */
#define USART_CR3_CTSIE               USART_CR3_CTSIE_Msk                      /*!< CTS Interrupt Enable */
#define USART_CR3_ONEBIT_Pos          (11U)                                    
#define USART_CR3_ONEBIT_Msk          (0x1U << USART_CR3_ONEBIT_Pos)           /*!< 0x00000800 */
#define USART_CR3_ONEBIT              USART_CR3_ONEBIT_Msk                     /*!< One sample bit method enable */
#define USART_CR3_OVRDIS_Pos          (12U)                                    
#define USART_CR3_OVRDIS_Msk          (0x1U << USART_CR3_OVRDIS_Pos)           /*!< 0x00001000 */
#define USART_CR3_OVRDIS              USART_CR3_OVRDIS_Msk                     /*!< Overrun Disable */
#define USART_CR3_DEM_Pos             (14U)                                    
#define USART_CR3_DEM_Msk             (0x1U << USART_CR3_DEM_Pos)              /*!< 0x00004000 */
#define USART_CR3_DEM                 USART_CR3_DEM_Msk                        /*!< Driver Enable Mode */
#define USART_CR3_DEP_Pos             (15U)                                    
#define USART_CR3_DEP_Msk             (0x1U << USART_CR3_DEP_Pos)              /*!< 0x00008000 */
#define USART_CR3_DEP                 USART_CR3_DEP_Msk                        /*!< Driver Enable Polarity Selection */
#define USART_CR3_SCARCNT_Pos         (17U)
#define USART_CR3_SCARCNT_Msk         (0x07<<USART_CR3_SCARCNT_Pos)
#define USART_CR3_SCARCNT             USART_CR3_SCARCNT_Msk
#define USART_CR3_SCARCNT_0           (0x01<<USART_CR3_SCARCNT_Pos)
#define USART_CR3_SCARCNT_1           (0x02<<USART_CR3_SCARCNT_Pos)
#define USART_CR3_SCARCNT_2           (0x04<<USART_CR3_SCARCNT_Pos)
#define USART_CR3_WUS_Pos             (20U)
#define USART_CR3_WUS_Msk             (0x03<<USART_CR3_WUS_Pos)								 /*!< 0x00300000 */
#define USART_CR3_WUS                 USART_CR3_WUS_Msk												 /*!< WUS[21:20] */
#define USART_CR3_WUS_0               (0x01<<USART_CR3_WUS_Pos)
#define USART_CR3_WUS_1               (0x02<<USART_CR3_WUS_Pos)
/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_FRACTION_Pos    (0U)                                     
#define USART_BRR_DIV_FRACTION_Msk    (0xFU << USART_BRR_DIV_FRACTION_Pos)     /*!< 0x0000000F */
#define USART_BRR_DIV_FRACTION        USART_BRR_DIV_FRACTION_Msk               /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_MANTISSA_Pos    (4U)                                     
#define USART_BRR_DIV_MANTISSA_Msk    (0xFFFU << USART_BRR_DIV_MANTISSA_Pos)   /*!< 0x0000FFF0 */
#define USART_BRR_DIV_MANTISSA        USART_BRR_DIV_MANTISSA_Msk               /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos            (0U)                                     
#define USART_GTPR_PSC_Msk            (0xFFU << USART_GTPR_PSC_Pos)            /*!< 0x000000FF */
#define USART_GTPR_PSC                USART_GTPR_PSC_Msk                       /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_GT_Pos             (8U)                                     
#define USART_GTPR_GT_Msk             (0xFFU << USART_GTPR_GT_Pos)             /*!< 0x0000FF00 */
#define USART_GTPR_GT                 USART_GTPR_GT_Msk                        /*!< GT[7:0] bits (Guard time value) */


/*******************  Bit definition for USART_RTOR register  *****************/
#define USART_RTOR_RTO_Pos            (0U)                                     
#define USART_RTOR_RTO_Msk            (0xFFFFFFU << USART_RTOR_RTO_Pos)        /*!< 0x00FFFFFF */
#define USART_RTOR_RTO                USART_RTOR_RTO_Msk                       /*!< Receiver Time Out Value */
#define USART_RTOR_BLEN_Pos           (24U)                                    
#define USART_RTOR_BLEN_Msk           (0xFFU << USART_RTOR_BLEN_Pos)           /*!< 0xFF000000 */
#define USART_RTOR_BLEN               USART_RTOR_BLEN_Msk                      /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define USART_RQR_ABRRQ_Pos           (0U)                                     
#define USART_RQR_ABRRQ_Msk           (0x1U << USART_RQR_ABRRQ_Pos)            /*!< 0x00000001 */
#define USART_RQR_ABRRQ               USART_RQR_ABRRQ_Msk                      /*!< Auto-Baud Rate Request */
#define USART_RQR_SBKRQ_Pos           (1U)                                     
#define USART_RQR_SBKRQ_Msk           (0x1U << USART_RQR_SBKRQ_Pos)            /*!< 0x00000002 */
#define USART_RQR_SBKRQ               USART_RQR_SBKRQ_Msk                      /*!< Send Break Request */
#define USART_RQR_MMRQ_Pos            (2U)                                     
#define USART_RQR_MMRQ_Msk            (0x1U << USART_RQR_MMRQ_Pos)             /*!< 0x00000004 */
#define USART_RQR_MMRQ                USART_RQR_MMRQ_Msk                       /*!< Mute Mode Request */
#define USART_RQR_RXFRQ_Pos           (3U)                                     
#define USART_RQR_RXFRQ_Msk           (0x1U << USART_RQR_RXFRQ_Pos)            /*!< 0x00000008 */
#define USART_RQR_RXFRQ               USART_RQR_RXFRQ_Msk                      /*!< Receive Data flush Request */
#define USART_RQR_TXFRQ_Pos           (4U)                                     
#define USART_RQR_TXFRQ_Msk           (0x1U << USART_RQR_TXFRQ_Pos)            /*!< 0x00000010 */
#define USART_RQR_TXFRQ               USART_RQR_TXFRQ_Msk                      /*!< Transmit Data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define USART_ISR_PE_Pos              (0U)                                     
#define USART_ISR_PE_Msk              (0x1U << USART_ISR_PE_Pos)               /*!< 0x00000001 */
#define USART_ISR_PE                  USART_ISR_PE_Msk                         /*!< Parity Error */
#define USART_ISR_FE_Pos              (1U)                                     
#define USART_ISR_FE_Msk              (0x1U << USART_ISR_FE_Pos)               /*!< 0x00000002 */
#define USART_ISR_FE                  USART_ISR_FE_Msk                         /*!< Framing Error */
#define USART_ISR_NE_Pos              (2U)                                     
#define USART_ISR_NE_Msk              (0x1U << USART_ISR_NE_Pos)               /*!< 0x00000004 */
#define USART_ISR_NE                  USART_ISR_NE_Msk                         /*!< Noise detected Flag */
#define USART_ISR_ORE_Pos             (3U)                                     
#define USART_ISR_ORE_Msk             (0x1U << USART_ISR_ORE_Pos)              /*!< 0x00000008 */
#define USART_ISR_ORE                 USART_ISR_ORE_Msk                        /*!< OverRun Error */
#define USART_ISR_IDLE_Pos            (4U)                                     
#define USART_ISR_IDLE_Msk            (0x1U << USART_ISR_IDLE_Pos)             /*!< 0x00000010 */
#define USART_ISR_IDLE                USART_ISR_IDLE_Msk                       /*!< IDLE line detected */
#define USART_ISR_RXNE_Pos            (5U)                                     
#define USART_ISR_RXNE_Msk            (0x1U << USART_ISR_RXNE_Pos)             /*!< 0x00000020 */
#define USART_ISR_RXNE                USART_ISR_RXNE_Msk                       /*!< Read Data Register Not Empty */
#define USART_ISR_TC_Pos              (6U)                                     
#define USART_ISR_TC_Msk              (0x1U << USART_ISR_TC_Pos)               /*!< 0x00000040 */
#define USART_ISR_TC                  USART_ISR_TC_Msk                         /*!< Transmission Complete */
#define USART_ISR_TXE_Pos             (7U)                                     
#define USART_ISR_TXE_Msk             (0x1U << USART_ISR_TXE_Pos)              /*!< 0x00000080 */
#define USART_ISR_TXE                 USART_ISR_TXE_Msk                        /*!< Transmit Data Register Empty */
#define USART_ISR_LBDF_Pos             (8U)                                     
#define USART_ISR_LBDF_Msk             (0x1U << USART_ISR_LBDF_Pos)              /*!< 0x00000080 */
#define USART_ISR_LBDF                 USART_ISR_LBDF_Msk                        /*!< LIN break detect flag */
#define USART_ISR_CTSIF_Pos           (9U)                                     
#define USART_ISR_CTSIF_Msk           (0x1U << USART_ISR_CTSIF_Pos)            /*!< 0x00000200 */
#define USART_ISR_CTSIF               USART_ISR_CTSIF_Msk                      /*!< CTS interrupt flag */
#define USART_ISR_CTS_Pos             (10U)                                    
#define USART_ISR_CTS_Msk             (0x1U << USART_ISR_CTS_Pos)              /*!< 0x00000400 */
#define USART_ISR_CTS                 USART_ISR_CTS_Msk                        /*!< CTS flag */
#define USART_ISR_RTOF_Pos            (11U)                                    
#define USART_ISR_RTOF_Msk            (0x1U << USART_ISR_RTOF_Pos)             /*!< 0x00000800 */
#define USART_ISR_RTOF                USART_ISR_RTOF_Msk                       /*!< Receiver Time Out */
#define USART_ISR_EOBF_Pos            (12U)                                    
#define USART_ISR_EOBF_Msk            (0x1U << USART_ISR_EOBF_Pos)             /*!< 0x00001000 */
#define USART_ISR_EOBF                USART_ISR_EOBF_Msk                       /*!< End of block flag */
#define USART_ISR_ABRE_Pos            (14U)                                    
#define USART_ISR_ABRE_Msk            (0x1U << USART_ISR_ABRE_Pos)             /*!< 0x00004000 */
#define USART_ISR_ABRE                USART_ISR_ABRE_Msk                       /*!< Auto-Baud Rate Error */
#define USART_ISR_ABRF_Pos            (15U)                                    
#define USART_ISR_ABRF_Msk            (0x1U << USART_ISR_ABRF_Pos)             /*!< 0x00008000 */
#define USART_ISR_ABRF                USART_ISR_ABRF_Msk                       /*!< Auto-Baud Rate Flag */
#define USART_ISR_BUSY_Pos            (16U)                                    
#define USART_ISR_BUSY_Msk            (0x1U << USART_ISR_BUSY_Pos)             /*!< 0x00010000 */
#define USART_ISR_BUSY                USART_ISR_BUSY_Msk                       /*!< Busy Flag */
#define USART_ISR_CMF_Pos             (17U)                                    
#define USART_ISR_CMF_Msk             (0x1U << USART_ISR_CMF_Pos)              /*!< 0x00020000 */
#define USART_ISR_CMF                 USART_ISR_CMF_Msk                        /*!< Character Match Flag */
#define USART_ISR_SBKF_Pos            (18U)                                    
#define USART_ISR_SBKF_Msk            (0x1U << USART_ISR_SBKF_Pos)             /*!< 0x00040000 */
#define USART_ISR_SBKF                USART_ISR_SBKF_Msk                       /*!< Send Break Flag */
#define USART_ISR_RWU_Pos             (19U)                                    
#define USART_ISR_RWU_Msk             (0x1U << USART_ISR_RWU_Pos)              /*!< 0x00080000 */
#define USART_ISR_RWU                 USART_ISR_RWU_Msk                        /*!< Receive Wake Up from mute mode Flag */
#define USART_ISR_WUF_Pos             (20U)                                    
#define USART_ISR_WUF_Msk             (0x1U << USART_ISR_WUF_Pos)              /*!< 0x00080000 */
#define USART_ISR_WUF                USART_ISR_WUF_Msk                        /*!< Receive Wakeup from Stop mode flag */
#define USART_ISR_TEACK_Pos           (21U)                                    
#define USART_ISR_TEACK_Msk           (0x1U << USART_ISR_TEACK_Pos)            /*!< 0x00200000 */
#define USART_ISR_TEACK               USART_ISR_TEACK_Msk                      /*!< Transmit Enable Acknowledge Flag */
#define USART_ISR_REACK_Pos           (22U)                                    
#define USART_ISR_REACK_Msk           (0x1U << USART_ISR_REACK_Pos)            /*!< 0x00400000 */
#define USART_ISR_REACK               USART_ISR_REACK_Msk                      /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define USART_ICR_PECF_Pos            (0U)                                     
#define USART_ICR_PECF_Msk            (0x1U << USART_ICR_PECF_Pos)             /*!< 0x00000001 */
#define USART_ICR_PECF                USART_ICR_PECF_Msk                       /*!< Parity Error Clear Flag */
#define USART_ICR_FECF_Pos            (1U)                                     
#define USART_ICR_FECF_Msk            (0x1U << USART_ICR_FECF_Pos)             /*!< 0x00000002 */
#define USART_ICR_FECF                USART_ICR_FECF_Msk                       /*!< Framing Error Clear Flag */
#define USART_ICR_NCF_Pos             (2U)                                     
#define USART_ICR_NCF_Msk             (0x1U << USART_ICR_NCF_Pos)              /*!< 0x00000004 */
#define USART_ICR_NCF                 USART_ICR_NCF_Msk                        /*!< Noise detected Clear Flag */
#define USART_ICR_ORECF_Pos           (3U)                                     
#define USART_ICR_ORECF_Msk           (0x1U << USART_ICR_ORECF_Pos)            /*!< 0x00000008 */
#define USART_ICR_ORECF               USART_ICR_ORECF_Msk                      /*!< OverRun Error Clear Flag */
#define USART_ICR_IDLECF_Pos          (4U)                                     
#define USART_ICR_IDLECF_Msk          (0x1U << USART_ICR_IDLECF_Pos)           /*!< 0x00000010 */
#define USART_ICR_IDLECF              USART_ICR_IDLECF_Msk                     /*!< IDLE line detected Clear Flag */
#define USART_ICR_TCCF_Pos            (6U)                                     
#define USART_ICR_TCCF_Msk            (0x1U << USART_ICR_TCCF_Pos)             /*!< 0x00000040 */
#define USART_ICR_TCCF                USART_ICR_TCCF_Msk                       /*!< Transmission Complete Clear Flag */
#define USART_ICR_CTSCF_Pos           (9U)                                     
#define USART_ICR_CTSCF_Msk           (0x1U << USART_ICR_CTSCF_Pos)            /*!< 0x00000200 */
#define USART_ICR_CTSCF               USART_ICR_CTSCF_Msk                      /*!< CTS Interrupt Clear Flag */
#define USART_ICR_RTOCF_Pos           (11U)                                    
#define USART_ICR_RTOCF_Msk           (0x1U << USART_ICR_RTOCF_Pos)            /*!< 0x00000800 */
#define USART_ICR_RTOCF               USART_ICR_RTOCF_Msk                      /*!< Receiver Time Out Clear Flag */
#define USART_ICR_CMCF_Pos            (17U)                                    
#define USART_ICR_CMCF_Msk            (0x1U << USART_ICR_CMCF_Pos)             /*!< 0x00020000 */
#define USART_ICR_CMCF                USART_ICR_CMCF_Msk                       /*!< Character Match Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define USART_RDR_RDR                 ((uint16_t)0x01FFU)                      /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define USART_TDR_TDR                 ((uint16_t)0x01FFU)                      /*!< TDR[8:0] bits (Transmit Data value) */

/*******************Bit definition for BEEP_CFGR register ************************/
#define BEEP_CFGR_CKSEL_Pos                (0U)
#define BEEP_CFGR_CKSEL_Msk                (0x01U << BEEP_CFGR_CKSEL_Pos)
#define BEEP_CFGR_CKSEL                    BEEP_CFGR_CKSEL_Msk
#define BEEP_CFGR_FREQ_Pos                 (1U)
#define BEEP_CFGR_FREQ_Msk                 (0x03U << BEEP_CFGR_FREQ_Pos)
#define BEEP_CFGR_FREQ                     BEEP_CFGR_FREQ_Msk
#define BEEP_CFGR_PRE_Pos                  (3U)
#define BEEP_CFGR_PRE_Msk                  (0x03U << BEEP_CFGR_PRE_Pos)
#define BEEP_CFGR_PRE                      BEEP_CFGR_PRE_Msk
/*******************Bit definition for BEEP_CR register ************************/
#define BEEP_CR_EN_Pos                      (0U)
#define BEEP_CR_EN_Msk                      (0x01U << BEEP_CR_EN_Pos)
#define BEEP_CR_EN                          BEEP_CR_EN_Msk
#define BEEP_CR_TRGO_Pos                    (1U)
#define BEEP_CR_TRGO_Msk                    (0x01U << BEEP_CR_TRGO_Pos)
#define BEEP_CR_TRGO                        BEEP_CR_TRGO_Msk


/******************************************************************************/
/*                                                                            */
/*                         Window WATCHDOG (WWDG)                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CR register  ********************/
#define WWDG_CR_T_Pos           (0U)                                           
#define WWDG_CR_T_Msk           (0x7FU << WWDG_CR_T_Pos)                       /*!< 0x0000007F */
#define WWDG_CR_T               WWDG_CR_T_Msk                                  /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CR_T_0             (0x01U << WWDG_CR_T_Pos)                       /*!< 0x00000001 */
#define WWDG_CR_T_1             (0x02U << WWDG_CR_T_Pos)                       /*!< 0x00000002 */
#define WWDG_CR_T_2             (0x04U << WWDG_CR_T_Pos)                       /*!< 0x00000004 */
#define WWDG_CR_T_3             (0x08U << WWDG_CR_T_Pos)                       /*!< 0x00000008 */
#define WWDG_CR_T_4             (0x10U << WWDG_CR_T_Pos)                       /*!< 0x00000010 */
#define WWDG_CR_T_5             (0x20U << WWDG_CR_T_Pos)                       /*!< 0x00000020 */
#define WWDG_CR_T_6             (0x40U << WWDG_CR_T_Pos)                       /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CR_T0 WWDG_CR_T_0
#define  WWDG_CR_T1 WWDG_CR_T_1
#define  WWDG_CR_T2 WWDG_CR_T_2
#define  WWDG_CR_T3 WWDG_CR_T_3
#define  WWDG_CR_T4 WWDG_CR_T_4
#define  WWDG_CR_T5 WWDG_CR_T_5
#define  WWDG_CR_T6 WWDG_CR_T_6

#define WWDG_CR_WDGA_Pos        (7U)                                           
#define WWDG_CR_WDGA_Msk        (0x1U << WWDG_CR_WDGA_Pos)                     /*!< 0x00000080 */
#define WWDG_CR_WDGA            WWDG_CR_WDGA_Msk                               /*!< Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define WWDG_CFR_W_Pos          (0U)                                           
#define WWDG_CFR_W_Msk          (0x7FU << WWDG_CFR_W_Pos)                      /*!< 0x0000007F */
#define WWDG_CFR_W              WWDG_CFR_W_Msk                                 /*!< W[6:0] bits (7-bit window value) */
#define WWDG_CFR_W_0            (0x01U << WWDG_CFR_W_Pos)                      /*!< 0x00000001 */
#define WWDG_CFR_W_1            (0x02U << WWDG_CFR_W_Pos)                      /*!< 0x00000002 */
#define WWDG_CFR_W_2            (0x04U << WWDG_CFR_W_Pos)                      /*!< 0x00000004 */
#define WWDG_CFR_W_3            (0x08U << WWDG_CFR_W_Pos)                      /*!< 0x00000008 */
#define WWDG_CFR_W_4            (0x10U << WWDG_CFR_W_Pos)                      /*!< 0x00000010 */
#define WWDG_CFR_W_5            (0x20U << WWDG_CFR_W_Pos)                      /*!< 0x00000020 */
#define WWDG_CFR_W_6            (0x40U << WWDG_CFR_W_Pos)                      /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CFR_W0 WWDG_CFR_W_0
#define  WWDG_CFR_W1 WWDG_CFR_W_1
#define  WWDG_CFR_W2 WWDG_CFR_W_2
#define  WWDG_CFR_W3 WWDG_CFR_W_3
#define  WWDG_CFR_W4 WWDG_CFR_W_4
#define  WWDG_CFR_W5 WWDG_CFR_W_5
#define  WWDG_CFR_W6 WWDG_CFR_W_6

#define WWDG_CFR_WDGTB_Pos      (7U)                                           
#define WWDG_CFR_WDGTB_Msk      (0x3U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000180 */
#define WWDG_CFR_WDGTB          WWDG_CFR_WDGTB_Msk                             /*!< WDGTB[1:0] bits (Timer Base) */
#define WWDG_CFR_WDGTB_0        (0x1U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000080 */
#define WWDG_CFR_WDGTB_1        (0x2U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000100 */

/* Legacy defines */
#define  WWDG_CFR_WDGTB0 WWDG_CFR_WDGTB_0
#define  WWDG_CFR_WDGTB1 WWDG_CFR_WDGTB_1

#define WWDG_CFR_EWI_Pos        (9U)                                           
#define WWDG_CFR_EWI_Msk        (0x1U << WWDG_CFR_EWI_Pos)                     /*!< 0x00000200 */
#define WWDG_CFR_EWI            WWDG_CFR_EWI_Msk                               /*!< Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define WWDG_SR_EWIF_Pos        (0U)                                           
#define WWDG_SR_EWIF_Msk        (0x1U << WWDG_SR_EWIF_Pos)                     /*!< 0x00000001 */
#define WWDG_SR_EWIF            WWDG_SR_EWIF_Msk                               /*!< Early Wakeup Interrupt Flag */






 /** 
  * @}
  *  */
  /* End of  Peripheral_Registers_Bits_Definition */

/**
  * @}
  */
   /* End of  Exported_constants */
   
/** @addtogroup Exported_macro
  * @{
  */

/****************************** ADC Instances *********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

/****************************** CRC Instances *********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)
                                      

/****************************** GPIO Instances ********************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE)  (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOC) || \
                                         ((INSTANCE) == GPIOD) )
                                         
/**************************** GPIO Alternate Function Instances ***************/
#define IS_GPIO_AF_INSTANCE(INSTANCE)   (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB))

/****************************** GPIO Lock Instances ***************************/
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB))

/****************************** I2C Instances *********************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) )


/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/****************************** SMBUS Instances *********************************/
#define IS_SMBUS_ALL_INSTANCE(INSTANCE) ((INSTANCE) == I2C1)

/****************************** SPI Instances *********************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) )

/****************************** TIM Instances *********************************/
#define IS_TIM_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
	((INSTANCE) == TIM2)    ||\
	((INSTANCE) == TIM6))

#define IS_TIM_CC1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
	((INSTANCE) == TIM2)   )

#define IS_TIM_CC2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
   ((INSTANCE) == TIM2)  )

#define IS_TIM_CC3_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
   ((INSTANCE) == TIM2) )

#define IS_TIM_CC4_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
	((INSTANCE) == TIM2)   	)

#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
    ((INSTANCE) == TIM2)   )

#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
    ((INSTANCE) == TIM2)   )

#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
   ((INSTANCE) == TIM2)    )

#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
  ((INSTANCE) == TIM2)    )

#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1) )

#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
   ((INSTANCE) == TIM2) )

#define IS_TIM_HALL_INTERFACE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1))
  
#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1))

#define IS_TIM_XOR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
   ((INSTANCE) == TIM2)  )

#define IS_TIM_MASTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
  ((INSTANCE) == TIM2)\
	)

#define IS_TIM_SLAVE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    ||\
  ((INSTANCE) == TIM2)   )

#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)


#define IS_TIM_BREAK_INSTANCE(INSTANCE)\
      (((INSTANCE) == TIM1) )

#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                   \
		(((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    )

#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
   )

#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)  )

#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    )

#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
	((INSTANCE) == TIM2)   \
   )

    
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)   )
    
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) )
                                     
/******************** USART Instances : auto Baud rate detection **************/                                     
#define IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(INSTANCE) ((INSTANCE) == USART1)
                                                                                              
/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE)   (((INSTANCE) == USART1))

/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE)   (((INSTANCE) == USART1))

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) )

/****************** UART Instances : Driver enable detection ********************/
#define IS_UART_DRIVER_ENABLE_INSTANCE(INSTANCE) (((INSTANCE) == USART1))

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/**
  * @}
  */
   /* End of  Exported_macro */


/******************************************************************************/
/*  For a painless codes migration between the HK32F030M device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */
/*  product lines within the same HK32F030M Family                              */
/******************************************************************************/

/* Aliases for __IRQn */
#define ADC1_COMP_IRQn             ADC1_IRQn
#define RCC_CRS_IRQn               RCC_IRQn
#define TIM6_DAC_IRQn              TIM6_IRQn


/* Aliases for __IRQHandler */
#define ADC1_COMP_IRQHandler             ADC1_IRQHandler
#define RCC_CRS_IRQHandler               RCC_IRQHandler
#define TIM6_DAC_IRQHandler              TIM6_IRQHandler



#include "hk32f030m_def.h"
#include "hk32f030m_conf.h"	




/** @addtogroup Exported_macro
  * @{
  */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/**
  * @}
  */
   /* End of  Exported_macro */
#ifdef __cplusplus
}
#endif

#endif /* HK32F030M_H */


/** @} */ /* End of group HK32F030M_H */

/** @} */ /* End of group HKMicroChip Ltd. */
