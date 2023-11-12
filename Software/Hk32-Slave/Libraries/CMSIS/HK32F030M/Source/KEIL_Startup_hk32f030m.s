;******************** (C) COPYRIGHT   HKMicroChip ********************
;* File Name          : KEIL_Startup_hk32f030m.s
;* Author             : MCD Application Team
;* Description        : hk32f030m devices vector table for MDK-ARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == Reset_Handler
;*                      - Set the vector table entries with the exceptions ISR address
;*                      - Branches to __main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the CortexM0 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;* <<< Use Configuration Wizard in Context Menu >>>
;*******************************************************************************

; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp                   ; Top of Stack
                DCD     Reset_Handler                  ; Reset Handler
                DCD     NMI_Handler                    ; NMI Handler
                DCD     HardFault_Handler              ; Hard Fault Handler
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     SVC_Handler                    ; SVCall Handler
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     PendSV_Handler                 ; PendSV Handler
                DCD     SysTick_Handler                ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler                ; Window Watchdog
                DCD     0                              ; Reserved
                DCD     EXTI11_IRQHandler              ; EXTI Line 11 interrupt(AWU_WKP)
                DCD     FLASH_IRQHandler               ; FLASH
                DCD     RCC_IRQHandler                 ; RCC
                DCD     EXTI0_IRQHandler               ; EXTI Line 0 
                DCD     EXTI1_IRQHandler               ; EXTI Line 1                 
                DCD     EXTI2_IRQHandler               ; EXTI Line 2 
                DCD     EXTI3_IRQHandler               ; EXTI Line 3 
                DCD     EXTI4_IRQHandler               ; EXTI Line 4 
                DCD     EXTI5_IRQHandler               ; EXTI Line 5
                DCD     TIM1_BRK_IRQHandler            ; TIM1 break interrupt
                DCD     ADC1_IRQHandler                ; ADC1 interrupt(combined with EXTI line 8)
                DCD     TIM1_UP_TRG_COM_IRQHandler     ; TIM1 Update, Trigger and Commutation
                DCD     TIM1_CC_IRQHandler             ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler                ; TIM2
                DCD     0                              ; Reserved
                DCD     TIM6_IRQHandler                ; TIM6
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved
                DCD     0                              ; Reserved 
                DCD     EXTI6_IRQHandler               ; EXTI Line 6 
                DCD     EXTI7_IRQHandler               ; EXTI Line 7
                DCD     I2C1_IRQHandler                ; I2C1(combined with EXTI line 10)
                DCD     0                              ; Reserved 
                DCD     SPI1_IRQHandler                ; SPI1
                DCD     0                              ; Reserved 
                DCD     USART1_IRQHandler              ; USART1(combined with EXTI line 9)
                DCD     0                              ; Reserved 
                DCD     0                              ; Reserved 
                DCD     0                              ; Reserved 
                DCD     0                              ; Reserved 

__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler routine
Reset_Handler    PROC
                 EXPORT  Reset_Handler                 [WEAK]
        IMPORT  __main
        IMPORT  SystemInit  
                 LDR     R0, =SystemInit
                 BLX     R0
                 LDR     R0, =__main
                 BX      R0
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                    [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler              [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                    [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler                 [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler                [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler                [WEAK]
                EXPORT  EXTI11_IRQHandler              [WEAK]
                EXPORT  FLASH_IRQHandler               [WEAK]
                EXPORT  RCC_IRQHandler                 [WEAK]
                EXPORT  EXTI0_IRQHandler               [WEAK]
                EXPORT  EXTI1_IRQHandler               [WEAK]
                EXPORT  EXTI2_IRQHandler               [WEAK]
                EXPORT  EXTI3_IRQHandler               [WEAK]
                EXPORT  EXTI4_IRQHandler               [WEAK]
                EXPORT  EXTI5_IRQHandler               [WEAK]
                EXPORT  TIM1_BRK_IRQHandler            [WEAK]
                EXPORT  ADC1_IRQHandler                [WEAK]
                EXPORT  TIM1_UP_TRG_COM_IRQHandler     [WEAK]
                EXPORT  TIM1_CC_IRQHandler             [WEAK]
                EXPORT  TIM2_IRQHandler                [WEAK]
                EXPORT  TIM6_IRQHandler                [WEAK]
                EXPORT  EXTI6_IRQHandler               [WEAK]
                EXPORT  EXTI7_IRQHandler               [WEAK]
                EXPORT  I2C1_IRQHandler                [WEAK]
                EXPORT  SPI1_IRQHandler                [WEAK]
                EXPORT  USART1_IRQHandler              [WEAK]

WWDG_IRQHandler   
EXTI11_IRQHandler         
FLASH_IRQHandler           
RCC_IRQHandler             
EXTI0_IRQHandler           
EXTI1_IRQHandler           
EXTI2_IRQHandler           
EXTI3_IRQHandler           
EXTI4_IRQHandler           
EXTI5_IRQHandler           
TIM1_BRK_IRQHandler        
ADC1_IRQHandler            
TIM1_UP_TRG_COM_IRQHandler 
TIM1_CC_IRQHandler 
TIM2_IRQHandler       
TIM6_IRQHandler            
EXTI6_IRQHandler           
EXTI7_IRQHandler           
I2C1_IRQHandler            
SPI1_IRQHandler            
USART1_IRQHandler          


                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB

                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit

                 ELSE

                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap

__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END

;************************ (C) COPYRIGHT HKMicroChip *****END OF FILE*****
