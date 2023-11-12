/**
  ******************************************************************************
  * @file    hk32f030m_flash.c
  * @author  Rakan.z/laura.C
  * @version V1.0  
  * @brief   API file of flash module
  * @changelist  
  ******************************************************************************
  */ 
 
/* Includes ------------------------------------------------------------------*/
#include "hk32f030m_flash.h"



/* FLASH driver modules  */ 
/* 
 ===============================================================================
               ##### FLASH Interface configuration functions #####
 ===============================================================================

    [..] FLASH_Interface configuration_Functions, includes the following functions:
       (+) void FLASH_SetLatency(uint32_t FLASH_Latency):
    [..] To correctly read data from Flash memory, the number of wait states (LATENCY) 
     must be correctly programmed according to the frequency of the CPU clock (HCLK) 
    [..]
        +--------------------------------------------- +
        |  Wait states  |   HCLK clock frequency (MHz) |
        |---------------|------------------------------|
        |0WS(1CPU cycle)|       0 < HCLK <= 24         |
        |---------------|------------------------------|
        |1WS(2CPU cycle)|       24 < HCLK <= 32        |
        +----------------------------------------------+
    [..]
       (+) void FLASH_PrefetchBufferCmd(FunctionalState NewState);
    [..]
     All these functions don't need the unlock sequence.

*/

/**
  * @brief  Sets the code latency value.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *          This parameter can be one of the following values:
  *             @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *             @arg FLASH_Latency_1: FLASH One Latency cycle
  *             @arg FLASH_Latency_2: FLASH two Latency cycle
  *             @arg FLASH_Latency_3: FLASH three Latency cycle
  * @retval None
  */
void FLASH_SetLatency(uint32_t FLASH_Latency)
{
   uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_FLASH_LATENCY(FLASH_Latency));

  /* Read the ACR register */
  tmpreg = FLASH->ACR;  

  /* Sets the Latency value */
  tmpreg &= (uint32_t) (~((uint32_t)FLASH_ACR_LATENCY));
  tmpreg |= FLASH_Latency;

  /* Write the ACR register */
  FLASH->ACR = tmpreg;
}


/* FLASH Memory Programming functions
 *
 ===============================================================================
                ##### FLASH Memory Programming functions #####
 ===============================================================================

    [..] The FLASH Memory Programming functions, includes the following functions:
       (+) void FLASH_Unlock(void);
       (+) void FLASH_Lock(void);
       (+) FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
       (+) FLASH_Status FLASH_EraseAllPages(void);
       (+) FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
       (+) FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
       (+) FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data)

    [..] Any operation of erase or program should follow these steps:
       
       (#) Call the FLASH_Unlock() function to enable the flash control register and 
           program memory access
       (#) Call the desired function to erase page or program data
       (#) Call the FLASH_Lock() to disable the flash program memory access 
      (recommended to protect the FLASH memory against possible unwanted operation)
  */

/**
  * @brief  Unlocks the FLASH control register and program memory access.
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Unlocking the program memory access */
    FLASH->KEYR = FLASH_FKEY1;
    FLASH->KEYR = FLASH_FKEY2;
  }
}

/**
  * @brief  Locks the Program memory access.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH control register and program memory access */
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
  * @brief  Erases a specified page in program memory.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  Page_Address: The page address in program memory to be erased.
  * @note   A Page is erased in the Program memory only if the address to load 
  *         is the start address of a page (multiple of 128 bytes).
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Page_Address));
 
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  { 
    /* If the previous operation is completed, proceed to erase the page */
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = Page_Address;
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PER Bit */
    FLASH->CR &= ~FLASH_CR_PER;
  }
    
  /* Return the Erase Status */
  return status;
}

/**
  * @brief  Erases all FLASH pages.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseAllPages(void)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase all pages */
     FLASH->CR |= FLASH_CR_MER;
     FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

    /* Disable the MER Bit */
    FLASH->CR &= ~FLASH_CR_MER;
  }

  /* Return the Erase Status */
  return status;
}


/**
  * @brief  Programs a half word at a specified address.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new data */
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint16_t*)Address = Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PG Bit */
    FLASH->CR &= ~FLASH_CR_PG;
  } 
  
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Programs a byte at a specified address.
  * @note   To correctly run this function, the FLASH_Unlock() function must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access (recommended
  *         to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new data */
    FLASH->ECR |= FLASH_ECR_BPG;
  
    *(__IO uint8_t*)Address = Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PG Bit */
    FLASH->ECR &= ~FLASH_ECR_BPG;
  } 
  
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Unlocks the option bytes block access.
  * @param  None
  * @retval None
  */
void FLASH_OB_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_OPTWRE) == RESET)
  { 
    /* Unlocking the option bytes block access */
    FLASH->OPTKEYR = FLASH_OPTKEY1;
    FLASH->OPTKEYR = FLASH_OPTKEY2;
  }
}

/**
  * @brief  Locks the option bytes block access.
  * @param  None
  * @retval None
  */
void FLASH_OB_Lock(void)
{
  /* Set the OPTWREN Bit to lock the option bytes block access */
  FLASH->CR &= ~FLASH_CR_OPTWRE;
}


/**
  * @brief  Erases the FLASH option bytes.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @note   This functions erases all option bytes except the Read protection (RDP).
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_EraseByte(uint32_t Address)
{
  	FLASH_Status status = FLASH_COMPLETE;
 
  	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
	if(status == FLASH_COMPLETE)
	{
	  FLASH->CR |= FLASH_CR_OPTER;
	  FLASH->AR = Address;
	  FLASH->CR |= FLASH_CR_STRT;
	  /* Wait for last operation to be completed */
	  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
	}
	/* If the erase operation is completed, disable the OPTER Bit */
    FLASH->CR &= ~FLASH_CR_OPTER;
  	return status;
}

/**
  * @brief  Write protects the desired pages
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  OB_WRP: specifies the address of the pages to be write protected.
  *          This parameter can be:
  *             @arg OB_WRP_Pages0to3..OB_WRP_Pages124to127
  *             @arg OB_WRP_AllPages
  *             @arg OB_WRP_None
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_WRPConfig(uint32_t OB_WRP)
{
	uint16_t WRP0_Data = 0xFFFF, WRP1_Data = 0xFFFF, WRP2_Data = 0xFFFF, WRP3_Data = 0xFFFF;
	uint32_t WRP_ADDR = FLASH_OB_WRP_ADDRESS;
	uint16_t i=0;

	FLASH_Status status = FLASH_COMPLETE;

	OB_WRP = (uint32_t)(~OB_WRP);
	WRP0_Data = (uint16_t)(OB_WRP & OB_WRP0_WRP0);
	WRP1_Data = (uint16_t)((OB_WRP >> 8) & OB_WRP0_WRP0);
	WRP2_Data = (uint16_t)((OB_WRP >> 16) & OB_WRP0_WRP0) ;
	WRP3_Data = (uint16_t)((OB_WRP >> 24) & OB_WRP0_WRP0) ;

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

	for(i=0; i<8; i++)
	{
		if(status == FLASH_COMPLETE)
		{
			FLASH->CR |= FLASH_CR_OPTER;
			FLASH->AR = WRP_ADDR;
			FLASH->CR |= FLASH_CR_STRT;
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

		}
		WRP_ADDR++;
	} 
	if(status == FLASH_COMPLETE)
	{

		/* If the erase operation is completed, disable the OPTER Bit */
    	FLASH->CR &= ~FLASH_CR_OPTER;    

		FLASH->CR |= FLASH_CR_OPTPG;

		if(WRP0_Data != 0xFF)
		{
			OB->WRP0 = WRP0_Data;

			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
		}
		if((status == FLASH_COMPLETE) && (WRP1_Data != 0xFF))
		{
			OB->WRP1 = WRP1_Data;

			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
		}
		if((status == FLASH_COMPLETE) && (WRP2_Data != 0xFF))
		{
			OB->WRP2 = WRP2_Data;

			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
		}    
		if((status == FLASH_COMPLETE) && (WRP3_Data != 0xFF))
		{
			OB->WRP3 = WRP3_Data;

			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
		}  
		if(status != FLASH_TIMEOUT)
		{
			/* if the program operation is completed, disable the OPTPG Bit */
			FLASH->CR &= ~FLASH_CR_OPTPG;
		}
	} 
	/* Return the write protection operation Status */
	return status;
}

/**
  * @brief  Enables or disables the read out protection.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  FLASH_ReadProtection_Level: specifies the read protection level. 
  *          This parameter can be:
  *             @arg OB_RDP_Level_0: No protection
  *             @arg OB_RDP_Level_1: Read protection of the memory
  *             @arg OB_RDP_Level_2: Chip protection
  * @note   When enabling OB_RDP level 2 it's no more possible to go back to level 1 or 0
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP)
{
  FLASH_Status status = FLASH_COMPLETE;
  uint32_t WRP_ADDR = FLASH_OB_RDP_ADDRESS;
  /* Check the parameters */
  assert_param(IS_OB_RDP(OB_RDP));
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
	if(status == FLASH_COMPLETE)
	{
		FLASH->CR |= FLASH_CR_OPTER;
		FLASH->AR = WRP_ADDR;
		FLASH->CR |= FLASH_CR_STRT;
		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT); 
	}
	if(status == FLASH_COMPLETE)
	{
		FLASH->CR |= FLASH_CR_OPTER;
		FLASH->AR = WRP_ADDR+1;
		FLASH->CR |= FLASH_CR_STRT;
		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);  
	}
    
    if(status == FLASH_COMPLETE)
    {
      /* If the erase operation is completed, disable the OPTER Bit */
      FLASH->CR &= ~FLASH_CR_OPTER;
      
      /* Enable the Option Bytes Programming operation */
      FLASH->CR |= FLASH_CR_OPTPG;
       
      OB->RDP = OB_RDP;

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT); 
    
      if(status != FLASH_TIMEOUT)
      {
        /* if the program operation is completed, disable the OPTPG Bit */
        FLASH->CR &= ~FLASH_CR_OPTPG;
      }
    }
    else 
    {
      if(status != FLASH_TIMEOUT)
      {
        /* Disable the OPTER Bit */
        FLASH->CR &= ~FLASH_CR_OPTER;
      }
    }
  }
  /* Return the protection operation Status */
  return status;
}

/**
  * @brief  Programs the FLASH User Option Byte: IWDG_SW / RST_STOP.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  OB_IWDG: Selects the WDG mode
  *          This parameter can be one of the following values:
  *             @arg OB_IWDG_SW: Software WDG selected
  *             @arg OB_IWDG_HW: Hardware WDG selected
  * @param  OB_STOP: Reset event when entering STOP mode.
  *          This parameter can be one of the following values:
  *             @arg OB_STOP_NoRST: No reset generated when entering in STOP
  *             @arg OB_STOP_RST: Reset generated when entering in STOP  
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP)
{
  FLASH_Status status = FLASH_COMPLETE; 
  uint32_t WRP_ADDR = FLASH_OB_USER_ADDRESS;

  /* Check the parameters */
  assert_param(IS_OB_IWDG_SOURCE(OB_IWDG));
  assert_param(IS_OB_STOP_SOURCE(OB_STOP));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  if(status == FLASH_COMPLETE)
  {
	  FLASH->CR |= FLASH_CR_OPTER;
	  FLASH->AR = WRP_ADDR;
	  FLASH->CR |= FLASH_CR_STRT;
	  /* Wait for last operation to be completed */
	  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  }
  if(status == FLASH_COMPLETE)
  {
	  FLASH->CR |= FLASH_CR_OPTER;
	  FLASH->AR = WRP_ADDR+1;
	  FLASH->CR |= FLASH_CR_STRT;
	  /* Wait for last operation to be completed */
	  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  }
  
  if(status == FLASH_COMPLETE)
  {
    /*clear the CR_OPTER bit*/
    FLASH->CR &= ~FLASH_CR_OPTER;
    /* Enable the Option Bytes Programming operation */
    FLASH->CR |= FLASH_CR_OPTPG; 

    OB->USER = (uint16_t)((uint16_t)(OB_IWDG | OB_STOP));
  
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

    if(status != FLASH_TIMEOUT)
    {
      /* If the program operation is completed, disable the OPTPG Bit */
      FLASH->CR &= ~FLASH_CR_OPTPG;
    }
  }  
  else
  {
  	if(status != FLASH_TIMEOUT)
      {
        /* Disable the OPTER Bit */
        FLASH->CR &= ~FLASH_CR_OPTER;
      }
  }  
  /* Return the Option Byte program Status */
  return status;
}

/**
  * @brief  Programs the FLASH User Option Byte: IWDG_RL_IV
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param    
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_IWDG_RLRConfig(uint16_t OB_IWDG_RLR, FunctionalState NewState)
{
  FLASH_Status status = FLASH_COMPLETE; 
  uint32_t WRP_ADDR = FLASH_OB_IWDG_ADDRESS;
  uint16_t i=0;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  	for(i=0; i<4; i++)
  	{
		if(status == FLASH_COMPLETE)
		{
			FLASH->CR |= FLASH_CR_OPTER;
			FLASH->AR = WRP_ADDR;
			FLASH->CR |= FLASH_CR_STRT;
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

		}
		WRP_ADDR++;
  	}
	/* If the erase operation is completed, disable the OPTER Bit */
    FLASH->CR &= ~FLASH_CR_OPTER;
    
  	if(NewState == ENABLE)
  	{
	  	if(status == FLASH_COMPLETE)
	  	{
			FLASH->CR |= FLASH_CR_OPTPG;
			OB->IWDG_RL_IV = (uint16_t)(OB_IWDG_RLR);
				  
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
	  	}
	  	if(status == FLASH_COMPLETE)
	  	{
			FLASH->CR |= FLASH_CR_OPTPG;
			OB->IWDG_INI_KEY = (uint16_t)(0x5b1e);
				  
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
	  	}

		if(status != FLASH_TIMEOUT)
		{
		  	/* If the program operation is completed, disable the OPTPG Bit */
		  	FLASH->CR &= ~FLASH_CR_OPTPG;
		}
	}    
	/* Return the Option Byte program Status */
	return status;
}


/**
  * @brief  Programs the FLASH User Option Byte: LSI_LP_CTL
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param    
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_LSILPConfig(FunctionalState NewState)
{
	FLASH_Status status = FLASH_COMPLETE; 
	uint32_t WRP_ADDR = FLASH_OB_LSI_LP_ADDRESS;
	uint16_t i=0;

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  	for(i=0; i<2; i++)
  	{
		if(status == FLASH_COMPLETE)
		{
			FLASH->CR |= FLASH_CR_OPTER;
			FLASH->AR = WRP_ADDR;
			FLASH->CR |= FLASH_CR_STRT;
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

    
		}
		WRP_ADDR++;
  	}
	/* If the erase operation is completed, disable the OPTER Bit */
    FLASH->CR &= ~FLASH_CR_OPTER;	
  	if(NewState == ENABLE)
  	{
	  	if(status == FLASH_COMPLETE)
	  	{
			FLASH->CR |= FLASH_CR_OPTPG;
			OB->LSI_LP_CTL = (uint16_t)(0x369C);
				  
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
	  	}

		if(status != FLASH_TIMEOUT)
		{
		  	/* If the program operation is completed, disable the OPTPG Bit */
		  	FLASH->CR &= ~FLASH_CR_OPTPG;
		}
	}    
	/* Return the Option Byte program Status */
	return status;
}

/**
  * @brief  Programs the FLASH User Option Byte: DBG_CLK_CTL
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param    
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_DBGCLKConfig(FunctionalState NewState)
{
	FLASH_Status status = FLASH_COMPLETE; 
	uint32_t WRP_ADDR = FLASH_OB_DBG_CLK_ADDRESS;
	uint16_t i=0;

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  	for(i=0; i<2; i++)
  	{
		if(status == FLASH_COMPLETE)
		{
			FLASH->CR |= FLASH_CR_OPTER;
			FLASH->AR = WRP_ADDR;
			FLASH->CR |= FLASH_CR_STRT;
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);  	

    
		}
		WRP_ADDR++;
  	}
  	/* If the erase operation is completed, disable the OPTER Bit */
    FLASH->CR &= ~FLASH_CR_OPTER;
  	if(NewState == ENABLE)
  	{
	  	if(status == FLASH_COMPLETE)
	  	{
			FLASH->CR |= FLASH_CR_OPTPG;
			OB->DBG_CLK_CTL = (uint16_t)(0x12de);
				  
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
	  	}

		if(status != FLASH_TIMEOUT)
		{
		  	/* If the program operation is completed, disable the OPTPG Bit */
		  	FLASH->CR &= ~FLASH_CR_OPTPG;
		}
	}    
	/* Return the Option Byte program Status */
	return status;
}


/**
  * @brief  Programs the FLASH User Option Byte: IWDG_SW, RST_STOP
  *         VDDA ANALOG monitoring.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  OB_USER: Selects all user option bytes
  *          This parameter is a combination of the following values:
  *             @arg OB_IWDG_SW / OB_IWDG_HW: Software / Hardware WDG selected
  *             @arg OB_STOP_NoRST / OB_STOP_RST: No reset / Reset generated when entering in STOP
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER)
{
  FLASH_Status status = FLASH_COMPLETE; 

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* Enable the Option Bytes Programming operation */
    FLASH->CR |= FLASH_CR_OPTPG; 

    OB->USER = OB_USER;
  
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

    if(status != FLASH_TIMEOUT)
    {
      /* If the program operation is completed, disable the OPTPG Bit */
      FLASH->CR &= ~FLASH_CR_OPTPG;
    }
  }    
  /* Return the Option Byte program Status */
  return status;

}

/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option
  *         bytes (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed.
  *          This parameter can be 0x1FFFF804 or 0x1FFFF806. 
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_OB_ProgramData(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Check the parameters */
  assert_param(IS_OB_DATA_ADDRESS(Address));
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

  if(status == FLASH_COMPLETE)
  {
    /* Enables the Option Bytes Programming operation */
    FLASH->CR |= FLASH_CR_OPTPG; 
    *(__IO uint16_t*)Address = Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    if(status != FLASH_TIMEOUT)
    {
      /* If the program operation is completed, disable the OPTPG Bit */
      FLASH->CR &= ~FLASH_CR_OPTPG;
    }
  }
  /* Return the Option Byte Data Program Status */
  return status;
}

/**
  * @brief  Returns the FLASH User Option Bytes values.
  * @param  None
  * @retval The FLASH User Option Bytes .
  */
uint8_t FLASH_OB_GetUser(void)
{
  /* Return the User Option Byte */
  return (uint8_t)(FLASH->OBR >> 8);
}

/**
  * @brief  Returns the FLASH Write Protection Option Bytes value.
  * @param  None
  * @retval The FLASH Write Protection Option Bytes value
  */
uint32_t FLASH_OB_GetWRP(void)
{
  /* Return the FLASH write protection Register value */
  return (uint32_t)(FLASH->WRPR);
}

/**
  * @brief  Checks whether the FLASH Read out Protection Status is set or not.
  * @param  None
  * @retval FLASH ReadOut Protection Status(SET or RESET)
  */
FlagStatus FLASH_OB_GetRDP(void)
{
  FlagStatus readstatus = RESET;
  
  if ((uint8_t)(FLASH->OBR & (FLASH_OBR_RDPRT1 | FLASH_OBR_RDPRT2)) != RESET)
  {
    readstatus = SET;
  }
  else
  {
    readstatus = RESET;
  }
  return readstatus;
}


/**  FLASH_Group4 Interrupts and flags management functions
 *   Interrupts and flags management functions
 *

 ===============================================================================
             ##### Interrupts and flags management functions #####
 ===============================================================================  
 */

/**
  * @brief  Enables or disables the specified FLASH interrupts.
  * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or 
  *         disabled.
  *          This parameter can be any combination of the following values:
  *             @arg FLASH_IT_EOP: FLASH end of programming Interrupt
  *             @arg FLASH_IT_ERR: FLASH Error Interrupt
  * @retval None 
  */
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FLASH_IT(FLASH_IT)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if(NewState != DISABLE)
  {
    /* Enable the interrupt sources */
    FLASH->CR |= FLASH_IT;
  }
  else
  {
    /* Disable the interrupt sources */
    FLASH->CR &= ~(uint32_t)FLASH_IT;
  }
}

/**
  * @brief  Checks whether the specified FLASH flag is set or not.
  * @param  FLASH_FLAG: specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *             @arg FLASH_FLAG_BSY: FLASH write/erase operations in progress flag 
  *             @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *             @arg FLASH_FLAG_EOP: FLASH End of Programming flag
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_FLASH_GET_FLAG(FLASH_FLAG));

  if((FLASH->SR & FLASH_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the new state of FLASH_FLAG (SET or RESET) */
  return bitstatus; 
}

/**
  * @brief  Clears the FLASH's pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *             @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *             @arg FLASH_FLAG_EOP: FLASH End of Programming flag
  * @retval None
  */
void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG));
  
  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_BUSY,  FLASH_ERROR_WRP or FLASH_COMPLETE.
  */
FLASH_Status FLASH_GetStatus(void)
{
  FLASH_Status FLASHstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    FLASHstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    { 
      FLASHstatus = FLASH_ERROR_WRP;
    }
    else 
    {
      FLASHstatus = FLASH_COMPLETE;
    }
  }
  /* Return the FLASH Status */
  return FLASHstatus;
}


/**
  * @brief  Waits for a FLASH operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH programming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout)
{ 
  FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the FLASH Status */
  status = FLASH_GetStatus();
  
  /* Wait for a FLASH operation to complete or a TIMEOUT to occur */
  while((status == FLASH_BUSY) && (Timeout != 0x00))
  {
    status = FLASH_GetStatus();
    Timeout--;
  }
  
  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}


/**
  * @brief  erase a byte at a EEPROM address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status EEPROM_EraseByte(uint32_t Address)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_EEPROM_PROGRAM_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new data */
    FLASH->ECR |= FLASH_ECR_EEPROM_ER;  
	  FLASH->AR = Address;
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PG Bit */
    FLASH->ECR &= ~FLASH_ECR_EEPROM_ER;
  } 
  
  /* Return the Program Status */
  return status;
}


/**
  * @brief  Programs a byte at a EEPROM address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status EEPROM_ProgramByte(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_EEPROM_PROGRAM_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new data */
    FLASH->ECR |= FLASH_ECR_EEPROM_BPG;
  
    *(__IO uint8_t*)Address = Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    /* Disable the PG Bit */
    FLASH->ECR &= ~FLASH_ECR_EEPROM_BPG;
  } 
  
  /* Return the Program Status */
  return status;
}

/**
  * @brief Return the unique device identifier (UID based on 64 bits)
  * @param UID: pointer to 2 words array.
  * @retval Device identifier
  */
void Sys_GetDevice64BitUID(uint32_t *UID)
{
  UID[0] = (uint32_t)(READ_REG(*((uint32_t *)UID_BASE)));
  UID[1] = (uint32_t)(READ_REG(*((uint32_t *)(UID_BASE + 4U)))); 
}


