/*
 * flash_data.c
 *
 *  Created on: Dec 1, 2021
 *      Author: igor.dymov
 */


#include "flash_data.h"


static uint8_t FisrtStart = 1;
uint16_t SettingsREG[]={VALID_CODE, 6, 4, 2, 2, 2,2, 20, 2, 60 , 13, 2,20,30,180};

static uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t MEM_If_Init_FS(void);
static uint16_t MEM_If_Erase_FS(void);
static uint16_t MEM_If_DeInit_FS(void);


void * cgetREGAdr(uint8_t adr)
{
	return ((void *)&SettingsREG[adr]);
}

void vFDWtiteReg(void)
{
	uint8_t * src = (uint8_t *) FLASH_DATA_ADR;
	MEM_If_Init_FS();
	MEM_If_Erase_FS();
	MEM_If_Write_FS(&SettingsREG[0], src, sizeof(SettingsREG));
	MEM_If_DeInit_FS();
}

void vFDInit( void )
{
	uint8_t * src =  (uint8_t *) FLASH_DATA_ADR;
	uint8_t  buff;
	if (FisrtStart)
	{
	   MEM_If_Read_FS(src, &buff, 1);
	   if (buff!= VALID_CODE)
	   {
		   vFDWtiteReg();
	   }
	   MEM_If_Read_FS(src, &SettingsREG[0],  sizeof(SettingsREG));
	   FisrtStart = 0;

	}
}
/*
 *
 */
void vFDSetRegState(uint8_t adr, uint16_t state)
{
	SettingsREG[adr+1]= state;
	vFDWtiteReg();
}


uint16_t vFDGetRegState(uint8_t adr)
{
	if (FisrtStart)
	{
	  vFDInit();
	}
	return SettingsREG[adr+1];
}



/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Memory initialization routine.
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Init_FS(void)
{

  HAL_StatusTypeDef flashStatus = HAL_ERROR;
  while ( flashStatus != HAL_OK )
  {
    flashStatus = HAL_FLASH_Unlock();
  }
  return HAL_OK;
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes Memory
  * @retval USBD_OK if operation is successful, MAL_FAIL else
  */
uint16_t MEM_If_DeInit_FS(void)
{
  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef flashStatus = HAL_ERROR;
  while ( flashStatus != HAL_OK )
  {
    flashStatus = HAL_FLASH_Lock();
  }
  return HAL_OK;
  /* USER CODE END 1 */
}

/**
  * @brief  Erase sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Erase_FS()
{
  /* USER CODE BEGIN 2 */
  uint32_t               pageError = 0U;
  HAL_StatusTypeDef      status    = HAL_ERROR;
  FLASH_EraseInitTypeDef eraseInit;


    eraseInit.TypeErase    = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks        = FLASH_BANK_1;
    eraseInit.PageAddress      = FLASH_DATA_ADR;
    eraseInit.NbPages    = 1U;
    status = HAL_FLASHEx_Erase( &eraseInit, &pageError );

  return status;
  /* USER CODE END 2 */
}

/**
  * @brief  Memory write routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* USER CODE BEGIN 3 */
  uint32_t           i      = 0U;
  HAL_StatusTypeDef      status    = HAL_ERROR;

  for ( i=0U; i<Len; i+=4U )
  {
	if ( ( uint32_t )( dest + i ) > FLASH_SIZE )
	{
      if ( HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, ( uint32_t )( dest + i ), *( uint32_t* )( src + i ) ) == HAL_OK )
      {
        if ( *( uint32_t* )( src + i ) != *( uint32_t* )( dest + i ) )
        {
        	status = HAL_ERROR;
          break;
        }
        else
        {
        	status = HAL_OK;
        }
      }
      else
      {
    	  status = HAL_ERROR;
        break;
      }
	}
  }
  return status;
  /* USER CODE END 3 */
}

/**
  * @brief  Memory read routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* Return a valid address to avoid HardFault */
  /* USER CODE BEGIN 4 */

    uint32_t i    = 0U;
    uint8_t *psrc = src;

    for ( i=0U; i<Len; i++ )
    {
      dest[i] = *psrc++;
    }
    return ( uint8_t* )( dest );

  /* USER CODE END 4 */
}



