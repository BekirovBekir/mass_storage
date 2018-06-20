#include "usbd_storage_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */



#define STORAGE_LUN_NBR                 1
#define STORAGE_BLK_NBR                 ((uint32_t)384)//128
#define STORAGE_BLK_SIZ                 ((uint32_t)512)
#define STORAGE_CAPACITY                                (STORAGE_BLK_NBR * STORAGE_BLK_SIZ)
#define RAM_FOR_STORE_DATA	((uint32_t)0x20000)	// 128 kbyte
#define PAGE_SIZE	((uint32_t)0x20000)
#define PAGE_CNT	2
#define BLK_PER_PAGE (PAGE_SIZE/STORAGE_BLK_SIZ)
#define NUM_SECTORS	2
//#define USE_NUM_SECTORS ((uint32_t)(RAM_FOR_STORE_DATA/STORAGE_CAPACITY))



/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */



#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_10  /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_10  +  GetSectorSize(ADDR_FLASH_SECTOR_11) -1 /* End @ of user Flash area : sector start address + sector size -1 */



uint8_t usbMassStorage[RAM_FOR_STORE_DATA];
//uint8_t usbMassStorage[0x8000];
//uint8_t    flashMemoryFlag[8];
//uint8_t    flashMemoryFlag[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t        fl = 0;


#define WAIT_TIMEOUT                                        5000



/* USB Mass storage Standard Inquiry Data */
const int8_t  STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
};


extern USBD_HandleTypeDef hUsbDeviceFS;



FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t SectorError=0;

static int8_t STORAGE_Init_FS (uint8_t lun);
static int8_t STORAGE_GetCapacity_FS (uint8_t lun,
                           uint32_t *block_num,
                           uint16_t *block_size);
static int8_t  STORAGE_IsReady_FS (uint8_t lun);
static int8_t  STORAGE_IsWriteProtected_FS (uint8_t lun);
static int8_t STORAGE_Read_FS (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_Write_FS (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS (void);



/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

USBD_StorageTypeDef USBD_Storage_Interface_fops_HS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS,
};



/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : STORAGE_Init_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Init_FS (uint8_t lun)
{
  HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_10;
    EraseInitStruct.NbSectors = 1;

    // check what blocks in flash is used
    /*uint8_t i = 0;
    uint16_t j = 0;
    uint32_t addr = 0;
    if(!fl)
    {

        for (i = 0; i < STORAGE_BLK_NBR; i++)
        {
                for(j = 0; j < STORAGE_BLK_SIZ; j++)
                {
                    addr = FLASH_USER_START_ADDR + (i*STORAGE_BLK_SIZ) + j;
                    if (*((volatile uint8_t*)(addr)) != 0xff)
                    {
                        flashMemoryFlag[i/8] |= (1 << (i%8));
                        break;
                    }

                }
        }
        fl = 1;
    }*/

    //HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

  return (USBD_OK);
}



/*******************************************************************************
* Function Name  : STORAGE_GetCapacity_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetCapacity_FS (uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}



/*******************************************************************************
* Function Name  : STORAGE_IsReady_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsReady_FS (uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}



/*******************************************************************************
* Function Name  : STORAGE_IsWriteProtected_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsWriteProtected_FS (uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}



/*******************************************************************************
* Function Name  : STORAGE_Read_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Read_FS (uint8_t lun,
                        uint8_t *buf,
                        uint32_t blk_addr,
                        uint16_t blk_len)
{
  uint32_t i;
    switch(lun)
    {
        case 0:
            for(i=0; i < (blk_len*STORAGE_BLK_SIZ); i+=1)
            {
                buf[i] = *((uint8_t*)(FLASH_USER_START_ADDR + (blk_addr*STORAGE_BLK_SIZ)+ i));
            }
            break;
        case 1:
            break;
        default:
            return (USBD_FAIL);
    }

  return (USBD_OK);

}



/*******************************************************************************
* Function Name  : STORAGE_Write_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Write_FS (uint8_t lun,
                         uint8_t *buf,
                         uint32_t blk_addr,
                         uint16_t blk_len)
{
  uint32_t sector_num=0;
  uint32_t addr;
  uint32_t i,j;

    switch(lun)
    {
        case 0:
        	HAL_FLASH_Unlock();
        	if (blk_addr<BLK_PER_PAGE)
        	{
        		sector_num=0;
        		i=0; j=0;
        		addr=blk_addr*STORAGE_BLK_SIZ;

        	}
        	else
        	{
        		sector_num=1;
        		i=1;
        		addr=(blk_addr*STORAGE_BLK_SIZ)-PAGE_SIZE;
        	}

            for (uint32_t k=i; k<=sector_num; k++)
            {
        	// copy
				for (uint32_t n = 0; n < RAM_FOR_STORE_DATA; n++)
				{
					usbMassStorage[n] = *((volatile uint8_t*)((FLASH_USER_START_ADDR+(sector_num*RAM_FOR_STORE_DATA)) + n));
				}
				// erase
			    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
			    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
			    EraseInitStruct.Sector = FLASH_SECTOR_10+sector_num;
			    EraseInitStruct.NbSectors = 1;
				HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

				FLASH_WaitForLastOperation(WAIT_TIMEOUT);

				//modify
				while ((addr<PAGE_SIZE)&&(j<(blk_len*STORAGE_BLK_SIZ)))
				{
					usbMassStorage[addr] = buf[j];
					j++;
					addr++;
				}

				//write
				for (uint32_t n = 0; n < RAM_FOR_STORE_DATA; n++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, ((FLASH_USER_START_ADDR+(sector_num*RAM_FOR_STORE_DATA))+ n), usbMassStorage[n]);
				}

				FLASH_WaitForLastOperation(WAIT_TIMEOUT);
				//break;

				if ((addr>=PAGE_SIZE)||(j<(blk_len*STORAGE_BLK_SIZ))) //(addr < (blk_addr*STORAGE_BLK_SIZ + (blk_len*STORAGE_BLK_SIZ)))
				{
					addr=0;
					sector_num=1;
				}
				else
				{
					break;
				}
            }
            HAL_FLASH_Lock();
            break;
        case 1:
            break;
        default:
            return (USBD_FAIL);
    }

  return (USBD_OK);
  /* USER CODE END 7 */
}



/*******************************************************************************
* Function Name  : STORAGE_GetMaxLun_FS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetMaxLun_FS (void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}



/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }



  return sector;
}



/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;



  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}



/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */



/**
  * @}
  */



/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
