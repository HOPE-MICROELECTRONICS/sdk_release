/***************************************************************************//**
* # License
* Copyright 2023 Shenzhen HOPE Microelectronics Co., Ltd. 
* All rights reserved.
* 
* IMPORTANT: All rights of this software belong to Shenzhen HOPE 
* Microelectronics Co., Ltd. ("HOPERF"). Your use of this Software is limited 
* to those specific rights granted under the terms of the business contract, 
* the confidential agreement, the non-disclosure agreement and any other forms 
* of agreements as a customer or a partner of HOPERF. You may not use this 
* Software unless you agree to abide by the terms of these agreements. 
* You acknowledge that the Software may not be modified, copied, 
* distributed or disclosed unless embedded on a HOPERF Bluetooth Low Energy 
* (BLE) integrated circuit, either as a product or is integrated into your 
* products.  Other than for the aforementioned purposes, you may not use, 
* reproduce, copy, prepare derivative works of, modify, distribute, perform, 
* display or sell this Software and/or its documentation for any purposes.
* 
* YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE 
* PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
* INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
* NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL 
* HOPERF OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT, 
* NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER 
* LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING 
* BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR 
* CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF 
* SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES 
* (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.  
*******************************************************************************/

/**
 * @file spi_flash.h
 
 * @version v1.0.0
 *
  */
#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmt453x.h"

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup
 * @{
 */

/** @addtogroup Common
 * @{
 */

/** @addtogroup SPI_FLASH
 * @{
 */

/** @addtogroup SPI_FLASH_Exported_Types
 * @{
 */
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Exported_Constants
 * @{
 */
/**
 * @brief  SPI Flash supported commands
 */
#define sFLASH_CMD_WRITE 0x02 /*!< Write to Memory instruction */
#define sFLASH_CMD_WRSR  0x01 /*!< Write Status Register instruction */
#define sFLASH_CMD_WREN  0x06 /*!< Write enable instruction */
#define sFLASH_CMD_READ  0x03 /*!< Read from Memory instruction */
#define sFLASH_CMD_RDSR  0x05 /*!< Read Status Register instruction  */
#define sFLASH_CMD_RDID  0x9F /*!< Read identification */
#define sFLASH_CMD_SE    0xD8 /*!< Sector Erase instruction */
#define sFLASH_CMD_BE    0xC7 /*!< Bulk Erase instruction */

#define sFLASH_WIP_FLAG 0x01 /*!< Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE   0xA5
#define sFLASH_SPI_PAGESIZE 0x100

#define sFLASH_W25Q128_ID 0x00EF4014
#define sFLASH_M25P64_ID  0x202017

#if 1
#define sFLASH_SPI                SPI1
#define sFLASH_SPI_CLK            RCC_APB2_PERIPH_SPI1
#define sFLASH_SPI_AF             GPIO_AF1_SPI1
#define sFLASH_SPI_SCK_PIN        GPIO_PIN_1 /* PA.01 */
#define sFLASH_SPI_SCK_GPIO_PORT  GPIOA      /* GPIOA */
#define sFLASH_SPI_SCK_GPIO_CLK   RCC_APB2_PERIPH_GPIOA
#define sFLASH_SPI_MISO_PIN       GPIO_PIN_3 /* PA.03 */
#define sFLASH_SPI_MISO_GPIO_PORT GPIOA      /* GPIOA */
#define sFLASH_SPI_MISO_GPIO_CLK  RCC_APB2_PERIPH_GPIOA
#define sFLASH_SPI_MOSI_PIN       GPIO_PIN_2 /* PA.02 */
#define sFLASH_SPI_MOSI_GPIO_PORT GPIOA      /* GPIOA */
#define sFLASH_SPI_MOSI_GPIO_CLK  RCC_APB2_PERIPH_GPIOA
#define sFLASH_CS_PIN             GPIO_PIN_0 /* PA.00 */
#define sFLASH_CS_GPIO_PORT       GPIOA      /* GPIOA */
#define sFLASH_CS_GPIO_CLK        RCC_APB2_PERIPH_GPIOA
#else
#define sFLASH_SPI                SPI2
#define sFLASH_SPI_CLK            RCC_APB2_PERIPH_SPI2
#define sFLASH_SPI_AF              GPIO_AF2_SPI2
#define sFLASH_SPI_SCK_PIN        GPIO_PIN_1 /* PB.01 */
#define sFLASH_SPI_SCK_GPIO_PORT  GPIOB      /* GPIOB */
#define sFLASH_SPI_SCK_GPIO_CLK   RCC_APB2_PERIPH_GPIOB
#define sFLASH_SPI_MISO_PIN       GPIO_PIN_3 /* PB.03 */
#define sFLASH_SPI_MISO_GPIO_PORT GPIOB      /* GPIOB */
#define sFLASH_SPI_MISO_GPIO_CLK  RCC_APB2_PERIPH_GPIOB
#define sFLASH_SPI_MOSI_PIN       GPIO_PIN_2 /* PB.02 */
#define sFLASH_SPI_MOSI_GPIO_PORT GPIOB      /* GPIOB */
#define sFLASH_SPI_MOSI_GPIO_CLK  RCC_APB2_PERIPH_GPIOB
#define sFLASH_CS_PIN             GPIO_PIN_0 /* PB.00 */
#define sFLASH_CS_GPIO_PORT       GPIOB      /* GPIOB */
#define sFLASH_CS_GPIO_CLK        RCC_APB2_PERIPH_GPIOB
#endif


/**
 * @}
 */

/** @addtogroup SPI_FLASH_Exported_Macros
 * @{
 */
/**
 * @brief  Select sFLASH: Chip Select pin low
 */
#define sFLASH_CS_LOW() GPIO_ResetBits(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN)
/**
 * @brief  Deselect sFLASH: Chip Select pin high
 */
#define sFLASH_CS_HIGH() GPIO_SetBits(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN)
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Exported_Functions
 * @{
 */
/**
 * @brief  High layer functions
 */
void sFLASH_DeInit(void);
void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
void sFLASH_StartReadSequence(uint32_t ReadAddr);

/**
 * @brief  Low layer functions
 */
uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
