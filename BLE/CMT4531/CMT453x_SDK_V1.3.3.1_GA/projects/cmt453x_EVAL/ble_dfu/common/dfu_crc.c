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
 * @file dfu_crc.c
 * @version v1.0.0
 *
  */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "dfu_crc.h"
#include "cmt453x_crc.h"

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/  

#define LITTLE_TO_BIG(u32)    \
(uint32_t) ((u32 & 0x000000FF) << 24) | \
(uint32_t) ((u32 & 0x0000FF00) << 8) | \
(uint32_t) ((u32 & 0x00FF0000) >> 8) | \
(uint32_t) ((u32 & 0xFF000000) >> 24)

/**
 * @brief Calculate crc32 for given data, using hardware crc32.
 * @param[in] p_data raw data.
 * @param[in] len raw data len. 
 * @return crc32 result.
 */
uint32_t dfu_crc32(uint8_t * p_data, uint32_t len)
{
    uint32_t u32p_data;
    RCC->AHBPCLKEN |= RCC_AHB_PERIPH_CRC;
    CRC->CRC32CTRL = CRC32_CTRL_RESET;
    uint32_t index = 0;

    //for if led%4 > 0
    switch (len&0x3)
    {
    	case 1:
            CRC->CRC32DAT = 0x6AA59E9D; 
            u32p_data = p_data[index];
            CRC->CRC32DAT = u32p_data;
            index += 1;
    		break;
        case 2:
            CRC->CRC32DAT = 0x9746CD0A; 
            u32p_data = (uint32_t)(p_data[index]<<8)| p_data[index + 1];
            CRC->CRC32DAT = u32p_data;
            index += 2;
    		break;
    	case 3:
            CRC->CRC32DAT = 0xCC6021D0; 
            u32p_data = (uint32_t)(p_data[index]<<16)| (p_data[index + 1]<<8)|(p_data[index + 2]);
            CRC->CRC32DAT = u32p_data;
            index += 3;
    		break;
    	default:
    		break;
    }
    
    while (index < len)
    {
        u32p_data = p_data[index]; 
        index++;
        u32p_data = (u32p_data<<8)|p_data[index]; 
        index++;
        u32p_data = (u32p_data<<8)|p_data[index]; 
        index++;
        u32p_data = (u32p_data<<8)|p_data[index]; 
        index++;
        CRC->CRC32DAT = u32p_data;
    }

    return CRC->CRC32DAT;
}

/**
 * @}
 */



