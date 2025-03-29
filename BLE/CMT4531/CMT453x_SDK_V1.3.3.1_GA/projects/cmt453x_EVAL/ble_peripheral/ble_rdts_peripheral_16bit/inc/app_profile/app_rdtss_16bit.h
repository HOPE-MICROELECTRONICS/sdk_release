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
 * @file app_rdtss_16bit.h
 
 * @version v1.0.1
 *
  */


#ifndef APP_RDTSS_16BIT_H_
#define APP_RDTSS_16BIT_H_

/**
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Battery Application Module entry point
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW configuration

#if (BLE_RDTSS_16BIT_SERVER)

/// Manufacturer Name Value
#define APP_RDTSS_16BIT_MANUFACTURER_NAME       ("HopeRF")
#define APP_RDTSS_16BIT_MANUFACTURER_NAME_LEN   (6)



#define ATT_SERVICE_AM_SPEED_16          0xfffa           /*!< Service UUID */
#define ATT_CHAR_AM_SPEED_WRITE_16       0xfffb           /*!< Characteristic value UUID */
#define ATT_CHAR_AM_SPEED_NTF_16         0xfffc           /*!< Characteristic value UUID */

/// rdtss 16bit uuid Service Attributes Indexes
enum
{
    RDTSS_16BIT_IDX_SVC,
    
    RDTSS_16BIT_IDX_WRITE_CHAR,
    RDTSS_16BIT_IDX_WRITE_VAL,
    RDTSS_16BIT_IDX_WRITE_CFG,
    
    RDTSS_16BIT_IDX_NTF_CHAR,
    RDTSS_16BIT_IDX_NTF_VAL,
    RDTSS_16BIT_IDX_NTF_CFG,
    
    RDTSS_16BIT_IDX_NB,
};

/**
 * @brief Initialize Device Information Service Application
 **/
void app_rdtss_16bit_init(void);

/**
 * @brief Add a Device Information Service instance in the DB
 **/
void app_rdtss_16bit_add_rdtss_16bit(void);

void rdtss_16bit_send_notify(uint8_t *data, uint16_t length);


#endif //BLE_RDTSS_16BIT_SERVER

/// @} APP

#endif // APP_RDTSS_16BIT_H_
