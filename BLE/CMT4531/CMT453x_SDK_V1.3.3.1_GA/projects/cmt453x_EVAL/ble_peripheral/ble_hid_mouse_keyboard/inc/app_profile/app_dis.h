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
* CMOSTEK OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT, 
* NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER 
* LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING 
* BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR 
* CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF 
* SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES 
* (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.  
*******************************************************************************/

/**
 * @file app_dis.h
 * @version v1.0.0
 *
 */


#ifndef APP_DIS_H_
#define APP_DIS_H_

/**
 * @addtogroup APP
 *
 * @brief Device Information Application Module Entry point.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW Configuration

#if (BLE_APP_DIS)

#include <stdint.h>

/* Define ------------------------------------------------------------*/

/// Manufacturer Name Value
#define APP_DIS_MANUFACTURER_NAME       ("HopeRF")
#define APP_DIS_MANUFACTURER_NAME_LEN   (6)

/// Model Number String Value
#define APP_DIS_MODEL_NB_STR            ("HP-BLE-1.0")
#define APP_DIS_MODEL_NB_STR_LEN        (10)

/// Serial Number
#define APP_DIS_SERIAL_NB_STR           ("1.0.0.0-LE")
#define APP_DIS_SERIAL_NB_STR_LEN       (10)

/// Firmware Revision
#define APP_DIS_FIRM_REV_STR            ("1.0.0")
#define APP_DIS_FIRM_REV_STR_LEN        (5)

/// System ID Value - LSB -> MSB
#define APP_DIS_SYSTEM_ID               ("\x12\x34\x56\xFF\xFE\x9A\xBC\xDE")
#define APP_DIS_SYSTEM_ID_LEN           (8)

/// Hardware Revision String
#define APP_DIS_HARD_REV_STR           ("1.0.0")
#define APP_DIS_HARD_REV_STR_LEN       (5)

/// Software Revision String
#define APP_DIS_SW_REV_STR              ("1.0.0")
#define APP_DIS_SW_REV_STR_LEN          (5)

/// IEEE
#define APP_DIS_IEEE                    ("\xFF\xEE\xDD\xCC\xBB\xAA")
#define APP_DIS_IEEE_LEN                (6)

/**
 * PNP ID Value - LSB -> MSB
 *      Vendor ID Source : 0x02 (USB Implementer’s Forum assigned Vendor ID value)
 *      Vendor ID : 0x045E      (Microsoft Corp)
 *      Product ID : 0x0040
 *      Product Version : 0x0300
 */
#define APP_DIS_PNP_ID               ("\x02\x5E\x04\x40\x00\x00\x03")
#define APP_DIS_PNP_ID_LEN           (7)

#if (BLE_APP_HID)
#define APP_DIS_FEATURES             (DIS_MANUFACTURER_NAME_CHAR_SUP_BIT |\
                                      DIS_MODEL_NB_STR_CHAR_SUP_BIT      |\
                                      DIS_SYSTEM_ID_CHAR_SUP_BIT         |\
                                      DIS_PNP_ID_CHAR_SUP_BIT)
    
#else
#define APP_DIS_FEATURES             (DIS_ALL_FEAT_SUP)
#endif //(BLE_APP_HID)


/* Public variables ---------------------------------------------------------*/

/// Table of message handlers
extern const struct app_subtask_handlers app_dis_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 * @brief Initialize Device Information Service Application
 **/
void app_dis_init(void);

/**
 * @brief Add a Device Information Service instance in the DB
 **/
void app_dis_add_dis(void);

/**
 * @brief Enable the Device Information Service
 **/
void app_dis_enable_prf(uint16_t conhdl);

#endif //BLE_APP_DIS

/// @} APP

#endif //APP_DIS_H_
