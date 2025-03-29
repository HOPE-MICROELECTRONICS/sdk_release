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
 * @file app_batt.h
 
 * @version v1.0.0
 *
  */

#ifndef APP_BATT_H_
#define APP_BATT_H_

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

#if (BLE_APP_BATT)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

/* Public typedef -----------------------------------------------------------*/

/// Battery Application Module Environment Structure
struct app_batt_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Current Battery Level
    uint8_t batt_lvl;
};

/* Public variables ---------------------------------------------------------*/

/// Battery Application environment
extern struct app_batt_env_tag app_batt_env;

/// Table of message handlers
extern const struct app_subtask_handlers app_batt_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 *
 * Health Thermometer Application Functions
 *
 **/

/**
 * @brief Initialize Battery Application Module
 **/
void app_batt_init(void);

/**
 * @brief Add a Battery Service instance in the DB
 **/
void app_batt_add_bas(void);

/**
 * @brief Enable the Battery Service
 **/
void app_batt_enable_prf(uint8_t conidx);

/**
 * @brief Send a Battery level value
 **/
void app_batt_send_lvl(uint8_t batt_lvl);

#endif //(BLE_APP_BATT)

/// @} APP

#endif // APP_BATT_H_
