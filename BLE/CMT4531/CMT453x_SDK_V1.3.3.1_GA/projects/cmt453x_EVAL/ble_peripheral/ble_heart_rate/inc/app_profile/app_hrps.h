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
 * @file app_hrps.h
 
 * @version v1.0.0
 *
  */


#ifndef APP_HRPS_H_
#define APP_HRPS_H_

/**
 * @addtogroup APP
 *
 * @brief Heart Rate Application Module Entry point.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW Configuration

#if (BLE_APP_HRPS)

#include <stdint.h>

/* Define ------------------------------------------------------------*/

#define APP_HRPS_FEATURES       (0x04)      //HRPS ALL FEATURES 0x07

#define HEART_RATE_SEND_DELAY   1000        

enum hrps_cfg
{
    HRPS_CFG_NTF_EN = 1,
};

/// Heart Rate Application Module Environment Structure
struct app_hrps_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Heart Rate Value
    uint8_t hrps_value;
};

/// Heart Rate Application environment
extern struct app_hrps_env_tag app_hrps_env;

extern uint16_t heart_rate_timer_id;

/* Public variables ---------------------------------------------------------*/

/// Table of message handlers
extern const struct app_subtask_handlers app_hrps_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 * @brief Initialize Heart Rate Service Application
 **/
void app_hrps_init(void);

/**
 * @brief Add a Heart Rate Service in the DB
 **/
void app_hrps_add_hrps(void);

/**
 * @brief Enable the Heart Rate profile
 **/
void app_hrps_enable_prf(uint8_t conidx, uint8_t cfg);

/**
 * @brief Send heart rate value
 */
void app_heart_rate_send(uint16_t heart_rate);

/**
 * @brief Send heart rate value periodically
 */
void app_heart_rate_timeout_handler(void);

#endif //BLE_APP_HRPS

/// @} APP

#endif //APP_HRPS_H_

