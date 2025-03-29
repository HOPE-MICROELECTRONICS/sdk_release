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
 * @file app_blps.h
 
 * @version v1.0.0
 *
  */


#ifndef APP_BLPS_H_
#define APP_BLPS_H_

/**
 * @addtogroup APP
 *
 * @brief Blood Pressure Application Module Entry point.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW Configuration

#if (BLE_APP_BLPS)

#include <stdint.h>
#include "blp_common.h"
/* Define ------------------------------------------------------------*/

#define APP_BLPS_ALL_FEATURES               0x1FF

#define APP_BLPS_SUP_FEATUEES               APP_BLPS_ALL_FEATURES

enum blps_cfg
{
    BLPS_CFG_NTFIND_STP,
    BLPS_CFG_NTFIND_NTF_EN,
    BLPS_CFG_NTFIND_IND_EN,
};

enum app_blps_prfl_cfg
{
    APP_BLPS_INTM_CUFF_PRESS_SUP = 1,
    APP_BLPS_RACP_SUP = 2,
};



/* Public variables ---------------------------------------------------------*/

/// Table of message handlers
extern const struct app_subtask_handlers app_blps_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 * @brief Initialize Blood Pressure Sensor Application
 **/
void app_blps_init(void);

/**
 * @brief Add a Blood Pressure Sensor instance in the DB
 **/
void app_blps_add_blps(void);

/**
 * @brief Enable Blood Pressure Sensor 
 */
void app_blps_enable_prf(uint8_t conidx);

/**
 * @brief  Send Blood Pressure Measurement value  
 */
void app_blps_measurement_send(uint8_t conidx, struct bps_bp_meas *data);

/**
 * @brief  Send Intermediate Cuff Pressure value 
 */
void app_blps_intm_cuff_press_send(uint8_t conidx, struct bps_bp_meas *data);

/**
 * @brief  send record access control point response value 
 */
void app_blps_racp_resp_send(uint8_t conidx, uint8_t *data, uint8_t data_len);

#endif //BLE_APP_BLPSS

/// @} APP

#endif //APP_BLPS_H_
