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
 * @file app_user_config.h
 
 * @version v1.0.1
 *
  */

#ifndef _APP_USER_CONFIG_H_
#define _APP_USER_CONFIG_H_

#include "hp_adv_data_def.h"

/* Device name */
#define CUSTOM_DEVICE_NAME                  "ImageUpdate"
#define CUSTOM_BLE_MAC_ADDRESS              "\x8A\x22\x77\x44\x55\x66"


/* adv configer*/
#define CUSTOM_ADV_FAST_INTERVAL               64                                     /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 40 ms.). */
#define CUSTOM_ADV_SLOW_INTERVAL               64                                     /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 40 ms.). */

#define CUSTOM_ADV_FAST_DURATION               0//30                                         /**< The advertising duration of fast advertising in units of 1 seconds. maximum is 655 seconds */
#define CUSTOM_ADV_SLOW_DURATION               180                                        /**< The advertising duration of slow advertising in units of 1 seconds. maximum is 655 seconds */


#define CUSTOM_USER_ADVERTISE_DATA \
            "\x03"\
            ADV_TYPE_SERVICE_DATA_16BIT_UUID\
            ADV_UUID_DEVICE_INFORMATION_SERVICE\


#define CUSTOM_USER_ADVERTISE_DATA_LEN (sizeof(CUSTOM_USER_ADVERTISE_DATA)-1)

/// Scan response data
#define CUSTOM_USER_ADV_SCNRSP_DATA  \
            "\x09"\
            ADV_TYPE_MANUFACTURER_SPECIFIC_DATA\
            "\xff\xffHopeRF"

/// Scan response data length- maximum 31 bytes
#define CUSTOM_USER_ADV_SCNRSP_DATA_LEN (sizeof(CUSTOM_USER_ADV_SCNRSP_DATA)-1)


/*  connection config  */
#define MIN_CONN_INTERVAL                   7.5                                        /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                   30                                         /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                       6                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    5000                                       /**< Connection supervisory timeout (5000ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      (0)                                        /**<  Time of initiating event to update connection params (5 seconds). */

//forfiles config
#define CFG_APP_HP_IUS     1


/* User config  */

#define HP_LOG_ERROR_ENABLE      1
#define HP_LOG_WARNING_ENABLE    1
#define HP_LOG_INFO_ENABLE       1
#define HP_LOG_DEBUG_ENABLE      0

#define HP_LOG_LPUART_ENABLE     0
#define HP_LOG_USART_ENABLE      0
#define HP_LOG_RTT_ENABLE        0

#define HP_TIMER_ENABLE          0

#define FIRMWARE_VERSION            "0.0.01"
#define HARDWARE_VERSION            "0.0.01"


#endif // _APP_USER_CONFIG_H_

