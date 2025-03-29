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

#define CUSTOM_DEVICE_NAME                  "HP_Blinky"   

/* adv configer*/
#define CUSTOM_ADV_FAST_INTERVAL               3200                                       /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds.). */
#define CUSTOM_ADV_SLOW_INTERVAL               3200                                       /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */

#define CUSTOM_ADV_FAST_DURATION               30                                         /**< The advertising duration of fast advertising in units of 1 seconds. maximum is 655 seconds */
#define CUSTOM_ADV_SLOW_DURATION               180                                        /**< The advertising duration of slow advertising in units of 1 seconds. maximum is 655 seconds */


#define CUSTOM_USER_ADVERTISE_DATA \
            "\x03"\
            ADV_TYPE_SERVICE_DATA_16BIT_UUID\
            ADV_UUID_DEVICE_INFORMATION_SERVICE\


#define CUSTOM_USER_ADVERTISE_DATA_LEN (sizeof(CUSTOM_USER_ADVERTISE_DATA)-1)

// Scan response data
#define CUSTOM_USER_ADV_SCNRSP_DATA  \
            "\x09"\
            ADV_TYPE_MANUFACTURER_SPECIFIC_DATA\
            "\xff\xffHopeRF"

// Scan response data length- maximum 31 bytes
#define CUSTOM_USER_ADV_SCNRSP_DATA_LEN (sizeof(CUSTOM_USER_ADV_SCNRSP_DATA)-1)


/*  connection config  */
#define MIN_CONN_INTERVAL                   15                                         /**< Minimum connection interval (15 ms) */
#define MAX_CONN_INTERVAL                   30                                         /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                       0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    5000                                       /**< Connection supervisory timeout (5000ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      (5000)                                     /**<  Time of initiating event to update connection params (5 seconds). */

//sec config
#define SEC_PARAM_IO_CAPABILITIES           GAP_IO_CAP_NO_INPUT_NO_OUTPUT               /**< No I/O capabilities. (@enum gap_io_cap) */
#define SEC_PARAM_OOB                       0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_KEY_SIZE                  16                                          /**< Minimum encryption key size. 7 to 16 */
#define SEC_PARAM_BOND                      1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                      1                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IKEY                      GAP_KDIST_NONE                              /**< Initiator Key Distribution. (@enum gap_kdist) */
#define SEC_PARAM_RKEY                      GAP_KDIST_ENCKEY                            /**< Responder Key Distribution. (@enum gap_kdist) */
#define SEC_PARAM_SEC_MODE_LEVEL            GAP_NO_SEC                                  /**< Device security requirements (minimum security level). (@enum see gap_sec_req) */

//bond conifg
#define MAX_BOND_PEER                       5
#define BOND_STORE_ENABLE                   0
#define BOND_DATA_BASE_ADDR                 0x0103B000

/* profiles config  */
#define CFG_APP_DIS       1
#define CFG_PRF_DISS      1
//#define CFG_APP_BATT    1
//#define CFG_PRF_BASS    1 


/* User config  */
#define HP_LOG_ERROR_ENABLE      1
#define HP_LOG_WARNING_ENABLE    1
#define HP_LOG_INFO_ENABLE       1
#define HP_LOG_DEBUG_ENABLE      0

#define HP_LOG_USART_ENABLE      1

#define HP_TIMER_ENABLE          1
//#define SLEEP_LP_TIMER_ENABLE    1

#define FIRMWARE_VERSION         "1.0.0"
#define HARDWARE_VERSION         "1.0.0"

#endif // _APP_USER_CONFIG_H_

