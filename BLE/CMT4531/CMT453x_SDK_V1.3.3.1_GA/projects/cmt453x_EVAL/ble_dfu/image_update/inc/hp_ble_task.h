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
 * @file hp_ble_task.h
 
 * @version v1.0.0
 *
  */


#ifndef APP_TASK_H_
#define APP_TASK_H_

/**
 * @addtogroup APPTASK Task
 * @ingroup APP
 * @brief Routes ALL messages to/from APP block.
 *
 * The APPTASK is the block responsible for bridging the final application with the
 * RWBLE software host stack. It communicates with the different modules of the BLE host,
 * i.e. @ref SMP, @ref GAP and @ref GATT.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)

#include <stdint.h>         // Standard Integer
#include "rwip_task.h"      // Task definitions
#include "ke_task.h"        // Kernel Task

/* Define ------------------------------------------------------------*/

/// Number of APP Task Instances
#define APP_IDX_MAX                 (1)



/* Typedef -----------------------------------------------------------*/
 
 /// Process event response
enum process_event_response
{
    /// Handled
    PR_EVENT_HANDLED = 0,

    /// Unhandled
    PR_EVENT_UNHANDLED
};


/// States of APP task
enum app_state
{
    /// Initialization state
    APP_INIT,
    /// Database create state
    APP_CREATE_DB,
    /// Ready State
    APP_READY,
    /// Connected state
    APP_CONNECTED,

    /// Number of defined states.
    APP_STATE_MAX
};


/// APP Task messages
/*@TRACE*/
enum app_msg_id
{
    APP_DUMMY_MSG = TASK_FIRST_MSG(TASK_ID_APP),

    #if (BLE_APP_PRF)
    #if (BLE_APP_HT)
    /// Timer used to refresh the temperature measurement value
    APP_HT_MEAS_INTV_TIMER,
    #endif //(BLE_APP_HT)

    #if (BLE_APP_HID)
    /// Timer used to disconnect the moue if no activity is detecter
    APP_HID_MOUSE_TIMEOUT_TIMER,
    #endif //(BLE_APP_HID)
    #endif //(BLE_APP_PRF)


    //user event
    APP_PARAMS_UPDATE_EVT,
    APP_CUSTS_TEST_EVT,
    APP_HID_DATA_PROCESS_EVT,
    APP_DFU_BLE_RESET_TIMER,

#if (HP_TIMER_ENABLE)
    /*ns timer*/
    APP_CANCEL_TIMER,
    APP_MODIFY_TIMER,
    //Do not alter the order of the next messages
    //they are considered a range
    HP_TIMER_API_MES0,
    HP_TIMER_API_MES1=HP_TIMER_API_MES0+1,
    HP_TIMER_API_MES2=HP_TIMER_API_MES0+2,
    HP_TIMER_API_MES3=HP_TIMER_API_MES0+3,
    HP_TIMER_API_MES4=HP_TIMER_API_MES0+4,
    HP_TIMER_API_MES5=HP_TIMER_API_MES0+5,
    HP_TIMER_API_MES6=HP_TIMER_API_MES0+6,
    HP_TIMER_API_MES7=HP_TIMER_API_MES0+7,
    HP_TIMER_API_MES8=HP_TIMER_API_MES0+8,
    HP_TIMER_API_MES9=HP_TIMER_API_MES0+9,
    HP_TIMER_API_LAST_MES=HP_TIMER_API_MES9,
#endif //HP_TIMER_ENABLE

};


/* Public variables ---------------------------------------------------------*/

/// @} APPTASK

#endif //(BLE_APP_PRESENT)

#endif // APP_TASK_H_
