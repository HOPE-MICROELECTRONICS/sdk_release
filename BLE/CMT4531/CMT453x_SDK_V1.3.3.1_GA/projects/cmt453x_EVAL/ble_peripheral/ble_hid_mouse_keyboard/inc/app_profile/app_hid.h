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
 * @file app_hid.h
 * @version v1.0.1
 *
 */


#ifndef APP_HID_H_
#define APP_HID_H_

/**
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief HID Application Module entry point
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_HID)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

#if (PS2_SUPPORT)
#include "ps2.h"             // PS2 Mouse Driver
#endif //(PS2_SUPPORT)

/* Public typedef -----------------------------------------------------------*/


/// HID Application Module Environment Structure
struct app_hid_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Mouse timeout value
    uint16_t timeout;
    /// Internal state of the module
    uint8_t state;
    /// Timer enabled
    bool timer_enabled;
    /// Number of report that can be sent
    uint8_t nb_report;
};

/// Mouse report (data packet)
struct ps2_mouse_msg
{
    /// Buttons
    uint8_t b;
    /// X Axis Relative Movement
    uint8_t x;
    /// Y Axis Relative Movement
    uint8_t y;
    /// Wheel Relative Movement
    uint8_t w;
};

struct keyboard_msg
{
		uint8_t modifier;		
		uint8_t reserved;
		uint8_t keycode1;
		uint8_t keycode2;
		uint8_t keycode3;
		uint8_t keycode4;
		uint8_t keycode5;
		uint8_t keycode6;	
};

/* Public variables ---------------------------------------------------------*/

/// Table of message handlers
extern const struct app_subtask_handlers app_hid_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 *
 * Health Thermometer Application Functions
 *
 **/

/**
 * @brief Initialize HID Application Module
 **/
void app_hid_init(void);

/**
 * @brief Add a HID Service instance in the DB
 **/
void app_hid_add_hids(void);

/**
 * @brief Enable the HID Over GATT Profile device role
 *
 * @param[in]:  conhdl - Connection handle for the connection
 **/
void app_hid_enable_prf(uint8_t conidx);

/**
 * @brief Send a mouse report to the peer device
 *
 * @param[in]:  report - Mouse report sent by the PS2 driver
 **/
void app_hid_send_mouse_report(struct ps2_mouse_msg report);

int app_hid_mouse_timeout_timer_handler(ke_msg_id_t const msgid,void const *param);
                                                   
void app_hid_send_consumer_report(uint8_t* report);

void app_hid_send_keyboard_report(struct keyboard_msg report);

void app_hid_send_voice_report(uint8_t* report, uint8_t len);
bool is_app_hid_ready(void);
#endif //(BLE_APP_HID)

/// @} APP

#endif // APP_HID_H_
