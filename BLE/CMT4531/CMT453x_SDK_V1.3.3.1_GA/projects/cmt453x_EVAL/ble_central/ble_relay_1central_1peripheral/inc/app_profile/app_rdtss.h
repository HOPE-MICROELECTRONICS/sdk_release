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
 * @file app_rdtss.h
 
 * @version v1.0.0
 *
  */


#ifndef APP_RDTSS_H_
#define APP_RDTSS_H_

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

#if (BLE_RDTSS_SERVER)

/// Manufacturer Name Value
#define APP_RDTSS_MANUFACTURER_NAME       ("HopeRF")
#define APP_RDTSS_MANUFACTURER_NAME_LEN   (6)



#define ATT_SERVICE_AM_SPEED_128          {0x01,0x10,0x2E,0xC7,0x8a,0x0E,  0x73,0x90,  0xE1,0x11,  0xC2,0x08,  0x60,0x27,0x00,0x00}        /*!< Service UUID */
#define ATT_CHAR_AM_SPEED_WRITE_128       {0x01,0x00,0x2E,0xC7,0x8a,0x0E,  0x73,0x90,  0xE1,0x11,  0xC2,0x08,  0x60,0x27,0x00,0x00}     /*!< Characteristic value UUID */
#define ATT_CHAR_AM_SPEED_NTF_128         {0x02,0x00,0x2E,0xC7,0x8a,0x0E,  0x73,0x90,  0xE1,0x11,  0xC2,0x08,  0x60,0x27,0x00,0x00}        /*!< Characteristic value UUID */
#define ATT_CHAR_AM_SPEED_CNTL_POINT_128  {0x03,0x00,0x2E,0xC7,0x8a,0x0E,  0x73,0x90,  0xE1,0x11,  0xC2,0x08,  0x60,0x27,0x00,0x00}        /*!< Characteristic value UUID */
#define ATT_CHAR_AM_SPEED_CMD_CMP_NTF_128 {0x04,0x00,0x2E,0xC7,0x8a,0x0E,  0x73,0x90,  0xE1,0x11,  0xC2,0x08,  0x60,0x27,0x00,0x00}        /*!< Characteristic value UUID */

/// rdtss Service Attributes Indexes
enum
{
    RDTSS_IDX_SVC,
    
    RDTSS_IDX_WRITE_CHAR,
    RDTSS_IDX_WRITE_VAL,
    RDTSS_IDX_WRITE_CFG,
    
    RDTSS_IDX_NTF_CHAR,
    RDTSS_IDX_NTF_VAL,
    RDTSS_IDX_NTF_CFG,
    
    RDTSS_IDX_CNTL_POINT_CHAR,
    RDTSS_IDX_CNTL_POINT_VAL,
    
    RDTSS_IDX_CMD_CMP_NTF_CHAR,
    RDTSS_IDX_CMD_CMP_NTF_VAL,
    RDTSS_IDX_CMD_CMP_NTF_CFG,
    
    RDTSS_IDX_NB,
};

enum cntl_point_op_status
{
    STATUS_NO_ERR                           = 0,
    STATUS_ERR_START_INIT_FAIL              = 1,
    STATUS_ERR_DISCONN_FAIL                 = 2,
    STATUS_ERR_NOT_CONNECTED                = 3,
    STATUS_ERR_MASTER_HAS_CONNECTED         = 4,
    STATUS_ERR_MASTER_IS_CONNECTING         = 5,
};

enum cntl_point_op
{
    CNTL_POINT_OP_CONN      = 0x01,
    CNTL_POINT_OP_DISCONN   = 0x02,
};

enum upload_evt_op
{
    CNTL_POINT_CMD_CMP      = 0x21,
};
/// Table of message handlers
extern const struct app_subtask_handlers app_rdtss_handlers;

/**
 * @brief Initialize Device Information Service Application
 **/
void app_rdtss_init(void);

/**
 * @brief Add a Device Information Service instance in the DB
 **/
void app_rdtss_add_rdts(void);

void rdtss_send_notify(uint8_t *data, uint16_t length);

void rdtss_notify_ctrl_point_cmd_cmp(uint8_t cmd, uint8_t status, uint8_t *data, uint16_t length);


#endif //BLE_RDTSS_SERVER

/// @} APP

#endif // APP_RDTSS_H_
