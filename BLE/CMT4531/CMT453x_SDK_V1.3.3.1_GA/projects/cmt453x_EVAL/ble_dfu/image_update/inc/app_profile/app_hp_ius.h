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
 * @file app_hp_ius.h
 
 * @version v1.0.1
 *
  */


#ifndef __APP_HP_IUS_H__
#define __APP_HP_IUS_H__


/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"     // SW configuration
#include "prf_types.h"
#include "prf.h"

#if (BLE_APP_HP_IUS)


#define SERVICE_HP_IUS       {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,  0x01,0x00,  0x11,0x11}        
#define CHAR_HP_IUS_RC       {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11  ,0x02,0x00,  0x11,0x11}     
#define CHAR_HP_IUS_CC       {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11  ,0x03,0x00,  0x11,0x11}        
/* Public typedef -----------------------------------------------------------*/
enum
{
    HP_IUS_IDX_SVC,
    HP_IUS_IDX_RC_CHAR,
    HP_IUS_IDX_RC_VAL,
    HP_IUS_IDX_RC_CFG,
    HP_IUS_IDX_CC_CHAR,
    HP_IUS_IDX_CC_VAL,
    HP_IUS_IDX_CC_CFG,
    HP_IUS_IDX_NB,
};

/* Public variables ---------------------------------------------------------*/
extern struct attm_desc_128 hp_ius_att_db[HP_IUS_IDX_NB];
extern const struct app_subtask_handlers hp_ius_app_handlers;


/* Public function prototypes -----------------------------------------------*/
/**
 * @brief SEND DATA THROUGH HP IUS SERVICE
 **/
void hp_ble_ius_app_cc_send(uint8_t *p_data, uint16_t length);
/**
 * @brief Add a HOPERF IMAGE UPDATE Service instance in the DB
 **/
void app_hp_ius_add_hp_ius(void);
/**
 * @brief Initialize HOPERF IMAGE UPDATE Service Application
 **/
void app_hp_ius_init(void);

#endif //BLE_APP_HP_IUS






#endif //__APP_HP_IUS_H__
