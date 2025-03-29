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
 * @file app_dis.c
 * @version v1.0.1
 *
 */

/**
 * @addtogroup APP
 * @{
 **/

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_DIS)
/* Includes ------------------------------------------------------------------*/
#include "hp_ble.h"                     // Application Manager Definitions
#include "app_dis.h"                 // Device Information Service Application Definitions
#include "diss_task.h"               // Device Information Profile Functions
#include "prf_types.h"               // Profile Common Types Definitions
#include "ke_task.h"                 // Kernel
#include "gapm_task.h"               // GAP Manager Task API
#include "diss.h"
#include <string.h>
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief  dis server value require indicate handler
 * @param  
 * @return 
 * @note   
 */
static int diss_value_req_ind_handler(ke_msg_id_t const msgid,
                                          struct diss_value_req_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    HP_LOG_DEBUG("%s\r\n", __func__);
    // Initialize length
    uint8_t len = 0;
    // Pointer to the data
    uint8_t *data = NULL;

    // Check requested value
    switch (param->value)
    {
        case DIS_MANUFACTURER_NAME_CHAR:
        {
            // Set information
            len = APP_DIS_MANUFACTURER_NAME_LEN;
            data = (uint8_t *)APP_DIS_MANUFACTURER_NAME;
        } break;

        case DIS_MODEL_NB_STR_CHAR:
        {
            // Set information
            len = APP_DIS_MODEL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_MODEL_NB_STR;
        } break;

        case DIS_SYSTEM_ID_CHAR:
        {
            // Set information
            len = APP_DIS_SYSTEM_ID_LEN;
            data = (uint8_t *)APP_DIS_SYSTEM_ID;
        } break;

        case DIS_PNP_ID_CHAR:
        {
            // Set information
            len = APP_DIS_PNP_ID_LEN;
            data = (uint8_t *)APP_DIS_PNP_ID;
        } break;

        case DIS_SERIAL_NB_STR_CHAR:
        {
            // Set information
            len = APP_DIS_SERIAL_NB_STR_LEN;
            data = (uint8_t *)APP_DIS_SERIAL_NB_STR;
        } break;

        case DIS_HARD_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_HARD_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_HARD_REV_STR;
        } break;

        case DIS_FIRM_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_FIRM_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_FIRM_REV_STR;
        } break;

        case DIS_SW_REV_STR_CHAR:
        {
            // Set information
            len = APP_DIS_SW_REV_STR_LEN;
            data = (uint8_t *)APP_DIS_SW_REV_STR;
        } break;

        case DIS_IEEE_CHAR:
        {
            // Set information
            len = APP_DIS_IEEE_LEN;
            data = (uint8_t *)APP_DIS_IEEE;
        } break;

        default:
            ASSERT_ERR(0);
            break;
    }

    // Allocate confirmation to send the value
    struct diss_value_cfm *cfm_value = KE_MSG_ALLOC_DYN(DISS_VALUE_CFM,
            src_id, dest_id,
            diss_value_cfm,
            len);

    // Set parameters
    cfm_value->value = param->value;
    cfm_value->length = len;
    if (len)
    {
        // Copy data
        memcpy(&cfm_value->data[0], data, len);
    }
    // Send message
    ke_msg_send(cfm_value);

    return (KE_MSG_CONSUMED);
}

/**
 * @brief  dis server Initialize
 * @param  
 * @return 
 * @note   
 */
void app_dis_init(void)
{
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_DISS;
    prf.prf_task_handler = &app_dis_handlers;
    hp_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_DISS;
    get_func.prf_itf_get_func = diss_prf_itf_get;
    prf_get_itf_func_register(&get_func);
}

/**
 * @brief  add dis server
 * @param  
 * @return 
 * @note   
 */
void app_dis_add_dis(void)
{
    struct diss_db_cfg* db_cfg;
    // Allocate the DISS_CREATE_DB_REQ
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                  TASK_GAPM, TASK_APP,
                                                  gapm_profile_task_add_cmd, sizeof(struct diss_db_cfg));
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl =  PERM(SVC_AUTH, NO_AUTH); 
    req->prf_task_id = TASK_ID_DISS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct diss_db_cfg* ) req->param;
    db_cfg->features = APP_DIS_FEATURES;

    // Send the message
    ke_msg_send(req);
    
    app_dis_init();
}

/* Public variables ---------------------------------------------------------*/

/// Default State handlers definition
const struct ke_msg_handler app_dis_msg_handler_list[] =
{
    {DISS_VALUE_REQ_IND,     (ke_msg_func_t)diss_value_req_ind_handler},
};

const struct app_subtask_handlers app_dis_handlers = APP_HANDLERS(app_dis);


#endif //BLE_APP_DIS

/// @} APP
