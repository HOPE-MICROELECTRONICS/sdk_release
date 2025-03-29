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
 * @file app_hp_ius.c
 * @version v1.0.1
 *
  */

/** 
 * @addtogroup APP
 * @{ 
 */
/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"     // SW configuration

#if (BLE_APP_HP_IUS)

#include "app_hp_ius.h"
#include <stdio.h>
#include "hp_iuss.h"
#include "hp_iuss_task.h"
#include "hp_dfu_ble.h"
#include "attm.h"
#include "ke_task.h"
#include "gapc.h"
#include "gapc_task.h"
#include "gattc_task.h"
#include "attm_db.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "co_utils.h"
#include "ke_msg.h"
#include "hp_ble.h"
#include "hp_log.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


struct attm_desc_128 hp_ius_att_db[HP_IUS_IDX_NB] =
{
    [HP_IUS_IDX_SVC] = {{0x00,0x28}, PERM(RD, ENABLE),0,0},

    [HP_IUS_IDX_RC_CHAR] = {{0x03,0x28}, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE),0, 0},
    [HP_IUS_IDX_RC_VAL] = {CHAR_HP_IUS_RC, PERM(WRITE_COMMAND, ENABLE), PERM_VAL(UUID_LEN, 0x02), 247},
    [HP_IUS_IDX_RC_CFG] = {{0x02,0x29}, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},

    [HP_IUS_IDX_CC_CHAR] = {{0x03,0x28}, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0 },
    [HP_IUS_IDX_CC_VAL] = {CHAR_HP_IUS_CC, PERM(WRITE_REQ, ENABLE) | PERM(NTF, ENABLE)  , PERM(RI, ENABLE)| PERM_VAL(UUID_LEN, 0x02), 20 },   
    [HP_IUS_IDX_CC_CFG] = {{0x02,0x29}, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},
};





static bool notification_enable = false;





static int hp_ius_val_write_ind_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    const struct hp_ius_val_write_ind *p_param = (const struct hp_ius_val_write_ind *)param;

    switch (p_param->handle)
    {
        case HP_IUS_IDX_CC_CFG:{
            
            if(p_param->length == 2)
            {
                uint16_t cfg_value = p_param->value[0] + p_param->value[1];
                
                if(cfg_value == PRF_CLI_START_NTF)
                {
                    notification_enable = true;
                }
                else if(cfg_value == PRF_CLI_STOP_NTFIND)
                {
                    notification_enable = false;
                }
                HP_LOG_DEBUG("notification_enable %d\r\n",notification_enable);
            }
            
        }break;
        
        
        case HP_IUS_IDX_CC_VAL:{
            uint8_t output[20];
            uint8_t output_len = 0;
            hp_dfu_ble_handler_cc(p_param->value, p_param->length, output, &output_len);
            if(output_len)hp_ble_ius_app_cc_send(output, output_len);
        }break;        
        
        case HP_IUS_IDX_RC_VAL:{
            hp_dfu_ble_handler_rc(p_param->value, p_param->length);
        }break;
        default:
            break;
        
    }    
    
    return KE_MSG_CONSUMED;
}



void hp_ble_ius_app_cc_send(uint8_t *p_data, uint16_t length)
{    
    uint8_t state = ke_state_get(hp_ble_ius_task);
    if (state == HP_IUS_BUSY || notification_enable == false)
    {
            return;
    }    
    ke_state_set(hp_ble_ius_task, HP_IUS_BUSY);    
    
    struct hp_ius_env_tag *hp_ius_env = PRF_ENV_GET(HP_IUS, hp_ius);
  struct gattc_send_evt_cmd * req = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD, KE_BUILD_ID(TASK_GATTC, app_env.conidx), hp_ius_env->prf_env.prf_task, gattc_send_evt_cmd, length);
    req->operation = GATTC_NOTIFY;
  req->handle = hp_ius_env->shdl + HP_IUS_IDX_CC_VAL;
  req->length = length;
    memcpy(req->value, p_data, length);
    ke_msg_send(req);        
}



static int hp_ius_disconnect_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    notification_enable = false;
  return KE_MSG_CONSUMED;
}


const struct ke_msg_handler hp_ius_app_msg_handler_list[] =
{
    {HP_IUS_VAL_WRITE_IND,          hp_ius_val_write_ind_handler},
    {HP_IUS_DISCONNECT,             hp_ius_disconnect_handler},    
};


const struct app_subtask_handlers hp_ius_app_handlers = APP_HANDLERS(hp_ius_app);







void app_hp_ius_add_hp_ius(void)
{
    Qflash_Init();
    
    struct hp_ius_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM,
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd,
                                                             sizeof(struct hp_ius_db_cfg));
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = PERM(SVC_AUTH, NO_AUTH);        
    req->prf_task_id = TASK_ID_HP_IUS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    db_cfg = (struct hp_ius_db_cfg *) req->param;
    db_cfg->att_tbl = NULL;
    db_cfg->cfg_flag = 0;
    db_cfg->features = 0;    
    ke_msg_send(req);

    app_hp_ius_init();
}

void app_hp_ius_init(void)
{
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_HP_IUS;
    prf.prf_task_handler = &hp_ius_app_handlers;
    hp_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_HP_IUS;
    get_func.prf_itf_get_func = hp_ius_prf_itf_get;
    prf_get_itf_func_register(&get_func);
}


#endif  //BLE_APP_HP_IUS


/// @} APP



