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
 * @file app_blps.c
 * @version v1.0.0
 *
  */

/** 
 * @addtogroup APP
 * @{ 
 */

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_BLPS)
/* Includes ------------------------------------------------------------------*/
#include "hp_ble.h"                     // Application Manager Definitions
#include "app_blps.h"                // BLPS Service Application Definitions
#include "blps_task.h"               // BLPS Profile Functions
#include "prf_types.h"               // Profile Common Types Definitions
#include "ke_task.h"                 // Kernel
#include "gapm_task.h"               // GAP Manager Task API
#include <string.h>
#include "co_utils.h"
#include "hp_timer.h"
#include "blps.h"
#include "hp_ble_task.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLOOD_PRESSURE_SEND_DELAY 1000
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
struct bps_bp_meas  app_meas;
    

void app_blood_pressure_timeout_handler(void)
{
    //send simulated data
    app_blps_measurement_send(app_env.conidx, &app_meas);
    
    //update simulated data  
    app_meas.diastolic -= 1;
    app_meas.systolic  += 1;
    app_meas.mean_arterial_pressure -= 1;
    app_meas.pulse_rate += 1;
    
    //update time_stamp 
    app_meas.time_stamp.sec++;
    if(app_meas.time_stamp.sec>59)
    {
        app_meas.time_stamp.sec = 0;
        app_meas.time_stamp.min++;
        if(app_meas.time_stamp.min>59) 
        {
            app_meas.time_stamp.min = 0;
        }
    }
    
    //reset the timer
    if(ke_state_get(TASK_APP)== APP_CONNECTED)
    {
        hp_timer_create(BLOOD_PRESSURE_SEND_DELAY,app_blood_pressure_timeout_handler);
    }
}




/**
 * @brief  BLPS config indicate handler
 * @param  
 * @return 
 * @note   
 */
static int blps_cfg_indntf_ind_handler(ke_msg_id_t const msgid,
                                          struct blps_cfg_indntf_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    if(param->char_code == BPS_BP_MEAS_CHAR)
    {
        if(param->cfg_val & PRF_CLI_START_IND)
        {
            HP_LOG_INFO("BPS_BP_MEAS_CHAR START_IND\r\n");
            hp_timer_create(BLOOD_PRESSURE_SEND_DELAY,app_blood_pressure_timeout_handler);
        }
        else
        {
            HP_LOG_INFO("BPS_BP_MEAS_CHAR STOP\r\n");
        }
    }
    else if(param->char_code == BPS_INTM_CUFF_MEAS_CHAR)
    {
        if(param->cfg_val & PRF_CLI_START_NTF)
        {
            HP_LOG_INFO("BPS_INTM_CUFF_MEAS_CHAR START_IND\r\n");
        }
        else
        {
            HP_LOG_INFO("BPS_INTM_CUFF_MEAS_CHAR STOP\r\n");
        }
    }
    else if(param->char_code == BPS_RACP_CHAR)
    {
        if(param->cfg_val & PRF_CLI_START_IND)
        {
            HP_LOG_INFO("BPS_RACP_CHAR START_IND\r\n");
        }
        else
        {
            HP_LOG_INFO("BPS_RACP_CHAR STOP\r\n");
        }
    }

    return KE_MSG_CONSUMED;
}


/**
 * @brief  enable blood pressure sensor
 * @param  
 * @return 
 * @note   
 */
void app_blps_enable_prf(uint8_t conidx)
{
    // Allocate the message
    struct blps_enable_req * req = KE_MSG_ALLOC(BLPS_ENABLE_REQ,
                                                prf_get_task_from_id(TASK_ID_BLPS),
                                                TASK_APP,
                                                blps_enable_req);

    // Fill in the parameter structure
    req->conidx             = conidx;

    req->bp_meas_ind_en     = PRF_CLI_STOP_NTFIND;
    req->interm_cp_ntf_en   = PRF_CLI_STOP_NTFIND;
    req->racp_ind_en        = PRF_CLI_STOP_NTFIND;
    
    // Send the message
    ke_msg_send(req);
}

/**
 * @brief  blood pressure sensor Initialize
 * @param  
 * @return 
 * @note
 */
void app_blps_init(void)
{
    //Initialize simulated data 
    app_meas.flags   = BPS_MEAS_PULSE_RATE_BIT|BPS_MEAS_FLAG_TIME_STAMP_BIT;
    app_meas.systolic               = 115;
    app_meas.diastolic              = 74;
    app_meas.mean_arterial_pressure = 102;
    app_meas.pulse_rate             = 61;
    //time stamp
    app_meas.time_stamp.year        = 2022;
    app_meas.time_stamp.month       = 3;
    app_meas.time_stamp.day         = 21;
    app_meas.time_stamp.hour        = 17;
    app_meas.time_stamp.sec         = 55;
    
    //register application subtask to app task
    struct prf_task_t prf;
    prf.prf_task_id = TASK_ID_BLPS;
    prf.prf_task_handler = &app_blps_handlers;
    hp_ble_prf_task_register(&prf);
    
    //register get itf function to prf.c
    struct prf_get_func_t get_func;
    get_func.task_id = TASK_ID_BLPS;
    get_func.prf_itf_get_func = blps_prf_itf_get;
    prf_get_itf_func_register(&get_func);
    
}

/**
 * @brief  add blood pressure sensor service
 * @param  
 * @return 
 * @note   
 */
void app_blps_add_blps(void)
{
    struct blps_db_cfg* db_cfg;
    
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                  TASK_GAPM, TASK_APP,
                                                  gapm_profile_task_add_cmd, sizeof(struct blps_db_cfg));
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl =  PERM(SVC_AUTH, NO_AUTH);

    req->prf_task_id = TASK_ID_BLPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct blps_db_cfg* ) req->param;
    
    db_cfg->prfl_cfg = APP_BLPS_INTM_CUFF_PRESS_SUP | APP_BLPS_RACP_SUP;
    db_cfg->features = APP_BLPS_SUP_FEATUEES;
    
    // Send the message
    ke_msg_send(req);
    
    app_blps_init();
}

/**
 * @brief  send blood pressure value 
 * @param  
 * @return 
 * @note   
 */
void app_blps_measurement_send(uint8_t conidx, struct bps_bp_meas *data)
{    
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);

    if (GETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_BP_MEAS_IND_CFG))
    {
        // Allocate the message
        struct blps_meas_send_cmd * cmd = KE_MSG_ALLOC(BLPS_MEAS_SEND_CMD,
                                                            prf_get_task_from_id(TASK_ID_BLPS),
                                                            TASK_APP,
                                                            blps_meas_send_cmd);
        
        // Fill in the parameter structure
        cmd->conidx = conidx;
        cmd->flag_interm_cp = 0;        //refer: flag_interm_cp
        memcpy(&(cmd->meas_val), data, sizeof(struct bps_bp_meas));

        // Send the message
        ke_msg_send(cmd);
    }
    else
    {
        HP_LOG_INFO("BP_MEAS_SEND ERR\r\n");
    }
}

/**
 * @brief  send intermediate cuff pressure value 
 * @param  
 * @return 
 * @note   
 */
void app_blps_intm_cuff_press_send(uint8_t conidx, struct bps_bp_meas *data)
{    
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);

    if (GETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_INTM_CUFF_PRESS_NTF_CFG))
    {
        // Allocate the message
        struct blps_meas_send_cmd * cmd = KE_MSG_ALLOC(BLPS_MEAS_SEND_CMD,
                                                            prf_get_task_from_id(TASK_ID_BLPS),
                                                            TASK_APP,
                                                            blps_meas_send_cmd);

        // Fill in the parameter structure
        cmd->conidx = conidx;
        cmd->flag_interm_cp = 1;        //refer: flag_interm_cp
        memcpy(&(cmd->meas_val), data, BLPS_BP_MEAS_MAX_LEN);

        // Send the message
        ke_msg_send(cmd);
    }
    else
    {
        HP_LOG_INFO("BP_INTF_CUFF_SEND ERR\r\n");
    }
}

/**
 * @brief  send record access control point response value 
 * @param  
 * @return 
 * @note
 */
void app_blps_racp_resp_send(uint8_t conidx, uint8_t *data, uint8_t data_len)
{    
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);

    if (GETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_RACP_IND_CFG))
    {
        // Allocate the message
        struct blps_racp_resp_send_cmd * cmd = KE_MSG_ALLOC_DYN(BLPS_RACP_RESP_SEND_CMD,
                                                            prf_get_task_from_id(TASK_ID_BLPS),
                                                            TASK_APP,
                                                            blps_racp_resp_send_cmd,
                                                            BLPS_RACP_MAX_LEN);
        
        
        // Fill in the parameter structure
        cmd->conidx = conidx;
        cmd->write_val_len = data_len;
        
        memcpy(&cmd->write_val, data, data_len);

        // Send the message
        ke_msg_send(cmd);
    }
    else
    {
        HP_LOG_INFO("BP_RACP_SEND ERR\r\n");
    }
}

/**
 * @brief  BLPS enable response handler
 * @param  
 * @return 
 * @note   
 */
static int blps_enable_rsp_handler(ke_msg_id_t const msgid,
                                          struct blps_enable_rsp const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    HP_LOG_INFO("BLPS PROFILE ENABLE STATUS: %x\r\n", param->status);
    return KE_MSG_CONSUMED;
}

/**
 * @brief  BLPS RACP write handler
 * @param  
 * @return 
 * @note   
 */
static int blps_racp_write_ind_handler(ke_msg_id_t const msgid,
                                          struct blps_racp_write_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    HP_LOG_INFO("RACP length: %x\r\n", param->write_val_len);
    HP_LOG_INFO("RACP opcode: %x\r\n", param->write_val.opcode);
    HP_LOG_INFO("RACP operator: %x\r\n", param->write_val.op_operator);
    
    if(param->write_val_len >= 3)
    {
        HP_LOG_INFO("RACP operand: %x\r\n", param->write_val.operand);
        
        HP_LOG_INFO("RACP data: ");
        for(uint8_t i=0; i<(param->write_val_len-3); i++)
        {
            HP_LOG_INFO("%x ", param->write_val.data[i]);
        }
        HP_LOG_INFO("\r\n");
    }
    
    return KE_MSG_CONSUMED;
}

/**
 * @brief  BLPS RACP write handler
 * @param  
 * @return 
 * @note   
 */
static int blps_cmp_evt_handler(ke_msg_id_t const msgid,
                                          struct blps_cmp_evt const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    
    HP_LOG_INFO("BLPS CMP operation: %x, code: %x, status: %x \r\n ", \
                        param->operation, param->operation_code, param->status);
    
    return KE_MSG_CONSUMED;
}

/// Default State handlers definition
const struct ke_msg_handler app_blps_msg_handler_list[] =
{
    {BLPS_ENABLE_RSP,         (ke_msg_func_t)blps_enable_rsp_handler},
    {BLPS_CFG_INDNTF_IND,     (ke_msg_func_t)blps_cfg_indntf_ind_handler},
    {BLPS_RACP_WRITE_IND,     (ke_msg_func_t)blps_racp_write_ind_handler},
    {BLPS_CMP_EVT,            (ke_msg_func_t)blps_cmp_evt_handler},
};

const struct app_subtask_handlers app_blps_handlers = APP_HANDLERS(app_blps);


#endif //BLE_APP_DIS

/// @} APP
