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
 * @file app_ble.c
 * @version v1.0.4
 *
  */
#include <string.h>
#include "cmt453x.h"
#include "gapm_task.h"               // GAP Manager Task API
#include "app_ble.h"
#include "app_usart.h"
#include "app_gpio.h"
#include "hp_sec.h"
#include "app_rdtsc.h"
#include "app_user_config.h"
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void app_ble_connected(void);
void app_ble_disconnected(void);

/**
 * @brief  user message handler
 * @param  
 * @return 
 * @note   
 */
void app_user_msg_handler(ke_msg_id_t const msgid, void const *p_param)
{
    
    switch (msgid)
    {
    	case APP_UART_TX_EVT:
            app_usart_tx_process();
    		break;
//    	case :
//    		break;
    	default:
    		break;
    }
 

}

/**
 * @brief  ble message handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_msg_handler(struct ble_msg_t const *p_ble_msg)
{
    switch (p_ble_msg->msg_id)
    {
        case APP_BLE_OS_READY:
            HP_LOG_INFO("APP_BLE_OS_READY\r\n");
            break;
        case APP_BLE_GAP_CONNECTED:
            app_rdtsc_enable_prf(app_env.conidx);
            app_ble_connected();
            break;
        case APP_BLE_GAP_DISCONNECTED:
            app_ble_disconnected();
            break;
         case APP_BLE_GAP_PARAMS_IND:
             // restart scan if not reach max connection
            if (hp_ble_get_connection_num() < BLE_MASTER_CONN)
            {
                HP_LOG_INFO("restart scan\r\n");
                hp_ble_start_scan();
            }
            break;
        default:
            break;
    }

}

/**
 * @brief  scan data handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_scan_data_handler(struct gapm_ext_adv_report_ind const* p_param)
{
//    HP_LOG_INFO("Found:%02x%02x%02x%02x%02x%02x, rssi:%d \r\n",
//                              p_param->trans_addr.addr.addr[0],
//                              p_param->trans_addr.addr.addr[1],
//                              p_param->trans_addr.addr.addr[2],
//                              p_param->trans_addr.addr.addr[3],
//                              p_param->trans_addr.addr.addr[4],
//                              p_param->trans_addr.addr.addr[5],
//                              p_param->rssi);

}

/**
 * @brief  scan state handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_scan_state_handler(enum scan_state_t state)
{
    switch (state)
    {
        case SCAN_STATE_SCANING:
            HP_LOG_INFO("Scaning.\r\n");
            break;
        case SCAN_STATE_FOUND:
            HP_LOG_INFO("Found target.\r\n");
            break;
        case SCAN_STATE_CONN_TIMEOUT:
            HP_LOG_INFO("Connect timeout.\r\n");
            break;
        default:
            break;
    }
}
    

/**
 * @brief  ble GAP initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_gap_params_init(void)
{
    struct hp_gap_params_t dev_info = {0};
    uint8_t *p_mac = SystemGetMacAddr();
    //get UUID from trim stored
    if(p_mac != NULL)
    {
        //set the uuid as mac address
        memcpy(dev_info.mac_addr.addr, p_mac , BD_ADDR_LEN); 
    }
    else{
        memcpy(dev_info.mac_addr.addr, "\x01\x02\x03\x0C\x05\x06" , BD_ADDR_LEN);
    }
    

    /* init params*/
    dev_info.mac_addr_type = GAPM_STATIC_ADDR;
    dev_info.appearance = 0;
    dev_info.dev_role = GAP_ROLE_ALL;
    
    dev_info.dev_name_len = sizeof(CUSTOM_DEVICE_NAME)-1;
    memcpy(dev_info.dev_name, CUSTOM_DEVICE_NAME, dev_info.dev_name_len); 
   
    dev_info.dev_conn_param.intv_min = MSECS_TO_UNIT(MIN_CONN_INTERVAL,MSECS_UNIT_1_25_MS);
    dev_info.dev_conn_param.intv_max = MSECS_TO_UNIT(MAX_CONN_INTERVAL,MSECS_UNIT_1_25_MS);
    dev_info.dev_conn_param.latency  = SLAVE_LATENCY;
    dev_info.dev_conn_param.time_out = MSECS_TO_UNIT(CONN_SUP_TIMEOUT,MSECS_UNIT_10_MS);
    dev_info.conn_param_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    
    hp_ble_gap_init(&dev_info);
    
}


/**
 * @brief  ble scan initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_scan_init(void)
{
    struct hp_scan_params_t init = {0};
//    static const uint8_t target_addr[] = {"\x11\x11\x11\x11\x11\x11"};
    static const uint8_t target_name[] = {"HP_RDTS_P"};
//    static const uint16_t target_uuid16 = 0x180A;
    
    init.type               = SCAN_PARAM_TYPE;
    init.dup_filt_pol       = SCAN_PARAM_DUP_FILT_POL;
    init.connect_enable     = SCAN_PARAM_CONNECT_EN;
    init.prop_active_enable = SCAN_PARAM_PROP_ACTIVE;
    init.scan_intv          = SCAN_PARAM_INTV;
    init.scan_wd            = SCAN_PARAM_WD;
    init.duration           = SCAN_PARAM_DURATION;
    init.initiating_timeout = 3000;
    
    init.filter_type    = SCAN_FILTER_BY_NAME;//SCAN_FILTER_BY_ADDRESS;
    init.filter_data    = (uint8_t*)&target_name;
    
    init.ble_scan_data_handler  = app_ble_scan_data_handler;
    init.ble_scan_state_handler = app_ble_scan_state_handler;
    
    hp_ble_scan_init(&init);
}

void app_ble_sec_init(void)
{
    struct hp_sec_init_t sec_init = {0};
    
    sec_init.rand_pin_enable = false;
    sec_init.pin_code = 123456;
    
    sec_init.pairing_feat.auth      = ( SEC_PARAM_BOND | (SEC_PARAM_MITM<<2) | (SEC_PARAM_LESC<<3) | (SEC_PARAM_KEYPRESS<<4) );
    sec_init.pairing_feat.iocap     = SEC_PARAM_IO_CAPABILITIES;
    sec_init.pairing_feat.key_size  = SEC_PARAM_KEY_SIZE;
    sec_init.pairing_feat.oob       = SEC_PARAM_OOB;
    sec_init.pairing_feat.ikey_dist = SEC_PARAM_IKEY;
    sec_init.pairing_feat.rkey_dist = SEC_PARAM_RKEY;
    sec_init.pairing_feat.sec_req   = SEC_PARAM_SEC_MODE_LEVEL;
    
    sec_init.bond_enable            = BOND_STORE_ENABLE;
    sec_init.bond_db_addr           = BOND_DATA_BASE_ADDR;
    sec_init.bond_max_peer          = MAX_BOND_PEER;
    sec_init.bond_sync_delay        = 2000;
    
    sec_init.hp_sec_msg_handler     = NULL;
    
    hp_sec_init(&sec_init);
}

void app_ble_prf_init(void)
{
    //add raw data transmit server(rdts)
    hp_ble_add_prf_func_register(app_rdtsc_add_rdts);
}


/**
 * @brief  ble initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_init(void)
{

    struct hp_stack_cfg_t app_handler = {0};
    app_handler.ble_msg_handler  = app_ble_msg_handler;
    app_handler.user_msg_handler = app_user_msg_handler;
    //initialization ble stack
    hp_ble_stack_init(&app_handler);
    
    app_ble_gap_params_init();
    app_ble_sec_init();
    app_ble_scan_init();
    app_ble_prf_init();
    //start scan
    hp_ble_start_scan();
}

/**
 * @brief  ble connected
 * @param  
 * @return 
 * @note   
 */
void app_ble_connected(void)
{
    //enable usart receive
    app_usart_dma_enable(ENABLE);
    LedOn(LED2_PORT,LED2_PIN);   
    
    #if (BLE_APP_BATT)
    // Enable Battery Service
    app_batt_enable_prf(app_env.conidx);
    #endif //(BLE_APP_BATT)
}

/**
 * @brief  ble disconnected
 * @param  
 * @return 
 * @note   
 */
void app_ble_disconnected(void)
{
    // Restart scan

    if (hp_ble_get_connection_num() < BLE_MASTER_CONN)
    {
        hp_ble_start_scan();
    }
    if (hp_ble_get_connection_num() == 0)
    {		
    //disable usart receive
    app_usart_dma_enable(DISABLE);
    LedOff(LED2_PORT,LED2_PIN);
    }
}





/**
 * @}
 */
