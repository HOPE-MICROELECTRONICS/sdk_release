/**
****************************************************************************************
*
* @file lld_user_rx_mode.c
*
* @brief LLD user_rx mode source code
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDUSER_RXMODE
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_user_config.h"

#if(HP_RF_CTRL_ENABLE)
#include "app_rf.h"
#include "user_rf.h"
#include "rwip_config.h"         // stack configuration


#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors

#include "reg_em_ble_rx_cte_desc.h" // BLE EM RX CTE descriptors

#define CHNL   20

void app_tx_start(uint8_t length, uint8_t * data)
{
    if(length < 6)
    {
        printf("app_tx_start  err_len \r\n");
        return;
    }    
    struct lld_user_tx_params tx_param;
    tx_param.aa_h = 0x6DEB;//0x8E89;//+0x100;
    tx_param.aa_l = 0x98E8;//0xBED6;
    tx_param.channel = CHNL;
    tx_param.crc_init[0] = 0x55;
    tx_param.crc_init[1] = 0x55;
    tx_param.crc_init[2] = 0x55;
    tx_param.phy = 0;   
    
    lld_user_tx_init(&tx_param);
    struct lld_user_tx_data txdata;
    txdata.length = length;
    for(int i = 0; i<length; i++)
    {
        txdata.data[i] = data[i];
    };
    lld_user_tx_start(&txdata);
}    

void app_rx_start(void)
{
    struct lld_user_rx_params rx_param;
    rx_param.aa_h = 0x6DEB;//0x8E89+0x100;
    rx_param.aa_l = 0x98E8;//0xBED6;
    rx_param.channel = CHNL;
    rx_param.crc_init[0] = 0x55;
    rx_param.crc_init[1] = 0x55;
    rx_param.crc_init[2] = 0x55;
    rx_param.phy = 0; 
    
    lld_user_rx_init(&rx_param);     
    em_ble_rxwincntl_pack(2, 1, 4);

    lld_user_rx_start();
}     

void app_rx_stop(void)
{
    lld_user_rx_stop(true);
}    

#endif // (HP_RF_CTRL_ENABLE)
