/***************************************************************************//**
* # License
* Copyright 2023 Shenzhen HOPE Microelectronics Co., Ltd. 
* All rights reserved.
* 
* IMPORTANT: All rights of this software belong to Shenzhen HOPE Microelectronics 
* CO., Ltd. ("HOPERF"). Your use of this Software is limited to those 
* specific rights granted under the terms of the business contract, the 
* confidential agreement, the non-disclosure agreement and any other forms 
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
 * @file user_rf.h
 
 * @version v1.0.1
 *
  */
 
 
#ifndef __USER_RF_H__
#define __USER_RF_H__
//------------------------------------------------------------------
//                        Headers
//------------------------------------------------------------------
#include <stdio.h>
#include "cmt453x.h"



struct lld_user_rx_params
{
    uint8_t channel;
    uint8_t phy;
    uint16_t rx_wind;
    uint16_t aa_h;
    uint16_t aa_l;
    uint8_t  crc_init[3];
};

struct lld_user_tx_params
{
    uint8_t  channel;
    uint8_t  phy;
    uint16_t aa_h;
    uint16_t aa_l;
    uint8_t  crc_init[3];
};

struct lld_user_tx_data
{
    uint16_t length;
    uint8_t data[264];
};    

struct rx_data_tag
{
    uint8_t  rx_stat;
    uint8_t  rx_rssi;
    uint16_t data_len;
    uint8_t  data[264]; 
};    

void lld_user_rx_start(void);
void lld_user_rx_init(struct lld_user_rx_params * p_param);
void lld_user_tx_start(struct lld_user_tx_data* params);
void lld_user_tx_init(struct lld_user_tx_params * p_param);

extern struct rx_data_tag  user_rx_data;

void lld_user_rx_stop(bool force);

#endif /*__USER_RF_H__*/
