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
 * @file app_fifo.c
 * @version v1.0.0
 *
  */
#include "app_fifo.h"
#include "app_rdtss.h"
#include "rwprf_config.h"
#include "string.h"
#include "ke_timer.h"
#include "rdtss_task.h"

/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t  master_rx_fifo_buf[MASTER_RX_FIFO_SIZE] = {0};
uint32_t master_rx_fifo_in = 0;
uint32_t master_rx_fifo_out = 0;
bool sending_to_master;
uint16_t num_send_to_peer_master = 20;      //MTU-3

/**
 * @brief  pop own master RX data, send data to peer master
 */
void pop_matser_rx_to_ble(void)
{
    uint32_t in_temp;
    uint16_t ble_send_len;
    
    in_temp = master_rx_fifo_in;
    if(master_rx_fifo_out < in_temp)
    {
        ble_send_len = in_temp-master_rx_fifo_out;
    }
    else if(master_rx_fifo_out > in_temp){
        ble_send_len = MASTER_RX_FIFO_SIZE-master_rx_fifo_out;
    }
    else if(master_rx_fifo_out == in_temp){
        // fifo empty, stop send loop
        sending_to_master = false;
        return;
    }
    if(ble_send_len > num_send_to_peer_master)
    {
        ble_send_len = num_send_to_peer_master;
    }
    sending_to_master = true;    
    #if (BLE_RDTSS_SERVER)
    rdtss_send_notify(  &master_rx_fifo_buf[master_rx_fifo_out], ble_send_len);
    #endif
    master_rx_fifo_out = (master_rx_fifo_out+ble_send_len)%MASTER_RX_FIFO_SIZE;

    return;
}

/**
 * @brief  push master RX data to fifo and active ble send first package if not active yet
 */
uint8_t push_master_rx_data_to_fifo(const uint8_t *p_data, uint16_t len)
{
    uint32_t in_len;
    //store data in fifo
    while(len)
    {
        if(master_rx_fifo_in >= master_rx_fifo_out )
        {
            in_len = MASTER_RX_FIFO_SIZE-master_rx_fifo_in;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&master_rx_fifo_buf[master_rx_fifo_in],p_data,in_len);
            len = len-in_len;
            p_data += in_len;
            master_rx_fifo_in = (master_rx_fifo_in + in_len)%MASTER_RX_FIFO_SIZE;
            
        }
        else if(master_rx_fifo_in < master_rx_fifo_out )
        {
            in_len = master_rx_fifo_out-master_rx_fifo_in-1;
            if(in_len > len)
            {
                in_len = len;
            }
            memcpy(&master_rx_fifo_buf[master_rx_fifo_in],p_data,in_len);
            len = len-in_len;
            master_rx_fifo_in = (master_rx_fifo_in + in_len)%MASTER_RX_FIFO_SIZE;
            
            //fifo full,drop the rest data 
            if(len)
            {
                HP_LOG_WARNING("F:%d,%d,%d\r\n",len,master_rx_fifo_in,master_rx_fifo_out);
            }
            break;
        }
    }
    
    if(master_rx_fifo_in != master_rx_fifo_out)       //data need to send
    {
        if(!sending_to_master)
        {
            ke_timer_set(RDTSS_VAL_NTF_CFM, TASK_APP, 10);
        }
    }

    return len;
}



/**
 * @}
 */
