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
 * @file main.c
 * @version v1.0.2
 *
  */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "cmt453x.h"
#include "dfu_crc.h"
#include "ble_stack_common.h"
#include "hp_ble.h"
#include "prf.h"
#include "hp_dfu_boot.h"


/**
 * @brief Set the bootsetting to default value, so next time boot up will not goin to imageupdate again.
 * @param[in] none.
 * @return none
 */
static void bootsetting_reset(void)
{
    HP_Bootsetting_t m_hp_bootsetting;
    memcpy(&m_hp_bootsetting,&hp_bootsetting,sizeof(HP_Bootsetting_t));    
    Qflash_Init();
    if(hp_bootsetting.app1.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.app1.start_address), hp_bootsetting.app1.size) )
    {
        m_hp_bootsetting.app1.activation = HP_BOOTSETTING_ACTIVATION_YES;
        m_hp_bootsetting.ImageUpdate.activation = HP_BOOTSETTING_ACTIVATION_NO;
    }
    else if(hp_bootsetting.app2.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.app2.start_address), hp_bootsetting.app2.size) )
    {
        m_hp_bootsetting.app2.activation = HP_BOOTSETTING_ACTIVATION_YES;
        m_hp_bootsetting.ImageUpdate.activation = HP_BOOTSETTING_ACTIVATION_NO;
    }
    if(m_hp_bootsetting.ImageUpdate.activation == HP_BOOTSETTING_ACTIVATION_NO)
    {
        m_hp_bootsetting.crc = dfu_crc32((uint8_t *)&m_hp_bootsetting.crc + 4, sizeof(HP_Bootsetting_t) - 4);
        Qflash_Erase_Sector(HP_BOOTSETTING_START_ADDRESS);
        Qflash_Write(HP_BOOTSETTING_START_ADDRESS, (uint8_t *)&m_hp_bootsetting, sizeof(HP_Bootsetting_t));        
    }
}

int main(void)
{
    bootsetting_reset();
    
    app_init();
    prf_init(RWIP_INIT);

    while(1)
    {
        rwip_schedule();        
    }
}


