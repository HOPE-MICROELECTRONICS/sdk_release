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
 * @version v1.0.4
 *
  */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "cmt453x.h"
#include "dfu_led.h"
#include "hp_dfu_boot.h"
#include "dfu_crc.h"
#include "hp_scheduler.h"
#include "hp_dfu_serial.h"
#include "dfu_usart.h"

/**
 * @brief Loop up the bootsetting and decide which bank to jump to , or stay in masterboot for usart image update.
 * @param[in] none.
 * @return none
 */
static void masterboot(void)
{
    uint32_t jump_address = NULL;
    if(hp_bootsetting.crc == dfu_crc32((uint8_t *)&hp_bootsetting.crc + 4, sizeof(HP_Bootsetting_t) - 4))
    {
        if(hp_bootsetting.master_boot_force_update != HP_BOOTSETTING_MASTER_BOOT_FORCE_UPDATE_YES)
        {
            if(hp_bootsetting.app1.activation == HP_BOOTSETTING_ACTIVATION_YES && hp_bootsetting.app1.start_address == HP_APP1_START_ADDRESS)
            {
                if(hp_bootsetting.app1.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.app1.start_address), hp_bootsetting.app1.size))
                {
                    jump_address = hp_bootsetting.app1.start_address;
                }
            }    
            else if(hp_bootsetting.app2.activation == HP_BOOTSETTING_ACTIVATION_YES && hp_bootsetting.app2.start_address == HP_APP2_START_ADDRESS)
            {
                if(hp_bootsetting.app2.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.app2.start_address), hp_bootsetting.app2.size))
                {
                    jump_address = hp_bootsetting.app2.start_address;
                }
            }    
            else if(hp_bootsetting.ImageUpdate.activation == HP_BOOTSETTING_ACTIVATION_YES && hp_bootsetting.ImageUpdate.start_address == HP_IMAGE_UPDATE_START_ADDRESS)
            {
                if(hp_bootsetting.ImageUpdate.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.ImageUpdate.start_address), hp_bootsetting.ImageUpdate.size))
                {
                    jump_address = hp_bootsetting.ImageUpdate.start_address;
                }
            }
            else{
                if(hp_bootsetting.app1.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.app1.start_address), hp_bootsetting.app1.size))
                {
                    jump_address = hp_bootsetting.app1.start_address;
                }        
                else if(hp_bootsetting.app2.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.app2.start_address), hp_bootsetting.app2.size))
                {
                    jump_address = hp_bootsetting.app2.start_address;
                }            
                else if(hp_bootsetting.ImageUpdate.crc == dfu_crc32((uint8_t *)((uint32_t *)hp_bootsetting.ImageUpdate.start_address), hp_bootsetting.ImageUpdate.size))
                {
                    jump_address = hp_bootsetting.ImageUpdate.start_address;
                }
            }
        }
    }
    
    if(jump_address != NULL)
    {
        dfu_usart1_default_config();
        hp_dfu_boot_jump(jump_address);
    }
}

#define POLL_UART_CYCLE (0x6FFF) //20ms
/**
 * @brief  main function
 * @param   
 * @return 
 * @note   Note
 */
int main(void)
{
    /// swd -> uart
    dfu_usart1_poll_config(USART1_GPIO_PA45,1000000);
    if(dfu_usart1_poll_boot_byte(0x7F, POLL_UART_CYCLE) == false)
    { 
        dfu_usart1_poll_config(USART1_GPIO_PA45,115200);
        if(dfu_usart1_poll_boot_byte(0x7F, POLL_UART_CYCLE) == false)
        { 
            dfu_usart1_poll_config(USART1_GPIO_PB67,1000000);
            if(dfu_usart1_poll_boot_byte(0x7F, POLL_UART_CYCLE) == false)
            { 
                dfu_usart1_poll_config(USART1_GPIO_PB67,115200);
                if(dfu_usart1_poll_boot_byte(0x7F, POLL_UART_CYCLE) == false)
                {
                    //normal boot
                    masterboot();
                }
            }
        }
    }
    HP_SCHED_INIT(256, 16);    
    hp_dfu_serial_init();
    
    while(1)
    {
        app_sched_execute();
        __WFE();
        __SEV();
        __WFE();    
    }
}





