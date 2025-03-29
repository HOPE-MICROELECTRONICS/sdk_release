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
 * @file main.c
 * @version v1.0.1
 *
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "dfu_led.h"
#include "cmt453x.h"
#include "hp_scheduler.h"
#include "hp_dfu_boot.h"
#include "dfu_usart.h"
#include "dfu_delay.h"
#include "hp_error.h"




static uint32_t serial_send_data(uint8_t *p_data, uint32_t length);




/**
 * @brief  main function
 * @param   
 * @return 
 * @note   Note
 */
int main(void)
{
    PWR->VTOR_REG = CURRENT_APP_START_ADDRESS | 0x80000000;
    
    dfu_leds_config();
    if(CURRENT_APP_START_ADDRESS == HP_APP1_START_ADDRESS){
        dfu_led_on(LED1_GPIO_PORT, LED_GPIO1_PIN);
    }else if(CURRENT_APP_START_ADDRESS == HP_APP2_START_ADDRESS){
        dfu_led_on(LED2_GPIO_PORT, LED_GPIO2_PIN);
    }    
    
    HP_SCHED_INIT(256, 16);    

    Qflash_Init();
    dfu_usart1_interrupt_config();
    dfu_usart1_enable();    
    while(1)
    {
        app_sched_execute();
        __WFE();
        __SEV();
        __WFI();    
    }
}


static uint8_t m_buffer[256];
#define SCHED_EVT_RX_DATA            1
#define  DFU_SERIAL_CMD_JumpToMasterBoot        0x07

/**
 * @brief Process data received from serial port.
 * @param[in] p_event_data event type.
 * @param[in] event_size event size.
 * @return none
 */
static void sched_evt(void * p_event_data, uint16_t event_size)
{
    switch(*(uint8_t *)p_event_data)
    {
        case SCHED_EVT_RX_DATA:{        
            if(m_buffer[0] == 0xAA)
            {
                switch(m_buffer[1]){
                    case DFU_SERIAL_CMD_JumpToMasterBoot:{
                        if(m_buffer[2] == 0x01 && m_buffer[3] == 0x02 && m_buffer[4] == 0x03)
                        {
                            uint8_t cmd[] = {0xAA,DFU_SERIAL_CMD_JumpToMasterBoot,0};
                            serial_send_data(cmd, sizeof(cmd));
                            
                            if(hp_dfu_boot_force_usart_dfu() == false){
                                uint8_t cmd[] = {0xAA,DFU_SERIAL_CMD_JumpToMasterBoot,2};
                                serial_send_data(cmd, sizeof(cmd));                            
                            
                            }
                        }else
                        {
                            uint8_t cmd[] = {0xAA,DFU_SERIAL_CMD_JumpToMasterBoot,1};
                            serial_send_data(cmd, sizeof(cmd));                        
                        
                        }
                    }break;
                }
            }
        }break;
    }
}




void USART1_IRQHandler(void)
{
    static uint32_t index = 0;
    static uint8_t buffer[256];
    
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXDNE) != RESET)
    {
        buffer[index] = USART_ReceiveData(USART1);
        
        if(buffer[0] == 0xAA)
            {
                index++;
                if(index >= 256)
                {
                    index = 0;    
                    memset(m_buffer,0,sizeof(m_buffer));
                    memcpy(m_buffer,buffer, 256);
                    
                    uint8_t event = SCHED_EVT_RX_DATA;
                    uint32_t    err_code = app_sched_event_put(&event ,sizeof(uint8_t),sched_evt);
                    ERROR_CHECK(err_code);
                }                            
            }        
        
        
    }    
}



static uint32_t serial_send_data(uint8_t *p_data, uint32_t length)
{
    static uint8_t cmd[256];
    memset(cmd,0,sizeof(cmd));
    memcpy(cmd,p_data,length);
    dfu_usart1_send(cmd,sizeof(cmd));
    
    return true;
}


