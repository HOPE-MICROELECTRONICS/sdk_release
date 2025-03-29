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
 * @file cmt453x_it.c
 * @version v1.0.0
 *
  */
#include "cmt453x_it.h"
#include "main.h"

/** @addtogroup CMT453X_StdPeriph_Template
 * @{
 */

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void)
{
}

/**
 * @brief  This function handles DMA interrupt request defined in main.h .
 */
void DMA_IRQ_HANDLER(void)
{
}

/******************************************************************************/
/*                 CMT453X Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_cmt453x.s).                                                 */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 */
/*void PPP_IRQHandler(void)
{
}*/


void KEYSCAN_IRQHandler(void)
{
    uint32_t key_data[5] = {0};
    uint8_t  num;
    if(KEYSCAN_GetInterruptState() == SET)
    {
        KEYSCAN_ClearInterrupt();
    }
    
    if(EXTI_GetITStatus(EXTI_LINE13))
    {
        EXTI_ClrITPendBit(EXTI_LINE13);
    }
    
    KEYSCAN_ReadKeyData(key_data);
    #if 0
    log_info("REG data: ");
    log_info("0x%08x,", key_data[0]);
    log_info("0x%08x,", key_data[1]);
    log_info("0x%08x,", key_data[2]);
    log_info("0x%08x,", key_data[3]);
    log_info("0x%08x ", key_data[4]);
    log_info("\r\n");
    #endif
    
    for(num = 1; num < KEYSCAN_KEY_MAX; num++)
    {
        if(memcmp(key_data,&keyscan_map[num][0],(KEYSCAN_INFO_CHECK<<2)) == 0)
        {
            break;
        }
    }
    
    if(num != KEYSCAN_KEY_MAX)
    {
        #if 1
        extern uint16_t key_receive ;
        key_receive = num;
        #else
        log_info("Detect key %d Pressed!\r\n", num);
        #endif
    }
}
/**
 * @}
 */
