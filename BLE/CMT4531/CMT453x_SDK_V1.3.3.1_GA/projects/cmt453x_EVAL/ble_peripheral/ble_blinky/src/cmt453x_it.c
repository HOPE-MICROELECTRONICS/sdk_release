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
#include "hp_log.h"
#include "app_lpuart.h"
#include "app_gpio.h"
#include "app_rtc.h"
#include "app_ble.h"
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
 * @brief  This function handles PPP interrupt request.
 */
/*void PPP_IRQHandler(void)
{
}*/



/**
 * @brief  LPUART interrupt handler
 * @param  
 * @return 
 * @note   
 */
void LPUART1_IRQHandler(void)
{
    uint8_t data; 
    while (LPUART_GetFlagStatus(LPUART1, LPUART_FLAG_FIFO_NE) != RESET)
    {
        /* Read one byte from the receive data register */
        data = LPUART_ReceiveData(LPUART1);
        app_lpuart_receive_data(&data,1);
    }
}

/**
 * @brief  EXTI4_12 interrupt handler
 * @param  
 * @return 
 * @note   
 */
void EXTI4_12_IRQHandler(void)
{
    /* EXTI_LINE10 as LPUART Wakeup */
    if ( EXTI_GetITStatus(EXTI_LINE10)!= RESET)
    {
        EXTI_ClrITPendBit(EXTI_LINE10);
    }
}


/**
 * @brief  External lines 1 interrupt. It will avtive rwip_schedule.
 */
void EXTI0_1_IRQHandler(void)
{
    if ( EXTI_GetITStatus(KEY1_INPUT_EXTI_LINE)!= RESET)
    {
        ke_msg_send_basic(APP_BUTTON_1_EVT,TASK_APP,TASK_APP);
        EXTI_ClrITPendBit(KEY1_INPUT_EXTI_LINE);
    }
}

/**
 * @brief  External lines 2 interrupt. It will avtive rwip_schedule.
 */
void EXTI2_3_IRQHandler(void)
{
    if ( EXTI_GetITStatus(KEY2_INPUT_EXTI_LINE)!= RESET)
    {
        ke_msg_send_basic(APP_BUTTON_2_EVT,TASK_APP,TASK_APP);
        EXTI_ClrITPendBit(KEY2_INPUT_EXTI_LINE);
    }
}

/**
 * @brief  This function handles RTC interrupt request.
 */

void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_INT_ALRA) != RESET)
    {
        RTC_ClrIntPendingBit(RTC_INT_ALRA);
        HP_LOG_INFO("%s->Alarm\r\n",__func__);
        RTC_TimeShow();
    }
    if (RESET != EXTI_GetITStatus(EXTI_LINE8))
    {
        EXTI_ClrITPendBit(EXTI_LINE8);
    }
    #ifndef SLEEP_LP_TIMER_ENABLE
    if (RTC_GetITStatus(RTC_INT_WUT) != RESET)
    {
        RTC_ClrIntPendingBit(RTC_INT_WUT);
        HP_LOG_INFO("%s->WakeUp\r\n",__func__);
    }
    if (RESET != EXTI_GetITStatus(EXTI_LINE9))
    {
        EXTI_ClrITPendBit(EXTI_LINE9);
    }
    #endif
}

/**
 * @}
 */
