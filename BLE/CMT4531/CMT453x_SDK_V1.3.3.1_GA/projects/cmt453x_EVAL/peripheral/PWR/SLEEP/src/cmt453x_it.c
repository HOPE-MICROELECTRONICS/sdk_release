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
#include "cmt453x.h"
#include "main.h"

/** @addtogroup CMT453X_StdPeriph_Template
 * @{
 */

extern __IO uint32_t CurrDataCounterEnd;

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
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void)
{
}


/**
 * @brief  This function handles the PVD Output interrupt request defined in main.h .
 */

void EXTI0_1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_LINE1) != RESET)
    {
        /* Clear the Key Button EXTI line pending bit */
        EXTI_ClrITPendBit(EXTI_LINE1);
    }
}

void EXTI2_3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_LINE2) != RESET)
    {
        /* Clear the Key Button EXTI line pending bit */
        EXTI_ClrITPendBit(EXTI_LINE2);
    }
}

void EXTI4_12_IRQHandler(void)
{
    /* NRST Wakeup Interrupt through EXTI line 12 */
    if (RESET != EXTI_GetITStatus(EXTI_LINE12))
    {
        NVIC_SystemReset();
        EXTI_ClrITPendBit(EXTI_LINE12);
    }
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

/**
 * @}
 */
