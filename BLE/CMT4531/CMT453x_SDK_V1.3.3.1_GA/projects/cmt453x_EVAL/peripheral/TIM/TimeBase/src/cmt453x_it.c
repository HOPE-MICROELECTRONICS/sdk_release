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
 * @version v1.0.1
 *
  */
#include "cmt453x_it.h"
#include "main.h"

/** @addtogroup CMT453X_StdPeriph_Template
 * @{
 */

uint16_t capture = 0;
extern __IO uint16_t CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern __IO uint16_t CCR3_Val;
extern __IO uint16_t CCR4_Val;

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

/******************************************************************************/
/*                 CMT453X Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_cmt453x.s).                                                 */
/******************************************************************************/

/**
 * @brief  This function handles TIM3 global interrupt request.
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM3, TIM_INT_CC1) != RESET)
    {
        TIM_ClrIntPendingBit(TIM3, TIM_INT_CC1);

        /* Pin PB.06 toggling with frequency = 88.88 Hz */
        GPIO_WriteBit(GPIOB, GPIO_PIN_6, (Bit_OperateType)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_PIN_6)));
        capture = TIM_GetCap1(TIM3);
        TIM_SetCmp1(TIM3, capture + CCR1_Val);
    }
    else if (TIM_GetIntStatus(TIM3, TIM_INT_CC2) != RESET)
    {
        TIM_ClrIntPendingBit(TIM3, TIM_INT_CC2);

        /* Pin PB.07 toggling with frequency = 133.33 Hz */
        GPIO_WriteBit(GPIOB, GPIO_PIN_7, (Bit_OperateType)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_PIN_7)));
        capture = TIM_GetCap2(TIM3);
        TIM_SetCmp2(TIM3, capture + CCR2_Val);
    }
    else if (TIM_GetIntStatus(TIM3, TIM_INT_CC3) != RESET)
    {
        TIM_ClrIntPendingBit(TIM3, TIM_INT_CC3);

        /* Pin PB.08 toggling with frequency = 266.66 Hz */
        GPIO_WriteBit(GPIOB, GPIO_PIN_11, (Bit_OperateType)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_PIN_11)));
        capture = TIM_GetCap3(TIM3);
        TIM_SetCmp3(TIM3, capture + CCR3_Val);
    }
    else
    {
        TIM_ClrIntPendingBit(TIM3, TIM_INT_CC4);

        /* Pin PB.09 toggling with frequency = 666.66 Hz */
        GPIO_WriteBit(GPIOB, GPIO_PIN_12, (Bit_OperateType)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_PIN_12)));
        capture = TIM_GetCap4(TIM3);
        TIM_SetCmp4(TIM3, capture + CCR4_Val);
    }
}

/**
 * @}
 */
