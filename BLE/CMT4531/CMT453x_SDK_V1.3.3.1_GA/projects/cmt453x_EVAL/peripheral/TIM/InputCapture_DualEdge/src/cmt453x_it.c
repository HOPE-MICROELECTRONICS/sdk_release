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

__IO uint16_t IC3ReadValueL = 0, IC3ReadValueH = 0;
__IO uint16_t CaptureNumber   = 0;
__IO uint32_t Capture         = 0, CaptureH         = 0, CaptureL         = 0;
__IO uint32_t TIM3Freq        = 0;
__IO uint32_t TIM3EnterIrqCnt = 0;

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
      /* TIM3 CH2 Rising Edge */
    if (TIM_GetIntStatus(TIM3, TIM_INT_CC2) == SET)
    {
        TIM3EnterIrqCnt++;
        /* Clear TIM3 Capture compare interrupt pending bit */
        TIM_ClrIntPendingBit(TIM3, TIM_INT_CC2);
        if (CaptureNumber == 0)
        {
            /* Get the Input Capture value */
            IC3ReadValueL = TIM_GetCap2(TIM3);
            CaptureNumber = 1;
        }
        else if (CaptureNumber == 1)
        {
            /* Get the Input Capture value */
            IC3ReadValueL = TIM_GetCap2(TIM3);

            /* Capture computation */
            if (IC3ReadValueL > IC3ReadValueH)
            {
                CaptureL = (IC3ReadValueL - IC3ReadValueH);
            }
            else
            {
                CaptureL = ((0xFFFF - IC3ReadValueH) + IC3ReadValueL);
            }
                        /* Period computation */
            Capture = CaptureH + CaptureL;
            /* Frequency computation */
            TIM3Freq = (uint32_t)SystemCoreClock / Capture;

        }
    }
    else if (TIM_GetIntStatus(TIM3, TIM_INT_CC1) == SET)
    {
        TIM3EnterIrqCnt++;
        /* Clear TIM3 Capture compare interrupt pending bit */
        TIM_ClrIntPendingBit(TIM3, TIM_INT_CC1);
        if (CaptureNumber == 0)
        {
            /* Get the Input Capture value */
            IC3ReadValueH = TIM_GetCap1(TIM3);
            CaptureNumber = 1;
        }
        else if (CaptureNumber == 1)
        {
            /* Get the Input Capture value */
            IC3ReadValueH = TIM_GetCap1(TIM3);

            /* Capture computation */
            if (IC3ReadValueH > IC3ReadValueL)
            {
                CaptureH = (IC3ReadValueH - IC3ReadValueL);
            }
            else
            {
                CaptureH = ((0xFFFF - IC3ReadValueL) + IC3ReadValueH);
            }
        }
    }
}

/**
 * @}
 */
