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

__IO uint32_t step = 1;

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
    /* Generate TIM1 COM event by software */
    TIM_GenerateEvent(TIM1, TIM_EVT_SRC_COM);
}

/******************************************************************************/
/*            cmt453x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
 * @brief  This function handles TIM1 Break Update Trigger and commutation interrupts
 *   requests.
 */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    /* Clear TIM1 COM pending bit */
    TIM_ClrIntPendingBit(TIM1, TIM_INT_COM);

    if (step == 1)
    {
        /* Next step: Step 2 Configuration ---------------------------- */
        /*  Channel3 configuration */
        TIM_EnableCapCmpCh(TIM1, TIM_CH_3, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_3, TIM_CAP_CMP_N_DISABLE);

        /*  Channel1 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_1, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_1, TIM_CAP_CMP_ENABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_1, TIM_CAP_CMP_N_DISABLE);

        /*  Channel2 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_2, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_2, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_2, TIM_CAP_CMP_N_ENABLE);
        step++;
    }
    else if (step == 2)
    {
        /* Next step: Step 3 Configuration ---------------------------- */
        /*  Channel2 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_2, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_2, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_2, TIM_CAP_CMP_N_ENABLE);

        /*  Channel3 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_3, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_3, TIM_CAP_CMP_ENABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_3, TIM_CAP_CMP_N_DISABLE);

        /*  Channel1 configuration */
        TIM_EnableCapCmpCh(TIM1, TIM_CH_1, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_1, TIM_CAP_CMP_N_DISABLE);
        step++;
    }
    else if (step == 3)
    {
        /* Next step: Step 4 Configuration ---------------------------- */
        /*  Channel3 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_3, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_3, TIM_CAP_CMP_ENABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_3, TIM_CAP_CMP_N_DISABLE);

        /*  Channel2 configuration */
        TIM_EnableCapCmpCh(TIM1, TIM_CH_2, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_2, TIM_CAP_CMP_N_DISABLE);

        /*  Channel1 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_1, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_1, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_1, TIM_CAP_CMP_N_ENABLE);
        step++;
    }
    else if (step == 4)
    {
        /* Next step: Step 5 Configuration ---------------------------- */
        /*  Channel3 configuration */
        TIM_EnableCapCmpCh(TIM1, TIM_CH_3, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_3, TIM_CAP_CMP_N_DISABLE);

        /*  Channel1 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_1, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_1, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_1, TIM_CAP_CMP_N_ENABLE);

        /*  Channel2 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_2, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_2, TIM_CAP_CMP_ENABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_2, TIM_CAP_CMP_N_DISABLE);
        step++;
    }
    else if (step == 5)
    {
        /* Next step: Step 6 Configuration ---------------------------- */
        /*  Channel3 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_3, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_3, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_3, TIM_CAP_CMP_N_ENABLE);

        /*  Channel1 configuration */
        TIM_EnableCapCmpCh(TIM1, TIM_CH_1, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_1, TIM_CAP_CMP_N_DISABLE);

        /*  Channel2 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_2, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_2, TIM_CAP_CMP_ENABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_2, TIM_CAP_CMP_N_DISABLE);
        step++;
    }
    else
    {
        /* Next step: Step 1 Configuration ---------------------------- */
        /*  Channel1 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_1, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_1, TIM_CAP_CMP_ENABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_2, TIM_CAP_CMP_N_DISABLE);

        /*  Channel3 configuration */
        TIM_SelectOcMode(TIM1, TIM_CH_3, TIM_OCMODE_PWM1);
        TIM_EnableCapCmpCh(TIM1, TIM_CH_3, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_3, TIM_CAP_CMP_N_ENABLE);

        /*  Channel2 configuration */
        TIM_EnableCapCmpCh(TIM1, TIM_CH_2, TIM_CAP_CMP_DISABLE);
        TIM_EnableCapCmpChN(TIM1, TIM_CH_2, TIM_CAP_CMP_N_DISABLE);
        step = 1;
    }
}
