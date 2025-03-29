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
 * @file bsp_key.c
 * @version v1.0.0
 *
  */

#include "bsp_key.h"

/**
 * @brief  NVIC configuration.
 */
static void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the KEY1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = KEY1_INT_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configure the IO as EXTI_Line and set the Priority.
 */
void EXTI_Key_Config(void)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;

    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(KEY1_INT_GPIO_CLK, ENABLE);

    /* Configure NVIC */
    NVIC_Configuration();

    /*--------------------------KEY1_PA0 Configuration-----------------------------*/
    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure KEY1 as input floating */
    GPIO_InitStructure.Pin       = KEY1_INT_GPIO_PIN;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitPeripheral(KEY1_INT_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ConfigEXTILine(KEY1_INT_EXTI_PORTSOURCE, KEY1_INT_EXTI_PINSOURCE);
    EXTI_InitStructure.EXTI_Line    = KEY1_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
}
