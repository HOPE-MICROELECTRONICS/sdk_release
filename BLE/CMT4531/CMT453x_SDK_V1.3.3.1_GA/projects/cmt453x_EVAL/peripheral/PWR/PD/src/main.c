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
 * @version v1.0.0
 *
  */
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "log.h"
/** @addtogroup PWR_PD
 * @{
 */


void delay(vu32 nCount);
void SYSCLKConfig_STOP(uint32_t RCC_PLLMULL);
void EXTI_Configuration(void);

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_cmt453x.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_cmt453x.c file
       */
    LedInit(LED1_PORT, LED1_PIN);
    
    EXTI_Configuration();
    while (1)
    {
        LedOn(LED1_PORT, LED1_PIN);
        delay(3000);
        LedOff(LED1_PORT, LED1_PIN);
        
        //Enter shutdown
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
        PWR_EnterPDMode(PWR_PDENTRY_WFI);
    }
}
    
void EXTI_Configuration(void)
{    
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    
    /* Configure NVIC */
    NVIC_InitStructure.NVIC_IRQChannel            = EXTI4_12_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority    = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable the NRST Wakeup Interrupt through EXTI line 12 */
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE12;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
}

/**
 * @brief  Configures LED GPIO.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else
    {
        return;
    }

    /* Configure the GPIO pin */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = Pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}

/**
 * @brief  Turns selected Led on.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedOn(GPIO_Module *GPIOx, uint16_t Pin)
{
    GPIO_SetBits(GPIOx, Pin);
}

/**
 * @brief  Turns selected Led Off.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_ResetBits(GPIOx, Pin);
}

/**
 * @brief  Toggles the selected Led.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_TogglePin(GPIOx, Pin);
}

/**
 * @brief  Delay function.
 *     @arg nCount
 */
void delay(vu32 nCount)
{
    vu32 index = 0;
    for (index = (3400 * nCount); index != 0; index--)
    {
    }
}

/**
 * @brief  Configures key GPIO.
 * @param key Specifies the Led to be configured.
 *   This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void KeyInputExtiInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    
    GPIO_InitStruct(&GPIO_InitStructure);
    /* Enable the GPIO Clock */

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    /*Configure the GPIO pin as input floating*/

    GPIO_InitStructure.Pin        = Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);

    /*Configure key EXTI Line to key input  Pin*/
    GPIO_ConfigEXTILine(GPIOA_PORT_SOURCE, GPIO_PIN_SOURCE0);

    /*Configure key EXTI line*/
    EXTI_InitStructure.EXTI_Line = EXTI_LINE0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /*Set key input interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}
