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
 * @file dfu_led.c
 * @version v1.0.0
 *
  */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "dfu_led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief Development board led init.
 * @param[in] none.
 * @return none.
 */
void dfu_leds_config(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    
    GPIO_InitStructure.Pin = LED_GPIO1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitPeripheral(LED1_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = LED_GPIO2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;    
    GPIO_InitPeripheral(LED2_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_ResetBits(LED1_GPIO_PORT, LED_GPIO1_PIN);
    GPIO_ResetBits(LED2_GPIO_PORT, LED_GPIO2_PIN);
    
}
/**
 * @brief Development board led turn on.
 * @param[in] led_port led port num.
 * @param[in] led_pin led pin num.
 * @return none.
 */
void dfu_led_on(GPIO_Module* led_port, uint32_t led_pin)
{
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_SetBits(led_port, led_pin);
}
/**
 * @brief Development board led turn off.
 * @param[in] led_port led port num.
 * @param[in] led_pin led pin num.
 * @return none.
 */
void dfu_led_off(GPIO_Module* led_port, uint32_t led_pin)
{
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_ResetBits(led_port, led_pin);
}
/**
 * @brief Development board led toggle.
 * @param[in] led_port led port num.
 * @param[in] led_pin led pin num.
 * @return none.
 */
void dfu_led_toggle(GPIO_Module* led_port, uint32_t led_pin)
{
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    GPIO_TogglePin(led_port, led_pin);
}



/**
 * @}
 */


