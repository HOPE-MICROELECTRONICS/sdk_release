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
#include "cmsis_os.h"

/**
 *  FreeRTOS Timers
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_1MS           ((uint32_t)1000)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Tick_num = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 */
int main(void)
{
    /*Configure the SysTick IRQ priority */
    HP_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0);
    /* Get SystemCoreClock */
    SystemCoreClockUpdate();

    /* Config 1s SysTick 1ms  */
    SysTick_Config(SystemCoreClock/SYSTICK_1MS);
  
    /* Initialize Led1~Led2 as output pushpull mode*/
    LedInit(LED1_PORT, LED1_PIN);
    LedInit(LED2_PORT, LED2_PIN);

    /*Turn on Led1~Led2*/
    LedOn(LED1_PORT, LED1_PIN);
    LedOn(LED2_PORT, LED2_PIN);

    /* Create Timer */
    osTimerDef(LEDTimer, osTimerCallback);
    osTimerId osTimer = osTimerCreate(osTimer(LEDTimer), osTimerPeriodic, NULL);
    /* Start Timer */
    osTimerStart(osTimer, 200);

    /* Create LED Thread */
    osThreadDef(LEDThread, ToggleLEDsThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    osThreadCreate(osThread(LEDThread), NULL);

    /* Start scheduler */
    osKernelStart();
  
    /* We should never get here as control is now taken by the scheduler */
    for(;;);
}

/**
  * @brief  Timer Callback that toggle LED1
  * @param  argument not used
  * @retval None
  */
static void osTimerCallback(void const *argument)
{
    (void) argument;

    /* Toggle LED1*/
    LedBlink(LED1_PORT,LED1_PIN);
}

/**
  * @brief  Toggle LED2 thread
  * @param  argument not used
  * @retval None
  */
static void ToggleLEDsThread(void const *argument)
{
    (void) argument;

    for (;;)
    {
        /* Toggle LED2 each 400ms */
        LedBlink(LED2_PORT,LED2_PIN);

        osDelay(400);
    }
}

/**
  * @brief  Sets the priority of an interrupt.
  * @param  IRQn External interrupt number .
  *         This parameter can be an enumerator of  IRQn_Type enumeration
  * @param  PreemptPriority The pre-emption priority for the IRQn channel.
  *         This parameter can be a value between 0 and 3.
  *         A lower priority value indicates a higher priority 
  * @param  SubPriority the subpriority level for the IRQ channel.
  *         this parameter is a dummy value and it is ignored, because 
  *         no subpriority supported in Cortex M0 based products.   
  * @retval None
  */
void HP_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
    /* Check the parameters */
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));
  NVIC_SetPriority(IRQn,PreemptPriority);
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
 * @}
 */

/**
 * @}
 */
