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

/**
 *  NVIC IRQ Mask
 */





volatile uint8_t Key_Status = DISABLE;

void TIM_Config(void);
void KeyInputExtiInit(GPIO_Module* GPIOx, uint16_t Pin);


/**
 * @brief  Main program.
 */
int main(void)
{
    /* log Init */
    log_init();
    log_info("\r\nNVIC IRQ Mask \r\n");

    KeyInputExtiInit(KEY_INPUT_PORT, KEY_INPUT_PIN);
    /* TIM configuration -------------------------------------------------------*/
    TIM_Config();

    while (1)
    {
        while (Key_Status == DISABLE)
        {
        }
        log_info("Disable irq \r\n");
        /* This instruction raises the execution priority to 0. This prevents all
           exceptions with configurable priority from activating, other than through
           the HardFault fault escalation mechanism. */
	    
		__disable_irq();
		
        Delay_ms(5000);		//Delay some time

        log_info("enable irq \r\n");
        /* This instruction will allow all exceptions with configurable priority to
           be activated. */
        __enable_irq();
		
		Key_Status = DISABLE;
    }
}
    
/**
 * @brief  Configures the used Timers.
 */
void TIM_Config(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* Enable TIM3 clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* TIM3 configuration */
    TIM_TimeBaseStructure.Period    = 0x4AF;
    TIM_TimeBaseStructure.Prescaler = ((SystemCoreClock / 1200) - 1);
    TIM_TimeBaseStructure.ClkDiv    = 0x0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);
    TIM_InitOcStruct(&TIM_OCInitStructure);

    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode = TIM_OCMODE_TIMING;
    TIM_OCInitStructure.Pulse  = 0x0;
    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    /* Immediate load of TIM3 Precaler values */
    TIM_ConfigPrescaler(TIM3, ((SystemCoreClock / 1200) - 1), TIM_PSC_RELOAD_MODE_IMMEDIATE);

    /* Clear TIM3 update pending flags */
    TIM_ClearFlag(TIM3, TIM_FLAG_UPDATE);

    /* Enable the TIM3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable TIM3 Update interrupts */
    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);

    /* TIM3 enable counters */
    TIM_Enable(TIM3, ENABLE);
}

/**
 * @brief  Configures key port.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
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
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);

    /*Configure the GPIO pin as input floating*/
    GPIO_InitStructure.Pin        = Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
	GPIO_InitStructure.GPIO_Pull  = GPIO_PULL_UP;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);

    /*Configure key EXTI Line to key input Pin*/
    GPIO_ConfigEXTILine(KEY_INPUT_PORT_SOURCE, KEY_INPUT_PIN_SOURCE);

    /*Configure key EXTI line*/
    EXTI_InitStructure.EXTI_Line    = KEY_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /*Set key input interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   = KEY_INPUT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**
 * @}
 */

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
*          line: assert_param error line source number
 * @return None
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}

/**
 * @}
 */
#endif
