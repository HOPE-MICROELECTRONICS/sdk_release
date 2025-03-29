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
 * @version v1.0.1
 *
  */
#include "main.h"

/** @addtogroup TIM_TimeBase
 * @{
 */

TIM_TimeBaseInitType TIM_TimeBaseStructure;
OCInitType TIM_OCInitStructure;
uint16_t PrescalerValue = 0;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void TIM_Configuration(void);

/**
 * @brief  Main program
 */
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* NVIC Configuration */
    NVIC_Configuration();

    /* GPIO Configuration */
    GPIO_Configuration();

    /* TIM Configuration */
    TIM_Configuration();

    while (1)
    {
        ;
    }
}

/**
 * @brief  Configures tim6 clocks.
 */
void TIM_Configuration(void)
{    
    /* Compute the prescaler value */
    PrescalerValue = 0;

    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = 65535;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(TIM6, &TIM_TimeBaseStructure);

    /* Prescaler configuration */
    TIM_ConfigPrescaler(TIM6, PrescalerValue, TIM_PSC_RELOAD_MODE_IMMEDIATE);

    /* TIM6 enable update irq */
    TIM_ConfigInt(TIM6, TIM_INT_UPDATE, ENABLE);

    /* TIM6 enable counter */
    TIM_Enable(TIM6, ENABLE);
}
/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* PCLK1 = HCLK/4 */
    RCC_ConfigPclk1(RCC_HCLK_DIV4);

    /* TIM6 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM6, ENABLE);

    /* GPIOB clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
}

/**
 * @brief  Configure the GPIO Pins.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* GPIOB Configuration:Pin6 as alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;

    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief  Configure the nested vectored interrupt controller.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the TIM6 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    while (1)
    {
    }
}

#endif

/**
 * @}
 */

/**
 * @}
 */
