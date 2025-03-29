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
 * @version v1.0.2
 *
  */
#include "main.h"

/** @addtogroup TIM_TimeBase
 * @{
 */

TIM_TimeBaseInitType TIM_TimeBaseStructure;
OCInitType TIM_OCInitStructure;
__IO uint16_t CCR1_Val  = 45000;
__IO uint16_t CCR2_Val  = 30000;
__IO uint16_t CCR3_Val  = 15000;
__IO uint16_t CCR4_Val  = 6000;
uint16_t PrescalerValue = 0;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

/**
 * @brief  Main program
 */
int main(void)
{
    RCC_ClocksType RCC_Clocks;
    /* System Clocks Configuration */
    RCC_Configuration();

    /* NVIC Configuration */
    NVIC_Configuration();

    /* GPIO Configuration */
    GPIO_Configuration();

    /* Compute the prescaler value */
    RCC_GetClocksFreqValue(&RCC_Clocks);
    if(RCC_Clocks.Pclk1Freq != RCC_Clocks.HclkFreq)
    {
        PrescalerValue = (uint16_t)(RCC_Clocks.Pclk1Freq*2 / 4000000) - 1;
    }
    else
    {
        PrescalerValue = (uint16_t)(SystemCoreClock / 4000000) - 1;
    }

    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = 65535;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);
    
    /* Prescaler configuration */
    TIM_ConfigPrescaler(TIM3, PrescalerValue, TIM_PSC_RELOAD_MODE_IMMEDIATE);

    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_TIMING;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR1_Val;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    TIM_EnableOc1Preload(TIM3, TIM_OC_PRE_LOAD_DISABLE);

    /* Output Compare Timing Mode configuration: Channel2 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR2_Val;
    TIM_InitOc2(TIM3, &TIM_OCInitStructure);

    TIM_ConfigOc2Preload(TIM3, TIM_OC_PRE_LOAD_DISABLE);

    /* Output Compare Timing Mode configuration: Channel3 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR3_Val;
    TIM_InitOc3(TIM3, &TIM_OCInitStructure);

    TIM_ConfigOc3Preload(TIM3, TIM_OC_PRE_LOAD_DISABLE);

    /* Output Compare Timing Mode configuration: Channel4 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR4_Val;
    TIM_InitOc4(TIM3, &TIM_OCInitStructure);

    TIM_ConfigOc4Preload(TIM3, TIM_OC_PRE_LOAD_DISABLE);

    /* TIM IT enable */
    TIM_ConfigInt(TIM3, TIM_INT_CC1 | TIM_INT_CC2 | TIM_INT_CC3 | TIM_INT_CC4, ENABLE);

    /* TIM3 enable counter */
    TIM_Enable(TIM3, ENABLE);

    while (1)
        ;
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* PCLK1 = HCLK/4 */
    RCC_ConfigPclk1(RCC_HCLK_DIV4);

    /* TIM clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* GPIO clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO , ENABLE);
}

/**
 * @brief  Configure the GPIO Pins.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    
    /* GPIOB Configuration:Pin6, 7, 11 and 12 as Output Push Pull Mode*/
    GPIO_InitStructure.Pin            = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Speed    = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Pull     = GPIO_PULL_UP;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief  Configure the nested vectored interrupt controller.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the TIM global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
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
