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

/** @addtogroup TIM_Cascade_Synchro
 * @{
 */

TIM_TimeBaseInitType TIM_TimeBaseStructure;
OCInitType TIM_OCInitStructure;

void RCC_Configuration(void);
void GPIO_Configuration(void);

/**
 * @brief  Main program
 */
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* GPIO Configuration */
    GPIO_Configuration();

    /* Timers synchronisation in cascade mode */
    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = 255;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM1, &TIM_TimeBaseStructure);
    
    TIM_TimeBaseStructure.Period = 3;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    /* Master Configuration in PWM1 Mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 64;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    TIM_InitOc1(TIM1, &TIM_OCInitStructure);

    /* Select the Master Slave Mode */
    TIM_SelectMasterSlaveMode(TIM1, TIM_MASTER_SLAVE_MODE_ENABLE);

    /* Master Mode selection */
    TIM_SelectOutputTrig(TIM1, TIM_TRGO_SRC_UPDATE);

    /* Slaves Configuration: PWM1 Mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 1;
    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    /* Slave Mode selection: TIM3 */
    TIM_SelectSlaveMode(TIM3, TIM_SLAVE_MODE_GATED);
    TIM_SelectInputTrig(TIM3, TIM_TRIG_SEL_IN_TR0);

    /* Select the Master Slave Mode */
    TIM_SelectMasterSlaveMode(TIM3, TIM_MASTER_SLAVE_MODE_ENABLE);

    /* Master Mode selection: TIM3 */
    TIM_SelectOutputTrig(TIM3, TIM_TRGO_SRC_UPDATE);

    /* TIM enable counter */
    TIM_EnableCtrlPwmOutputs(TIM1,ENABLE);
    TIM_Enable(TIM1, ENABLE);
    TIM_Enable(TIM3, ENABLE);

    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* TIM3 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* TIM1 GPIOB and AFIO clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1 | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
}

/**
 * @brief  Configure the GPIOD Pins.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* GPIOB Configuration: PB8(TIM1 CH1) as alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_TIM1;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);    
    
    /* GPIOB Configuration: PB4(TIM3 CH1) as alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_4;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
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
