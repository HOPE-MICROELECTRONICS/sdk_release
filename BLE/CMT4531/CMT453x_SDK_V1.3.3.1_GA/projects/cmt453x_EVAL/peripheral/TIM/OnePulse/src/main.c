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

/** @addtogroup TIM_OnePulse
 * @{
 */

TIM_TimeBaseInitType TIM_TimeBaseStructure;
TIM_ICInitType TIM_ICInitStructure;
OCInitType TIM_OCInitStructure;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void SendOneTrig(void);
uint16_t gSendTrigEn = 1;

/**
 * @brief  Main program
 */
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* TIM3 configuration: One Pulse mode */

    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = 65535;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    /* TIM3 PWM2 Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 16383;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    TIM_InitOc3(TIM3, &TIM_OCInitStructure);

    /* TIM3 configuration in Input Capture Mode */

    TIM_InitIcStruct(&TIM_ICInitStructure);
    TIM_ICInitStructure.Channel     = TIM_CH_2;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    TIM_ICInitStructure.IcFilter    = 0;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /* One Pulse Mode selection */
    TIM_SelectOnePulseMode(TIM3, TIM_OPMODE_SINGLE);

    /* Input Trigger selection */
    TIM_SelectInputTrig(TIM3, TIM_TRIG_SEL_TI2FP2);

    /* Slave Mode selection: Trigger Mode */
    TIM_SelectSlaveMode(TIM3, TIM_SLAVE_MODE_TRIG);

    while (1)
    {
        SendOneTrig();
    }
}

/**
 * @brief  Send one trig byself.
 */
void SendOneTrig(void)
{
    if (gSendTrigEn)
    {
        gSendTrigEn = 0;
        GPIO_SetBits(GPIOA, GPIO_PIN_6);
        {
            uint32_t i;
            while (i++ < 10)
                ;
        }
        GPIO_ResetBits(GPIOA, GPIO_PIN_6);
    }
}
/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    RCC_ConfigPclk1(RCC_HCLK_DIV4);
    
    /* TIM3 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* GPIOB clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
}

/**
 * @brief  Configure the GPIOA Pins.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* TIM3_CH3 pin (PB.06) configuration as Alternate Function Push Pull Mode*/
    GPIO_InitStructure.Pin        = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* TIM3_CH2 pin (PB.05) configuration as Alternate Function Push Pull Mode*/
    GPIO_InitStructure.Pin       = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* pin (PA.06) configuration as Output Push Pull Mode */
    GPIO_InitStructure.Pin       = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
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
