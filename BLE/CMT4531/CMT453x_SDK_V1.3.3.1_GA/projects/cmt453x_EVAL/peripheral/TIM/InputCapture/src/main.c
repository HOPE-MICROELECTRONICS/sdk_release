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

/** @addtogroup TIM_Input_Capture
 * @{
 */

TIM_ICInitType TIM_ICInitStructure;
uint16_t gOnePulsEn = 0;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void GenTwoRisingEdge(void);

/**
 * @brief  Main program
 */
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* TIM3 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM3 CH2 pin (PB.05)
     The Rising edge is used as active edge,
     The TIM3 CCDAT2 is used to compute the frequency value

    gOnePulsEn = 1,will do GenTwoRisingEdge
    then
    see Capture that is dist time
    ------------------------------------------------------------ */

    TIM_ICInitStructure.Channel     = TIM_CH_2;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    TIM_ICInitStructure.IcFilter    = 0x0;

    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    /* TIM enable counter */
    TIM_Enable(TIM3, ENABLE);

    /* Enable the CC2 Interrupt Request */
    TIM_ConfigInt(TIM3, TIM_INT_CC2, ENABLE);

    while (1)
    {
        GenTwoRisingEdge();
    }
}

// generation two rising edge
void GenTwoRisingEdge(void)
{
    if (gOnePulsEn)
    {
        gOnePulsEn = 0;
        GPIO_SetBits(GPIOA, GPIO_PIN_6);
              {
            uint32_t i = 0;
            while (i++ < 50)
                ;
        }
        GPIO_ResetBits(GPIOA, GPIO_PIN_6);
        {
            uint32_t i = 0;
            while (i++ < 500)
                ;
        }

        GPIO_SetBits(GPIOA, GPIO_PIN_6);
                {
            uint32_t i = 0;
            while (i++ < 50)
                ;
        }
        GPIO_ResetBits(GPIOA, GPIO_PIN_6);
                {
            uint32_t i = 0;
            while (i++ < 500)
                ;
        }
    }
}
/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* TIM3 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* GPIOA and GPIOB clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA|RCC_APB2_PERIPH_GPIOB, ENABLE);
}

/**
 * @brief  Configure the GPIOA Pins.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* TIM3 channel 2 pin (PB.05) configuration */
    GPIO_InitStructure.Pin        = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    GPIO_InitStruct(&GPIO_InitStructure);
    
    // PA6
    GPIO_InitStructure.Pin       = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief  Configure the nested vectored interrupt controller.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the TIM3 global Interrupt */
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
