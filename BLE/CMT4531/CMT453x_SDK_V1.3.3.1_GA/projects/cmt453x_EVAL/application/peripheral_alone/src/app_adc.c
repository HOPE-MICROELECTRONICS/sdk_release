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
 * @file app_adc.c
 * @version v1.0.1
 *
  */
#include "app_adc.h"
#include "main.h"
#include <stdio.h>


/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration_ADC(void)
{
    /* Enable peripheral clocks */
    /* Enable GPIOB clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    RCC_Enable_ADC_CLK_SRC_AUDIOPLL(ENABLE);    
    
    RCC_ConfigAdcClk(RCC_ADCCLK_SRC_AUDIOPLL);

}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration_ADC(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure PB.10 (ADC Channel1) as analog input --------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief  Configures the ADC and Read CH1 CH2.
 */
void ADC_Configuration(void)
{
    RCC_Configuration_ADC();
    GPIO_Configuration_ADC();

    /* Select the ADC channel */
    ADC_ConfigChannel(ADC, ADC_CTRL_CH_1);
    
    ADC_EnableBypassFilter(ADC, ENABLE);


}

/**
 * @brief  Configures and read the selected ADC channel
 * @param  ADC_Channel the ADC channel to read.
 *   This parameter can be any combination of the following values:
 *     @arg ADC_CTRL_CH_0 ADC Channel0 selected (PGA PB11/PB13)
 *     @arg ADC_CTRL_CH_1 ADC Channel1 selected (PB10)
 *     @arg ADC_CTRL_CH_2 ADC Channel2 selected (PB9)
 *     @arg ADC_CTRL_CH_3 ADC Channel3 selected (PB8)
 *     @arg ADC_CTRL_CH_4 ADC Channel4 selected (PB7)
 *     @arg ADC_CTRL_CH_5 ADC Channel5 selected (PB6)
 *     @arg ADC_CTRL_CH_6 ADC Channel6 selected (external voltage)
 *     @arg ADC_CTRL_CH_7 ADC Channel7 selected (temperature)
 */
uint16_t ADC_GetDataBlocking(uint32_t ADC_Channel)
{
    uint16_t dat;
    uint32_t timer = 0;

    /* Select the ADC channel */
    ADC_ConfigChannel(ADC, ADC_Channel);
    /* Enable ADC */
    ADC_Enable(ADC, ENABLE);

    while(ADC_GetFlagStatus(ADC,ADC_FLAG_DONE)== RESET)
    {
      if(++timer > 0xfffff)
            return 0xffff;
    }
    ADC_ClearFlag(ADC,ADC_FLAG_DONE);
    dat=ADC_GetDat(ADC);
    return dat;
}


/**
 * @}
 */

