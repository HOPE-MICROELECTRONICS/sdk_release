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
#include "log.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);

uint16_t ADCConvertedValue[2];
uint32_t voltage[2] = {0}; 
/**
 * @brief  Main program
 */
int main(void)
{
	log_init();
    log_info("\nthis is adc single read Demo.\n");
	log_info("Please make sure J15 and J16 connect the IO to pin on board!\n");
	/* System Clocks Configuration */
	RCC_Configuration();
	
	/* Configure the GPIO ports */
    GPIO_Configuration();
	
	ADC_EnableBypassFilter(ADC, ENABLE);
    while (1)
    {
		ADC_ConfigChannel(ADC, ADC_CTRL_CH_3);
		ADC_Enable(ADC, ENABLE);
		while(ADC_GetFlagStatus(ADC,ADC_FLAG_DONE) == RESET);
		ADC_ClearFlag(ADC,ADC_FLAG_DONE);  
		ADCConvertedValue[1] = ADC_GetDat(ADC);
    
		voltage[1] = ADC_ConverValueToVoltage(ADCConvertedValue[1], ADC_CTRL_CH_3);
		log_info("ADC CH3 value: %4d  |  ADC CH3 vol_mV: %4d .\r\n",ADCConvertedValue[1],voltage[1]);  
	
		ADC_ConfigChannel(ADC, ADC_CTRL_CH_1);
		ADC_Enable(ADC, ENABLE);
		while(ADC_GetFlagStatus(ADC,ADC_FLAG_DONE) == RESET);
		ADC_ClearFlag(ADC,ADC_FLAG_DONE);  
		ADCConvertedValue[0] = ADC_GetDat(ADC);

		voltage[0] = ADC_ConverValueToVoltage(ADCConvertedValue[0], ADC_CTRL_CH_1);
		log_info("ADC CH1 value: %4d  |  ADC CH1 vol_mV: %4d .\r\n\r\n",ADCConvertedValue[0],voltage[0]);    
		Delay_ms(1000);
    }
}


/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
	/* Enable peripheral clocks */
    /* Enable GPIOB clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
        
    RCC_ConfigAdcClk(RCC_ADCCLK_SRC_AUDIOPLL);

    /* enable ADC 4M clock */
    RCC_Enable_ADC_CLK_SRC_AUDIOPLL(ENABLE);
}


/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure PB.10 (ADC Channel1) PB.8 (ADC Channel3) as analog input --------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_10|GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
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

/*************** (C) COPYRIGHT HopeRF Technologies Inc *****END OF FILE***************/


