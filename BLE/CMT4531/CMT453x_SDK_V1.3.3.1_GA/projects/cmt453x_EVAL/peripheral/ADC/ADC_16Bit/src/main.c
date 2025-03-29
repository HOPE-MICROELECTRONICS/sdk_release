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
/* NOTE: 
1. 16bit ADC must DISABLE BypassFilter
2. 16bit ADC must ENABLE  ContinuousMode
3. every 248 16Bit ADC samples will output 2*16bit invalid data, 
   if you use DMA, you can set data len as 250, then drop the last 2 samples.*/

#define USE_ADC_16BIT_DMA   1
#define ADC_16BIT_DROP      8
#define ADC_DMA_BUF_SIZE    250
#define ADC_AVG_NUM         200

#if ((ADC_AVG_NUM+ADC_16BIT_DROP) > ADC_DMA_BUF_SIZE)
#error "ADC_AVG_NUM+ADC_16BIT_DROP must less than ADC_DMA_BUF_SIZE!!!"
#endif

extern void system_delay_n_10us(uint32_t value);

int16_t ADCBuf[ADC_DMA_BUF_SIZE];


int16_t ADC16bit_DMA_Get_AVG_Val(uint16_t Channel, uint8_t avg_num)
{
    uint16_t i, null_num;
    int16_t avg;
    int64_t sum = 0;
    DMA_InitType DMA_InitStructure = {0};   

	/* Enable peripheral clocks */
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    RCC_ConfigAdcClk(RCC_ADCCLK_SRC_AUDIOPLL);
    /* enable ADC 4M clock */
    RCC_Enable_ADC_CLK_SRC_AUDIOPLL(ENABLE);
    /* Enable peripheral clocks */
	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);
    
	/* DMA channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA_CH1);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&ADC->DAT;
    DMA_InitStructure.MemAddr        = (uint32_t)&ADCBuf;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = avg_num+ADC_16BIT_DROP;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(DMA_CH1, &DMA_InitStructure);
	DMA_RequestRemap(DMA_REMAP_ADC, DMA, DMA_CH1, ENABLE);
    /* Enable DMA channel1 */
    DMA_EnableChannel(DMA_CH1, ENABLE);   
    /* configer ADC */
	ADC_EnableBypassFilter(ADC, DISABLE);
    ADC_ConfigContinuousMode(ADC, ENABLE);
    ADC_ConfigChannel(ADC, Channel);
	ADC_EnableDMA(ADC, ENABLE);    
    ADC_Enable(ADC, ENABLE);
    /* wait DMA finish */
    while(DMA_GetFlagStatus(DMA_FLAG_TC1, DMA) == RESET);
    ADC_Enable(ADC, DISABLE);
    DMA_EnableChannel(DMA_CH1, DISABLE);
    null_num = 0;

    for( i = ADC_16BIT_DROP;i<(avg_num+ADC_16BIT_DROP);i++)
    {
        if(ADCBuf[i] == 0)
        {
            null_num++;
            continue;
        }
        sum += ADCBuf[i];
    }
    avg = sum/(avg_num-null_num);

    return avg;
}


uint16_t ADC16bit_SingleRead_Val(uint16_t Channel , uint8_t avg_num)
{
    int32_t  sum = 0;
    uint16_t i,null_num;
    int16_t adc_val;
	/* Enable peripheral clocks */
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    RCC_ConfigAdcClk(RCC_ADCCLK_SRC_AUDIOPLL);
    /* enable ADC 4M clock */
    RCC_Enable_ADC_CLK_SRC_AUDIOPLL(ENABLE);
    /* configer ADC */
	ADC_EnableBypassFilter(ADC, DISABLE);
    ADC_ConfigContinuousMode(ADC, ENABLE);
    ADC_ConfigChannel(ADC, Channel);
    ADC_Enable(ADC, ENABLE);
    /* drop invalid data  */
    system_delay_n_10us(100); //1ms 
    /* get adc data  */
    null_num = 0;
    for(i = 0; i<avg_num; i++)
    {
        adc_val = ADC_GetDat(ADC);
        if(adc_val == 0)
        {
            null_num++;
            continue;
        }
        sum += adc_val;
        system_delay_n_10us(10);
    }
    ADC_Enable(ADC, DISABLE);
    adc_val = sum/(avg_num-null_num);
    return adc_val;
}


int16_t ADC16Bit_GetOffsetAtVCC(uint16_t volmV)
{
    uint16_t get_vol;
    int16_t adc_val, offset;
    #if 1
    adc_val = ADC16bit_SingleRead_Val(ADC_CTRL_CH_6,20);
    #else
    adc_val = ADC16bit_DMA_Get_AVG_Val(ADC_CTRL_CH_6,20);
    #endif
    get_vol = ADC16Bit_ConverValueToVoltage(adc_val, ADC_CTRL_CH_6,0);
    offset = volmV-get_vol;
    
    return offset;
}

/**
 * @brief  Main program
 */
int main(void)
{
    int16_t offset;
    uint16_t voltage = 0;
    int16_t adc_val = 0;
	log_init();
    log_info("\nthis is 16bit adc read Demo. \n");
	log_info("Please make sure J15 and J16 connect the IO to pin on board!\n");
    log_info("CH2 is PB9 which is support 0-1V.\n");
    log_info("CH3 is PB8 which is support 0-3.6V.\n");
    
    offset = ADC16Bit_GetOffsetAtVCC(3300);
    log_info("ADC16Bit offset:%d\n",offset);
    while (1)
    {
        #if USE_ADC_16BIT_DMA
        
        adc_val = ADC16bit_DMA_Get_AVG_Val(ADC_CTRL_CH_2,ADC_AVG_NUM);
        voltage = ADC16Bit_ConverValueToVoltage(adc_val, ADC_CTRL_CH_2,offset);
		log_info("DMA Read CH2(PB9),value: %d, vol_mV: %d\t",adc_val,voltage); 
        adc_val = ADC16bit_DMA_Get_AVG_Val(ADC_CTRL_CH_3,ADC_AVG_NUM);
        voltage = ADC16Bit_ConverValueToVoltage(adc_val, ADC_CTRL_CH_3,offset);
        log_info("CH3(PB8),value: %d, vol_mV: %d\r\n",adc_val,voltage);
        
        #else
        
        adc_val = ADC16bit_SingleRead_Val(ADC_CTRL_CH_2,ADC_AVG_NUM);
        voltage = ADC16Bit_ConverValueToVoltage(adc_val, ADC_CTRL_CH_2,offset);
		log_info("Single Read CH2(PB9),value: %d, vol_mV: %d\t",adc_val,voltage); 
        adc_val = ADC16bit_SingleRead_Val(ADC_CTRL_CH_3,ADC_AVG_NUM);
        voltage = ADC16Bit_ConverValueToVoltage(adc_val, ADC_CTRL_CH_3,offset);
        log_info("CH3(PB8),value: %d, vol_mV: %d\r\n",adc_val,voltage);   
        
        #endif
		Delay_ms(1000);
    }
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


