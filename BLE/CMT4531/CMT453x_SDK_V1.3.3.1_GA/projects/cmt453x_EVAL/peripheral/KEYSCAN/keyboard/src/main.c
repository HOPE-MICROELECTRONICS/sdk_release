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
#include <stdio.h>
#include "main.h"

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */
extern void system_delay_n_10us(uint32_t value);
void KEYSCAN_Configuration(uint8_t key_mask, uint8_t mode, uint8_t dts, uint8_t wts, bool int_en);
NVIC_InitType NVIC_InitStructure;
EXTI_InitType EXTI_InitStructure;
GPIO_InitType GPIO_InitStructure; 
KEYSCAN_InitType KEYSCAN_InitStructure;
uint16_t key_receive = 0;

   
/**
 * @brief  Main program.
 */
int main(void)
{
    log_init();
    log_info("\nThis is KEYSCAN demo in 44 key mode with 8 GPIO!\n");
    log_info("Connect each other PA0, PA1, PA2, PA3, PA6, PB8, PB9, PB10,\n");
    log_info("Or connect one of then to GND or VCC as one key pressed. \n");
    log_info("                                                                               VCC\n");
    log_info("PA0-----------------------------------------------------------KEY_29-|--KEY_37--|\n");
    log_info("PA1---KEY_1---------------------------------------------------KEY_30-|--KEY_38--|\n");
    log_info("PA2---KEY_2---KEY_3-------------------------------------------KEY_31-|--KEY_39--|\n");
    log_info("PA3---KEY_4---KEY_5---KEY_6-----------------------------------KEY_32-|--KEY_40--|\n");
    log_info("PA6---KEY_7---KEY_8---KEY_9---KEY_10--------------------------KEY_33-|--KEY_41--|\n");
    log_info("PB10--KEY_11--KEY_12--KEY_13--KEY_14--KEY_15------------------KEY_34-|--KEY_42--|\n");
    log_info("PB8---KEY_16--KEY_17--KEY_18--KEY_19--KEY_20--KEY_21----------KEY_35-|--KEY_43--|\n");
    log_info("PB9---KEY_22--KEY_23--KEY_24--KEY_25--KEY_26--KEY_27--KEY_28--KEY_36-|--KEY_44--|\n");
    log_info("                                                                    GND          \n");
    
    KEYSCAN_Configuration(KEY_44,MODE_FIXED_INTV, DTS_10MS, WTS_0MS, INT_EN);
    while (1)
    {
        if(key_receive)
        {
            log_info("Keyboard detect KEY_%d pressed.\n",key_receive);
            key_receive = 0;// clear last key value
        }
    }
}

/**
 * @brief  KEYSCAN Configuration
 * @param  @KEY_MASK     :   KEY_65, KEY_44, KEY_104
 * @param  @KEY_MODE     :   MODE_FIXED_INTV, MODE_SW_TRIG, MODE_PRESS_TRIG,
 * @param  @KEY_DTS      :   DTS_10MS, DTS_20MS, DTS_40MS, DTS_80MS, DTS_160MS, DTS_320MS, DTS_640MS, DTS_640MS_2
 * @param  @KEY_WTS      :   WTS_0MS, WTS_32MS, WTS_64MS, WTS_96MS, WTS_128MS, WTS_160MS, WTS_192MS, WTS_224MS
 * @param  @KEY_INT      :   INT_DIS, INT_EN
 * @return Return value
 * @note   Note
 */
void KEYSCAN_Configuration(uint8_t key_mask, uint8_t mode, uint8_t dts, uint8_t wts, bool int_en)
{
    /* KEYSCAN CLK EN */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC->LSCTRL |= RCC_LSCTRL_KEYSCANEN;
     /* config GPIO */
    GPIO_InitStruct(&GPIO_InitStructure);
    if(key_mask == KEY_65)
    {
        GPIO_InitStructure.Pin =     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;        
    }
    else if(key_mask == KEY_44)
    {
        GPIO_InitStructure.Pin =     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6;
    }
    else
    {
        GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    }
    
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = 0; //deinit
    if(key_mask == KEY_65 ||key_mask == KEY_44)
    {
        GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;        
    }
    else
    {
        GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    }
    
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* config KEYSCAN */
    KEYSCAN_InitStructure.Mask = key_mask;
    KEYSCAN_InitStructure.Mode = mode;
    KEYSCAN_InitStructure.Wts = wts;
    KEYSCAN_InitStructure.Dts = dts;
    KEYSCAN_InitStructure.Int_en = int_en;
    KEYSCAN_Init(&KEYSCAN_InitStructure);
    /* config intterrupt */
    if(int_en)
    {
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Line = EXTI_LINE13;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitPeripheral(&EXTI_InitStructure);
            
        
        NVIC_DisableIRQ(KEYSCAN_IRQn);
        /* Enable the KEYSCAN Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel                   = KEYSCAN_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        NVIC_EnableIRQ(KEYSCAN_IRQn);
    }
    /* Enable KEYSCAN */
    KEYSCAN_Enable(ENABLE);
}


/**
 * @}
 */

/**
 * @}
 */
