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
 * @file dfu_usart.c
 * @version v1.0.4
 *
  */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "dfu_usart.h"
#include "cmt453x.h"
#include <stdio.h>
#include <string.h>
extern void system_delay_cycles(uint32_t i);

/**
 * @brief  config rcc of usart1
 * @param  
 * @return 
 * @note   
 */
static void rcc_configure_usart1(void)
{ 
    RCC->APB2PCLKEN |= RCC_APB2_PERIPH_GPIOA|RCC_APB2_PERIPH_GPIOB;
    RCC->APB2PCLKEN |= RCC_APB2_PERIPH_USART1;
}

/**
 * @brief  config gpio of usart1
 * @param  
 * @return 
 * @note   
 */
static void gpio_configure_usart1(uint8_t gpio)
{
    GPIO_InitType GPIO_InitStructure;
    
    GPIO_InitStruct(&GPIO_InitStructure);
    if(gpio == USART1_GPIO_PA45)
    {
        GPIO_InitStructure.Pin              =    GPIO_PIN_4;
        GPIO_InitStructure.GPIO_Mode        =    GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF3_USART1;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.Pin              =    GPIO_PIN_5;
        GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF3_USART1;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure); 
        
    }
    else {
        
        GPIO_InitStructure.Pin              =    GPIO_PIN_4;
        GPIO_InitStructure.GPIO_Mode        =    GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF0_SWD;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.Pin              =    GPIO_PIN_5;
        GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF0_SWD;
        GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure); 
        
        GPIO_InitStructure.Pin              =    GPIO_PIN_6;
        GPIO_InitStructure.GPIO_Mode        =    GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF4_USART1;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
        GPIO_InitStructure.Pin              =    GPIO_PIN_7;
        GPIO_InitStructure.GPIO_Alternate   =    GPIO_AF4_USART1;
        GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);           
    }        
    
    
}

/**
 * @brief  config usart1
 * @param  
 * @return 
 * @note   
 */
static void config_usart1(uint32_t baud)
{
    USART_InitType USART_InitStructure;
    
    USART_InitStructure.BaudRate                   =      baud;
    USART_InitStructure.WordLength                 =      USART_WL_8B;
    USART_InitStructure.StopBits                   =      USART_STPB_1;
    USART_InitStructure.Parity                     =      USART_PE_NO;
    USART_InitStructure.HardwareFlowControl        =      USART_HFCTRL_NONE;
    USART_InitStructure.Mode                       =      USART_MODE_RX | USART_MODE_TX;
    
    USART_Init(USART1, &USART_InitStructure);

}

/**
 * @brief  config usart1 for dfu 
 * @param  
 * @return 
 * @note   
 */

void dfu_usart1_poll_config(uint8_t gpio,uint32_t baud)
{
    
    rcc_configure_usart1();
    gpio_configure_usart1(gpio);
    dfu_usart1_disable();
    config_usart1(baud);
    dfu_usart1_enable();
}

/**
 * @brief  enable usart1
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_enable(void)
{
    USART_Enable(USART1, ENABLE);
}


/**
 * @brief  disable usart1
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_disable(void)
{
    USART_Enable(USART1, DISABLE);
}

/**
 * @brief  send data via usart1
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_send(uint8_t *p_data, uint32_t len)
{
    for(uint32_t i=0;i<len;i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXDE) != SET);
        USART_SendData(USART1, p_data[i]);
    }
}


/**
 * @brief  receive data from usart1
 * @param  
 * @return 
 * @note   
 */
uint8_t dfu_usart1_receive(uint8_t *p_data)
{
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXDNE) != RESET)
    {
        *p_data = USART_ReceiveData(USART1);
        return 1;
    }
    return 0;
}

uint8_t dfu_usart1_poll_boot_byte(uint8_t byte, uint32_t timeout)
{  
    while(timeout--){
        if(USART_GetFlagStatus(USART1, USART_FLAG_RXDNE) != RESET)
        {
            if(USART_ReceiveData(USART1) == byte){
                system_delay_cycles(10);
                while(USART_GetFlagStatus(USART1, USART_FLAG_TXDE) != SET);
                USART_SendData(USART1, 0x79);              
                return 1;
            }
        }    
    }
    return 0;
}


/**
 * @brief  config nvic of usart1
 * @param  
 * @return 
 * @note   
 */
static void nvic_configure_usart1(void)
{
    NVIC_InitType NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel              =            USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority      =            0;
    NVIC_InitStructure.NVIC_IRQChannelCmd           =            ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/**
 * @brief  enable interrupt of usart1
 * @param  
 * @return 
 * @note   
 */
static void config_interrupt_usart1(void)
{
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
}

/**
 * @brief  config usart1 and enable interrupt 
 * @param  
 * @return 
 * @note   
 */
void dfu_usart1_interrupt_config(void)
{
    nvic_configure_usart1();
    config_interrupt_usart1();
}



void dfu_usart1_default_config(void)
{   
    
    RCC->APB2PRST |= RCC_APB2_PERIPH_AFIO|RCC_APB2_PERIPH_GPIOA|RCC_APB2_PERIPH_GPIOB|RCC_APB2_PERIPH_USART1;
    RCC->APB2PRST &= ~(RCC_APB2_PERIPH_AFIO|RCC_APB2_PERIPH_GPIOA|RCC_APB2_PERIPH_GPIOB|RCC_APB2_PERIPH_USART1);

    RCC->APB2PCLKEN &= ~(RCC_APB2_PERIPH_AFIO|RCC_APB2_PERIPH_GPIOA|RCC_APB2_PERIPH_GPIOB|RCC_APB2_PERIPH_USART1);
}

