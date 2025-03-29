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
 * @file main.h
 
 * @version v1.0.0
 *
  */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmt453x.h"


#define USARTy             USART1
#define USARTy_CLK         RCC_APB2_PERIPH_USART1
#define USARTy_GPIO        GPIOB
#define USARTy_GPIO_CLK    RCC_APB2_PERIPH_GPIOB
#define USARTy_RxPin       GPIO_PIN_7
#define USARTy_TxPin       GPIO_PIN_6
#define USARTy_RTSPin      GPIO_PIN_8
#define USARTy_CTSPin      GPIO_PIN_9                         
#define USARTy_Rx_GPIO_AF  GPIO_AF4_USART1
#define USARTy_Tx_GPIO_AF  GPIO_AF4_USART1
#define USARTy_RTS_GPIO_AF GPIO_AF3_USART1
#define USARTy_CTS_GPIO_AF GPIO_AF3_USART1
#define USARTy_APBxClkCmd  RCC_EnableAPB2PeriphClk
	

#define USARTz              USART2
#define USARTz_CLK          RCC_APB1_PERIPH_USART2
#define USARTz_GPIO         GPIOB
#define USARTz_RTS_GPIO     GPIOB
#define USARTz_CTS_GPIO     GPIOB
#define USARTz_GPIO_CLK     RCC_APB2_PERIPH_GPIOB
#define USARTz_RTS_GPIO_CLK RCC_APB2_PERIPH_GPIOB
#define USARTz_CTS_GPIO_CLK RCC_APB2_PERIPH_GPIOB
#define USARTz_RxPin        GPIO_PIN_5
#define USARTz_TxPin        GPIO_PIN_4
#define USARTz_RTSPin       GPIO_PIN_11
#define USARTz_CTSPin       GPIO_PIN_12
#define USARTz_Rx_GPIO_AF   GPIO_AF3_USART2
#define USARTz_Tx_GPIO_AF   GPIO_AF3_USART2
#define USARTz_RTS_GPIO_AF  GPIO_AF3_USART2
#define USARTz_CTS_GPIO_AF  GPIO_AF3_USART2
#define USARTz_APBxClkCmd  RCC_EnableAPB1PeriphClk


typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

void RCC_Configuration(void);
void GPIO_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Delay(__IO uint32_t nCount);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
