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

#define _LPUART1_USART1_

#ifdef _LPUART1_USART1_
#define LPUARTy                 LPUART1
#define LPUARTy_CLK             RCC_LPUART1CLK
#define LPUARTy_GPIO            GPIOB
#define LPUARTy_GPIO_CLK        RCC_APB2_PERIPH_GPIOB
#define LPUARTy_RxPin           GPIO_PIN_2
#define LPUARTy_TxPin           GPIO_PIN_1
#define LPUARTy_Rx_GPIO_AF      GPIO_AF4_LPUART1
#define LPUARTy_Tx_GPIO_AF      GPIO_AF4_LPUART1
#define LPUARTy_DAT_Base        (LPUART1_BASE + 0x10)
#define LPUARTy_Tx_DMA_Channel  DMA_CH1
#define LPUARTy_Tx_DMA_FLAG     DMA_FLAG_TC1
#define LPUARTy_Rx_DMA_Channel  DMA_CH2
#define LPUARTy_Rx_DMA_FLAG     DMA_FLAG_TC2
#define LPUARTy_Tx_DMA_REMAP    DMA_REMAP_LPUART_TX
#define LPUARTy_Rx_DMA_REMAP    DMA_REMAP_LPUART_RX
#define LPUARTy_IRQn            LPUART1_IRQn
#define LPUARTy_IRQHandler      LPUART1_IRQHandler

#define USARTz                  USART2
#define USARTz_CLK              RCC_APB1_PERIPH_USART2
#define USARTz_GPIO             GPIOB
#define USARTz_GPIO_CLK         RCC_APB2_PERIPH_GPIOB
#define USARTz_RxPin            GPIO_PIN_5
#define USARTz_TxPin            GPIO_PIN_4
#define USARTz_Rx_GPIO_AF       GPIO_AF3_USART2
#define USARTz_Tx_GPIO_AF       GPIO_AF3_USART2
#define USARTz_APBxClkCmd       RCC_EnableAPB1PeriphClk
#define USARTz_DAT_Base         (USART2_BASE + 0x04)
#define USARTz_Tx_DMA_Channel   DMA_CH4
#define USARTz_Tx_DMA_FLAG      DMA_FLAG_TC4
#define USARTz_Rx_DMA_Channel   DMA_CH5
#define USARTz_Rx_DMA_FLAG      DMA_FLAG_TC5
#define USARTz_Tx_DMA_REMAP     DMA_REMAP_USART2_TX
#define USARTz_Rx_DMA_REMAP     DMA_REMAP_USART2_RX
#endif

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

void RCC_Configuration(uint32_t LPUART_CLK_SRC);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Delay(__IO uint32_t nCount);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
