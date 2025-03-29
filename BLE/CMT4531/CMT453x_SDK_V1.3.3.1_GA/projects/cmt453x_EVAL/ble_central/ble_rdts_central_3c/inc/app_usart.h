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
 * @file app_usart.h
 
 * @version v1.0.0
 *
  */
#ifndef __APP_USART_H__
#define __APP_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmt453x.h"

    
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define _USART1_COM_
//#define _USART2_COM_

#ifdef _USART1_COM_
#define USARTx              USART1
#define USARTx_CLK          RCC_APB2_PERIPH_USART1
#define USARTx_GPIO         GPIOB
#define USARTx_GPIO_CLK     RCC_APB2_PERIPH_GPIOB
#define USARTx_RxPin        GPIO_PIN_7
#define USARTx_TxPin        GPIO_PIN_6
#define USARTx_Rx_GPIO_AF   GPIO_AF4_USART1
#define USARTx_Tx_GPIO_AF   GPIO_AF4_USART1
#define GPIO_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USART_APBxClkCmd    RCC_EnableAPB2PeriphClk

#define USARTx_DAT_Base         (USART1_BASE + 0x04)
#define USARTx_Tx_DMA_Channel   DMA_CH1
#define USARTx_Tx_DMA_FLAG      DMA_FLAG_TC1
#define USARTx_Rx_DMA_Channel   DMA_CH2
#define USARTx_Rx_DMA_FLAG      DMA_FLAG_TC2
#define USARTx_Tx_DMA_REMAP     DMA_REMAP_USART1_TX
#define USARTx_Rx_DMA_REMAP     DMA_REMAP_USART1_RX
#define USARTx_IRQn             USART1_IRQn
#define USARTx_IRQHandler       USART1_IRQHandler
#endif

#ifdef _USART2_COM_
#define USARTx              USART2
#define USARTx_CLK          RCC_APB1_PERIPH_USART2
#define USARTx_GPIO         GPIOB
#define USARTx_GPIO_CLK     RCC_APB2_PERIPH_GPIOB
#define USARTx_RxPin        GPIO_PIN_5
#define USARTx_TxPin        GPIO_PIN_4
#define USARTx_Rx_GPIO_AF   GPIO_AF3_USART2
#define USARTx_Tx_GPIO_AF   GPIO_AF3_USART2
#define GPIO_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USART_APBxClkCmd    RCC_EnableAPB1PeriphClk

#define USARTx_DAT_Base         (USART2_BASE + 0x04)
#define USARTx_Tx_DMA_Channel   DMA_CH1
#define USARTx_Tx_DMA_FLAG      DMA_FLAG_TC1 
#define USARTx_Rx_DMA_Channel   DMA_CH2
#define USARTx_Rx_DMA_FLAG      DMA_FLAG_TC2
#define USARTx_Tx_DMA_REMAP     DMA_REMAP_USART2_TX
#define USARTx_Rx_DMA_REMAP     DMA_REMAP_USART2_RX
#define USARTx_IRQn             USART2_IRQn
#define USARTx_IRQHandler       USART2_IRQHandler
#endif


#define USART_RX_DMA_SIZE  256
#define USART_RX_FIFO_SIZE 1000
#define USART_TX_FIFO_SIZE 1000


/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/

void app_usart_configuration(void);
uint8_t app_usart_rx_data_fifo_enter(const uint8_t *p_data, uint16_t len);
uint8_t usart_tx_dma_send(uint8_t *p_data, uint16_t len);
void app_usart_tx_data_blocking(uint8_t *p_data, uint16_t len);

uint8_t app_usart_tx_fifo_enter(const uint8_t *p_data, uint16_t len);
void usart_forward_to_ble_loop(void);
void app_usart_dma_enable(FunctionalState Cmd);
void app_usart_tx_process(void);
#ifdef __cplusplus
}
#endif

#endif /* __APP_USART_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
