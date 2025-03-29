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

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;

#define I2Cx_MASTER                 I2C1
#define I2Cx_MASTER_CLK             RCC_APB1_PERIPH_I2C1
#define I2Cx_MASTER_GPIO            GPIOB
#define I2Cx_MASTER_GPIO_AF         GPIO_AF3_I2C
#define I2Cx_MASTER_GPIO_CLK        RCC_APB2_PERIPH_GPIOB
#define I2Cx_MASTER_SDA_PIN         GPIO_PIN_6
#define I2Cx_MASTER_SCL_PIN         GPIO_PIN_7
#define I2Cx_MASTER_IRQ             I2C1_IRQn

#define DMA_CHANNEL_USED            DMA1_CH1
#define DMA_IRQ_HANDLER             DMA1_Channel1_IRQHandler
#define DMA_IT_FLAG_TC              DMA1_INT_TXC1
#define DMA_IT_FLAG_GL              DMA1_INT_GLB1
#define DMA_IRQN                    DMA1_Channel1_IRQn

typedef enum
{
    C_READY = 0,
    C_START_BIT,
    C_STOP_BIT
}CommCtrl_t;

typedef enum
{
    MASTER_OK = 0,
    MASTER_BUSY,
    MASTER_MODE,
    MASTER_TXMODE,
    MASTER_RXMODE,
    MASTER_SENDING,
    MASTER_SENDED,
    MASTER_RECVD,
    MASTER_BYTEF,
    MASTER_BUSERR,
    MASTER_UNKNOW,
    SLAVE_OK = 20,
    SLAVE_BUSY,
    SLAVE_MODE,
    SLAVE_BUSERR,
    SLAVE_UNKNOW,

}ErrCode_t;

#define LOG_USARTx                USART2
#define LOG_PERIPH                RCC_APB1_PERIPH_USART2
#define LOG_GPIO                  GPIOB
#define LOG_PERIPH_GPIO           RCC_APB2_PERIPH_GPIOB
#define LOG_TX_PIN                GPIO_PIN_4
#define LOG_RX_PIN                GPIO_PIN_5
#define LOG_GPIO_AF               GPIO_AF3_USART2            //GPIO_AF4_USART1

#define log_info(...)     printf(__VA_ARGS__)
#define log_error(...)    printf(__VA_ARGS__)
void log_init(void);
void log_buff(uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
