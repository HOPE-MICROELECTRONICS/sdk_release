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



#define SPI_MASTER              SPI1
#define SPI_MASTER_CLK          RCC_APB2_PERIPH_SPI1
#define SPI_MASTER_GPIO         GPIOA
#define SPI_MASTER_GPIO_AF        GPIO_AF1_SPI1
#define SPI_MASTER_GPIO_CLK     RCC_APB2_PERIPH_GPIOA
#define SPI_MASTER_PIN_SCK      GPIO_PIN_1
#define SPI_MASTER_PIN_MISO     GPIO_PIN_3
#define SPI_MASTER_PIN_MOSI     GPIO_PIN_2
#define SPI_MASTER_IRQn         SPI1_IRQn

#define SPI_SLAVE                  SPI2
#define SPI_SLAVE_CLK              RCC_APB2_PERIPH_SPI2
#define SPI_SLAVE_GPIO             GPIOB
#define SPI_SLAVE_GPIO_AF        GPIO_AF2_SPI2
#define SPI_SLAVE_GPIO_CLK         RCC_APB2_PERIPH_GPIOB
#define SPI_SLAVE_PIN_SCK          GPIO_PIN_1
#define SPI_SLAVE_PIN_MISO         GPIO_PIN_3
#define SPI_SLAVE_PIN_MOSI         GPIO_PIN_2
#define SPI_SLAVE_IRQn             SPI2_IRQn

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
