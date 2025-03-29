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

#include "cmt453x.h"
#include "cmt453x_conf.h"

#define SPIy          SPI1
#define SPIy_CLK      RCC_APB2_PERIPH_SPI1
#define SPIy_GPIO     GPIOA
#define SPIy_GPIO_AF  GPIO_AF1_SPI1
#define SPIy_GPIO_CLK RCC_APB2_PERIPH_GPIOA
#define SPIy_PIN_SCK  GPIO_PIN_1
#define SPIy_PIN_MISO GPIO_PIN_3
#define SPIy_PIN_MOSI GPIO_PIN_2

#define SPIz          SPI2
#define SPIz_CLK      RCC_APB2_PERIPH_SPI2
#define SPIz_GPIO     GPIOB
#define SPIz_GPIO_AF  GPIO_AF2_SPI2
#define SPIz_GPIO_CLK RCC_APB2_PERIPH_GPIOB
#define SPIz_PIN_SCK  GPIO_PIN_1
#define SPIz_PIN_MISO GPIO_PIN_3
#define SPIz_PIN_MOSI GPIO_PIN_2

#endif /* __MAIN_H__ */
