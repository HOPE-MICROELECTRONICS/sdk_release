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
 * @file bsp_led.h
 
 * @version v1.0.0
 *
  */
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "cmt453x.h"

//************************************************************************************//
/* Define the LED that is connected GPIO*/

#define LED1_GPIO_PORT GPIOB
#define LED1_GPIO_CLK  RCC_APB2_PERIPH_GPIOB
#define LED1_GPIO_PIN  GPIO_PIN_0

#define LED2_GPIO_PORT GPIOA
#define LED2_GPIO_CLK  RCC_APB2_PERIPH_GPIOA
#define LED2_GPIO_PIN  GPIO_PIN_6


#define digitalHi(p, i)                                                                                                \
    {                                                                                                                  \
        p->PBSC = i;                                                                                                   \
    } // output high
#define digitalLo(p, i)                                                                                                \
    {                                                                                                                  \
        p->PBC = i;                                                                                                    \
    } // output low
#define digitalToggle(p, i)                                                                                            \
    {                                                                                                                  \
        p->POD ^= i;                                                                                                   \
    } // toggle

#define LED1_TOGGLE digitalToggle(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED1_OFF    digitalLo(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED1_ON     digitalHi(LED1_GPIO_PORT, LED1_GPIO_PIN)

#define LED2_TOGGLE digitalToggle(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED2_OFF    digitalLo(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED2_ON     digitalHi(LED2_GPIO_PORT, LED2_GPIO_PIN)

void LED_GPIO_Config(void);

#endif /* __BSP_LED_H__ */
