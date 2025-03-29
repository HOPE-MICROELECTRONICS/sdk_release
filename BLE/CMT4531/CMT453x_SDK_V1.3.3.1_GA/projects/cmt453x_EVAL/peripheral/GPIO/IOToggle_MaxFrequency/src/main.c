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
#include "main.h"
#include <stdio.h>
#include <stdint.h>

GPIO_InitType GPIO_InitStructure;

/**
 * @brief  Main program.
 */
int main(void)
{
    /* -1- Enable GPIOx Clock (to be able to program the configuration registers) */
    RCC_EnableAPB2PeriphClk(GPIOx_CLK, ENABLE);

    /* -2- Configure GPIOx_PIN in output push-pull mode */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIOx_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);

    /* -3- Toggle GPIOx_PIN in an infinite loop */
    while (1)
    {          
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
        GPIOA->POD = 1;
        GPIOA->POD = 0;
    }
}

/**
 * @brief Assert failed function by user.
 * @param file The name of the call that failed.
 * @param line The source line number of the call that failed.
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    while (1)
    {
    }
}
#endif // USE_FULL_ASSERT
/**
 * @}
 */
