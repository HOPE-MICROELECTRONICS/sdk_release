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
#include "log.h"
/**
 *  Cortex-M0 SysTick
 */


#define SYSTICK_1MS           ((uint32_t)1000)
#define DEMO_USART_BAUDRATE    ((uint32_t)115200)

uint32_t Tick_num = 0;

void rcc_debug(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
    
    GPIO_InitStruct(&GPIO_InitStructure);
    
    /* Configure rcc_mco pin*/
    GPIO_InitStructure.Pin               = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode         = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed        = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate    = GPIO_AF4_MCO;
    GPIO_InitStructure.GPIO_Pull         = GPIO_NO_PULL;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    
    RCC_ConfigMco(RCC_MCO_SYSCLK);
    printf("RCC->CFG:0x%02x\r\n", RCC->CFG);
}

/**
 * @brief  Main program.
 */
int main(void)
{
    /* USART Init */
    log_init();
    printf("Cortex-M0 SysTick \r\n");
    
    rcc_debug();
    /* Get SystemCoreClock */
    SystemCoreClockUpdate();
    printf("SystemCoreClock:%d\r\n", SystemCoreClock);
    /* Config 1s SysTick  */
    SysTick_Config(SystemCoreClock/SYSTICK_1MS);

    while (1)
    {
        if((Tick_num%1000) == 0)
        {
            printf("Cortex-M0 SysTick IRQ \r\n");
        }
    }
}

/**
 * @}
 */

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
*          line: assert_param error line source number
 * @return None
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}

/**
 * @}
 */
#endif
