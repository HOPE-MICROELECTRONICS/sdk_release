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
#include <stdio.h>
#include "main.h"
#include "bsp_led.h"
#include "bsp_key.h"

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup USART_MultiProcessor
 * @{
 */

USART_InitType USART_InitStructure;

/**
 * @brief  Main program
 */
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* Initialize Leds, Wakeup and Key Buttons mounted on board */
    LED_GPIO_Config();
    EXTI_Key_Config();
    
    /* USARTy and USARTz configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_9B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTy and USARTz */
    USART_Init(USARTy, &USART_InitStructure);
    USART_Init(USARTz, &USART_InitStructure);

    /* Enable the USARTy and USARTz */
    USART_Enable(USARTy, ENABLE);
    USART_Enable(USARTz, ENABLE);

    /* Set the USARTy and USARTz Address */
    USART_SetAddr(USARTy, 0x1);
    USART_SetAddr(USARTz, 0x2);

    /* Select the USARTz WakeUp Method */
    USART_ConfigWakeUpMode(USARTz, USART_WUM_ADDRMASK);

    while (1)
    {
        /* Send one byte from USARTy to USARTz */
        USART_SendData(USARTy, 0x33);

        /* Wait while USARTy TXE = 0 */
        while (USART_GetFlagStatus(USARTy, USART_FLAG_TXDE) == RESET)
        {
        }

        if (USART_GetFlagStatus(USARTz, USART_FLAG_RXDNE) != RESET)
        {
            if (USART_ReceiveData(USARTz) == 0x33)
            {
                LED1_ON;
                Delay(0xFFFFF);
                LED1_OFF;
                Delay(0xFFFFF);

                LED2_ON;
                Delay(0xFFFFF);
                LED2_OFF;
                Delay(0xFFFFF);
            }
        }
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(USARTy_GPIO_CLK | USARTz_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    /* Enable USARTy and USARTz Clock */
    USARTy_APBxClkCmd(USARTy_CLK, ENABLE);
    USARTz_APBxClkCmd(USARTz_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTy_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTz_TxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);

    /* Configure USARTy Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = USARTy_RxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);    
}

/**
 * @brief  Inserts a delay time.
 * @param nCount specifies the delay time length.
 */
void Delay(__IO uint32_t nCount)
{
    /* Decrement nCount value */
    for (; nCount != 0; nCount--)
        ;
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

/**
 * @}
 */

/**
 * @}
 */
