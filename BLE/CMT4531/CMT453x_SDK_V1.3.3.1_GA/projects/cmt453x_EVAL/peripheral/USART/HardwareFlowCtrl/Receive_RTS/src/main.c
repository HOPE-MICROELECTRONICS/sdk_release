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

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup USART_Receive_RTS
 * @{
 */

#define TxBufferSize1 (countof(TxBuffer1) - 1)

#define countof(a) (sizeof(a) / sizeof(*(a)))

USART_InitType USART_InitStructure;
uint8_t TxBuffer1[] = "USART Flow_Control Mode Example: Board1_USARTy -> Board2_USARTz using CTS and RTS Flags";
uint8_t RxBuffer1[TxBufferSize1];
__IO uint8_t RxCounter             = 0;
volatile TestStatus TransferStatus = FAILED;

/**
 * @brief  Main program
 */
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* USARTy and USARTz configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_RTS;
    USART_InitStructure.Mode                = USART_MODE_RX;

    /* Configure USARTz */
    USART_Init(USARTz, &USART_InitStructure);
    /* Enable the USARTz */
    USART_Enable(USARTz, ENABLE);

    while (RxCounter < TxBufferSize1)
    {
        /* Loop until the USARTz Receive Data Register is not empty */
        while (USART_GetFlagStatus(USARTz, USART_FLAG_RXDNE) == RESET)
        {
        }

        Delay(0x2FFFF);

        /* Store the received byte in RxBuffer */
        RxBuffer1[RxCounter++] = USART_ReceiveData(USARTz);
    }

    /* Check the received data with the send ones */
    /* TransferStatus = PASSED, if the data transmitted from USARTy and
       received by USARTz are the same */
    /* TransferStatus = FAILED, if the data transmitted from USARTy and
       received by USARTz are different */
    TransferStatus = Buffercmp(TxBuffer1, RxBuffer1, TxBufferSize1);

    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable GPIOx and AFIO clocks */
    RCC_EnableAPB2PeriphClk(USARTz_GPIO_CLK | USARTz_RTS_GPIO_CLK, ENABLE);

    /* Enable USARTz clocks */
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

    /* Configure USARTy RTS as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTz_RTSPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTz_RTS_GPIO_AF;
    GPIO_InitPeripheral(USARTz_RTS_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
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
