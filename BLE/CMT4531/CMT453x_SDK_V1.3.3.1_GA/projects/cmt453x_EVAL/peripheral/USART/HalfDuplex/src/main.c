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

/** @addtogroup USART_HalfDuplex
 * @{
 */

#define TxBufferSize1 (countof(TxBuffer1) - 1)
#define TxBufferSize2 (countof(TxBuffer2) - 1)

#define countof(a) (sizeof(a) / sizeof(*(a)))

USART_InitType USART_InitStructure;
uint8_t TxBuffer1[] = "USART Half Duplex: USARTy -> USARTz using HalfDuplex mode";
uint8_t TxBuffer2[] = "USART Half Duplex: USARTz -> USARTy using HalfDuplex mode";
uint8_t RxBuffer1[TxBufferSize2];
uint8_t RxBuffer2[TxBufferSize1];
uint32_t NbrOfDataToRead1 = TxBufferSize2;
uint32_t NbrOfDataToRead2 = TxBufferSize1;
uint8_t TxCounter1 = 0, RxCounter1 = 0;
uint8_t TxCounter2 = 0, RxCounter2 = 0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;

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
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTy and USARTz */
    USART_Init(USARTy, &USART_InitStructure);
    USART_Init(USARTz, &USART_InitStructure);
    
    /* Enable the USARTy and USARTz */
    USART_Enable(USARTy, ENABLE);
    USART_Enable(USARTz, ENABLE);

    /* Enable USARTy Half Duplex Mode*/
    USART_EnableHalfDuplex(USARTy, ENABLE);
    /* Enable USARTz Half Duplex Mode*/
    USART_EnableHalfDuplex(USARTz, ENABLE);
        
    while (NbrOfDataToRead2--)
    {
        /* Wait until end of transmit */
        while (USART_GetFlagStatus(USARTy, USART_FLAG_TXDE) == RESET)
        {
        }
        /* Write one byte in the USARTy Transmit Data Register */
        USART_SendData(USARTy, TxBuffer1[TxCounter1++]);

        /* Wait the byte is entirely received by USARTz */
        while (USART_GetFlagStatus(USARTz, USART_FLAG_RXDNE) == RESET)
        {
        }
        /* Store the received byte in the RxBuffer2 */
        RxBuffer2[RxCounter2++] = USART_ReceiveData(USARTz);
    }

    /* Clear the USARTy Data Register */
    USART_ReceiveData(USARTy);

    while (NbrOfDataToRead1--)
    {
        /* Wait until end of transmit */
        while (USART_GetFlagStatus(USARTz, USART_FLAG_TXDE) == RESET)
        {
        }
        /* Write one byte in the USARTz Transmit Data Register */
        USART_SendData(USARTz, TxBuffer2[TxCounter2++]);

        /* Wait the byte is entirely received by USARTy */
        while (USART_GetFlagStatus(USARTy, USART_FLAG_RXDNE) == RESET)
        {
        }
        /* Store the received byte in the RxBuffer1 */
        RxBuffer1[RxCounter1++] = USART_ReceiveData(USARTy);
    }

    /* Check the received data with the send ones */
    TransferStatus1 = Buffercmp(TxBuffer1, RxBuffer2, TxBufferSize1);
    /* TransferStatus = PASSED, if the data transmitted from USARTy and
       received by USARTz are the same */
    /* TransferStatus = FAILED, if the data transmitted from USARTy and
       received by USARTz are different */
    TransferStatus2 = Buffercmp(TxBuffer2, RxBuffer1, TxBufferSize2);
    /* TransferStatus = PASSED, if the data transmitted from USARTz and
       received by USARTy are the same */
    /* TransferStatus = FAILED, if the data transmitted from USARTz and
       received by USARTy are different */

    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(USARTy_GPIO_CLK | USARTz_GPIO_CLK, ENABLE);
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

    /* Configure USARTy Tx as alternate function open-drain and pull-up */
    GPIO_InitStructure.Pin            = USARTy_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Tx as alternate function open-drain */
    GPIO_InitStructure.Pin            = USARTz_TxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Tx_GPIO_AF;
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
