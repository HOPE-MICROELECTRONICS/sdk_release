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
#include "log.h"

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup LPUART_Receive_RTS
 * @{
 */


#define TxBufferSize1 (countof(TxBuffer1))

#define countof(a) (sizeof(a) / sizeof(*(a)))

LPUART_InitType LPUART_InitStructure;
uint8_t TxBuffer1[] = "LPUART Flow_Control Mode Example: Board1_LPUARTy -> Board2_LPUARTz using CTS and RTS Flags";
uint8_t RxBuffer1[TxBufferSize1];
__IO uint8_t RxCounter             = 0;
volatile TestStatus TransferStatus = FAILED;

/**
 * @brief  Main program
 */
int main(void)
{
    log_init();
    printf("\n This is lpuart hardware FlowCtrl rx demo\r\n");
    
    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* System Clocks Configuration */   
    RCC_Configuration(RCC_LPUARTCLK_SRC_APB1); 

    /* LPUARTz configuration ------------------------------------------------------*/    
    LPUART_DeInit(LPUARTz);
    LPUART_InitStructure.BaudRate            = 9600;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_RTS;
    LPUART_InitStructure.Mode                = LPUART_MODE_RX;
    /* Configure LPUARTz */
    LPUART_Init(LPUARTz, &LPUART_InitStructure);

    while (RxCounter < TxBufferSize1)
    {
        /* Loop until the Board2_LPUARTz Receive Data Register is not empty */
        while (LPUART_GetFlagStatus(LPUARTz, LPUART_FLAG_FIFO_NE) == RESET)
        {
        }

        /* Store the received byte in RxBuffer */
        RxBuffer1[RxCounter++] = LPUART_ReceiveData(LPUARTz);
    }

    /* Check the received data with the send ones */
    TransferStatus = Buffercmp(TxBuffer1, RxBuffer1, TxBufferSize1);
    
    printf("test result TransferStatus:%d\r\n", TransferStatus);
    printf("RxBuffer1:%s\r\n", RxBuffer1);
    
    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 * @param  LPUART_CLK_SRC: specifies the LPUARTz clock source.
 */
void RCC_Configuration(uint32_t LPUART_CLK_SRC)
{
    switch(LPUART_CLK_SRC)
    {
        case RCC_LPUARTCLK_SRC_LSI_LSE:
        {  
#if 0            
            RCC_ConfigLSXSEL(RCC_RTCCLK_SRC_LSE);
            /* Configures the External Low Speed oscillator (LSE) */
            RCC_ConfigLse(RCC_LSE_ENABLE);        //32.768KHz
            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSERD) == RESET)
            {
            }
#else            
            RCC_ConfigLSXSEL(RCC_RTCCLK_SRC_LSI);
            RCC_EnableLsi(ENABLE);        //32KHz
            while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
            {
            }
#endif
            /* Specifies the LPUARTz clock source, LSE selected as LPUARTz clock */
            RCC_ConfigLpuartClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_LSI_LSE);
        }
        break;        
        
        default:
        {
            /* Specifies the LPUARTz clock source, APB1 selected as LPUARTz clock */
            RCC_ConfigLpuartClk(LPUARTz_CLK, RCC_LPUARTCLK_SRC_APB1);
        }
        break;
    }  
    
    /* Enable LPUARTz Clock */
    RCC_EnableLpuartClk(ENABLE);    
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTz_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure LPUARTz RTS as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTz_RTSPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTz_RTS_GPIO_AF;
    GPIO_InitPeripheral(LPUARTz_GPIO, &GPIO_InitStructure);

    /* Configure LPUARTz Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = LPUARTz_RxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTz_Rx_GPIO_AF;    
    GPIO_InitPeripheral(LPUARTz_GPIO, &GPIO_InitStructure);
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
 * @}
 */

/**
 * @}
 */
