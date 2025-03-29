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

/** @addtogroup LPUART_Transmit_CTS
 * @{
 */

#define TxBufferSize1 (countof(TxBuffer1))

#define countof(a) (sizeof(a) / sizeof(*(a)))

LPUART_InitType LPUART_InitStructure;
uint8_t TxBuffer1[]    = "LPUART Flow_Control Mode Example: Board1_LPUARTy -> Board2_LPUARTz using CTS and RTS Flags";
__IO uint8_t TxCounter = 0;

/**
 * @brief  Main program
 */
int main(void)
{
    log_init();
    printf("\n This is lpuart hardware FlowCtrl Tx demo\r\n");
    
    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* System Clocks Configuration */
    RCC_Configuration(RCC_LPUARTCLK_SRC_APB1);  

    /* LPUART configuration ------------------------------------------------------*/    
    LPUART_DeInit(LPUARTy);
    LPUART_InitStructure.BaudRate            = 9600;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_CTS;
    LPUART_InitStructure.Mode                = LPUART_MODE_TX;
    /* Configure LPUART */
    LPUART_Init(LPUARTy, &LPUART_InitStructure);

    while (TxCounter < TxBufferSize1)
    {
        /* Send one byte from Board1_LPUART to Board2_LPUART */
        LPUART_SendData(LPUARTy, TxBuffer1[TxCounter++]);
        /* Loop until Board1_LPUART DAT register is empty */
        while (LPUART_GetFlagStatus(LPUARTy, LPUART_FLAG_TXC) == RESET)
        {
        }
        LPUART_ClrFlag(LPUARTy, LPUART_FLAG_TXC); 
    }
    
    printf("send finish\r\n");
    
    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 * @param  LPUART_CLK_SRC: specifies the LPUARTy clock source.
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
            /* Specifies the LPUARTy clock source, LSE selected as LPUARTy clock */
            RCC_ConfigLpuartClk(LPUARTy_CLK, RCC_LPUARTCLK_SRC_LSI_LSE);
        }
        break;        
        
        default:
        {
            /* Specifies the LPUARTy clock source, APB1 selected as LPUARTy clock */
            RCC_ConfigLpuartClk(LPUARTy_CLK, RCC_LPUARTCLK_SRC_APB1);
        }
        break;
    }  
    
    /* Enable LPUARTy Clock */
    RCC_EnableLpuartClk(ENABLE);    
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTy_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure LPUARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTy_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTy_Tx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTy_GPIO, &GPIO_InitStructure);

    /* Configure LPUARTy CTS as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = LPUARTy_CTSPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTy_CTS_GPIO_AF;
    GPIO_InitPeripheral(LPUARTy_GPIO, &GPIO_InitStructure);
}


/**
 * @}
 */

/**
 * @}
 */
