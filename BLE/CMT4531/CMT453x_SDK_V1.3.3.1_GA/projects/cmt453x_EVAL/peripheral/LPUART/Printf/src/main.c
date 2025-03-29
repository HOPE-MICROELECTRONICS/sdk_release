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

/** @addtogroup LPUART_Printf
 * @{
 */

LPUART_InitType LPUART_InitStructure;

/**
 * @brief  Main program
 */
int main(void)
{
    /* System Clocks Configuration */
#if 0    
    RCC_Configuration(RCC_LPUARTCLK_SRC_LSI_LSE);
#else
    RCC_Configuration(RCC_LPUARTCLK_SRC_APB1);
#endif
    
    /* Configure the GPIO ports */
    GPIO_Configuration();

    LPUART_DeInit(LPUARTx);
    /* LPUARTx configuration ------------------------------------------------------*/
    LPUART_InitStructure.BaudRate            = 9600;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    LPUART_InitStructure.Mode                = LPUART_MODE_RX | LPUART_MODE_TX;
    /* Configure LPUARTx */
    LPUART_Init(LPUARTx, &LPUART_InitStructure);

    /* Output a message on Hyperterminal using printf function */
    printf("\n\rLPUART Printf Example: retarget the C library printf function to the LPUART mode:%d, src:0x%08x\n\r", _LPUART1_COM_, RCC_GetLpuartClkSrc(LPUARTx_CLK));

    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 * @param  LPUART_CLK_SRC: specifies the LPUARTx clock source.
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
            /* Specifies the LPUARTx clock source, LSE selected as LPUARTx clock */
            RCC_ConfigLpuartClk(LPUARTx_CLK, RCC_LPUARTCLK_SRC_LSI_LSE);
        }
        break;        
        
        default:
        {
            /* Specifies the LPUARTx clock source, APB1 selected as LPUARTx clock */
            RCC_ConfigLpuartClk(LPUARTx_CLK, RCC_LPUARTCLK_SRC_APB1);
        }
        break;
    }    
    
    /* Enable LPUARTx Clock */
    RCC_EnableLpuartClk(ENABLE);    
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTx_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    
    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);  
    /* Configure LPUARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTx_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTx_Tx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTx_GPIO, &GPIO_InitStructure);

    /* Configure LPAURTx Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = LPUARTx_RxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;    
    GPIO_InitStructure.GPIO_Alternate = LPUARTx_Rx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTx_GPIO, &GPIO_InitStructure);
}

/* retarget the C library printf function to the LPUARTx */
int fputc(int ch, FILE* f)
{
    LPUART_SendData(LPUARTx, (uint8_t)ch);
    while (LPUART_GetFlagStatus(LPUARTx, LPUART_FLAG_TXC) == RESET)
        ;
    LPUART_ClrFlag(LPUARTx, LPUART_FLAG_TXC);      

    return (ch);
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
