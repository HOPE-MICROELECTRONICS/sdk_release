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
 * @file app_usart.c
 * @version v1.0.0
 *
  */
#include "app_usart.h"
#include "main.h"
#include <stdio.h>


/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USART_fifo_buf[256] = {0};
__IO uint8_t USART_fifo_in = 0;
__IO uint8_t USART_fifo_out = 0;
/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/



/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration_USART(void)
{
    /* Enable GPIO clock */
    GPIO_APBxClkCmd(USARTx_GPIO_CLK, ENABLE);
    /* Enable USARTx Clock */
    USART_APBxClkCmd(USARTx_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration_USART(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    

    /* Configure USARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTx_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);   

    /* Configure USARTx Rx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTx_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure); 
}

/**
 * @brief  Deinitializes the USART.
 */
void USART_Deinitializes(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_Enable(USARTx, DISABLE);

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    

    /* Configure USARTx Tx Rx as ANALOG to save power */
    GPIO_InitStructure.Pin            = USARTx_TxPin|USARTx_RxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_ANALOG;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);   


}

/**
 * @brief  Configures the USART as 115200 8n1.
 */
void USART_Configuration(void)
{

    USART_InitType USART_InitStructure;
         
    /* Configure and enable RCC */
    RCC_Configuration_USART();
    /* Configure GPIO for USART */
    GPIO_Configuration_USART();

        /* USARTy and USARTz configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTx */
    USART_Init(USARTx, &USART_InitStructure);
    /* Enable the USARTx */
    USART_Enable(USARTx, ENABLE);

}

/**
 * @brief  USART
 */
void USART_EchoTask(void)
{
    /* USART receive */
    if(USART_GetFlagStatus(USARTx, USART_FLAG_RXDNE) != RESET)
    {
        /*buffer not full yet*/
        if((USART_fifo_in+1) != USART_fifo_out)
        {
            /* Store the received byte in USART_fifo_buf */
            USART_fifo_buf[USART_fifo_in++] = USART_ReceiveData(USARTx);            
        }
    }
    
        /* USART send */
    if(USART_GetFlagStatus(USARTx, USART_FLAG_TXDE) != RESET)
    {
        /*buffer not empty yet*/
        if(USART_fifo_out != USART_fifo_in)
        {
            /*Send one byte to USART from buf */
            USART_SendData(USARTx, USART_fifo_buf[USART_fifo_out++]);
        }
    }

}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE* f)
{
    USART_SendData(USARTx, (uint8_t)ch);
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXDE) == RESET);

    return (ch);
}



/**
 * @}
 */

