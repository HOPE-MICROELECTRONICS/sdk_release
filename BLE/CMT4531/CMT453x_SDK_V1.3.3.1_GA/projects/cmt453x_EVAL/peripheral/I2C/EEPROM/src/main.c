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
 * @version v1.0.1
 *
  */
#include "cmt453x.h"
#include "i2c_eeprom.h"
#include "stdio.h"
#include "main.h"
/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup I2C_EEPROM
 * @{
 */

uint8_t tx_buf[TEST_EEPROM_SIZE] = {0};
uint8_t rx_buf[TEST_EEPROM_SIZE] = {0};
volatile Status test_status      = FAILED;

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/**
 * @brief   Main program
 */
int main(void)
{
    uint16_t i = 0;
    log_init();
    log_info("\n this is a I2C EEPROM demo, mode:%d\r\n", PROCESS_MODE);
    
    /* Initialize the I2C EEPROM driver ----------------------------------------*/
    I2C_EE_Init();

    /* Fill the buffer to send */
    for (i = 0; i < TEST_EEPROM_SIZE; i++)
    {
        tx_buf[i] = i;
    }

    /* First write in the memory followed by a read of the written data --------*/
    /* Write to I2C EEPROM from TEST_EEPROM_ADDR */
    I2C_EE_WriteBuffer(tx_buf, TEST_EEPROM_ADDR, TEST_EEPROM_SIZE);

    /* Read from I2C EEPROM from sEE_READ_ADDRESS1 */
    I2C_EE_ReadBuffer(rx_buf, TEST_EEPROM_ADDR, TEST_EEPROM_SIZE);

    /* Check if the data written to the memory is read correctly */
    test_status = Buffercmp(tx_buf, rx_buf, TEST_EEPROM_SIZE);

    if (test_status == PASSED)
    {
        log_info("the write and read data are the same,I2C EEPROM test pass\r\n");
    }
    else
    {
        log_info("the write and read data are different,I2C EEPROM test fail\r\n");
    }

    while (1)
    {
    }
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer, pBuffer1: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer identical to pBuffer1
 *         FAILED: pBuffer differs from pBuffer1
 */
Status Buffercmp(uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}


void log_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;
    
    
    GPIO_InitStruct(&GPIO_InitStructure);
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | LOG_PERIPH_GPIO, ENABLE);
    
    RCC_EnableAPB1PeriphClk(LOG_PERIPH, ENABLE);
   
    GPIO_InitStructure.Pin            = LOG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LOG_GPIO_AF;
    GPIO_InitPeripheral(LOG_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.Pin             = LOG_RX_PIN;
    GPIO_InitStructure.GPIO_Alternate  = LOG_GPIO_AF;
    GPIO_InitPeripheral(LOG_GPIO, &GPIO_InitStructure);

    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    // init uart
    USART_Init(LOG_USARTx, &USART_InitStructure);

    // enable uart
    USART_Enable(LOG_USARTx, ENABLE);
}

int fputc(int ch, FILE* f)
{
    USART_SendData(LOG_USARTx, (uint8_t)ch);
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(LOG_USARTx, USART_FLAG_TXC) == RESET)
    {
    }
    return ch;
}

#ifdef USE_FULL_ASSERT

void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    log_error("assertion failed: `%s` at %s:%d", expr, file, line);
    while (1)
    {
    }
}
#endif // USE_FULL_ASSERT

/**
 * @}
 */
