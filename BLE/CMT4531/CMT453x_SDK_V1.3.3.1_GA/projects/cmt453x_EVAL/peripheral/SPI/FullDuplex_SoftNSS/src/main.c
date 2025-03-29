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
#include "log.h"

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup FullDuplex_SoftNSS
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define BufferSize 32

SPI_InitType SPI_InitStructure;
uint8_t SPIy_Buffer_Tx[BufferSize] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
                                      0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
                                      0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20};
uint8_t SPIz_Buffer_Tx[BufferSize] = {0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B,
                                      0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66,
                                      0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70};
uint8_t SPIy_Buffer_Rx[BufferSize], SPIz_Buffer_Rx[BufferSize];
__IO uint8_t TxIdx = 0, RxIdx = 0, k = 0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;
volatile TestStatus TransferStatus3 = FAILED, TransferStatus4 = FAILED;

void RCC_Configuration(void);
void GPIO_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_cmt453x.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_cmt453x.c file
       */
    
    log_init();
    log_info("\n this is a SPI FullDuplex_SoftNSS Demo.\n");
    
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* 1st phase: SPIy Master and SPIz Slave */
    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* SPIy Config -------------------------------------------------------------*/
    SPI_I2S_DeInit(SPIy);
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_8;
    SPI_InitStructure.FirstBit      = SPI_FB_LSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(SPIy, &SPI_InitStructure);
        
    /* SPIz Config -------------------------D------------------------------------*/
    SPI_I2S_DeInit(SPIz);
    SPI_InitStructure.SpiMode = SPI_MODE_SLAVE;
    SPI_Init(SPIz, &SPI_InitStructure);
    
    /* Enable SPIy */
    SPI_Enable(SPIy, ENABLE);
        /* Enable SPIz */
    SPI_Enable(SPIz, ENABLE);

    /* Transfer procedure */
    while (TxIdx < BufferSize)
    {
        /* Wait for SPIy Tx buffer empty */
        while (SPI_I2S_GetStatus(SPIy, SPI_I2S_TE_FLAG) == RESET)
            ;
        /* Send SPIz data */
        SPI_I2S_TransmitData(SPIz, SPIz_Buffer_Tx[TxIdx]);
        /* Send SPIy data */
        SPI_I2S_TransmitData(SPIy, SPIy_Buffer_Tx[TxIdx++]);
        /* Wait for SPIz data reception */
        while (SPI_I2S_GetStatus(SPIz, SPI_I2S_RNE_FLAG) == RESET)
            ;
        /* Read SPIz received data */
        SPIz_Buffer_Rx[RxIdx] = SPI_I2S_ReceiveData(SPIz);
        /* Wait for SPIy data reception */
        while (SPI_I2S_GetStatus(SPIy, SPI_I2S_RNE_FLAG) == RESET)
            ;
        /* Read SPIy received data */
        SPIy_Buffer_Rx[RxIdx++] = SPI_I2S_ReceiveData(SPIy);
    }

    /* Check the correctness of written dada */
    TransferStatus1 = Buffercmp(SPIz_Buffer_Rx, SPIy_Buffer_Tx, BufferSize);
    TransferStatus2 = Buffercmp(SPIy_Buffer_Rx, SPIz_Buffer_Tx, BufferSize);
    /* TransferStatus1, TransferStatus2 = PASSED, if the transmitted and received data
       are equal */
    /* TransferStatus1, TransferStatus2 = FAILED, if the transmitted and received data
       are different */
    
    printf("SPIy->SPIz result TransferStatus1:%d, TransferStatus2:%d\r\n", TransferStatus1, TransferStatus2);

    /* SPIy Re-configuration ---------------------------------------------------*/
    SPI_InitStructure.SpiMode = SPI_MODE_SLAVE;
    SPI_Init(SPIy, &SPI_InitStructure);

    /* SPIz Re-configuration ---------------------------------------------------*/
    SPI_InitStructure.SpiMode = SPI_MODE_MASTER;
    SPI_Init(SPIz, &SPI_InitStructure);

    /* Reset TxIdx, RxIdx indexes and receive tables values */
    TxIdx = 0;
    RxIdx = 0;
    for (k = 0; k < BufferSize; k++)
        SPIz_Buffer_Rx[k] = 0;
    for (k = 0; k < BufferSize; k++)
        SPIy_Buffer_Rx[k] = 0;

    /* Transfer procedure */
    while (TxIdx < BufferSize)
    {
        /* Wait for SPIz Tx buffer empty */
        while (SPI_I2S_GetStatus(SPIz, SPI_I2S_TE_FLAG) == RESET)
            ;
        /* Send SPIy data */
        SPI_I2S_TransmitData(SPIy, SPIy_Buffer_Tx[TxIdx]);
        /* Send SPIz data */
        SPI_I2S_TransmitData(SPIz, SPIz_Buffer_Tx[TxIdx++]);
        /* Wait for SPIy data reception */
        while (SPI_I2S_GetStatus(SPIy, SPI_I2S_RNE_FLAG) == RESET)
            ;
        /* Read SPIy received data */
        SPIy_Buffer_Rx[RxIdx] = SPI_I2S_ReceiveData(SPIy);
        /* Wait for SPIz data reception */
        while (SPI_I2S_GetStatus(SPIz, SPI_I2S_RNE_FLAG) == RESET)
            ;
        /* Read SPIz received data */
        SPIz_Buffer_Rx[RxIdx++] = SPI_I2S_ReceiveData(SPIz);
    }

    /* Check the correctness of written dada */
    TransferStatus3 = Buffercmp(SPIz_Buffer_Rx, SPIy_Buffer_Tx, BufferSize);
    TransferStatus4 = Buffercmp(SPIy_Buffer_Rx, SPIz_Buffer_Tx, BufferSize);
    /* TransferStatus3, TransferStatus4 = PASSED, if the transmitted and received data
       are equal */
    /* TransferStatus3, TransferStatus4 = FAILED, if the transmitted and received data
       are different */
    printf("SPIz->SPIy result TransferStatus3:%d, TransferStatus4:%d\r\n", TransferStatus3, TransferStatus4);
    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks --------------------------------------------------*/

    /* Enable GPIO clock for SPIy and SPIz */
    RCC_EnableAPB2PeriphClk(SPIy_GPIO_CLK | SPIz_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Enable SPIy and SPIz Periph clock */
    RCC_EnableAPB2PeriphClk(SPIy_CLK | SPIz_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration()
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
    /* Confugure SCK , MISO and MOSI pins as Alternate Function Push Pull */
    GPIO_InitStructure.Pin            = SPIy_PIN_SCK | SPIy_PIN_MISO | SPIy_PIN_MOSI;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate = SPIy_GPIO_AF;
    GPIO_InitPeripheral(SPIy_GPIO, &GPIO_InitStructure);
    
    /* Configure SPI2 pins: SCK, MISO and MOSI ---------------------------------*/
    /* Confugure SCK , MISO and MOSI pins as Alternate Function Push Pull */
    GPIO_InitStructure.Pin            = SPIz_PIN_SCK | SPIz_PIN_MISO | SPIz_PIN_MOSI;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate = SPIz_GPIO_AF;
    GPIO_InitPeripheral(SPIz_GPIO, &GPIO_InitStructure);
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
