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
#include "main.h"
#include "log.h"

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup SPI_CRC
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define BufferSize 32

SPI_InitType SPI_InitStructure;
uint16_t SPI1_Buffer_Tx[BufferSize] = {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
                                       0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
                                       0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
                                       0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40};
uint16_t SPI2_Buffer_Tx[BufferSize] = {0x5152, 0x5354, 0x5556, 0x5758, 0x595A, 0x5B5C, 0x5D5E, 0x5F60,
                                       0x6162, 0x6364, 0x6566, 0x6768, 0x696A, 0x6B6C, 0x6D6E, 0x6F70,
                                       0x7172, 0x7374, 0x7576, 0x7778, 0x797A, 0x7B7C, 0x7D7E, 0x7F80,
                                       0x8182, 0x8384, 0x8586, 0x8788, 0x898A, 0x8B8C, 0x8D8E, 0x8F90};
uint16_t SPI1_Buffer_Rx[BufferSize], SPI2_Buffer_Rx[BufferSize];
uint32_t TxIdx = 0, RxIdx = 0;
__IO uint16_t CRC1Value = 0, CRC2Value = 0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;

void RCC_Configuration(void);
void GPIO_Configuration(void);
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength);

uint32_t sts;
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
    log_info("\n this is a SPI CRC Demo.\n");
    
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();
    
    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();            
    
    /* SPI1 configuration ------------------------------------------------------*/
    SPI_I2S_DeInit(SPI1);
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_16BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_32;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    /* SPI2 configuration ------------------------------------------------------*/
    SPI_I2S_DeInit(SPI2);
    SPI_InitStructure.SpiMode = SPI_MODE_SLAVE;
    SPI_Init(SPI2, &SPI_InitStructure);
    
    /* Enable SPI1 CRC calculation */
    SPI_EnableCalculateCrc(SPI1, ENABLE);
    /* Enable SPI1 */
    SPI_Enable(SPI1, ENABLE);
    
    /* Enable SPI2 CRC calculation */
    SPI_EnableCalculateCrc(SPI2, ENABLE);
    /* Enable SPI2 */
    SPI_Enable(SPI2, ENABLE);

    /* Transfer procedure */
    while (TxIdx < BufferSize - 1)
    {
        /* Wait for SPI1 Tx buffer empty */
        while (SPI_I2S_GetStatus(SPI1, SPI_I2S_TE_FLAG) == RESET)
            ;
        /* Send SPI2 data */
        SPI_I2S_TransmitData(SPI2, SPI2_Buffer_Tx[TxIdx]);
        /* Send SPI1 data */
        SPI_I2S_TransmitData(SPI1, SPI1_Buffer_Tx[TxIdx++]);
        /* Wait for SPI2 data reception */
        while (SPI_I2S_GetStatus(SPI2, SPI_I2S_RNE_FLAG) == RESET)
            ;
        /* Read SPI2 received data */
        SPI2_Buffer_Rx[RxIdx] = SPI_I2S_ReceiveData(SPI2);
        /* Wait for SPI1 data reception */
        while (SPI_I2S_GetStatus(SPI1, SPI_I2S_RNE_FLAG) == RESET)
            ;
        /* Read SPI1 received data */
        SPI1_Buffer_Rx[RxIdx++] = SPI_I2S_ReceiveData(SPI1);
    }
    
    /* Wait for SPI1 Tx buffer empty */
    while (SPI_I2S_GetStatus(SPI1, SPI_I2S_TE_FLAG) == RESET)
        ;
    /* Wait for SPI2 Tx buffer empty */
    while (SPI_I2S_GetStatus(SPI2, SPI_I2S_TE_FLAG) == RESET)
        ;
    
    /* Send last SPI2_Buffer_Tx data */
    SPI_I2S_TransmitData(SPI2, SPI2_Buffer_Tx[TxIdx]);
    /* Enable SPI2 CRC transmission */
    SPI_TransmitCrcNext(SPI2);
    
    /* Send last SPI1_Buffer_Tx data */
    SPI_I2S_TransmitData(SPI1, SPI1_Buffer_Tx[TxIdx]);
    /* Enable SPI1 CRC transmission */
    SPI_TransmitCrcNext(SPI1);

    /* Wait for SPI1 last data reception */
    while (SPI_I2S_GetStatus(SPI1, SPI_I2S_RNE_FLAG) == RESET)
        ;
    /* Read SPI1 last received data */
    SPI1_Buffer_Rx[RxIdx] = SPI_I2S_ReceiveData(SPI1);
    /* Read SPI2 last received data */
    SPI2_Buffer_Rx[RxIdx] = SPI_I2S_ReceiveData(SPI2);
    
    /* Wait for SPI2 data reception: CRC transmitted by SPI1 */
    while (SPI_I2S_GetStatus(SPI2, SPI_I2S_RNE_FLAG) == RESET)
        ;
    /* Wait for SPI1 data reception: CRC transmitted by SPI2 */
    while (SPI_I2S_GetStatus(SPI1, SPI_I2S_RNE_FLAG) == RESET)
        ;
    
    /* Check the received data with the send ones */
    TransferStatus1 = Buffercmp(SPI1_Buffer_Rx, SPI2_Buffer_Tx, BufferSize);
    TransferStatus2 = Buffercmp(SPI2_Buffer_Rx, SPI1_Buffer_Tx, BufferSize);
    /* TransferStatus1, TransferStatus2 = PASSED, if the data transmitted and received
       are correct */
    /* TransferStatus1, TransferStatus2 = FAILED, if the data transmitted and received
       are different */
    printf("data SPI1 TransferStatus1:%d, SPI2 TransferStatus2:%d\r\n", TransferStatus1, TransferStatus2);
    
    /* Test on the SPI1 CRC Error flag */
    if ((SPI_I2S_GetStatus(SPI1, SPI_CRCERR_FLAG)) == SET)
    {
        TransferStatus1 = FAILED;
    }

    /* Test on the SPI2 CRC Error flag */
    if ((SPI_I2S_GetStatus(SPI2, SPI_CRCERR_FLAG)) == SET)
    {
        TransferStatus2 = FAILED;
    }
    
    printf("CRC  SPI1 TransferStatus1:%d, SPI2 TransferStatus2:%d\r\n", TransferStatus1, TransferStatus2);
    
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
    /* GPIOA, GPIOB, AFIO clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* SPI1 Periph clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1 | RCC_APB2_PERIPH_SPI2, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/
    /* Confugure SCK , MISO and MOSI pins as Alternate Function Push Pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_SPI1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    
    /* Configure SPI2 pins: SCK, MISO and MOSI ---------------------------------*/
    GPIO_InitStructure.Pin            = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_SPI2;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength)
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
