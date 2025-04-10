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
#include "main.h"
#include "log.h"

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup I2S_DMA
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define BufferSize 32

DMA_InitType DMA_InitStructure;
GPIO_InitType GPIO_InitStructure;
I2S_InitType I2S_InitStructure;

const uint16_t I2S_MASTER_Buffer_Tx[BufferSize] = {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
                                                   0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
                                                   0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
                                                   0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40};
uint16_t I2S_SLAVE_Buffer_Rx[BufferSize];
volatile TestStatus TransferStatus = FAILED;
ErrorStatus HSEStartUpStatus;

void RCC_Configuration(void);
void GPIO_Configuration(void);
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength);

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
    int i = 0;
    log_init();
    log_info("\n this is a I2S DMA SLAVE Demo.\n");
    
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* Deinitializes the I2S slave peripheral registers --------------------*/
    SPI_I2S_DeInit(I2S_SLAVE);

    /* I2S_SLAVE_Rx_DMA_Channel configuration ---------------------------------------------*/
    DMA_DeInit(I2S_SLAVE_Rx_DMA_Channel);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&SPI1->DAT;
    DMA_InitStructure.MemAddr        = (uint32_t)I2S_SLAVE_Buffer_Rx;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = BufferSize;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(I2S_SLAVE_Rx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(DMA_REMAP_SPI1_RX,DMA,I2S_SLAVE_Rx_DMA_Channel,ENABLE);
        
    /* I2S peripheral configuration */
    I2S_InitStructure.Standard       = I2S_STD_PHILLIPS;
    I2S_InitStructure.DataFormat     = I2S_DATA_FMT_16BITS_EXTENDED;
    I2S_InitStructure.AudioFrequency = I2S_AUDIO_FREQ_48K;
    I2S_InitStructure.CLKPOL         = I2S_CLKPOL_LOW;

    /* Master Transmitter to Slave Receiver communication ------------*/
    /* I2S configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_SlAVE_RX;
    I2S_Init(I2S_SLAVE, &I2S_InitStructure);

    /* Enable I2S_SLAVE Rx request */
    SPI_I2S_EnableDma(I2S_SLAVE, SPI_I2S_DMA_RX, ENABLE);

    /* Enable the I2S */
    I2S_Enable(I2S_SLAVE, ENABLE);

    /* Enable DMA rx Channel */
    DMA_EnableChannel(I2S_SLAVE_Rx_DMA_Channel, ENABLE);

    /* Wait for DMA rx channel transfer complete */
    while (!DMA_GetFlagStatus(I2S_SLAVE_Rx_DMA_FLAG, DMA))
        ;

    /* Check the correctness of written data */
    TransferStatus = Buffercmp(I2S_SLAVE_Buffer_Rx, (uint16_t*)I2S_MASTER_Buffer_Tx, BufferSize);
    /* TransferStatus = PASSED, if the transmitted and received data
       are equal */
    /* TransferStatus = FAILED, if the transmitted and received data
       are different */

    /* I2S_MASTER_Tx_DMA_Channel configuration ---------------------------------------------*/
    DMA_DeInit(I2S_SLAVE_Tx_DMA_Channel);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&SPI1->DAT;
    DMA_InitStructure.MemAddr        = (uint32_t)I2S_SLAVE_Buffer_Rx;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = BufferSize;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(I2S_SLAVE_Tx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(DMA_REMAP_SPI1_TX,DMA,I2S_SLAVE_Tx_DMA_Channel,ENABLE);
    
    DMA_EnableChannel(I2S_SLAVE_Rx_DMA_Channel, DISABLE);
    SPI_I2S_EnableDma(I2S_SLAVE, SPI_I2S_DMA_RX, DISABLE);
    I2S_Enable(I2S_SLAVE, DISABLE);
        
    /* Master Transmitter to Slave Receiver communication ------------*/
    /* I2S configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_SlAVE_TX;
    I2S_Init(I2S_SLAVE, &I2S_InitStructure);

    /* Enable I2S_SLAVE tx request */
    SPI_I2S_EnableDma(I2S_SLAVE, SPI_I2S_DMA_TX, ENABLE);

    /* Enable the I2S */
    I2S_Enable(I2S_SLAVE, ENABLE);

    /* Enable DMA tx Channel */
    DMA_EnableChannel(I2S_SLAVE_Tx_DMA_Channel, ENABLE);

    /* Wait for DMA tx channel transfer complete */
    while (!DMA_GetFlagStatus(I2S_SLAVE_Tx_DMA_FLAG, DMA))
        ;
        
    if(TransferStatus == PASSED)
    {
        log_info("I2S DMA SLAVE Demo test Succeed.\n");
    }
    else
    {
        log_info("I2S DMA SLAVE Demo test Fail.\n");
    }
    
    
    for(i=0; i<32; i++)
    {
        log_info("0x%04x ",I2S_SLAVE_Buffer_Rx[i]);
    }

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
    /* Enable I2S_SLAVE DMA clock */
    RCC_EnableAHBPeriphClk(I2S_SLAVE_DMA_CLK, ENABLE);

    /* GPIOA and AFIO clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* I2S clocks enable */
    RCC_EnableAPB2PeriphClk(I2S_SLAVE_CLK , ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure I2S pins: CK, WS and SD ---------------------------------*/
    GPIO_InitStructure.Pin            = I2S_SLAVE_PIN_CK | I2S_SLAVE_PIN_SD | I2S_SLAVE_PIN_WS;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate = I2S_SLAVE_PIN_AF;
    GPIO_InitPeripheral(I2S_SLAVE_PIN_PORT, &GPIO_InitStructure);
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
