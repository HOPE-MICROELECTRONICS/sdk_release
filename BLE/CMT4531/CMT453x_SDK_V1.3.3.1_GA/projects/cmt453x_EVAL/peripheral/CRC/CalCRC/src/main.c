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
#include "cmt453x.h"
#include "log.h"

/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup CRC_Calculation
 * @{
 */

#define BUFFER_SIZE  114
#define CRC_DFE_POLY 0x8005

static const uint32_t DataBuffer[BUFFER_SIZE] = {
    0x00001021, 0x20423063, 0x408450a5, 0x60c670e7, 0x9129a14a, 0xb16bc18c, 0xd1ade1ce, 0xf1ef1231, 0x32732252,
    0x52b54294, 0x72f762d6, 0x93398318, 0xa35ad3bd, 0xc39cf3ff, 0xe3de2462, 0x34430420, 0x64e674c7, 0x44a45485,
    0xa56ab54b, 0x85289509, 0xf5cfc5ac, 0xd58d3653, 0x26721611, 0x063076d7, 0x569546b4, 0xb75ba77a, 0x97198738,
    0xf7dfe7fe, 0xc7bc48c4, 0x58e56886, 0x78a70840, 0x18612802, 0xc9ccd9ed, 0xe98ef9af, 0x89489969, 0xa90ab92b,
    0x4ad47ab7, 0x6a961a71, 0x0a503a33, 0x2a12dbfd, 0xfbbfeb9e, 0x9b798b58, 0xbb3bab1a, 0x6ca67c87, 0x5cc52c22,
    0x3c030c60, 0x1c41edae, 0xfd8fcdec, 0xad2abd0b, 0x8d689d49, 0x7e976eb6, 0x5ed54ef4, 0x2e321e51, 0x0e70ff9f,
    0xefbedfdd, 0xcffcbf1b, 0x9f598f78, 0x918881a9, 0xb1caa1eb, 0xd10cc12d, 0xe16f1080, 0x00a130c2, 0x20e35004,
    0x40257046, 0x83b99398, 0xa3fbb3da, 0xc33dd31c, 0xe37ff35e, 0x129022f3, 0x32d24235, 0x52146277, 0x7256b5ea,
    0x95a88589, 0xf56ee54f, 0xd52cc50d, 0x34e224c3, 0x04817466, 0x64475424, 0x4405a7db, 0xb7fa8799, 0xe75ff77e,
    0xc71dd73c, 0x26d336f2, 0x069116b0, 0x76764615, 0x5634d94c, 0xc96df90e, 0xe92f99c8, 0xb98aa9ab, 0x58444865,
    0x78066827, 0x18c008e1, 0x28a3cb7d, 0xdb5ceb3f, 0xfb1e8bf9, 0x9bd8abbb, 0x4a755a54, 0x6a377a16, 0x0af11ad0,
    0x2ab33a92, 0xed0fdd6c, 0xcd4dbdaa, 0xad8b9de8, 0x8dc97c26, 0x5c644c45, 0x3ca22c83, 0x1ce00cc1, 0xef1fff3e,
    0xdf7caf9b, 0xbfba8fd9, 0x9ff86e17, 0x7e364e55, 0x2e933eb2, 0x0ed11ef0};

static const uint8_t Buffer[8] = {0x7e, 0x11, 0x36, 0x4e, 0x55, 0x0a, 0xf1, 0x1a};

__IO uint32_t CRCValue   = 0;
__IO uint16_t CRC16Value = 0;

void Delay(__IO uint32_t nCount);

uint8_t ReverseBits(uint8_t bufin)
{
    uint8_t bufout = 0;
    uint8_t j      = 0;
    for (j = 0; j < 4; j++)
    {
        bufout |= (bufin & (0x01 << j)) << (7 - 2 * j);
        bufout |= (bufin & (0x10 << (j))) >> (2 * j + 1);
    }
    return bufout;
}
uint16_t GetCRC16_Software(uint8_t* buf, uint32_t len, uint16_t initcrc)
{
    uint16_t crc16 = initcrc;
    uint8_t i = 0, j = 0;

    for (i = 0; i < len; i++)
    {
        crc16 ^= (uint16_t)(buf[i] << 8);
        for (j = 0; j < 8; j++)
        {
            if ((crc16 & 0x8000) > 0)
            {
                crc16 = (uint16_t)((crc16 << 1) ^ CRC_DFE_POLY);
            }
            else
            {
                crc16 <<= 1;
            }
        }
    }
    return crc16;
}


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
    log_info("\nthis is a CRC Demo.\n");
    
    /* Enable CRC clock */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_CRC, ENABLE);

    /* Compute the 32bit CRC of "DataBuffer" */
    CRC32_ResetCrc();
    CRCValue = CRC32_CalcBufCrc((uint32_t*)DataBuffer, BUFFER_SIZE);
    printf("Step1 hardware CRC32Value:0x%08x\r\n", CRCValue);
    
    /* Compute the 16bit CRC of an array */
    CRC16Value = CRC16_CalcBufCrc((uint8_t*)Buffer, 8);
    printf("Step2 hardware CRC16Value:0x%04x\r\n", CRC16Value);
    
    /* Compute the 16bit CRC of an array by software*/
    CRCValue   = GetCRC16_Software((uint8_t*)Buffer, 8, 0x00);
    printf("Step3 software CRC16Value:0x%04x\r\n", CRCValue);
    
    while (1)
    {
    }
}

/**
 * @brief  Inserts a delay time.
 * @param nCount specifies the delay time length.
 */

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
