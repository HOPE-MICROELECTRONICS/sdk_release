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
 * @file i2c_eeprom.h
 
 * @version v1.0.0
 *
  */
#ifndef __I2C_EEPROM_H__
#define __I2C_EEPROM_H__

#include "cmt453x.h"
#include <stdio.h>

typedef enum i2c_state
{
    COMM_DONE       = 0, /// done successfully
    COMM_PRE        = 1,
    COMM_IN_PROCESS = 2,
    COMM_EXIT       = 3 /// exit since failure
} I2C_STATE;

typedef enum i2c_direction
{
    Transmitter = 0x00,
    Receiver    = 0x01
} I2C_DIRECTION;

/**
 * PROCESS MODE
 * 0=polling
 * 1=interrupt
 * 2=DMA
 */

#define PROCESS_MODE     (0)

#define I2C1_TEST
//#define I2C1_REMAP

#ifdef I2C1_TEST
#define I2Cx            I2C1
#define I2Cx_SCL_PIN    GPIO_PIN_7
#define I2Cx_SDA_PIN    GPIO_PIN_6
#define GPIOx           GPIOB
#define I2Cx_GPIO_AF    GPIO_AF3_I2C
#define I2Cx_IRQ        I2C1_IRQn

#define I2Cx_peripheral_clk_en() RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C1, ENABLE)
#define I2Cx_scl_clk_en()        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE)
#define I2Cx_sda_clk_en()        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE)
#endif

#define TEST_EEPROM_SIZE 256
#define TEST_EEPROM_ADDR 0x00
#define I2C_Speed        100000
#define EEPROM_ADDRESS   0xA0
#define I2C_PageSize     16//32 /// eeprom IC type M24C32
#define sEE_FLAG_TIMEOUT ((uint32_t)0x1000)
#define sEE_LONG_TIMEOUT ((uint32_t)(100 * sEE_FLAG_TIMEOUT))

/** Maximum number of trials for sEE_WaitEepromStandbyState() function */
#define sEE_MAX_TRIALS_NUMBER 150
#define FALSE                 0
#define TRUE                  1

void I2C_EE_Init(void);
void I2C_EE_WriteBuffer(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void I2C_EE_WriteOnePage(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void I2C_EE_PageWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite);
void I2C_EE_WriteOnePageCompleted(void);

void I2C_EE_ReadBuffer(u8* pBuffer, u16 ReadAddr, u16 NumByteToRead);
void I2C_EE_WaitOperationIsCompleted(void);
void I2C_EE_WaitEepromStandbyState(void);

void i2c1_evt_handle(void);
void i2c1_err_handle(void);
void i2c1_send_dma_handle(void);
void i2c1_receive_dma_handle(void);

#endif /* __I2C_EEPROM_H__ */
