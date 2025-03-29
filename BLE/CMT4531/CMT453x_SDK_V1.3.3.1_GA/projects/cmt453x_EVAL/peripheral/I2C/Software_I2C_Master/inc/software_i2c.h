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
 * @file software_i2c.h
 
 * @version v1.0.0
 *
  */
#ifndef __SOFTWARE_I2C_H__
#define __SOFTWARE_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmt453x.h"


 
 
//??IIC??????
typedef struct 
{
    GPIO_Module    *SDA_GPIOx;       //SDA port
    GPIO_Module    *SCL_GPIOx;       //SCL port
    uint32_t        SDA_PINx;        //SDA pin
    uint32_t        SCL_PINx;        //SCL pin
    uint32_t        DelayUS;         //delay time in US
    //port function
    void (*Start)(void *pHandle);                //SI2C start signal
    void (*Stop)(void *pHandle);                //SI2C stop signal
    bool (*SendByte)(void *pHandle, uint8_t data);    //SI2C send a byte
    uint8_t (*ReadByte)(void *pHandle,bool isAck);    //SI2C read a byte
}SI2C_HANDLE;
 
 
bool SI2C_Init(SI2C_HANDLE *pHandle, GPIO_Module *SDA_GPIOx, GPIO_Module *SCL_GPIOx,\
                uint32_t SDA_Pin, uint32_t SCL_Pin,uint8_t DelayUS);    //??IIC???
 
void SI2C_Start(SI2C_HANDLE *pHandle);                //SI2C start signal
void SI2C_Stop(SI2C_HANDLE *pHandle);                //SI2C stop signal
bool SI2C_WaitAck(SI2C_HANDLE *pHandle);            //SI2C wait ack signal
void SI2C_Ack(SI2C_HANDLE *pHandle);                //SI2C send ack signal
void SI2C_NAck(SI2C_HANDLE *pHandle);                //SI2C send nack signal
bool SI2C_SendByte(SI2C_HANDLE *pHandle, uint8_t data);    //SI2C send a byte
uint8_t SI2C_ReadByte(SI2C_HANDLE *pHandle,bool isAck);    //SI2C read a byte
bool SI2C_ReadReg(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint16_t RegAddr, \
                  bool is8bitRegAddr, uint8_t *pDataBuff, uint16_t ReadByteNum); //SI2C read reg
bool SI2C_WriteReg(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint16_t RegAddr, \
                   bool is8bitRegAddr, uint8_t *pDataBuff, uint16_t WriteByteNum); //SI2C write reg

bool SI2C_MasterWrite(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint8_t *pDataBuff, uint16_t WriteByteNum);
bool SI2C_MasterRead(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint8_t *pDataBuff, uint16_t ReadByteNum);
#ifdef __cplusplus
}
#endif

#endif /* __SOFTWARE_I2C_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
