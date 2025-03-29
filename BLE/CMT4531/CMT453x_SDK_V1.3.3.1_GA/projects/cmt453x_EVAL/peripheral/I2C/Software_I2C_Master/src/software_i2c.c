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
 * @file software_i2c.c
 * @version v1.0.2
 *
  */
#include "cmt453x.h"
#include "software_i2c.h"
#include "main.h"
#include <stdio.h>
/** @addtogroup CMT453X_StdPeriph_Examples
 * @{
 */

/** @addtogroup software_i2c
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SI2C_DBUG    1
#if SI2C_DBUG
#include "main.h"
#define SI2C_Debug(format,...)    printf(format,##__VA_ARGS__)
#else
#define SI2C_Debug(format,...)    
#endif    //SI2C_DBUG


#define GPIOx_MODE_SET(GPIOx,bit,mode) do{     \
GPIOx->POTYPE &= ~(1U << bit);                 \
GPIOx->POTYPE |= (((mode >> 4U) & 1U) << bit); \
GPIOx->PMODE&= ~(03U<<(bit * 2U));             \
GPIOx->PMODE|= (mode& 03U)<<(bit*2U);          \
}while(0);

//IO control 
#define SDA_OUT_MODE(GPIOx,bit)  GPIOx_MODE_SET(GPIOx,bit,GPIO_MODE_OUTPUT_OD)    //SDA output
#define SDA_IN_MODE(GPIOx,bit)   GPIOx_MODE_SET(GPIOx,bit,GPIO_MODE_INPUT)        //SDA input
#define SCL_OUT_MODE(GPIOx,bit)  GPIOx_MODE_SET(GPIOx,bit,GPIO_MODE_OUTPUT_PP)    //SCL output
#define SDA_OUT_H(GPIOx,bit)     (GPIOx->PBSC = 1<<bit)                           //SDA output high
#define SDA_OUT_L(GPIOx,bit)     (GPIOx->PBC  = 1<<bit)                           //SDA output low
#define SCL_OUT_H(GPIOx,bit)     (GPIOx->PBSC = 1<<bit)                           //SCL output high
#define SCL_OUT_L(GPIOx,bit)     (GPIOx->PBC  = 1<<bit)                           //SCL output low
#define SDA_IN(GPIOx,bit)        (((GPIOx->PID) & (1<<bit))?1:0)                  //SDA input
 
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void SI2C_DelayUS(uint32_t i)
{
    i=i*7;
    while(--i);
}

/**
 * @brief  software i2c init
 * @param  
 * @return  true: success, false: fail.
 * @note   
 */
bool SI2C_Init(SI2C_HANDLE *pHandle, GPIO_Module *SDA_GPIOx, GPIO_Module *SCL_GPIOx,\
                uint32_t SDA_Pin, uint32_t SCL_Pin,uint8_t DelayUS)
{
    GPIO_InitType GPIO_InitStructure;
    uint8_t bit = 0;
    if(pHandle == NULL) 
    {
        SI2C_Debug("pHandle should not be null\r\n");
        return false;
    }
    if(DelayUS < 1) DelayUS = 1;
    if(DelayUS > 100) DelayUS = 100;
    pHandle->DelayUS = DelayUS;
    pHandle->SDA_GPIOx = SDA_GPIOx;                //SDA port
    pHandle->SCL_GPIOx = SCL_GPIOx;                //SCL port

    for(bit = 0; (SDA_Pin >> bit)!=1 ;bit++ )
    {
        if(bit>13)
            break;
    }
    pHandle->SDA_PINx = bit;        //SDA pin
    for(bit = 0; (SCL_Pin >> bit)!=1 ;bit++ )
    {
        if(bit>13)
        break;
    }
    pHandle->SCL_PINx = bit;        //SCL pin
    //init GPIO
    if (SDA_GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (SDA_GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    if (SCL_GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (SCL_GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx);        //SDA=1
    SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);        //SCL=1
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitPeripheral(SDA_GPIOx, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = SCL_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitPeripheral(SCL_GPIOx, &GPIO_InitStructure);

    SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx);        //SDA=1
    SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);        //SCL=1
    //fucntion init
    pHandle->Start = (void (*)(void *))SI2C_Start; 
    pHandle->Stop  = (void (*)(void *))SI2C_Stop; 
    pHandle->SendByte  = (bool (*)(void *,uint8_t))SI2C_SendByte;
    pHandle->ReadByte  = (uint8_t (*)(void *, bool))SI2C_ReadByte;
 
    return true;
}
 
 
 
/**
 * @brief  software i2c send start singal
 * @param  
 * @return
 * @note   
 */
void SI2C_Start(SI2C_HANDLE *pHandle)
{
    SDA_OUT_MODE(pHandle->SDA_GPIOx, pHandle->SDA_PINx); 
    SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx); //SDA=1 
    SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=1
    SI2C_DelayUS(pHandle->DelayUS);
    SDA_OUT_L(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=0 
    SI2C_DelayUS(pHandle->DelayUS);
    SCL_OUT_L(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=0
}    
 
 
/**
 * @brief  software i2c send stop singal
 * @param  
 * @return
 * @note   
 */
void SI2C_Stop(SI2C_HANDLE *pHandle)
{
    SCL_OUT_L(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=0
    SI2C_DelayUS(1);
    SDA_OUT_L(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=0
    SI2C_DelayUS(pHandle->DelayUS);
    SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=1
    SI2C_DelayUS(pHandle->DelayUS);
    SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=1    
    SI2C_DelayUS(1);
}
 
 
/**
 * @brief  software i2c wait ack signal
 * @param  
 * @return
 * @note   
 */
bool SI2C_WaitAck(SI2C_HANDLE *pHandle)
{
    uint8_t ucErrTime=0;
    
    SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=1    
    SDA_IN_MODE(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA input
    SI2C_DelayUS(pHandle->DelayUS);
    SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=1
    SI2C_DelayUS(pHandle->DelayUS);
    
    while(SDA_IN(pHandle->SDA_GPIOx, pHandle->SDA_PINx)) //wait low
    {
        ucErrTime++;
        if(ucErrTime>50)
        {
            SI2C_Stop(pHandle);
            return false;
        }
        SI2C_DelayUS(1);
    }    
    SCL_OUT_L(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=0      
    SI2C_DelayUS(1);
    SDA_OUT_MODE(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA output
    
    return true;  
} 
 
 
/**
 * @brief  software i2c send ack singal
 * @param  
 * @return
 * @note   
 */
void SI2C_Ack(SI2C_HANDLE *pHandle)
{
    SDA_OUT_L(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=0
    SI2C_DelayUS(pHandle->DelayUS);   
    SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=1
    SI2C_DelayUS(pHandle->DelayUS); 
    SCL_OUT_L(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=0
}
 
/**
 * @brief  software i2c send nack singal
 * @param  
 * @return
 * @note   
 */  
void SI2C_NAck(SI2C_HANDLE *pHandle)
{    
    SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=1
    SI2C_DelayUS(pHandle->DelayUS);
    SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=1
    SI2C_DelayUS(pHandle->DelayUS);
    SCL_OUT_L(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=0      
}    
 
 
 
 
/**
 * @brief  software i2c send a byte
 * @param  
 * @return
 * @note   
 */       
bool SI2C_SendByte(SI2C_HANDLE *pHandle, uint8_t data)
{                        
    uint8_t t; 
    
    for(t=0;t<8;t++)
    {         
        if(data & 0X80)
        {
            SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=1
        }
        else
        {
            SDA_OUT_L(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=0
        }
        data <<= 1; 
        SI2C_DelayUS(pHandle->DelayUS);
        SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=1
        SI2C_DelayUS(pHandle->DelayUS);
        SCL_OUT_L(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=0    
    }    
    return SI2C_WaitAck(pHandle);
}     
 
 
 
/**
 * @brief  software i2c read a byte
 * @param  
 * @return
 * @note   
 */
uint8_t SI2C_ReadByte(SI2C_HANDLE *pHandle,bool isAck)
{
    uint8_t i,receive=0;
    SDA_OUT_H(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA=1 pull up
    SDA_IN_MODE(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA input
    for(i=0;i<8;i++ )
    {
        receive<<=1;
        SI2C_DelayUS(pHandle->DelayUS); 
        SCL_OUT_H(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=1
        SI2C_DelayUS(pHandle->DelayUS);
        if(SDA_IN(pHandle->SDA_GPIOx, pHandle->SDA_PINx))
        {
            receive++;
        }
        __nop();
        SCL_OUT_L(pHandle->SCL_GPIOx, pHandle->SCL_PINx);//SCL=0    
    }
    __nop();__nop();__nop();
    
    SDA_OUT_MODE(pHandle->SDA_GPIOx, pHandle->SDA_PINx);//SDA output 
    if (!isAck)
        SI2C_NAck(pHandle);//send nack
    else
        SI2C_Ack(pHandle); //send ack  
    
    return receive;
}
 
 
/**
 * @brief  software i2c read a reg
 * @param  
 * @return
 * @note   
 */
bool SI2C_ReadReg(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint16_t RegAddr,\
                  bool is8bitRegAddr, uint8_t *pDataBuff, uint16_t ReadByteNum)
{
    uint16_t i;
    
    SI2C_Start(pHandle);
    if(SI2C_SendByte(pHandle, SlaveAddr) == false)
    {
        SI2C_Debug("[SI2C ERROR]:address MBS write fail\r\n");
        return false;
    }
    if(is8bitRegAddr == false)
    {
        if(SI2C_SendByte(pHandle, RegAddr>>8) == false)
        {
            SI2C_Debug("[SI2C ERROR]:reg write fail\r\n");
            return false;
        }
    }
    if(SI2C_SendByte(pHandle, RegAddr) == false)
    {
        SI2C_Debug("[SI2C ERROR]:address LSB write fail\r\n");
        return false;
    }

    SI2C_Start(pHandle);
    if(SI2C_SendByte(pHandle, SlaveAddr|1) == false)
    {
        SI2C_Debug("[SI2C ERROR]:address read fail\r\n");
        return false;
    }
    for(i = 0;i < ReadByteNum;i ++)
    {
        if(i == (ReadByteNum-1))
        {
            pDataBuff[i] = SI2C_ReadByte(pHandle, false);
        }
        else
        {
            pDataBuff[i] = SI2C_ReadByte(pHandle, true);
        }    
    }
    SI2C_Stop(pHandle);
    
    return true;
}
 
 
 
/**
 * @brief  software i2c write a reg
 * @param  
 * @return
 * @note   
 */
bool SI2C_WriteReg(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint16_t RegAddr,\
                   bool is8bitRegAddr, uint8_t *pDataBuff, uint16_t WriteByteNum)
{
    uint16_t i;
    
    SI2C_Start(pHandle);
    if(SI2C_SendByte(pHandle, SlaveAddr) == false)
    {
        SI2C_Debug("[SI2C ERROR]:address write fail\r\n");
        return false;
    }
    if(is8bitRegAddr == false)
    {
        if(SI2C_SendByte(pHandle, RegAddr>>8) == false)
        {
            SI2C_Debug("[SI2C ERROR]:reg MSB write fail\r\n");
            return false;
        }
    }
    if(SI2C_SendByte(pHandle, RegAddr) == false)
    {
        SI2C_Debug("[SI2C ERROR]:reg LSB write fail\r\n");
        return false;
    }
    for(i = 0;i < WriteByteNum;i ++)
    {
        if(SI2C_SendByte(pHandle, pDataBuff[i]) == false) 
        {
            SI2C_Debug("[SI2C ERROR]:data write fail\r\n");
            return false;
        }
    }
    SI2C_Stop(pHandle);
    
    return true;
}


/**
 * @brief  software i2c master read
 * @param  
 * @return
 * @note   
 */
bool SI2C_MasterRead(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint8_t *pDataBuff, uint16_t ReadByteNum)
{
    uint16_t i;

    SI2C_Start(pHandle);
    if(SI2C_SendByte(pHandle, SlaveAddr|0x01) == false)
    {
        SI2C_Debug("[SI2C ERROR]:address read fail\r\n");
        return false;
    }
    for(i = 0;i < ReadByteNum;i ++)
    {
        if(i == (ReadByteNum-1))
        {
            pDataBuff[i] = SI2C_ReadByte(pHandle, false);
        }
        else
        {
            pDataBuff[i] = SI2C_ReadByte(pHandle, true);
        }    
    }
    SI2C_Stop(pHandle);
    
    return true;
}
/**
 * @brief  software i2c master write
 * @param  
 * @return
 * @note   
 */
bool SI2C_MasterWrite(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint8_t *pDataBuff, uint16_t WriteByteNum)
{
    uint16_t i;
    
    SI2C_Start(pHandle);
    if(SI2C_SendByte(pHandle, SlaveAddr&0xFE) == false)
    {
        SI2C_Debug("[SI2C ERROR]:address write fail\r\n");
        return false;
    }

    for(i = 0;i < WriteByteNum;i ++)
    {
        if(SI2C_SendByte(pHandle, pDataBuff[i]) == false) 
        {
            SI2C_Debug("[SI2C ERROR]:data write fail\r\n");
            return false;
        }
    }
    SI2C_Stop(pHandle);
    
    return true;
}


/**
 * @}
 */
