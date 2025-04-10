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
 * @file i2c_eeprom.c
 * @version v1.0.4
 *
  */
#include "cmt453x.h"
#include "cmt453x_dma.h"
#include "i2c_eeprom.h"
#include "string.h"
#include "stdbool.h"

#include "main.h"

/** @addtogroup I2C_EEPROM
 * @{
 */

/* when EEPROM is writing data inside,it won't response the request from the master.check the ack,
if EEPROM response,make clear that EEPROM finished the last data-writing,allow the next operation */
static bool check_begin = FALSE;
static bool DeviceDone  = FALSE;
static bool addressDone = FALSE;
u32 sEETimeout = sEE_LONG_TIMEOUT;

static u8 MasterDirection = Transmitter;
static u16 SlaveADDR;
static u16 DeviceOffset = 0x0;

u16 I2C_NumByteToWrite   = 0;
u8 I2C_NumByteWritingNow = 0;
u8* I2C_pBuffer          = 0;
u16 I2C_WriteAddr        = 0;

static u8 SendBuf[32]         = {0};
static u8 *recvBuf            = NULL;
static u16 BufCount          = 0;
static volatile u16 Int_NumByteToWrite = 0;
static volatile u16 Int_NumByteToRead  = 0;
/* global state variable i2c_comm_state */
volatile I2C_STATE i2c_comm_state;

/**
 * @brief  Timeout callback used by the I2C EEPROM driver.
 */
u8 sEE_TIMEOUT_UserCallback(void)
{
    printf("error!!!\r\n");
    /* Block communication and all processes */
    while (1)
    {
    }
}

/**
 * @brief  Initializes peripherals used by the I2C EEPROM driver.
 */
void I2C_EE_Init(void)
{
    /** GPIO configuration and clock enable */
    GPIO_InitType GPIO_InitStructure;
    I2C_InitType I2C_InitStructure;
    
#if (PROCESS_MODE == 1) // interrupt
    NVIC_InitType NVIC_InitStructure;
#endif

    GPIO_InitStruct(&GPIO_InitStructure);
    
    /** enable peripheral clk*/
    I2Cx_peripheral_clk_en();
    I2C_DeInit(I2Cx);

    I2Cx_scl_clk_en();
    I2Cx_sda_clk_en();

    GPIO_InitStructure.Pin            = I2Cx_SCL_PIN | I2Cx_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed     = GPIO_SPEED_LOW;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.GPIO_Pull     = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = I2Cx_GPIO_AF;
    GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);

    /** I2C periphral configuration */
    I2C_DeInit(I2Cx);
    I2C_InitStructure.BusMode     = I2C_BUSMODE_I2C;
    I2C_InitStructure.FmDutyCycle = I2C_FMDUTYCYCLE_1;
    I2C_InitStructure.OwnAddr1    = 0xff;
    I2C_InitStructure.AckEnable   = I2C_ACKEN;
    I2C_InitStructure.AddrMode    = I2C_ADDR_MODE_7BIT;
    I2C_InitStructure.ClkSpeed    = I2C_Speed;
    I2C_Init(I2Cx, &I2C_InitStructure);
    
#if (PROCESS_MODE == 1) /* interrupt */
    /** I2C NVIC configuration */
    NVIC_InitStructure.NVIC_IRQChannel                   = I2Cx_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
#elif PROCESS_MODE == 2 /* DMA */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);
#endif
}

/**
 * @brief  Writes buffer of data to the I2C EEPROM.
 * @param pBuffer pointer to the buffer  containing the data to be
 *                  written to the EEPROM.
 * @param WriteAddr EEPROM's internal address to write to.
 * @param NumByteToWrite number of bytes to write to the EEPROM.
 */
void I2C_EE_WriteBuffer(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
    if (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
    {
        printf("I2C busy\r\n");
        I2C_ClrFlag(I2Cx, I2C_FLAG_BUSY);
        return;
    }
    I2C_pBuffer        = pBuffer;
    I2C_WriteAddr      = WriteAddr;
    I2C_NumByteToWrite = NumByteToWrite;

    while (I2C_NumByteToWrite > 0)
    {
        I2C_EE_WriteOnePage(I2C_pBuffer, I2C_WriteAddr, I2C_NumByteToWrite);
    }
}

/**
 * @brief  Writes a page of data to the I2C EEPROM, general called by
 *         I2C_EE_WriteBuffer.
 * @param pBuffer pointer to the buffer  containing the data to be
 *                  written to the EEPROM.
 * @param WriteAddr EEPROM's internal address to write to.
 * @param NumByteToWrite number of bytes to write to the EEPROM.
 */
void I2C_EE_WriteOnePage(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
    u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

    Addr        = WriteAddr % I2C_PageSize;
    count       = I2C_PageSize - Addr;
    NumOfPage   = NumByteToWrite / I2C_PageSize;
    NumOfSingle = NumByteToWrite % I2C_PageSize;

    I2C_NumByteWritingNow = 0;
    /** If WriteAddr is I2C_PageSize aligned */
    if (Addr == 0)
    {
        /** If NumByteToWrite < I2C_PageSize */
        if (NumOfPage == 0)
        {
            I2C_NumByteWritingNow = NumOfSingle;
            I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        }
        /** If NumByteToWrite > I2C_PageSize */
        else
        {
            I2C_NumByteWritingNow = I2C_PageSize;
            I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize);
        }
    }
    /** If WriteAddr is not I2C_PageSize aligned */
    else
    {
        /* If NumByteToWrite < I2C_PageSize */
        if (NumOfPage == 0)
        {
            I2C_NumByteWritingNow = NumOfSingle;
            I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        }
        /* If NumByteToWrite > I2C_PageSize */
        else
        {
            if (count != 0)
            {
                I2C_NumByteWritingNow = count;
                I2C_EE_PageWrite(pBuffer, WriteAddr, count);
            }
        }
    }
}

/**
 * @brief  Writes more than one byte to the EEPROM with a single WRITE
 *         cycle. The number of byte can't exceed the EEPROM page size.
 * @param pBuffer pointer to the buffer containing the data to be
 *                  written to the EEPROM.
 * @param WriteAddr EEPROM's internal address to write to (1-16).
 * @param NumByteToWrite number of bytes to write to the EEPROM.
 */
void I2C_EE_PageWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
#if (PROCESS_MODE == 0) /* polling */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send START condition */
    I2C_GenerateStart(I2Cx, ENABLE);
    /** Test on EV5 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** Send EEPROM address for write */
    I2C_SendAddr7bit(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_SEND);
    /** Test on EV6 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_TXMODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** Send the EEPROM's internal address to write to */
    I2C_SendData(I2Cx, WriteAddr>>8);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    I2C_SendData(I2Cx, WriteAddr);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** While there is data to be written */
    while (NumByteToWrite--)
    {
        /** Send the current byte */
        I2C_SendData(I2Cx, *pBuffer);
        /** Point to the next byte to be written */
        pBuffer++;
        /** Test on EV8 and clear it */
        sEETimeout = sEE_LONG_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
        {
            if ((sEETimeout--) == 0)
                sEE_TIMEOUT_UserCallback();
        }
    }
    /** Send STOP condition */
    I2C_GenerateStop(I2Cx, ENABLE);
    I2C_EE_WaitEepromStandbyState();
    I2C_EE_WriteOnePageCompleted();

#elif PROCESS_MODE == 1 /* interrupt */
    /** initialize static parameter */
    MasterDirection = Transmitter;
    i2c_comm_state  = COMM_PRE;
    /** initialize static parameter according to input parameter */
    SlaveADDR        = EEPROM_ADDRESS; /// this byte shoule be send by F/W (in loop or INTSTS way)
    DeviceOffset     = WriteAddr;      /// this byte can be send by both F/W and DMA
    check_begin        = FALSE;
    
    memcpy(SendBuf, pBuffer, NumByteToWrite);
    BufCount           = 0;
    Int_NumByteToWrite = NumByteToWrite;
    
    I2C_ConfigAck(I2Cx, ENABLE);
    I2C_ConfigInt(I2Cx, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);

    /** Send START condition */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    I2C_GenerateStart(I2Cx, ENABLE);
    I2C_EE_WaitOperationIsCompleted();
    I2C_EE_WriteOnePageCompleted();

#elif PROCESS_MODE == 2 /* DMA */
    DMA_InitType DMA_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    /** DMA initialization */
    DMA_DeInit(DMA_CH2);
    DMA_InitStructure.PeriphAddr                  = (u32)&I2Cx->DAT;            /// (u32)I2C1_DR_Address;
    DMA_InitStructure.MemAddr                     = (u32)pBuffer;              /// from function input parameter
    DMA_InitStructure.Direction                     = DMA_DIR_PERIPH_DST;     /// fixed for send function
    DMA_InitStructure.BufSize                     = NumByteToWrite;            /// from function input parameter
    DMA_InitStructure.PeriphInc                   = DMA_PERIPH_INC_DISABLE; // fixed
    DMA_InitStructure.DMA_MemoryInc               = DMA_MEM_INC_ENABLE;      /// fixed
    DMA_InitStructure.PeriphDataSize               = DMA_PERIPH_DATA_SIZE_BYTE;  /// fixed
    DMA_InitStructure.MemDataSize                 = DMA_MemoryDataSize_Byte;  /// fixed
    DMA_InitStructure.CircularMode                = DMA_MODE_NORMAL;           /// fixed
    DMA_InitStructure.Priority                    = DMA_PRIORITY_HIGH;     /// up to user
    DMA_InitStructure.Mem2Mem                    = DMA_M2M_DISABLE;      /// fixed
    DMA_Init(DMA_CH2, &DMA_InitStructure);
    DMA_RequestRemap(DMA_REMAP_I2C_TX, DMA, DMA_CH2, ENABLE);
    DMA_ConfigInt(DMA_CH2, DMA_INT_TXC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel          = DMA_Channel1_2_3_4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority    = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd       = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    i2c_comm_state = COMM_PRE;
    sEETimeout     = sEE_LONG_TIMEOUT;
    while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send START condition */
    I2C_GenerateStart(I2Cx, ENABLE);
    /** Test on EV5 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** Send EEPROM address for write */
    I2C_SendAddr7bit(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_SEND);
    /** Test on EV6 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_TXMODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** Send the EEPROM's internal address to write to */
    I2C_SendData(I2Cx, WriteAddr>>8);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    I2C_SendData(I2Cx, WriteAddr);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }

    I2C_EnableDMA(I2C1, ENABLE);
    DMA_EnableChannel(DMA_CH2, ENABLE);
    
    sEETimeout = sEE_LONG_TIMEOUT;
    while (i2c_comm_state != COMM_IN_PROCESS)
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    I2C_EE_WaitEepromStandbyState();
    I2C_EE_WriteOnePageCompleted();
#endif
}

/**
 * @brief  Process Write one page completed.
 */
void I2C_EE_WriteOnePageCompleted(void)
{
    I2C_pBuffer += I2C_NumByteWritingNow;
    I2C_WriteAddr += I2C_NumByteWritingNow;
    I2C_NumByteToWrite -= I2C_NumByteWritingNow;
}

/**
 * @brief  Reads a block of data from the EEPROM.
 * @param pBuffer pointer to the buffer that receives the data read
 *                  from the EEPROM.
 * @param ReadAddr EEPROM's internal address to read from.
 * @param NumByteToRead number of bytes to read from the EEPROM.
 */
void I2C_EE_ReadBuffer(u8* pBuffer, u16 ReadAddr, u16 NumByteToRead)
{
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
        sEE_TIMEOUT_UserCallback();
    }
    
    /** Send START condition */
    I2C_GenerateStart(I2Cx, ENABLE);
    /** Test on EV5 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** Send EEPROM address for write */
    I2C_SendAddr7bit(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_SEND);
    /** Test on EV6 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_TXMODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** Clear EV6 by setting again the PE bit */
    I2C_Enable(I2Cx, ENABLE);
    /** Send the EEPROM's internal address to write to */
    I2C_SendData(I2Cx, ReadAddr>>8);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    I2C_SendData(I2Cx, ReadAddr);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    
    /** Send STRAT condition a second time */
    I2C_GenerateStart(I2Cx, ENABLE);
    /** Test on EV5 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send EEPROM address for read */
    I2C_SendAddr7bit(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_RECV);
    /* Test on EV6 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_GetFlag(I2Cx, I2C_FLAG_ADDRF))    //EV6
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
        
    /* While there is data to be read */
    while (NumByteToRead)
    {
        /** One byte */
        if (NumByteToRead == 1)
        {
            /** Disable Acknowledgement */
            I2C_ConfigAck(I2Cx, DISABLE);
            /** Send STOP Condition */
            I2C_GenerateStop(I2Cx, ENABLE);
        }
        /** Test on EV7 and clear it */
        sEETimeout = sEE_LONG_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_DATA_RECVD_FLAG))
        {
            if ((sEETimeout--) == 0)
                    sEE_TIMEOUT_UserCallback();
        }
        /** Read data from DAT */
        *pBuffer = I2C_RecvData(I2Cx);
        /** Point to the next location where the byte read will be saved */
        pBuffer++;
        /** Decrement the read bytes counter */
        NumByteToRead--;
    }
    /* Enable Acknowledgement to be ready for another reception */
    I2C_ConfigAck(I2Cx, ENABLE);
}

/**
 * @brief  wait operation is completed.
 */
void I2C_EE_WaitOperationIsCompleted(void)
{
    sEETimeout = sEE_LONG_TIMEOUT*1000;
    while (i2c_comm_state != COMM_DONE)
    {
        if ((sEETimeout--) == 0)
        {
            printf("wait error\r\n");
            sEE_TIMEOUT_UserCallback();
        }
    }
}

/**
 * @brief  I2c1 event interrupt Service Routines.
 */
void I2C1_IRQHandler(void)
{
    uint32_t lastevent = I2C_GetLastEvent(I2Cx);
    
    //printf("0x%08x, %d, %d\r\n", lastevent, Int_NumByteToWrite, Int_NumByteToRead);
    switch (lastevent)
    {
        /** Master Invoke */
        case I2C_EVT_MASTER_MODE_FLAG: /// 0x00030001 EV5
            if (!check_begin)
            {
                i2c_comm_state = COMM_IN_PROCESS;
            }
            if (MasterDirection == Receiver)
            {
                if (DeviceDone == FALSE)
                {
                    DeviceDone = TRUE;
                    I2C_SendAddr7bit(I2Cx, SlaveADDR, I2C_DIRECTION_SEND);
                }
                else
                {
                    /** Send slave Address for read */
                    I2C_SendAddr7bit(I2Cx, SlaveADDR, I2C_DIRECTION_RECV);
                }
            }
            else
            {
                /** Send slave Address for write */
                I2C_SendAddr7bit(I2Cx, SlaveADDR, I2C_DIRECTION_SEND);        
            }
            break;
            
        /** Master Transmitter events */
        case I2C_EVT_MASTER_TXMODE_FLAG:     // 0x00070082. EV6 Send first addr
            if(check_begin)
            {
                check_begin = FALSE;
                I2C_ConfigInt(I2C1, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);
                I2C_GenerateStop(I2C1, ENABLE);
                i2c_comm_state = COMM_DONE;
                break;
            }
            if (MasterDirection == Receiver)
            {
                I2C_SendData(I2Cx, DeviceOffset);  
                addressDone = TRUE;
            }
            else
            {
                I2C_SendData(I2Cx, DeviceOffset); 
                addressDone = TRUE;
            }
            break;
            
        case I2C_EVT_MASTER_DATA_SENDING:     // 0x00070080. EV8 Sending data 
            if(addressDone == TRUE)
            {
                I2C_SendData(I2Cx, DeviceOffset);
                addressDone = FALSE;
            }
            break;
            
        case I2C_EVT_MASTER_DATA_SENDED:     // 0x00070084.EV8_2 Send finish
            if (MasterDirection == Receiver)
            {
                I2C_GenerateStart(I2Cx, ENABLE);
            }
            else
            {
                if (Int_NumByteToWrite == 0)
                {
                    I2C_GenerateStop(I2Cx, ENABLE);
                    sEETimeout = sEE_LONG_TIMEOUT;
                    while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
                    {
                        if ((sEETimeout--) == 0)
                            sEE_TIMEOUT_UserCallback();
                    }
                    check_begin = TRUE;
                    I2C_GenerateStart(I2C1, ENABLE);
                }
                else
                {
                    I2C_SendData(I2Cx, SendBuf[BufCount++]);
                    Int_NumByteToWrite--;
                }
            }
            break;
            
        case 0x00030400:    
            if (check_begin) /// EEPROM write busy
            {
                I2C_GenerateStart(I2C1, ENABLE);
            }
            I2C_ClrFlag(I2Cx, I2C_FLAG_ACKFAIL);
            break;
            
        /** Master Receiver events */
        case I2C_EVT_MASTER_RXMODE_FLAG: /// EV6 0x00030002
            if(Int_NumByteToRead == 1)
            {
                I2C_ConfigAck(I2C1, DISABLE);
                I2C_GenerateStop(I2Cx, ENABLE);
            }
            else if(Int_NumByteToRead == 2)
            {
                I2C1->CTRL1 |= I2C_NACK_POS_NEXT; /// set ACKPOS
                I2C_ConfigAck(I2Cx, DISABLE);
            }
            break;
            
        case I2C_EVT_MASTER_DATA_RECVD_FLAG: // 0x00030040. EV7.one byte recved 
            break;
        
        case I2C_EVT_MASTER_DATA_RECVD_BSF_FLAG: // 0x00030044. EV7.When the I2C communication rate is too high, BSF = 1
            *recvBuf = I2C_RecvData(I2Cx);
            recvBuf++;
            Int_NumByteToRead--;
            if (Int_NumByteToRead == 1)
            {
                /** Disable Acknowledgement */
                I2C_ConfigAck(I2Cx, DISABLE);
            }
            if (Int_NumByteToRead == 0)
            {
                I2C_GenerateStop(I2Cx, ENABLE);
                I2C_ConfigInt(I2Cx, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);
                i2c_comm_state = COMM_DONE;
            }
            break;
            
        case 0x00000044:
            I2C_ConfigInt(I2Cx, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);
            i2c_comm_state = COMM_DONE;
            break;    
        
        default:
            break;
    }
}

/**
 * @brief  I2c1 dma send interrupt Service Routines.
 */
void DMA_Channel1_2_3_4_IRQHandler()
{
    if (DMA_GetFlagStatus(DMA_FLAG_TC2, DMA))
    {
        if (I2Cx->STS2 & 0x01) /// master send DMA finish, check process later
        {
            /** DMA1-6 (I2Cx Tx DMA)transfer complete INTSTS */
            I2C_EnableDMA(I2Cx, DISABLE);
            DMA_EnableChannel(DMA_CH2, DISABLE);
            /** wait until BTF */
            while (!I2C_GetFlag(I2Cx, I2C_FLAG_BYTEF))
                ;
            I2C_GenerateStop(I2Cx, ENABLE);
            /** wait until BUSY clear */
            while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
                ;
            i2c_comm_state = COMM_IN_PROCESS;
        }
        DMA_ClearFlag(DMA_FLAG_TC2, DMA);
    }
    else if (DMA_GetFlagStatus(DMA_FLAG_TC3, DMA))
    {
        if (I2Cx->STS2 & 0x01) /// master send DMA finish, check process later
        {
            /** DMA1-6 (I2Cx Tx DMA)transfer complete INTSTS */
            I2C_EnableDMA(I2Cx, DISABLE);
            DMA_EnableChannel(DMA_CH2, DISABLE);
            /** wait until BTF */
            while (!I2C_GetFlag(I2Cx, I2C_FLAG_BYTEF))
                ;
            I2C_GenerateStop(I2Cx, ENABLE);
            /** wait until BUSY clear */
            while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
                ;
            i2c_comm_state = COMM_IN_PROCESS;
        }
        DMA_ClearFlag(DMA_FLAG_TC3, DMA);
    }
}

/**
 * @brief  Wait eeprom standby state.
 */
void I2C_EE_WaitEepromStandbyState(void)
{
    __IO uint16_t tmpSR1    = 0;
    __IO uint32_t sEETrials = 0;

    /** While the bus is busy */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_GetFlag(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }

    /** Keep looping till the slave acknowledge his address or maximum number
       of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER) */
    while (1)
    {
        /** Send START condition */
        I2C_GenerateStart(I2Cx, ENABLE);

        /** Test on EV5 and clear it */
        sEETimeout = sEE_LONG_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
        {
            if ((sEETimeout--) == 0)
                sEE_TIMEOUT_UserCallback();
        }

        /** Send EEPROM address for write */
        I2C_SendAddr7bit(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_SEND);
        /** Wait for ADDR flag to be set (Slave acknowledged his address) */
        sEETimeout = sEE_LONG_TIMEOUT;
        do
        {
            /** Get the current value of the STS1 register */
            tmpSR1 = I2Cx->STS1;

            /** Update the timeout value and exit if it reach 0 */
            if ((sEETimeout--) == 0)
                sEE_TIMEOUT_UserCallback();
        }
        /** Keep looping till the Address is acknowledged or the AF flag is
           set (address not acknowledged at time) */
        while ((tmpSR1 & (I2C_STS1_ADDRF | I2C_STS1_ACKFAIL)) == 0);

        /** Check if the ADDR flag has been set */
        if (tmpSR1 & I2C_STS1_ADDRF)
        {
            /** Clear ADDR Flag by reading STS1 then STS2 registers (STS1 have already
               been read) */
            (void)I2Cx->STS2;

            /** STOP condition */
            I2C_GenerateStop(I2Cx, ENABLE);

            /** Exit the function */
            return;
        }
        else
        {
            /** Clear AF flag */
            I2C_ClrFlag(I2Cx, I2C_FLAG_ACKFAIL);
        }

        /** Check if the maximum allowed numbe of trials has bee reached */
        if (sEETrials++ == sEE_MAX_TRIALS_NUMBER)
        {
            /** If the maximum number of trials has been reached, exit the function */
            sEE_TIMEOUT_UserCallback();
        }
    }
}

/**
 * @}
 */
