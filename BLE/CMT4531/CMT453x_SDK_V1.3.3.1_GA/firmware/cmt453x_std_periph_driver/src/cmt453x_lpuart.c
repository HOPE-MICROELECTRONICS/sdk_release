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
 * @file cmt453x_lpuart.c
 * @version v1.0.3
 *
  */
#include "cmt453x_lpuart.h"
#include "cmt453x_rcc.h"

/** @addtogroup CMT453X_StdPeriph_Driver
 * @{
 */

/** @addtogroup LPUART
 * @brief LPUART driver modules
 * @{
 */

/** @addtogroup LPUART_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @addtogroup LPUART_Private_Defines
 * @{
 */

#define STS_CLR_MASK        ((uint16_t)0x01BF) /*!< LPUART STS Mask */

#define INTEN_CLR_MASK      ((uint16_t)0x0000) /*!< LPUART INTEN Mask */
#define INT_MASK            ((uint16_t)0x007F) /*!< LPUART Interrupt Mask */

#define CTRL_CLR_MASK           ((uint16_t)0x70F4) /*!< LPUART CTRL Mask */
#define CTRL_SMPCNT_MASK        ((uint16_t)0x3FFF) /*!< LPUART Sampling Method Mask */
#define CTRL_WUSTP_MASK         ((uint16_t)0x4FFF) /*!< LPUART WakeUp Method Mask */
#define CTRL_WUSTP_SET          ((uint16_t)0x0080) /*!< LPUART stop mode Enable Mask */
#define CTRL_WUSTP_RESET        ((uint16_t)0x7F7F) /*!< LPUART stop mode Disable Mask */
#define CTRL_LOOPBACK_SET       ((uint16_t)0x0010) /*!< LPUART Loopback Test Enable Mask */
#define CTRL_LOOPBACK_RESET     ((uint16_t)0xFFEF) /*!< LPUART Loopback Test Disable Mask */
#define CTRL_FLUSH_SET          ((uint16_t)0x0004) /*!< LPUART Flush Receiver FIFO Enable Mask */
#define CTRL_FLUSH_RESET        ((uint16_t)0x7FFB) /*!< LPUART Flush Receiver FIFO Disable Mask */

/**
 * @}
 */

/** @addtogroup LPUART_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup LPUART_Private_Variables
 * @{
 */

/**
 * @}
 */

/** @addtogroup LPUART_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup LPUART_Private_Functions
 * @{
 */

/**
 * @brief  Deinitializes the LPUART peripheral registers to their default reset values. 
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 */
void LPUART_DeInit(LPUART_Module* LPUARTx)
{
    /* Check the parameters */
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx));

    if(LPUARTx == LPUART1)
    {
        RCC->LSCTRL |=RCC_LSCTRL_LPUARTRST;
        RCC->LSCTRL &=~RCC_LSCTRL_LPUARTRST;
    }
    else
    {
    }
}

/**
 * @brief  Initializes the LPUART peripheral according to the specified
 *         parameters in the LPUART_InitStruct.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_InitStruct pointer to a LPUART_InitType structure
 *         that contains the configuration information for the specified LPUART
 *         peripheral.
 */
void LPUART_Init(LPUART_Module* LPUARTx, LPUART_InitType* LPUART_InitStruct)
{
    uint32_t tmpregister = 0x00, clocksrc = 0x00, apbclock = 0x00;
    uint32_t integerdivider    = 0x00;
    uint32_t fractionaldivider = 0x00;
    uint32_t tmpdivider = 0x00, lastdivider = 0x00, i = 0x00;
    uint32_t lpuartxbase       = 0;
    RCC_ClocksType RCC_ClocksStatus;

    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx));  
    assert_param(IS_LPUART_BAUDRATE(LPUART_InitStruct->BaudRate));
    assert_param(IS_LPUART_PARITY(LPUART_InitStruct->Parity));
    assert_param(IS_LPUART_MODE(LPUART_InitStruct->Mode));
    assert_param(IS_LPUART_RTSTHRESHOLD(LPUART_InitStruct->RtsThreshold));
    assert_param(IS_LPUART_HARDWARE_FLOW_CONTROL(LPUART_InitStruct->HardwareFlowControl));  

    lpuartxbase = (uint32_t)LPUARTx;

    /*---------------------------- LPUART CTRL Configuration -----------------------*/
    tmpregister = LPUARTx->CTRL;
    /* Clear FC_RXEN, FC_TXEN, RTS_THSEL[1:0], PCDIS, TRS and PSEL bits */
    tmpregister &= CTRL_CLR_MASK;
    /* Configure the LPUART Parity, Mode, RtsThrehold and HardwareFlowControl ----------------------- */
    /* Set PCDIS and PSEL bits according to Parity value */
    /* Set the TRS bit according to Mode */
    /* Set RTS_THSEL[1:0] bits according to RtsThrehold */
    /* Set FC_RXEN and FC_TXEN bits according to HardwareFlowControl */
    tmpregister |= (uint32_t)LPUART_InitStruct->Parity | LPUART_InitStruct->Mode | LPUART_InitStruct->RtsThreshold | LPUART_InitStruct->HardwareFlowControl;
    /* Write to LPUART CTRL */
    LPUARTx->CTRL = (uint16_t)tmpregister;
    
    /*---------------------------- LPUART BRCFG1 & 2 Configuration -----------------------*/
    /* Configure the LPUART Baud Rate -------------------------------------------*/
    if(lpuartxbase == LPUART1_BASE)
    {
        clocksrc = RCC_GetLpuartClkSrc(RCC_LPUART1CLK);
    }   
    if (clocksrc == RCC_LSCTRL_LPUART_LSI)
    {
        #ifdef LSI_TRIM_32768HZ
        apbclock = 0x8000; // 32.768kHz, should the same with your LSI trim
        #elif defined LSI_TRIM_28800HZ
        apbclock = 28800;  // 28.8kHz, should the same with your LSI trim
        #elif defined LSI_TRIM_32000HZ
        apbclock = 0x7D00;  // 32kHz, should the same with your LSI trim
        #else
        apbclock = 0x8000; // LSI default is 32.768kHz
        #endif
    }
    else if (clocksrc == RCC_LSCTRL_LPUART_LSE)
    {
        apbclock = 0x8000; // 32.768kHz
    }
    else // APB1
    {
        RCC_GetClocksFreqValue(&RCC_ClocksStatus);
        apbclock = RCC_ClocksStatus.Pclk1Freq;
    }

    /* Determine the integer part */
    integerdivider = apbclock / (LPUART_InitStruct->BaudRate);  

    /* Configure sampling method */
    if(integerdivider <= 10)
    {
        LPUART_ConfigSamplingMethod(LPUARTx, LPUART_SMPCNT_1B);
    } 
    else
    {
        LPUART_ConfigSamplingMethod(LPUARTx, LPUART_SMPCNT_3B);
    } 

    /* Check baudrate */ 
    assert_param(IS_LPUART_BAUDRATE(integerdivider));
    /* Write to LPUART BRCFG1 */
    LPUARTx->BRCFG1 = (uint16_t)integerdivider; 

    /* Determine the fractional part */
    fractionaldivider = ((apbclock % (LPUART_InitStruct->BaudRate)) * 10000) / (LPUART_InitStruct->BaudRate);
    
    tmpregister = 0x00;
    tmpdivider = fractionaldivider;
    /* Implement the fractional part in the register */
    for( i = 0; i < 8; i++)
    {   
        lastdivider = tmpdivider;     
        tmpdivider = lastdivider + fractionaldivider;
        if((tmpdivider / 10000) ^ (lastdivider / 10000))
        {
            tmpregister |= (0x01 << i);
        }        
    } 
    /* Write to LPUART BRCFG2 */
    LPUARTx->BRCFG2 = (uint8_t)tmpregister; 
}

/**
 * @brief  Fills each LPUART_InitStruct member with its default value.
 * @param LPUART_InitStruct pointer to a LPUART_InitType structure
 *         which will be initialized.
 */
void LPUART_StructInit(LPUART_InitType* LPUART_InitStruct)
{
    /* LPUART_InitStruct members default value */
    LPUART_InitStruct->BaudRate            = 9600;
    LPUART_InitStruct->Parity              = LPUART_PE_NO;
    LPUART_InitStruct->Mode                = LPUART_MODE_RX | LPUART_MODE_TX;
    LPUART_InitStruct->RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStruct->HardwareFlowControl = LPUART_HFCTRL_NONE;
}

/**
 * @brief  Flushes Receiver FIFO.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 */
void LPUART_FlushRxFifo(LPUART_Module* LPUARTx)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 

    /* Clear LPUART Flush Receiver FIFO */
    LPUARTx->CTRL |= CTRL_FLUSH_SET;
    while(LPUART_GetFlagStatus(LPUARTx, LPUART_FLAG_FIFO_NE) != RESET)
    {        
    }
    LPUARTx->CTRL &= CTRL_FLUSH_RESET;
}

/**
 * @brief  Enables or disables the specified LPUART interrupts.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_INT specifies the LPUART interrupt sources to be enabled or disabled.
 *   This parameter can be one of the following values:
 *     @arg LPUART_INT_WUF Wake-Up Interrupt
 *     @arg LPUART_INT_FIFO_NE FIFO Non-Empty Interrupt
 *     @arg LPUART_INT_FIFO_HF FIFO Half Full Interrupt
 *     @arg LPUART_INT_FIFO_FU FIFO Full Interrupt Enable
 *     @arg LPUART_INT_FIFO_OV FIFO Overflow Interrupt
 *     @arg LPUART_INT_TXC TX Complete Interrupt
 *     @arg LPUART_INT_PE Parity Check Error Interrupt
 * @param Cmd new state of the specified LPUART interrupts.
 *   This parameter can be: ENABLE or DISABLE.
 */
void LPUART_ConfigInt(LPUART_Module* LPUARTx, uint16_t LPUART_INT, FunctionalState Cmd)
{    
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx));   
    assert_param(IS_LPUART_CFG_INT(LPUART_INT));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));    
    
    if (Cmd != DISABLE)
    {
       LPUARTx->INTEN |= (uint8_t)LPUART_INT;
    }
    else
    {
        LPUARTx->INTEN &= (uint8_t)(~LPUART_INT);
    }
}

/**
 * @brief  Enables or disables the LPUART's DMA interface.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_DMAReq specifies the DMA request.
 *   This parameter can be any combination of the following values:
 *     @arg LPUART_DMAREQ_TX LPUART DMA transmit request
 *     @arg LPUART_DMAREQ_RX LPUART DMA receive request
 * @param Cmd new state of the DMA Request sources.
 *   This parameter can be: ENABLE or DISABLE.
 */
void LPUART_EnableDMA(LPUART_Module* LPUARTx, uint16_t LPUART_DMAReq, FunctionalState Cmd)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx));    
    assert_param(IS_LPUART_DMAREQ(LPUART_DMAReq));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enable the DMA transfer by setting the DMA_RXEN and/or DMA_TXEN bits in the LPUART_CTRL register */
        LPUARTx->CTRL |= LPUART_DMAReq;
    }
    else
    {
        /* Disable the DMA transfer by clearing the DMA_RXEN and/or DMA_TXEN bits in the LPUART_CTRL register */
        LPUARTx->CTRL &= (uint16_t)(~LPUART_DMAReq);
    }
}

/**
 * @brief  Selects the LPUART WakeUp method.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_WakeUpMethod specifies the LPUART wakeup method.
 *   This parameter can be one of the following values:
 *     @arg LPUART_WUSTP_STARTBIT WakeUp by Start Bit Detection
 *     @arg LPUART_WUSTP_RXNE WakeUp by RXNE Detection
 *     @arg LPUART_WUSTP_BYTE WakeUp by A Configurable Received Byte
 *     @arg LPUART_WUSTP_FRAME WakeUp by A Programmed 4-Byte Frame
 */
void LPUART_ConfigWakeUpMethod(LPUART_Module* LPUARTx, uint16_t LPUART_WakeUpMethod)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_LPUART_WAKEUP(LPUART_WakeUpMethod));

    LPUARTx->CTRL &= CTRL_WUSTP_MASK;
    LPUARTx->CTRL |= LPUART_WakeUpMethod;
}

/**
 * @brief  Enables or disables LPUART Wakeup in STOP mode.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param Cmd new state of the LPUART Wakeup in STOP mode.
 *   This parameter can be: ENABLE or DISABLE.
 */
void LPUART_EnableWakeUpStop(LPUART_Module* LPUARTx, FunctionalState Cmd)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enable Wakeup in STOP mode by setting the WUSTP bit in the CTRL register */        
        LPUARTx->CTRL |= CTRL_WUSTP_SET;
    }
    else
    {
        /* Disable Wakeup in STOP mode by clearing the WUSTP bit in the CTRL register */
        LPUARTx->CTRL &= CTRL_WUSTP_RESET;
    }
}

/**
 * @brief  Selects the LPUART Sampling method.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_SamplingMethod specifies the LPAURT sampling method.
 *   This parameter can be one of the following values:
 *     @arg LPUART_SMPCNT_3B 3 Sample bit
 *     @arg LPUART_SMPCNT_1B 1 Sample bit
 */
void LPUART_ConfigSamplingMethod(LPUART_Module* LPUARTx, uint16_t LPUART_SamplingMethod)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_LPUART_SAMPLING(LPUART_SamplingMethod));

    LPUARTx->CTRL &= CTRL_SMPCNT_MASK;
    LPUARTx->CTRL |= LPUART_SamplingMethod;
}

/**
 * @brief  Enables or disables LPUART Loop Back Self-Test.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param Cmd new state of the LPUART Loop Back Self-Test.
 *   This parameter can be: ENABLE or DISABLE.
 */
void LPUART_EnableLoopBack(LPUART_Module* LPUARTx, FunctionalState Cmd)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enable LPUART Loop Back Self-Test by setting the LOOKBACK bit in the CTRL register */        
        LPUARTx->CTRL |= CTRL_LOOPBACK_SET;
    }
    else
    {
        /* Disable LPUART Loop Back Self-Test by clearing the LOOKBACK bit in the CTRL register */
        LPUARTx->CTRL &= CTRL_LOOPBACK_RESET;
    }
}

/**
 * @brief  Transmits single data through the LPUART peripheral.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param Data the data to transmit.
 */
void LPUART_SendData(LPUART_Module* LPUARTx, uint8_t Data)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx));    
    assert_param(IS_LPUART_DATA(Data));

    /* Transmit Data */
    LPUARTx->DAT = (Data & (uint8_t)0xFF);
}

/**
 * @brief  Returns the most recent received data by the LPUART peripheral.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @return The received data.
 */
uint8_t LPUART_ReceiveData(LPUART_Module* LPUARTx)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 

    /* Receive Data */
    return (uint8_t)(LPUARTx->DAT & (uint8_t)0xFF);
}

/**
 * @brief  SConfigures LPUART detected byte or frame match for wakeup CPU from STOPS mode.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_WakeUpData specifies the LPUART detected byte or frame match for wakeup CPU from STOP mode.
 */
void LPUART_ConfigWakeUpData(LPUART_Module* LPUARTx, uint32_t LPUART_WakeUpData)
{     
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 

    LPUARTx->WUDAT = LPUART_WakeUpData;
}

/**
 * @brief  Checks whether the specified LPUART flag is set or not.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_FLAG specifies the flag to check.
 *   This parameter can be one of the following values:
 *     @arg LPUART_FLAG_PEF Parity Check Error Flag.
 *     @arg LPUART_FLAG_TXC TX Complete Flag.
 *     @arg LPUART_FLAG_FIFO_OV FIFO Overflow Flag.
 *     @arg LPUART_FLAG_FIFO_FU FIFO Full Flag.
 *     @arg LPUART_FLAG_FIFO_HF FIFO Half Full Flag.
 *     @arg LPUART_FLAG_FIFO_NE FIFO Non-Empty Flag.
 *     @arg LPUART_FLAG_CTS CTS Change(Hardware Flow Control) Flag.
 *     @arg LPUART_FLAG_WUFWakeup from STOP mode Flag.
 *     @arg LPUART_FLAG_NF Noise Detection Flag.
 * @return The new state of LPUART_FLAG (SET or RESET).
 */
FlagStatus LPUART_GetFlagStatus(LPUART_Module* LPUARTx, uint16_t LPUART_FLAG)
{
    FlagStatus bitstatus = RESET;
    
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_LPUART_FLAG(LPUART_FLAG));

    if ((LPUARTx->STS & LPUART_FLAG) != (uint16_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}

/**
 * @brief  Clears the LPUART's pending flags.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_FLAG specifies the flag to clear.
 *   This parameter can be any combination of the following values:
 *     @arg LPUART_FLAG_PEF Parity Check Error Flag.
 *     @arg LPUART_FLAG_TXC TX Complete Flag.
 *     @arg LPUART_FLAG_FIFO_OV FIFO Overflow Flag.
 *     @arg LPUART_FLAG_FIFO_FU FIFO Full Flag.
 *     @arg LPUART_FLAG_FIFO_HF FIFO Half Full Flag.
 *     @arg LPUART_FLAG_FIFO_NE FIFO Non-Empty Flag.
 *     @arg LPUART_FLAG_CTS CTS Change(Hardware Flow Control) Flag.
 *     @arg LPUART_FLAG_WUFWakeup from STOP mode Flag.
 *     @arg LPUART_FLAG_NF Noise Detection Flag.
 */
void LPUART_ClrFlag(LPUART_Module* LPUARTx, uint16_t LPUART_FLAG)
{
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_LPUART_CLEAR_FLAG(LPUART_FLAG));

    LPUARTx->STS = (uint16_t)LPUART_FLAG;
}

/**
 * @brief  Checks whether the specified LPUART interrupt has occurred or not.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_INT specifies the LPUART interrupt source to check.
 *   This parameter can be one of the following values:
 *     @arg LPUART_INT_WUF Wake-Up Interrupt
 *     @arg LPUART_INT_FIFO_NE FIFO Non-Empty Interrupt
 *     @arg LPUART_INT_FIFO_HF FIFO Half Full Interrupt
 *     @arg LPUART_INT_FIFO_FU FIFO Full Interrupt Enable
 *     @arg LPUART_INT_FIFO_OV FIFO Overflow Interrupt
 *     @arg LPUART_INT_TXC TX Complete Interrupt
 *     @arg LPUART_INT_PE Parity Check Error Interrupt
 * @return The new state of LPUART_INT (SET or RESET).
 */
INTStatus LPUART_GetIntStatus(LPUART_Module* LPUARTx, uint16_t LPUART_INT)
{
    uint32_t bitpos = 0x00, itmask = 0x00;
    INTStatus bitstatus = RESET;
    
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_LPUART_GET_INT(LPUART_INT));
    
    /* Get the interrupt position */
    itmask = (uint8_t)(LPUART_INT >> 0x08) & INT_MASK;
    itmask = (uint32_t)0x01 << itmask;
    itmask &= LPUARTx->INTEN;

    bitpos = ((uint8_t)LPUART_INT) & 0xFF;
    bitpos &= LPUARTx->STS;
    if ((itmask != (uint16_t)RESET) && (bitpos != (uint16_t)RESET))
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/**
 * @brief  Clears the LPUART's interrupt pending bits.
 * @param LPUARTx Select the LPUART peripheral.
 *   This parameter can be one of the following values: LPUART1.
 * @param LPUART_INT specifies the interrupt pending bit to clear.
 *   This parameter can be one of the following values:
 *     @arg LPUART_INT_WUF Wake-Up Interrupt
 *     @arg LPUART_INT_FIFO_NE FIFO Non-Empty Interrupt
 *     @arg LPUART_INT_FIFO_HF FIFO Half Full Interrupt
 *     @arg LPUART_INT_FIFO_FU FIFO Full Interrupt Enable
 *     @arg LPUART_INT_FIFO_OV FIFO Overflow Interrupt
 *     @arg LPUART_INT_TXC TX Complete Interrupt
 *     @arg LPUART_INT_PE Parity Check Error Interrupt 
 */
void LPUART_ClrIntPendingBit(LPUART_Module* LPUARTx, uint16_t LPUART_INT)
{
    uint16_t itmask = 0x00;
    
    /* Check the parameters */  
    assert_param(IS_LPUART_ALL_PERIPH(LPUARTx)); 
    assert_param(IS_LPUART_CLR_INT(LPUART_INT));
    
    itmask      = ((uint8_t)LPUART_INT) & 0xFF;
    LPUARTx->STS = (uint16_t)itmask;
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
