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
 * @file cmt453x_dma.c
 * @version v1.0.1
 *
  */
#include "cmt453x_dma.h"
#include "cmt453x_rcc.h"

/** @addtogroup CMT453X_StdPeriph_Driver
 * @{
 */

/** @addtogroup DMA
 * @brief DMA driver modules
 * @{
 */

/** @addtogroup DMA_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @addtogroup DMA_Private_Defines
 * @{
 */

/* DMA Channelx interrupt pending bit masks */
#define DMA_CH1_INT_MASK ((uint32_t)(DMA_INTSTS_GLBF1 | DMA_INTSTS_TXCF1 | DMA_INTSTS_HTXF1 | DMA_INTSTS_ERRF1))
#define DMA_CH2_INT_MASK ((uint32_t)(DMA_INTSTS_GLBF2 | DMA_INTSTS_TXCF2 | DMA_INTSTS_HTXF2 | DMA_INTSTS_ERRF2))
#define DMA_CH3_INT_MASK ((uint32_t)(DMA_INTSTS_GLBF3 | DMA_INTSTS_TXCF3 | DMA_INTSTS_HTXF3 | DMA_INTSTS_ERRF3))
#define DMA_CH4_INT_MASK ((uint32_t)(DMA_INTSTS_GLBF4 | DMA_INTSTS_TXCF4 | DMA_INTSTS_HTXF4 | DMA_INTSTS_ERRF4))
#define DMA_CH5_INT_MASK ((uint32_t)(DMA_INTSTS_GLBF5 | DMA_INTSTS_TXCF5 | DMA_INTSTS_HTXF5 | DMA_INTSTS_ERRF5))


/* DMA CHCFGx registers Masks, MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
#define CCR_CLEAR_Mask ((uint32_t)0xFFFF800F)

/**
 * @}
 */

/** @addtogroup DMA_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup DMA_Private_Variables
 * @{
 */

/**
 * @}
 */

/** @addtogroup DMA_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup DMA_Private_Functions
 * @{
 */

/**
 * @brief  Deinitializes the DMAy Channelx registers to their default reset
 *         values.
 * @param DMAyChx where y can be 1 to select the DMA and
 *   x can be 1 to 5 for DMA  Channel.
 */
void DMA_DeInit(DMA_ChannelType* DMAChx)
{
    /* Check the parameters */
    assert_param(IS_DMA_ALL_PERIPH(DMAChx));

    /* Disable the selected DMAy Channelx */
    DMAChx->CHCFG &= (uint16_t)(~DMA_CHCFG1_CHEN);

    /* Reset DMAy Channelx control register */
    DMAChx->CHCFG = 0;

    /* Reset DMAy Channelx remaining bytes register */
    DMAChx->TXNUM = 0;

    /* Reset DMAy Channelx peripheral address register */
    DMAChx->PADDR = 0;

    /* Reset DMAy Channelx memory address register */
    DMAChx->MADDR = 0;

    if (DMAChx == DMA_CH1)
    {
        /* Reset interrupt pending bits for DMA1 Channel1 */
        DMA->INTCLR |= DMA_CH1_INT_MASK;
    }
    else if (DMAChx == DMA_CH2)
    {
        /* Reset interrupt pending bits for DMA1 Channel2 */
        DMA->INTCLR |= DMA_CH2_INT_MASK;
    }
    else if (DMAChx == DMA_CH3)
    {
        /* Reset interrupt pending bits for DMA1 Channel3 */
        DMA->INTCLR |= DMA_CH3_INT_MASK;
    }
    else if (DMAChx == DMA_CH4)
    {
        /* Reset interrupt pending bits for DMA1 Channel4 */
        DMA->INTCLR |= DMA_CH4_INT_MASK;
    }
    else if (DMAChx == DMA_CH5)
    {
        /* Reset interrupt pending bits for DMA1 Channel5 */
        DMA->INTCLR |= DMA_CH5_INT_MASK;
    }
    else
    {
    }
}

/**
 * @brief  Initializes the DMAy Channelx according to the specified
 *         parameters in the DMA_InitParam.
 * @param DMAChx where x can be 1 to 5 for DMA to select the DMA Channel.
 * @param DMA_InitParam pointer to a DMA_InitType structure that
 *         contains the configuration information for the specified DMA Channel.
 */
void DMA_Init(DMA_ChannelType* DMAChx, DMA_InitType* DMA_InitParam)
{
    uint32_t tmpregister = 0;

    /* Check the parameters */
    assert_param(IS_DMA_ALL_PERIPH(DMAChx));
    assert_param(IS_DMA_DIR(DMA_InitParam->Direction));
    assert_param(IS_DMA_BUF_SIZE(DMA_InitParam->BufSize));
    assert_param(IS_DMA_PERIPH_INC_STATE(DMA_InitParam->PeriphInc));
    assert_param(IS_DMA_MEM_INC_STATE(DMA_InitParam->DMA_MemoryInc));
    assert_param(IS_DMA_PERIPH_DATA_SIZE(DMA_InitParam->PeriphDataSize));
    assert_param(IS_DMA_MEMORY_DATA_SIZE(DMA_InitParam->MemDataSize));
    assert_param(IS_DMA_MODE(DMA_InitParam->CircularMode));
    assert_param(IS_DMA_PRIORITY(DMA_InitParam->Priority));
    assert_param(IS_DMA_M2M_STATE(DMA_InitParam->Mem2Mem));

    /*--------------------------- DMAy Channelx CHCFG Configuration -----------------*/
    /* Get the DMAyChx CHCFG value */
    tmpregister = DMAChx->CHCFG;
    /* Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
    tmpregister &= CCR_CLEAR_Mask;
    /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
    /* Set DIR bit according to Direction value */
    /* Set CIRC bit according to CircularMode value */
    /* Set PINC bit according to PeriphInc value */
    /* Set MINC bit according to DMA_MemoryInc value */
    /* Set PSIZE bits according to PeriphDataSize value */
    /* Set MSIZE bits according to MemDataSize value */
    /* Set PL bits according to Priority value */
    /* Set the MEM2MEM bit according to Mem2Mem value */
    tmpregister |= DMA_InitParam->Direction | DMA_InitParam->CircularMode | DMA_InitParam->PeriphInc
                   | DMA_InitParam->DMA_MemoryInc | DMA_InitParam->PeriphDataSize | DMA_InitParam->MemDataSize
                   | DMA_InitParam->Priority | DMA_InitParam->Mem2Mem;

    /* Write to DMAy Channelx CHCFG */
    DMAChx->CHCFG = tmpregister;

    /*--------------------------- DMAy Channelx TXNUM Configuration ---------------*/
    /* Write to DMAy Channelx TXNUM */
    DMAChx->TXNUM = DMA_InitParam->BufSize;

    /*--------------------------- DMAy Channelx PADDR Configuration ----------------*/
    /* Write to DMAy Channelx PADDR */
    DMAChx->PADDR = DMA_InitParam->PeriphAddr;

    /*--------------------------- DMAy Channelx MADDR Configuration ----------------*/
    /* Write to DMAy Channelx MADDR */
    DMAChx->MADDR = DMA_InitParam->MemAddr;
}

/**
 * @brief  Fills each DMA_InitParam member with its default value.
 * @param DMA_InitParam pointer to a DMA_InitType structure which will
 *         be initialized.
 */
void DMA_StructInit(DMA_InitType* DMA_InitParam)
{
    /*-------------- Reset DMA init structure parameters values ------------------*/
    /* Initialize the PeriphAddr member */
    DMA_InitParam->PeriphAddr = 0;
    /* Initialize the MemAddr member */
    DMA_InitParam->MemAddr = 0;
    /* Initialize the Direction member */
    DMA_InitParam->Direction = DMA_DIR_PERIPH_SRC;
    /* Initialize the BufSize member */
    DMA_InitParam->BufSize = 0;
    /* Initialize the PeriphInc member */
    DMA_InitParam->PeriphInc = DMA_PERIPH_INC_DISABLE;
    /* Initialize the DMA_MemoryInc member */
    DMA_InitParam->DMA_MemoryInc = DMA_MEM_INC_DISABLE;
    /* Initialize the PeriphDataSize member */
    DMA_InitParam->PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    /* Initialize the MemDataSize member */
    DMA_InitParam->MemDataSize = DMA_MemoryDataSize_Byte;
    /* Initialize the CircularMode member */
    DMA_InitParam->CircularMode = DMA_MODE_NORMAL;
    /* Initialize the Priority member */
    DMA_InitParam->Priority = DMA_PRIORITY_LOW;
    /* Initialize the Mem2Mem member */
    DMA_InitParam->Mem2Mem = DMA_M2M_DISABLE;
}

/**
 * @brief  Enables or disables the specified DMAy Channelx.
 * @param DMAChx where x can be 1 to 5 for DMA to select the DMA Channel.
 * @param Cmd new state of the DMA Channelx.
 *   This parameter can be: ENABLE or DISABLE.
 */
void DMA_EnableChannel(DMA_ChannelType* DMAChx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_DMA_ALL_PERIPH(DMAChx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enable the selected DMAy Channelx */
        DMAChx->CHCFG |= DMA_CHCFG1_CHEN;
    }
    else
    {
        /* Disable the selected DMAy Channelx */
        DMAChx->CHCFG &= (uint16_t)(~DMA_CHCFG1_CHEN);
    }
}

/**
 * @brief  Enables or disables the specified DMAy Channelx interrupts.
 * @param DMAChx where x can be 1 to 5 for DMA to select the DMA Channel.
 * @param DMAInt specifies the DMA interrupts sources to be enabled
 *   or disabled.
 *   This parameter can be any combination of the following values:
 *     @arg DMA_INT_TXC Transfer complete interrupt mask
 *     @arg DMA_INT_HTX Half transfer interrupt mask
 *     @arg DMA_INT_ERR Transfer error interrupt mask
 * @param Cmd new state of the specified DMA interrupts.
 *   This parameter can be: ENABLE or DISABLE.
 */
void DMA_ConfigInt(DMA_ChannelType* DMAChx, uint32_t DMAInt, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_DMA_ALL_PERIPH(DMAChx));
    assert_param(IS_DMA_CONFIG_INT(DMAInt));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        /* Enable the selected DMA interrupts */
        DMAChx->CHCFG |= DMAInt;
    }
    else
    {
        /* Disable the selected DMA interrupts */
        DMAChx->CHCFG &= ~DMAInt;
    }
}

/**
 * @brief  Sets the number of data units in the current DMAy Channelx transfer.
 * @param DMAChx where x can be 1 to 5 for DMA to select the DMA Channel.
 * @param DataNumber The number of data units in the current DMAy Channelx
 *         transfer.
 * @note   This function can only be used when the DMAyChx is disabled.
 */
void DMA_SetCurrDataCounter(DMA_ChannelType* DMAChx, uint16_t DataNumber)
{
    /* Check the parameters */
    assert_param(IS_DMA_ALL_PERIPH(DMAChx));

    /*--------------------------- DMAy Channelx TXNUM Configuration ---------------*/
    /* Write to DMA Channelx TXNUM */
    DMAChx->TXNUM = DataNumber;
}

/**
 * @brief  Returns the number of remaining data units in the current
 *         DMA Channelx transfer.
 * @param DMAChx where x can be 1 to 5 for DMA to select the DMA Channel.
 * @return The number of remaining data units in the current DMA Channelx
 *         transfer.
 */
uint16_t DMA_GetCurrDataCounter(DMA_ChannelType* DMAChx)
{
    /* Check the parameters */
    assert_param(IS_DMA_ALL_PERIPH(DMAChx));
    /* Return the number of remaining data units for DMAy Channelx */
    return ((uint16_t)(DMAChx->TXNUM));
}

/**
 * @brief  Checks whether the specified DMA Channelx flag is set or not.
 * @param DMAFlag specifies the flag to check.
 *   This parameter can be one of the following values:
 *     @arg DMA_FLAG_GL1 DMA Channel1 global flag.
 *     @arg DMA_FLAG_TC1 DMA Channel1 transfer complete flag.
 *     @arg DMA_FLAG_HT1 DMA Channel1 half transfer flag.
 *     @arg DMA_FLAG_TE1 DMA Channel1 transfer error flag.
 *     @arg DMA_FLAG_GL2 DMA Channel2 global flag.
 *     @arg DMA_FLAG_TC2 DMA Channel2 transfer complete flag.
 *     @arg DMA_FLAG_HT2 DMA Channel2 half transfer flag.
 *     @arg DMA_FLAG_TE2 DMA Channel2 transfer error flag.
 *     @arg DMA_FLAG_GL3 DMA Channel3 global flag.
 *     @arg DMA_FLAG_TC3 DMA Channel3 transfer complete flag.
 *     @arg DMA_FLAG_HT3 DMA Channel3 half transfer flag.
 *     @arg DMA_FLAG_TE3 DMA Channel3 transfer error flag.
 *     @arg DMA_FLAG_GL4 DMA Channel4 global flag.
 *     @arg DMA_FLAG_TC4 DMA Channel4 transfer complete flag.
 *     @arg DMA_FLAG_HT4 DMA Channel4 half transfer flag.
 *     @arg DMA_FLAG_TE4 DMA Channel4 transfer error flag.
 *     @arg DMA_FLAG_GL5 DMA Channel5 global flag.
 *     @arg DMA_FLAG_TC5 DMA Channel5 transfer complete flag.
 *     @arg DMA_FLAG_HT5 DMA Channel5 half transfer flag.
 *     @arg DMA_FLAG_TE5 DMA Channel5 transfer error flag.
 * @param DMAy DMA
 *   This parameter can be one of the following values:
 *     @arg DMA .
 * @return The new state of DMAFlag (SET or RESET).
 */
FlagStatus DMA_GetFlagStatus(uint32_t DMAFlag, DMA_Module* DMAy)
{
    FlagStatus bitstatus = RESET;
    uint32_t tmpregister = 0;

    /* Check the parameters */
    assert_param(IS_DMA_GET_FLAG(DMAFlag));
    assert_param(IS_DMA_PERIPH(DMAy));
    /* Calculate the used DMAy */
    /* Get DMAy INTSTS register value */
    tmpregister = DMAy->INTSTS;

    /* Check the status of the specified DMAy flag */
    if ((tmpregister & DMAFlag) != (uint32_t)RESET)
    {
        /* DMAyFlag is set */
        bitstatus = SET;
    }
    else
    {
        /* DMAyFlag is reset */
        bitstatus = RESET;
    }

    /* Return the DMAyFlag status */
    return bitstatus;
}

/**
 * @brief  Clears the DMA Channelx's pending flags.
 * @param DMAFlag specifies the flag to clear.
 *   This parameter can be any combination (for the same DMA) of the following values:
 *     @arg DMA_FLAG_GL1 DMA Channel1 global flag.
 *     @arg DMA_FLAG_TC1 DMA Channel1 transfer complete flag.
 *     @arg DMA_FLAG_HT1 DMA Channel1 half transfer flag.
 *     @arg DMA_FLAG_TE1 DMA Channel1 transfer error flag.
 *     @arg DMA_FLAG_GL2 DMA Channel2 global flag.
 *     @arg DMA_FLAG_TC2 DMA Channel2 transfer complete flag.
 *     @arg DMA_FLAG_HT2 DMA Channel2 half transfer flag.
 *     @arg DMA_FLAG_TE2 DMA Channel2 transfer error flag.
 *     @arg DMA_FLAG_GL3 DMA Channel3 global flag.
 *     @arg DMA_FLAG_TC3 DMA Channel3 transfer complete flag.
 *     @arg DMA_FLAG_HT3 DMA Channel3 half transfer flag.
 *     @arg DMA_FLAG_TE3 DMA Channel3 transfer error flag.
 *     @arg DMA_FLAG_GL4 DMA Channel4 global flag.
 *     @arg DMA_FLAG_TC4 DMA Channel4 transfer complete flag.
 *     @arg DMA_FLAG_HT4 DMA Channel4 half transfer flag.
 *     @arg DMA_FLAG_TE4 DMA Channel4 transfer error flag.
 *     @arg DMA_FLAG_GL5 DMA Channel5 global flag.
 *     @arg DMA_FLAG_TC5 DMA Channel5 transfer complete flag.
 *     @arg DMA_FLAG_HT5 DMA Channel5 half transfer flag.
 *     @arg DMA_FLAG_TE5 DMA Channel5 transfer error flag.
 * @param DMA DMA
 *   This parameter can be one of the following values:
 *     @arg DMA .
 */
void DMA_ClearFlag(uint32_t DMAFlag, DMA_Module* DMAy)
{
    /* Check the parameters */
    assert_param(IS_DMA_CLEAR_FLAG(DMAFlag));
    assert_param(IS_DMA_PERIPH(DMAy));
    /* Calculate the used DMAy */
    /* Clear the selected DMAy flags */
    DMAy->INTCLR = DMAFlag;
}

/**
 * @brief  Checks whether the specified DMA Channelx interrupt has occurred or not.
 * @param DMA_IT specifies the DMAy interrupt source to check.
 *   This parameter can be one of the following values:
 *     @arg DMA_INT_GLB1 DMA Channel1 global interrupt.
 *     @arg DMA_INT_TXC1 DMA Channel1 transfer complete interrupt.
 *     @arg DMA_INT_HTX1 DMA Channel1 half transfer interrupt.
 *     @arg DMA_INT_ERR1 DMA Channel1 transfer error interrupt.
 *     @arg DMA_INT_GLB2 DMA Channel2 global interrupt.
 *     @arg DMA_INT_TXC2 DMA Channel2 transfer complete interrupt.
 *     @arg DMA_INT_HTX2 DMA Channel2 half transfer interrupt.
 *     @arg DMA_INT_ERR2 DMA Channel2 transfer error interrupt.
 *     @arg DMA_INT_GLB3 DMA Channel3 global interrupt.
 *     @arg DMA_INT_TXC3 DMA Channel3 transfer complete interrupt.
 *     @arg DMA_INT_HTX3 DMA Channel3 half transfer interrupt.
 *     @arg DMA_INT_ERR3 DMA Channel3 transfer error interrupt.
 *     @arg DMA_INT_GLB4 DMA Channel4 global interrupt.
 *     @arg DMA_INT_TXC4 DMA Channel4 transfer complete interrupt.
 *     @arg DMA_INT_HTX4 DMA Channel4 half transfer interrupt.
 *     @arg DMA_INT_ERR4 DMA Channel4 transfer error interrupt.
 *     @arg DMA_INT_GLB5 DMA Channel5 global interrupt.
 *     @arg DMA_INT_TXC5 DMA Channel5 transfer complete interrupt.
 *     @arg DMA_INT_HTX5 DMA Channel5 half transfer interrupt.
 *     @arg DMA_INT_ERR5 DMA Channel5 transfer error interrupt.
 * @param DMA DMA
 *   This parameter can be one of the following values:
 *     @arg DMA .
 * @return The new state of DMA_IT (SET or RESET).
 */
INTStatus DMA_GetIntStatus(uint32_t DMA_IT, DMA_Module* DMAy)
{
    INTStatus bitstatus  = RESET;
    uint32_t tmpregister = 0;

    /* Check the parameters */
    assert_param(IS_DMA_GET_IT(DMA_IT));
    assert_param(IS_DMA_PERIPH(DMAy));
    /* Calculate the used DMA */
    /* Get DMAy INTSTS register value */
    tmpregister = DMAy->INTSTS;

    /* Check the status of the specified DMAy interrupt */
    if ((tmpregister & DMA_IT) != (uint32_t)RESET)
    {
        /* DMAy_IT is set */
        bitstatus = SET;
    }
    else
    {
        /* DMAy_IT is reset */
        bitstatus = RESET;
    }
    /* Return the DMAInt status */
    return bitstatus;
}

/**
 * @brief  Clears the DMA Channelx's interrupt pending bits.
 * @param DMA_IT specifies the DMA interrupt pending bit to clear.
 *   This parameter can be any combination (for the same DMA) of the following values:
 *     @arg DMA_INT_GLB1 DMA Channel1 global interrupt.
 *     @arg DMA_INT_TXC1 DMA Channel1 transfer complete interrupt.
 *     @arg DMA_INT_HTX1 DMA Channel1 half transfer interrupt.
 *     @arg DMA_INT_ERR1 DMA Channel1 transfer error interrupt.
 *     @arg DMA_INT_GLB2 DMA Channel2 global interrupt.
 *     @arg DMA_INT_TXC2 DMA Channel2 transfer complete interrupt.
 *     @arg DMA_INT_HTX2 DMA Channel2 half transfer interrupt.
 *     @arg DMA_INT_ERR2 DMA Channel2 transfer error interrupt.
 *     @arg DMA_INT_GLB3 DMA Channel3 global interrupt.
 *     @arg DMA_INT_TXC3 DMA Channel3 transfer complete interrupt.
 *     @arg DMA_INT_HTX3 DMA Channel3 half transfer interrupt.
 *     @arg DMA_INT_ERR3 DMA Channel3 transfer error interrupt.
 *     @arg DMA_INT_GLB4 DMA Channel4 global interrupt.
 *     @arg DMA_INT_TXC4 DMA Channel4 transfer complete interrupt.
 *     @arg DMA_INT_HTX4 DMA Channel4 half transfer interrupt.
 *     @arg DMA_INT_ERR4 DMA Channel4 transfer error interrupt.
 *     @arg DMA_INT_GLB5 DMA Channel5 global interrupt.
 *     @arg DMA_INT_TXC5 DMA Channel5 transfer complete interrupt.
 *     @arg DMA_INT_HTX5 DMA Channel5 half transfer interrupt.
 *     @arg DMA_INT_ERR5 DMA Channel5 transfer error interrupt.
 * @param DMAy DMA
 *   This parameter can be one of the following values:
 *     @arg DMA .
 */
void DMA_ClrIntPendingBit(uint32_t DMA_IT, DMA_Module* DMAy)
{
    /* Check the parameters */
    assert_param(IS_DMA_CLR_INT(DMA_IT));
    assert_param(IS_DMA_PERIPH(DMAy));
    /* Calculate the used DMA */
    /* Clear the selected DMA interrupt pending bits */
    DMAy->INTCLR = DMA_IT;
}

/**
 * @brief  Set the DMA Channelx's remap request.
 * @param DMA_REMAP specifies the DMA request.
 *   This parameter can be set by the following values:
 *     @arg DMA_REMAP_ADC            DMA Request For ADC
 *     @arg DMA_REMAP_USART1_TX      DMA Request For USART1_TX  
 *     @arg DMA_REMAP_USART1_RX      DMA Request For USART1_RX  
 *     @arg DMA_REMAP_LPUART_TX      DMA Request For LPUART_TX  
 *     @arg DMA_REMAP_LPUART_RX      DMA Request For LPUART_RX  
 *     @arg DMA_REMAP_USART2_TX      DMA Request For USART2_TX  
 *     @arg DMA_REMAP_USART2_RX      DMA Request For USART2_RX  
 *     @arg DMA_REMAP_SPI1_TX        DMA Request For SPI1_TX    
 *     @arg DMA_REMAP_SPI1_RX        DMA Request For SPI1_RX    
 *     @arg DMA_REMAP_SPI2_TX        DMA Request For SPI2_TX    
 *     @arg DMA_REMAP_SPI2_RX        DMA Request For SPI2_RX    
 *     @arg DMA_REMAP_I2C_TX         DMA Request For I2C_TX     
 *     @arg DMA_REMAP_I2C_RX         DMA Request For I2C_RX     
 *     @arg DMA_REMAP_TIM1_CH1       DMA Request For TIM1_CH1   
 *     @arg DMA_REMAP_TIM1_CH2       DMA Request For TIM1_CH2   
 *     @arg DMA_REMAP_TIM1_CH3       DMA Request For TIM1_CH3   
 *     @arg DMA_REMAP_TIM1_CH4       DMA Request For TIM1_CH4   
 *     @arg DMA_REMAP_TIM1_COM       DMA Request For TIM1_COM   
 *     @arg DMA_REMAP_TIM1_UP        DMA Request For TIM1_UP    
 *     @arg DMA_REMAP_TIM1_TRIG      DMA Request For TIM1_TRIG  
 *     @arg DMA_REMAP_TIM3_CH1       DMA Request For TIM3_CH1   
 *     @arg DMA_REMAP_TIM3_CH3       DMA Request For TIM3_CH3   
 *     @arg DMA_REMAP_TIM3_CH4       DMA Request For TIM3_CH4   
 *     @arg DMA_REMAP_TIM3_UP        DMA Request For TIM3_UP    
 *     @arg DMA_REMAP_TIM3_TRIG      DMA Request For TIM3_TRIG  
 *     @arg DMA_REMAP_TIM6           DMA Request For TIM6       
 * @param DMAy DMA
 *   This parameter can be one of the following values:
 *     @arg DMA .
 * @param DMAChx where x can be 1 to 5 for DMA to select the DMA Channel.
 * @param Cmd new state of the DMA Channelx.
 *   This parameter can be: ENABLE or DISABLE.
 */
void DMA_RequestRemap(uint32_t DMA_REMAP, DMA_Module* DMAy, DMA_ChannelType* DMAChx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_DMA_REMAP(DMA_REMAP));
    assert_param(IS_DMA_PERIPH(DMAy));
    assert_param(IS_DMA_ALL_PERIPH(DMAChx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    
    if (Cmd != DISABLE)
    {
        /* Calculate the used DMAy */
        /* Set the selected DMAy remap request */
        DMAChx->CHSEL = DMA_REMAP;
    }
    else
    {
        /* Clear DMAy remap */
        DMAChx->CHSEL = 0;
    }
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
