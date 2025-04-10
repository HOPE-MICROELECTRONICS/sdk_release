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
 * @file cmt453x_dma.h
 
 * @version v1.0.1
 *
  */
#ifndef __CMT453X_DMA_H__
#define __CMT453X_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmt453x.h"

/** @addtogroup CMT453X_StdPeriph_Driver
 * @{
 */

/** @addtogroup DMA
 * @{
 */

/** @addtogroup DMA_Exported_Types
 * @{
 */

/**
 * @brief  DMA Init structure definition
 */

typedef struct
{
    uint32_t PeriphAddr; /*!< Specifies the peripheral base address for DMAy Channelx. */

    uint32_t MemAddr; /*!< Specifies the memory base address for DMAy Channelx. */

    uint32_t Direction; /*!< Specifies if the peripheral is the source or destination.
                           This parameter can be a value of @ref DMA_data_transfer_direction */

    uint32_t BufSize; /*!< Specifies the buffer size, in data unit, of the specified Channel.
                                  The data unit is equal to the configuration set in PeriphDataSize
                                  or MemDataSize members depending in the transfer direction. */

    uint32_t PeriphInc; /*!< Specifies whether the Peripheral address register is incremented or not.
                                     This parameter can be a value of @ref DMA_peripheral_incremented_mode */

    uint32_t DMA_MemoryInc; /*!< Specifies whether the memory address register is incremented or not.
                                 This parameter can be a value of @ref DMA_memory_incremented_mode */

    uint32_t PeriphDataSize; /*!< Specifies the Peripheral data width.
                                          This parameter can be a value of @ref DMA_peripheral_data_size */

    uint32_t MemDataSize; /*!< Specifies the Memory data width.
                                      This parameter can be a value of @ref DMA_memory_data_size */

    uint32_t CircularMode; /*!< Specifies the operation mode of the DMAy Channelx.
                            This parameter can be a value of @ref DMA_circular_normal_mode.
                            @note: The circular buffer mode cannot be used if the memory-to-memory
                                   data transfer is configured on the selected Channel */

    uint32_t Priority; /*!< Specifies the software priority for the DMAy Channelx.
                                This parameter can be a value of @ref DMA_priority_level */

    uint32_t Mem2Mem; /*!< Specifies if the DMAy Channelx will be used in memory-to-memory transfer.
                           This parameter can be a value of @ref DMA_memory_to_memory */
} DMA_InitType;

/**
 * @}
 */

/** @addtogroup DMA_Exported_Constants
 * @{
 */
#define IS_DMA_PERIPH(PERIPH)   ((PERIPH) == DMA)

#define IS_DMA_ALL_PERIPH(PERIPH)                                                                                      \
    (((PERIPH) == DMA_CH1) || ((PERIPH) == DMA_CH2) || ((PERIPH) == DMA_CH3) || ((PERIPH) == DMA_CH4)              \
     || ((PERIPH) == DMA_CH5))

/** @addtogroup DMA_data_transfer_direction
 * @{
 */

#define DMA_DIR_PERIPH_DST ((uint32_t)0x00000010)
#define DMA_DIR_PERIPH_SRC ((uint32_t)0x00000000)
#define IS_DMA_DIR(DIR)    (((DIR) == DMA_DIR_PERIPH_DST) || ((DIR) == DMA_DIR_PERIPH_SRC))
/**
 * @}
 */

/** @addtogroup DMA_peripheral_incremented_mode
 * @{
 */

#define DMA_PERIPH_INC_ENABLE          ((uint32_t)0x00000040)
#define DMA_PERIPH_INC_DISABLE         ((uint32_t)0x00000000)
#define IS_DMA_PERIPH_INC_STATE(STATE) (((STATE) == DMA_PERIPH_INC_ENABLE) || ((STATE) == DMA_PERIPH_INC_DISABLE))
/**
 * @}
 */

/** @addtogroup DMA_memory_incremented_mode
 * @{
 */

#define DMA_MEM_INC_ENABLE          ((uint32_t)0x00000080)
#define DMA_MEM_INC_DISABLE         ((uint32_t)0x00000000)
#define IS_DMA_MEM_INC_STATE(STATE) (((STATE) == DMA_MEM_INC_ENABLE) || ((STATE) == DMA_MEM_INC_DISABLE))
/**
 * @}
 */

/** @addtogroup DMA_peripheral_data_size
 * @{
 */

#define DMA_PERIPH_DATA_SIZE_BYTE     ((uint32_t)0x00000000)
#define DMA_PERIPH_DATA_SIZE_HALFWORD ((uint32_t)0x00000100)
#define DMA_PERIPH_DATA_SIZE_WORD     ((uint32_t)0x00000200)
#define IS_DMA_PERIPH_DATA_SIZE(SIZE)                                                                                  \
    (((SIZE) == DMA_PERIPH_DATA_SIZE_BYTE) || ((SIZE) == DMA_PERIPH_DATA_SIZE_HALFWORD)                                \
     || ((SIZE) == DMA_PERIPH_DATA_SIZE_WORD))
/**
 * @}
 */

/** @addtogroup DMA_memory_data_size
 * @{
 */

#define DMA_MemoryDataSize_Byte     ((uint32_t)0x00000000)
#define DMA_MemoryDataSize_HalfWord ((uint32_t)0x00000400)
#define DMA_MemoryDataSize_Word     ((uint32_t)0x00000800)
#define IS_DMA_MEMORY_DATA_SIZE(SIZE)                                                                                  \
    (((SIZE) == DMA_MemoryDataSize_Byte) || ((SIZE) == DMA_MemoryDataSize_HalfWord)                                    \
     || ((SIZE) == DMA_MemoryDataSize_Word))
/**
 * @}
 */

/** @addtogroup DMA_circular_normal_mode
 * @{
 */

#define DMA_MODE_CIRCULAR ((uint32_t)0x00000020)
#define DMA_MODE_NORMAL   ((uint32_t)0x00000000)
#define IS_DMA_MODE(MODE) (((MODE) == DMA_MODE_CIRCULAR) || ((MODE) == DMA_MODE_NORMAL))
/**
 * @}
 */

/** @addtogroup DMA_priority_level
 * @{
 */

#define DMA_PRIORITY_VERY_HIGH ((uint32_t)0x00003000)
#define DMA_PRIORITY_HIGH      ((uint32_t)0x00002000)
#define DMA_PRIORITY_MEDIUM    ((uint32_t)0x00001000)
#define DMA_PRIORITY_LOW       ((uint32_t)0x00000000)
#define IS_DMA_PRIORITY(PRIORITY)                                                                                      \
    (((PRIORITY) == DMA_PRIORITY_VERY_HIGH) || ((PRIORITY) == DMA_PRIORITY_HIGH)                                       \
     || ((PRIORITY) == DMA_PRIORITY_MEDIUM) || ((PRIORITY) == DMA_PRIORITY_LOW))
/**
 * @}
 */

/** @addtogroup DMA_memory_to_memory
 * @{
 */

#define DMA_M2M_ENABLE          ((uint32_t)0x00004000)
#define DMA_M2M_DISABLE         ((uint32_t)0x00000000)
#define IS_DMA_M2M_STATE(STATE) (((STATE) == DMA_M2M_ENABLE) || ((STATE) == DMA_M2M_DISABLE))

/**
 * @}
 */

/** @addtogroup DMA_interrupts_definition
 * @{
 */

#define DMA_INT_TXC           ((uint32_t)0x00000002)
#define DMA_INT_HTX           ((uint32_t)0x00000004)
#define DMA_INT_ERR           ((uint32_t)0x00000008)
#define IS_DMA_CONFIG_INT(IT) ((((IT)&0xFFFFFFF1) == 0x00) && ((IT) != 0x00))

#define DMA_INT_GLB1 ((uint32_t)0x00000001)
#define DMA_INT_TXC1 ((uint32_t)0x00000002)
#define DMA_INT_HTX1 ((uint32_t)0x00000004)
#define DMA_INT_ERR1 ((uint32_t)0x00000008)
#define DMA_INT_GLB2 ((uint32_t)0x00000010)
#define DMA_INT_TXC2 ((uint32_t)0x00000020)
#define DMA_INT_HTX2 ((uint32_t)0x00000040)
#define DMA_INT_ERR2 ((uint32_t)0x00000080)
#define DMA_INT_GLB3 ((uint32_t)0x00000100)
#define DMA_INT_TXC3 ((uint32_t)0x00000200)
#define DMA_INT_HTX3 ((uint32_t)0x00000400)
#define DMA_INT_ERR3 ((uint32_t)0x00000800)
#define DMA_INT_GLB4 ((uint32_t)0x00001000)
#define DMA_INT_TXC4 ((uint32_t)0x00002000)
#define DMA_INT_HTX4 ((uint32_t)0x00004000)
#define DMA_INT_ERR4 ((uint32_t)0x00008000)
#define DMA_INT_GLB5 ((uint32_t)0x00010000)
#define DMA_INT_TXC5 ((uint32_t)0x00020000)
#define DMA_INT_HTX5 ((uint32_t)0x00040000)
#define DMA_INT_ERR5 ((uint32_t)0x00080000)



#define IS_DMA_CLR_INT(IT) (((((IT)&0xF0000000) == 0x00) || (((IT)&0xEFF00000) == 0x00)) && ((IT) != 0x00))

#define IS_DMA_GET_IT(IT)                                                                                              \
    (((IT) == DMA_INT_GLB1) || ((IT) == DMA_INT_TXC1) || ((IT) == DMA_INT_HTX1) || ((IT) == DMA_INT_ERR1)          \
     || ((IT) == DMA_INT_GLB2) || ((IT) == DMA_INT_TXC2) || ((IT) == DMA_INT_HTX2) || ((IT) == DMA_INT_ERR2)       \
     || ((IT) == DMA_INT_GLB3) || ((IT) == DMA_INT_TXC3) || ((IT) == DMA_INT_HTX3) || ((IT) == DMA_INT_ERR3)       \
     || ((IT) == DMA_INT_GLB4) || ((IT) == DMA_INT_TXC4) || ((IT) == DMA_INT_HTX4) || ((IT) == DMA_INT_ERR4)       \
     || ((IT) == DMA_INT_GLB5) || ((IT) == DMA_INT_TXC5) || ((IT) == DMA_INT_HTX5) || ((IT) == DMA_INT_ERR5)       \
)

/**
 * @}
 */

/** @addtogroup DMA_flags_definition
 * @{
 */
#define DMA_FLAG_GL1 ((uint32_t)0x00000001)
#define DMA_FLAG_TC1 ((uint32_t)0x00000002)
#define DMA_FLAG_HT1 ((uint32_t)0x00000004)
#define DMA_FLAG_TE1 ((uint32_t)0x00000008)
#define DMA_FLAG_GL2 ((uint32_t)0x00000010)
#define DMA_FLAG_TC2 ((uint32_t)0x00000020)
#define DMA_FLAG_HT2 ((uint32_t)0x00000040)
#define DMA_FLAG_TE2 ((uint32_t)0x00000080)
#define DMA_FLAG_GL3 ((uint32_t)0x00000100)
#define DMA_FLAG_TC3 ((uint32_t)0x00000200)
#define DMA_FLAG_HT3 ((uint32_t)0x00000400)
#define DMA_FLAG_TE3 ((uint32_t)0x00000800)
#define DMA_FLAG_GL4 ((uint32_t)0x00001000)
#define DMA_FLAG_TC4 ((uint32_t)0x00002000)
#define DMA_FLAG_HT4 ((uint32_t)0x00004000)
#define DMA_FLAG_TE4 ((uint32_t)0x00008000)
#define DMA_FLAG_GL5 ((uint32_t)0x00010000)
#define DMA_FLAG_TC5 ((uint32_t)0x00020000)
#define DMA_FLAG_HT5 ((uint32_t)0x00040000)
#define DMA_FLAG_TE5 ((uint32_t)0x00080000)


#define IS_DMA_CLEAR_FLAG(FLAG) (((((FLAG)&0xF0000000) == 0x00) || (((FLAG)&0xEFF00000) == 0x00)) && ((FLAG) != 0x00))

#define IS_DMA_GET_FLAG(FLAG)                                                                                          \
    (((FLAG) == DMA_FLAG_GL1) || ((FLAG) == DMA_FLAG_TC1) || ((FLAG) == DMA_FLAG_HT1) || ((FLAG) == DMA_FLAG_TE1)  \
     || ((FLAG) == DMA_FLAG_GL2) || ((FLAG) == DMA_FLAG_TC2) || ((FLAG) == DMA_FLAG_HT2)                            \
     || ((FLAG) == DMA_FLAG_TE2) || ((FLAG) == DMA_FLAG_GL3) || ((FLAG) == DMA_FLAG_TC3)                            \
     || ((FLAG) == DMA_FLAG_HT3) || ((FLAG) == DMA_FLAG_TE3) || ((FLAG) == DMA_FLAG_GL4)                            \
     || ((FLAG) == DMA_FLAG_TC4) || ((FLAG) == DMA_FLAG_HT4) || ((FLAG) == DMA_FLAG_TE4)                            \
     || ((FLAG) == DMA_FLAG_GL5) || ((FLAG) == DMA_FLAG_TC5) || ((FLAG) == DMA_FLAG_HT5)                            \
     || ((FLAG) == DMA_FLAG_TE5))
/**
 * @}
 */

/** @addtogroup DMA_Buffer_Size
 * @{
 */

#define IS_DMA_BUF_SIZE(SIZE) (((SIZE) >= 0x1) && ((SIZE) < 0x10000))

/**
 * @}
 */

/** @addtogroup DMA_remap_request_definition
 * @{
 */
#define DMA_REMAP_ADC                     ((uint32_t)0x00000000)
#define DMA_REMAP_USART1_TX               ((uint32_t)0x00000002)
#define DMA_REMAP_USART1_RX               ((uint32_t)0x00000003)
#define DMA_REMAP_LPUART_TX               ((uint32_t)0x00000004)
#define DMA_REMAP_LPUART_RX               ((uint32_t)0x00000005)
#define DMA_REMAP_USART2_TX               ((uint32_t)0x00000006)
#define DMA_REMAP_USART2_RX               ((uint32_t)0x00000007)
#define DMA_REMAP_SPI1_TX                 ((uint32_t)0x00000008)
#define DMA_REMAP_SPI1_RX                 ((uint32_t)0x00000009)
#define DMA_REMAP_SPI2_TX                 ((uint32_t)0x0000000A)
#define DMA_REMAP_SPI2_RX                 ((uint32_t)0x0000000B)
#define DMA_REMAP_I2C_TX                  ((uint32_t)0x0000000C)
#define DMA_REMAP_I2C_RX                  ((uint32_t)0x0000000D)
#define DMA_REMAP_TIM1_CH1                ((uint32_t)0x0000000E)
#define DMA_REMAP_TIM1_CH2                ((uint32_t)0x0000000F)
#define DMA_REMAP_TIM1_CH3                ((uint32_t)0x00000010)
#define DMA_REMAP_TIM1_CH4                ((uint32_t)0x00000011)
#define DMA_REMAP_TIM1_COM                ((uint32_t)0x00000012)
#define DMA_REMAP_TIM1_UP                 ((uint32_t)0x00000013)
#define DMA_REMAP_TIM1_TRIG               ((uint32_t)0x00000014)
#define DMA_REMAP_TIM3_CH1                ((uint32_t)0x00000015)
#define DMA_REMAP_TIM3_CH3                ((uint32_t)0x00000016)
#define DMA_REMAP_TIM3_CH4                ((uint32_t)0x00000017)
#define DMA_REMAP_TIM3_UP                 ((uint32_t)0x00000018)
#define DMA_REMAP_TIM3_TRIG               ((uint32_t)0x00000019)
#define DMA_REMAP_TIM6                    ((uint32_t)0x0000001A)
 
#define IS_DMA_REMAP(FLAG)    (((FLAG) == DMA_REMAP_ADC)       ||  \
                               ((FLAG) == DMA_REMAP_USART1_TX) || ((FLAG) == DMA_REMAP_USART1_RX) ||       \
                               ((FLAG) == DMA_REMAP_LPUART_TX) || ((FLAG) == DMA_REMAP_LPUART_RX)||       \
                               ((FLAG) == DMA_REMAP_USART2_TX) || ((FLAG) == DMA_REMAP_USART2_RX)||       \
                               ((FLAG) == DMA_REMAP_SPI1_TX)   || ((FLAG) == DMA_REMAP_SPI1_RX)  ||       \
                               ((FLAG) == DMA_REMAP_SPI2_TX)   || ((FLAG) == DMA_REMAP_SPI2_RX)  ||       \
                               ((FLAG) == DMA_REMAP_I2C_TX)    || ((FLAG) == DMA_REMAP_I2C_RX)   ||       \
                               ((FLAG) == DMA_REMAP_TIM1_CH1)  || ((FLAG) == DMA_REMAP_TIM1_CH2)   ||       \
                               ((FLAG) == DMA_REMAP_TIM1_CH3)  || ((FLAG) == DMA_REMAP_TIM1_CH4)   ||       \
                               ((FLAG) == DMA_REMAP_TIM1_COM)  || ((FLAG) == DMA_REMAP_TIM1_UP)   ||       \
                               ((FLAG) == DMA_REMAP_TIM1_TRIG) || ((FLAG) == DMA_REMAP_TIM3_CH1)   ||       \
                               ((FLAG) == DMA_REMAP_TIM3_CH3)  || ((FLAG) == DMA_REMAP_TIM3_CH4)  ||       \
                               ((FLAG) == DMA_REMAP_TIM3_UP)   || ((FLAG) == DMA_REMAP_TIM3_TRIG)  ||       \
                               ((FLAG) == DMA_REMAP_TIM6))
/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup DMA_Exported_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup DMA_Exported_Functions
 * @{
 */

void DMA_DeInit(DMA_ChannelType* DMAChx);
void DMA_Init(DMA_ChannelType* DMAChx, DMA_InitType* DMA_InitParam);
void DMA_StructInit(DMA_InitType* DMA_InitParam);
void DMA_EnableChannel(DMA_ChannelType* DMAChx, FunctionalState Cmd);
void DMA_ConfigInt(DMA_ChannelType* DMAChx, uint32_t DMAInt, FunctionalState Cmd);
void DMA_SetCurrDataCounter(DMA_ChannelType* DMAChx, uint16_t DataNumber);
uint16_t DMA_GetCurrDataCounter(DMA_ChannelType* DMAChx);
FlagStatus DMA_GetFlagStatus(uint32_t DMAFlag, DMA_Module* DMAy);
void DMA_ClearFlag(uint32_t DMAFlag, DMA_Module* DMAy);
INTStatus DMA_GetIntStatus(uint32_t DMA_IT, DMA_Module* DMAy);
void DMA_ClrIntPendingBit(uint32_t DMA_IT, DMA_Module* DMAy);
void DMA_RequestRemap(uint32_t DMA_REMAP, DMA_Module* DMAy, DMA_ChannelType* DMAChx, FunctionalState Cmd);

#ifdef __cplusplus
}
#endif

#endif /*__CMT453X_DMA_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
