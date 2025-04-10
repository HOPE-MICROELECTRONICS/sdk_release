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
 * @file cmt453x_gpio.c
 * @version v1.0.1
 *
  */
#include "cmt453x_gpio.h"

/** @addtogroup CMT453X_StdPeriph_Driver
 * @{
 */

/** @addtogroup GPIO
 * @brief GPIO driver modules
 * @{
 */

/** @addtogroup GPIO_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @addtogroup GPIO_Private_Defines
 * @{
 */
      
#define GPIO_MODE                         ((uint32_t)0x00000003)
#define EXTI_MODE                         ((uint32_t)0x10000000)
#define GPIO_MODE_IT                      ((uint32_t)0x00010000)
#define GPIO_MODE_EVT                     ((uint32_t)0x00020000)
#define RISING_EDGE                       ((uint32_t)0x00100000) 
#define FALLING_EDGE                      ((uint32_t)0x00200000)
#define GPIO_OUTPUT_TYPE                  ((uint32_t)0x00000010)

/**
 * @}
 */

/** @addtogroup GPIO_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup GPIO_Private_Variables
 * @{
 */

/**
 * @}
 */

/** @addtogroup GPIO_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup GPIO_Private_Functions
 * @{
 */

/**
 * @brief  Deinitializes the GPIOx peripheral registers to their default reset values.
 * @param GPIOx where x can be (A,B,C,D,F) to select the GPIO peripheral.
 */
void GPIO_DeInit(GPIO_Module* GPIOx)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphReset(RCC_APB2_PERIPH_GPIOA, ENABLE);
        RCC_EnableAPB2PeriphReset(RCC_APB2_PERIPH_GPIOA, DISABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphReset(RCC_APB2_PERIPH_GPIOB, ENABLE);
        RCC_EnableAPB2PeriphReset(RCC_APB2_PERIPH_GPIOB, DISABLE);
    }
    else
    {
        return;
    }
}

/**
 * @brief  Deinitializes the GPIOx peripheral registers to their default reset values.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param GPIO_Pin specifies the port bit to be written.
  *                This parameter can be one of GPIO_PIN_x where x can be (0..13).
  *                All port bits are not necessarily available on all GPIOs.
 */
void GPIO_DeInitPin(GPIO_Module* GPIOx, uint32_t Pin)
{
    uint32_t pos = 0x00U, currentpin = 0x00U;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
        assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));

    while((Pin >> pos) != 0)
        {
            /* Get the IO position */
            currentpin = (Pin) & (1U << pos);

            if(currentpin)
            {
                /*------------------------- GPIO Mode Configuration --------------------*/
                /* Configure IO Direction in Analog Mode */
                GPIOx->PMODE |= (GPIO_PMODE0 << (pos * 2U));

                /* Configure the default Alternate Function in current IO */ 
                if(pos & 0x08)
                    GPIOx->AFH |= ((0x0FUL) << ((pos & 0x07U) * 4U));
                else
                    GPIOx->AFL |= ((0x0FUL) << ((pos & 0x07U) * 4U));

                /* Configure the default value IO Output Type */
                GPIOx->POTYPE &= ~(GPIO_POTYPE_POT_0 << pos);

                /* Deactivate the Pull-up and Pull-down resistor for the current IO */
                GPIOx->PUPD &= ~(GPIO_PUPD0 << (pos * 2U));

                /* Configure the default value for IO Speed */
                GPIOx->SR |= (GPIO_SR_SR0 << pos);

                /* Configure the default value for IO Speed */
                GPIOx->DS &= ~(GPIO_DS_DS0 << pos);
            } 
            pos++;
        }
    
}

/**
 * @brief  Deinitializes the Alternate Functions (remap, event control
 *   and EXTI configuration) registers to their default reset values.
 */
void GPIO_AFIOInitDefault(void)
{
    RCC_EnableAPB2PeriphReset(RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC_EnableAPB2PeriphReset(RCC_APB2_PERIPH_AFIO, DISABLE);
}

/**
 * @brief  Initializes the GPIOx peripheral according to the specified
 *         parameters in the GPIO_InitStruct.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param GPIO_InitStruct pointer to a GPIO_InitType structure that
 *         contains the configuration information for the specified GPIO peripheral.
 */
 
void GPIO_InitPeripheral(GPIO_Module* GPIOx, GPIO_InitType * GPIO_InitStruct)
{
    uint32_t pos = 0x00U, currentpin = 0x00U;
    uint32_t tmpregister = 0x00U; 
    
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, GPIO_InitStruct->Pin));
    assert_param(IS_GPIO_AF(GPIO_InitStruct->GPIO_Alternate)); 
    assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));    
    assert_param(IS_GPIO_PULL(GPIO_InitStruct->GPIO_Pull));                
    assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));
    assert_param(IS_GPIO_CURRENT(GPIO_InitStruct->GPIO_Current));

    while(((GPIO_InitStruct->Pin) >> pos) != 0)
    {
        /* Get the IO position */
        currentpin = (GPIO_InitStruct->Pin) & (1U << pos);

        if(currentpin)
        {
            /*--------------------- GPIO Mode Configuration ------------------------*/
              /* In case of Alternate function mode selection */
            if((GPIO_InitStruct->GPIO_Mode == GPIO_MODE_AF_PP) || (GPIO_InitStruct->GPIO_Mode == GPIO_MODE_AF_OD))
            {
                /* Check the Alternate function parameters */
                assert_param(IS_GPIO_AF(GPIO_InitStruct->GPIO_Alternate));

                /* Configure Alternate function mapped with the current IO */
                if(pos & 0x08)
                {
                    tmpregister = GPIOx->AFH;
                    tmpregister &= ~((uint32_t)0xF << ((uint32_t)(pos & (uint32_t)0x07) * 4U));
                    tmpregister |= ((uint32_t)(GPIO_InitStruct->GPIO_Alternate) << ((uint32_t)(pos & (uint32_t)0x07) * 4U)) ;
                    GPIOx->AFH = tmpregister;
                }
                else
                {
                  tmpregister = GPIOx->AFL;
                    tmpregister &= ~((uint32_t)0xF << ((uint32_t)(pos & (uint32_t)0x07) * 4U)) ;
                    tmpregister |= ((uint32_t)(GPIO_InitStruct->GPIO_Alternate) << ((uint32_t)(pos & (uint32_t)0x07) * 4U)) ;
                    GPIOx->AFL = tmpregister;
                }
            }

            /* In case of Output or Alternate function mode selection */
            if ((GPIO_InitStruct->GPIO_Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_InitStruct->GPIO_Mode == GPIO_MODE_OUTPUT_OD)
                 ||(GPIO_InitStruct->GPIO_Mode == GPIO_MODE_AF_PP) || (GPIO_InitStruct->GPIO_Mode == GPIO_MODE_AF_OD))
            {
                /* Configure the IO Output Type */
                tmpregister = GPIOx->POTYPE;
                tmpregister &= ~(GPIO_POTYPE_POT_0 << pos);                
                tmpregister |= (((GPIO_InitStruct->GPIO_Mode >> 4U) & 0x01U) << pos);
                GPIOx->POTYPE = tmpregister;
            }

            /*---------------------------- GPIO Mode Configuration -----------------------*/
            /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
            tmpregister = GPIOx->PMODE;
            tmpregister &= ~(GPIO_PMODE0 << (pos * 2U));
            tmpregister |= (((GPIO_InitStruct->GPIO_Mode & 0x03U) << (pos * 2U)));
            GPIOx->PMODE = tmpregister;

            /* Configure pull-down mode */
            tmpregister = GPIOx->PUPD;    
            tmpregister &= ~(GPIO_PUPD0 << (pos * 2U));
            tmpregister |= (GPIO_InitStruct->GPIO_Pull << (pos * 2U));
            GPIOx->PUPD = tmpregister;    

            /* Configure slew rate */
            tmpregister = GPIOx->SR;
            tmpregister &= ~(GPIO_SR_SR0 << pos);
            tmpregister |= (GPIO_InitStruct->GPIO_Speed << pos);
            GPIOx->SR = tmpregister;

            /* Configure driver current */
            tmpregister = GPIOx->DS;
            tmpregister &= ~(GPIO_DS_DS0 << pos* 2U);
            tmpregister |= (GPIO_InitStruct->GPIO_Current << pos* 2U);
            GPIOx->DS = tmpregister;
        }
        pos++;      
    }    
}

/**
 * @brief  Fills each GPIO_InitStruct member with its default value.
 * @param GPIO_InitStruct pointer to a GPIO_InitType structure which will
 *         be initialized.
 */
void GPIO_InitStruct(GPIO_InitType* GPIO_InitStruct)
{    
    /* Reset GPIO init structure parameters values */
    GPIO_InitStruct->Pin             = GPIO_PIN_ALL;
    GPIO_InitStruct->GPIO_Alternate  = GPIO_NO_AF;
    GPIO_InitStruct->GPIO_Mode       = GPIO_MODE_INPUT;
    GPIO_InitStruct->GPIO_Pull       = GPIO_NO_PULL;
    GPIO_InitStruct->GPIO_Speed      = GPIO_SPEED_LOW;    
    GPIO_InitStruct->GPIO_Current    = GPIO_DC_LOW;
}

/**
 * @brief  Reads the specified input port pin.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param Pin specifies the port bit to read.
 *   This parameter can be GPIO_Pin_x where x can be (0..13).
 * @return The input port pin value.
 */
uint8_t GPIO_ReadInputDataBit(GPIO_Module* GPIOx, uint16_t Pin)
{
    uint8_t bitstatus = 0x00;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));

    if ((GPIOx->PID & Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }
    return bitstatus;
}

/**
 * @brief  Reads the specified GPIO input data port.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @return GPIO input data port value.
 */
uint16_t GPIO_ReadInputData(GPIO_Module* GPIOx)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    return ((uint16_t)GPIOx->PID);
}

/**
 * @brief  Reads the specified output data port bit.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param Pin specifies the port bit to read.
 *   This parameter can be GPIO_Pin_x where x can be (0..13).
 * @return The output port pin value.
 */
uint8_t GPIO_ReadOutputDataBit(GPIO_Module* GPIOx, uint16_t Pin)
{
    uint8_t bitstatus = 0x00;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));

    if ((GPIOx->POD & Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }
    return bitstatus;
}

/**
 * @brief  Reads the specified GPIO output data port.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @return GPIO output data port value.
 */
uint16_t GPIO_ReadOutputData(GPIO_Module* GPIOx)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    return ((uint16_t)GPIOx->POD);
}

/**
 * @brief  Sets the selected data port bits.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param Pin specifies the port bits to be written.
 *   This parameter can be any combination of GPIO_Pin_x where x can be (0..13).
 */
void GPIO_SetBits(GPIO_Module* GPIOx, uint16_t Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));

    GPIOx->PBSC = Pin;
}

/**
 * @brief  Clears the selected data port bits.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param Pin specifies the port bits to be written.
 *   This parameter can be any combination of GPIO_Pin_x where x can be (0..13).
 */
void GPIO_ResetBits(GPIO_Module* GPIOx, uint16_t Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));

    GPIOx->PBC = Pin;
}

/**
 * @brief  Sets or clears the selected data port bit.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param Pin specifies the port bit to be written.
 *   This parameter can be one of GPIO_Pin_x where x can be (0..13).
 * @param BitCmd specifies the value to be written to the selected bit.
 *   This parameter can be one of the Bit_OperateType enum values:
 *     @arg Bit_RESET to clear the port pin
 *     @arg Bit_SET to set the port pin
 */
void GPIO_WriteBit(GPIO_Module* GPIOx, uint16_t Pin, Bit_OperateType BitCmd)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));
    assert_param(IS_GPIO_BIT_OPERATE(BitCmd));

    if (BitCmd != Bit_RESET)
    {
        GPIOx->PBSC = Pin;
    }
    else
    {
        GPIOx->PBC = Pin;
    }
}

/**
 * @brief  Writes data to the specified GPIO data port.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param PortVal specifies the value to be written to the port output data register.
 */
void GPIO_Write(GPIO_Module* GPIOx, uint16_t PortVal)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));    

    GPIOx->POD = PortVal;
}

/**
  * @brief  Toggles the specified GPIO pins.
  * @param GPIOx Where x can be (A,B) to select the GPIO peripheral.
  *              All port bits are not necessarily available on all GPIOs.
  * @param GPIO_Pin Specifies the pins to be toggled.
  * @retval None
  */
void GPIO_TogglePin(GPIO_Module *GPIOx, uint16_t Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));
    GPIOx->POD ^= Pin;
}

/**
 * @brief  Locks GPIO Pins configuration registers.
 * @param GPIOx where x can be (A,B) to select the GPIO peripheral.
 * @param Pin specifies the port bit to be written.
 *   This parameter can be any combination of GPIO_Pin_x where x can be (0..13).
 */
void GPIO_ConfigPinLock(GPIO_Module* GPIOx, uint16_t Pin)
{
    uint32_t tmp = 0x00010000;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN_AVAILABLE(GPIOx, Pin));

    tmp |= Pin;
    /* Set LCKK bit */
    GPIOx->PLOCK = tmp;
    /* Reset LCKK bit */
    GPIOx->PLOCK = Pin;
    /* Set LCKK bit */
    GPIOx->PLOCK = tmp;
    /* Read LCKK bit*/
    tmp = GPIOx->PLOCK;
    /* Read LCKK bit*/
    tmp = GPIOx->PLOCK;
}

/**
 * @brief  Selects the GPIO pin used as EXTI Line.
 * @param PortSource selects the GPIO port to be used as source for EXTI lines.
 *   This parameter can be GPIO_PortSourceGPIOx where x can be (A,B).
 * @param PinSource specifies the EXTI line to be configured.
 *   This parameter can be GPIO_PinSourcex where x can be (0..13).
 */
void GPIO_ConfigEXTILine(uint8_t PortSource, uint8_t PinSource)
{
    /* Check the parameters */
    assert_param(IS_GPIO_EXTI_PORT_SOURCE(PortSource));
    assert_param(IS_GPIO_PIN_SOURCE(PinSource));
    
    if(PortSource == GPIOA_PORT_SOURCE)
    {
        switch(PinSource)
        {
            case 0:
                AFIO->EXTI_CFG[0] &= ~(0x03U);
                break;
            case 1:
                AFIO->EXTI_CFG[0] &= ~(0x03U);
                AFIO->EXTI_CFG[0] |= (0x01U);
                break;
            case 2:
                AFIO->EXTI_CFG[0] &= ~(0x03U << 4U);
                break;
            case 3:
                AFIO->EXTI_CFG[0] &= ~(0x03U << 4U);
                AFIO->EXTI_CFG[0] |= (0x01U << 4U);
                break;
            case 4:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 8U);   
                AFIO->EXTI_CFG[1] |= (0x02U << 8U);
                break;
            case 5:
                AFIO->EXTI_CFG[0] &= ~(0x03U << 12U);  
                break;
            case 6:
                AFIO->EXTI_CFG[0] &= ~(0x03U << 8U);           
                break;
             default:
                break;
        }            
    }
    else{
        switch(PinSource)
        {
            case 0:
                AFIO->EXTI_CFG[0] &= ~(0x03U);
                AFIO->EXTI_CFG[0] |= (0x02U);
                break;
            case 1:
                AFIO->EXTI_CFG[0] &= ~(0x03U << 4U);
                AFIO->EXTI_CFG[0] |= (0x02U << 4U);
                break;
            case 2:
                AFIO->EXTI_CFG[0] &= ~(0x03U << 8U);
                AFIO->EXTI_CFG[0] |= (0x01U << 8U);
                break;
            case 3:            
                AFIO->EXTI_CFG[0] &= ~(0x03U << 12U);
                AFIO->EXTI_CFG[0] |= (0x01U << 12U);
                break;
            case 4:
                AFIO->EXTI_CFG[1] &= ~(0x03U);
                AFIO->EXTI_CFG[1] |= (0x01U);
                break;
            case 5:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 4U);
                AFIO->EXTI_CFG[1] |= (0x02U << 4U);
                break;
            case 6:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 8U);
                AFIO->EXTI_CFG[1] |= (0x01U << 8U);
                break;
            case 7:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 12U);
                AFIO->EXTI_CFG[1] |= (0x01U << 12U);
                break;
            case 8:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 12U);
                AFIO->EXTI_CFG[1] |= (0x02U << 12U);
                break;
            case 9:
                AFIO->EXTI_CFG[1] &= ~(0x03U);
                break;
            case 10:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 4U);
                break;
            case 11:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 4U);
                AFIO->EXTI_CFG[1] |= (0x01U << 4U);
                break;
            case 12:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 8U);
                break;
            case 13:
                AFIO->EXTI_CFG[1] &= ~(0x03U << 12U); 
                break;
             default:
                break;
        }
    }
}

/**
 * @brief  Changes the mapping of the specified pin.
 * @param PortSource selects the GPIO port to be used.
 * @param PinSource specifies the pin for the remaping.
 *   This parameter can be GPIO_PinSourcex where x can be (0..13).
 * @param AlternateFunction specifies the alternate function for the remaping.
 */
void GPIO_ConfigPinRemap(uint8_t PortSource, uint8_t PinSource, uint32_t AlternateFunction)
{
    uint32_t tmp = 0x00, tmpregister = 0x00;
    GPIO_Module *GPIOx;
    /* Check the parameters */
    assert_param(IS_GPIO_REMAP_PORT_SOURCE(PortSource));
    assert_param(IS_GPIO_PIN_SOURCE(PinSource));
    assert_param(IS_GPIO_AF(AlternateFunction));
    /*Get Peripheral point*/
    GPIOx = GPIO_GET_PERIPH(PortSource);
    /**/
     if(PinSource & (uint8_t)0x08)
    {
        tmp = (uint32_t)(PinSource & (uint8_t)0x07);
        /*Read GPIO_AFH register*/
        tmpregister  = GPIOx->AFH;
        /*Reset corresponding bits*/
        tmpregister &=~((uint32_t)0x0F <<(tmp*4U));
        /*Set corresponding bits*/
        tmpregister |= AlternateFunction;
        /*Write to the GPIO_AFH register*/
        GPIOx->AFH = tmpregister;
    }
    else
    {
        tmp = (uint32_t)(PinSource & (uint8_t)0x07);
        /*Read GPIO_AFL register*/
        tmpregister  = GPIOx->AFL;
        /*Reset corresponding bits*/
        tmpregister &=~((uint32_t)0x0F <<(tmp*4U));
        /*Set corresponding bits*/
        tmpregister |= AlternateFunction;
        /*Write to the GPIO_AFL register*/
        GPIOx->AFL = tmpregister;
    }
}


/**
 * @brief  Selects the alternate function SPIx NSS mode.
 * @param AFIO_SPIx_NSS choose which SPI configuration.
 *   This parameter can be AFIO_SPI1_NSS and AFIO_SPI2_NSS.
 * @param SpiNssMode specifies the SPI_NSS mode to be configured.
 *   This parameter can be AFIO_SPI_NSS_High_IMPEDANCE and AFIO_SPI_NSS_High_LEVEL.
 */
void AFIO_ConfigSPINSSMode(uint32_t AFIO_SPIx_NSS, uint32_t SpiNssMode)
{
    uint32_t tmpregister = 0x00U;

    /* Check the parameters */
    assert_param(IS_AFIO_SPIX(AFIO_SPIx_NSS));
    assert_param(IS_AFIO_SPI_NSS(SpiNssMode));

    tmpregister = AFIO->CFG;    
    if(SpiNssMode != AFIO_SPI_NSS_High_IMPEDANCE)
    {
        tmpregister |= AFIO_SPIx_NSS;
    }
    else 
    {
        tmpregister &= ~AFIO_SPIx_NSS;
    }    
    AFIO->CFG = tmpregister;
}


/**
 * @}
 */

/**
 * @}
 */
