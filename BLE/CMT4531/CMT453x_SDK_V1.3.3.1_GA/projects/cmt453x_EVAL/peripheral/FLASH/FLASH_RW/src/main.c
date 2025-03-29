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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "cmt453x.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED1_PORT GPIOB
#define LED1_PIN  GPIO_PIN_0
#define LED2_PORT GPIOA
#define LED2_PIN  GPIO_PIN_6

#define FLASH_TEST_ADDRESS       0x1020000
#define BUFFER_SIZE              4096
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t m_buffer[BUFFER_SIZE];
//Allocate a buffer for read flash memory and for compare with a static buffer.
static uint8_t buffer[BUFFER_SIZE];
GPIO_InitType GPIO_InitStructure;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void assert_handle(void)
{
    GPIO_SetBits(LED1_PORT, LED1_PIN);
    while(1){

    }
}


/**
 * @brief  Main program.
 */
int main(void)
{
    //Initialize dev board leds
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);    
    GPIO_InitStructure.Pin = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;    
    GPIO_InitPeripheral(LED2_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED1_PORT, LED1_PIN);
    GPIO_ResetBits(LED2_PORT, LED2_PIN);    
    
    //Initialize Qflash.
    Qflash_Init();
    //Erase one Flash Sector
    Qflash_Erase_Sector(FLASH_TEST_ADDRESS);
    //Read test flash sector into buffer ram
    Qflash_Read(FLASH_TEST_ADDRESS, buffer, BUFFER_SIZE);
    for(uint32_t i=0;i<BUFFER_SIZE;i++){
        if(buffer[i] != 0xFF){
            assert_handle();
        }
    }
    //Assign value to m_buffer
    for(uint32_t i=0;i<BUFFER_SIZE;i++){
        m_buffer[i] = i;
    }    
    //Write buffer data to flash.
    Qflash_Write(FLASH_TEST_ADDRESS, m_buffer, BUFFER_SIZE); 
    //Read test flash sector into buffer ram
    Qflash_Read(FLASH_TEST_ADDRESS, buffer, BUFFER_SIZE);
    if(memcmp(buffer,m_buffer,BUFFER_SIZE) != 0){
        assert_handle();
    }
    GPIO_SetBits(LED2_PORT, LED2_PIN);
    while(1){
        
    }
    
    
    
}



/**
 * @}
 */
