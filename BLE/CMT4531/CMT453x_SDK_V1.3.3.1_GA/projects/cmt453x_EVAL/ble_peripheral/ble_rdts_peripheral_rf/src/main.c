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

/** @addtogroup 
 * @{
 */
 
/* Includes ------------------------------------------------------------------*/
#include "cmt453x.h"
#include "rwip.h"
#include "hp_ble.h"
#include "hp_sleep.h"
#include "hp_delay.h"
#include "hp_log.h"
#include "app_usart.h"
#include "app_gpio.h"
#include "app_ble.h"
#if  (CFG_APP_HP_IUS)
#include "hp_dfu_boot.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DEMO_STRING  "\r\n HopeRF raw data transfer server(128bit UUID) demo \r\n"

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#if(HP_RF_CTRL_ENABLE)
#include "app_rf.h"
#define KEY_INPUT_PORT        GPIOB
#define KEY_INPUT_PIN         GPIO_PIN_2
#define KEY_INPUT_EXTI_LINE   EXTI_LINE2
#define KEY_INPUT_PORT_SOURCE GPIOB_PORT_SOURCE
#define KEY_INPUT_PIN_SOURCE  GPIO_PIN_SOURCE2
#define KEY_INPUT_IRQn        EXTI2_3_IRQn

uint8_t flag_rx_tx_mode = false;

extern void system_delay_n_10us(uint32_t value);

void KeyInputExtiInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_AFIO, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
    }
    else
    {
        return;
    }

    /*Configure the GPIO pin as input floating*/
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin          = Pin;
        GPIO_InitStructure.GPIO_Pull    = GPIO_PULL_UP;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }

    /*Configure key EXTI Line to key input Pin*/
    GPIO_ConfigEXTILine(KEY_INPUT_PORT_SOURCE, KEY_INPUT_PIN_SOURCE);

    /*Configure key EXTI line*/
    EXTI_InitStructure.EXTI_Line    = KEY_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /*Set key input interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   = KEY_INPUT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI2_3_IRQHandler(void)
{
    if (RESET != EXTI_GetITStatus(KEY_INPUT_EXTI_LINE))
    {
        HP_LOG_INFO("EXTI2_3_IRQHandler\r\n");

        system_delay_n_10us(20);

        flag_rx_tx_mode  = !flag_rx_tx_mode;
        if(flag_rx_tx_mode)
        {
						HP_LOG_INFO("Switch to RF tx/rx mode, the active connection will be terminated\r\n");
						HP_LOG_INFO("app_env.adv_mode = %x\r\n", app_env.adv_mode);
            
            HP_LOG_INFO("app state = %x\r\n", ke_state_get(TASK_APP));
            //STOP ADV AND CONNECT 
            if(ke_state_get(TASK_APP) == APP_CONNECTED)
            {
                hp_ble_disconnect();
            } 
            else
            {
                hp_ble_adv_stop();
            }
						
            //start 2.4g tx rx 
            ke_timer_set(APP_RF_INTV_TIMER, TASK_APP, RF_SWITCH_DELAY);
        } 
        else
        {
						HP_LOG_INFO("Switch to BLE mode, start advertising\r\n");
					
            ke_timer_clear(APP_RF_INTV_TIMER, TASK_APP);
            ke_timer_clear(APP_RF_RX_WIND_TIMER, TASK_APP);
            app_rx_stop();
            delay_n_10us(20*100);
					
            //START ADV
            hp_ble_adv_start(); 
        }

        EXTI_ClrITPendBit(KEY_INPUT_EXTI_LINE);
    }
}
#endif //HP_RF_CTRL_ENABLE

/**
 * @brief  main function
 * @param   
 * @return 
 * @note   Note
 */
int main(void)
{
    //for hold the SWD before sleep
    delay_n_10us(200*1000);
    
    HP_LOG_INIT();

#if  (CFG_APP_HP_IUS)
    if(CURRENT_APP_START_ADDRESS == HP_APP1_START_ADDRESS){
        HP_LOG_INFO("application 1 start new ...\r\n");
    }else if(CURRENT_APP_START_ADDRESS == HP_APP2_START_ADDRESS){
        HP_LOG_INFO("application 2 start new ...\r\n");
    }
#endif
    
#if (HP_RF_CTRL_ENABLE)
    KeyInputExtiInit(KEY_INPUT_PORT, KEY_INPUT_PIN);
#endif ///HP_RF_CTRL_ENABLE

    app_ble_init();
    
    HP_LOG_INFO(DEMO_STRING);

    // periph init 
    LedInit(LED1_PORT,LED1_PIN);  // power led
    LedInit(LED2_PORT,LED2_PIN);  // connection state
    LedOn(LED1_PORT,LED1_PIN);    
    app_usart_dma_enable(ENABLE);
    //init text
    usart_tx_dma_send((uint8_t*)DEMO_STRING, sizeof(DEMO_STRING)); 

    delay_n_10us(500);
    
    //disable usart for enter sleep
    app_usart_dma_enable(DISABLE);   

    while (1)
    {
        /*schedule all pending events*/
        rwip_schedule();
				hp_sleep();
    }
}

/**
 * @brief  user handle before enter sleep mode
 * @param  
 * @return 
 * @note   
 */
void app_sleep_prepare_proc(void)
{

}

/**
 * @brief  user handle after wake up from sleep mode
 * @param  
 * @return 
 * @note   
 */
void app_sleep_resume_proc(void)
{

}

/**
 * @}
 */

