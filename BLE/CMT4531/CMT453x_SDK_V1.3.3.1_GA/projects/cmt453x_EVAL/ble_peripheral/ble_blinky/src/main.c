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
#include "app_ble.h"
#include "app_lpuart.h"
#include "app_gpio.h"
#include "app_rtc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DEMO_STRING  "\r\n HopeRF device low power timer demo \r\n"
#define LPT_PERIOD_MS  100

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Low power timer handler.
 * You can drive peripherals and run logic code, do not disable interrupt here.
 * You send message but not set up ke_timer or hp_timer here. The rwip_schedule not 
 * active yet, the message you sent will handle after the rwip_schedule wake-up. 
 * You can wake-up the rwip_schedule manully with function hp_sleep_lp_timer_wake_ble.
 */
void user_lpt_wakeup_cb(void)
{
    LedBlink(LED1_PORT,LED1_PIN);
    #if 0
    // active rwip_schedule if necessary     
    hp_sleep_lp_timer_wake_ble();
    #endif
}

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
    app_ble_init();
    
    HP_LOG_INFO(DEMO_STRING);
    HP_LOG_INFO("advertising interval:%dms\r\n",CUSTOM_ADV_FAST_INTERVAL*5/8);
    HP_LOG_INFO("low power timer interval:%dms\r\n",LPT_PERIOD_MS);
    
    LedInit(LED1_PORT,LED1_PIN);
    LedInit(LED2_PORT,LED2_PIN);    
    app_key_configuration();
    app_lpuart_init();
    app_lpuart_send_data("lpuart_test\r\n",13);
    RTC_Configuration();    

    #ifdef SLEEP_LP_TIMER_ENABLE
    hp_sleep_lp_timer_config(LPT_PERIOD_MS,user_lpt_wakeup_cb);
    #endif

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
    app_key_reinit_after_sleep();
}



/**
 * @}
 */

