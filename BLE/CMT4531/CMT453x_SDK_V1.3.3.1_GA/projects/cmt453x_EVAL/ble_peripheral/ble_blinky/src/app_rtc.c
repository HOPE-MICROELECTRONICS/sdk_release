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
 * @file app_rtc.c
 * @version v1.0.0
 *
  */
#include "app_rtc.h"
#include "hp_log.h"
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define RCT_LOG_ENABLE
#ifdef RCT_LOG_ENABLE
#define log_info HP_LOG_INFO
#else
#define log_info(str, ...) 
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
ErrorStatus RTC_DateTimeRegulate(void);
ErrorStatus RTC_AlarmRegulate(void);
void EXTI9_RTCWKUP_Configuration(FunctionalState Cmd);
void EXTI8_RTCAlarm_Configuration(FunctionalState Cmd);
void WakeUpClockSelect(uint8_t WKUPClkSrcSel);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  app rtc init
 */
void RTC_Configuration(void)
{
    RTC_InitType RTC_InitStructure;
    /* Enable the RTC Clock */
    RCC_EnableRtcClk(ENABLE);
    RTC_WaitForSynchro();    
    if((RTC->INITSTS&RTC_INITSTS_INITSF) == 0)
    {
        /* Disable RTC Wakeup and before  */
        RTC_EnableWakeUp(DISABLE);
        RTC_ConfigInt(RTC_INT_WUT, DISABLE);   
        /* Configure the RTC data register and RTC prescaler */
        RTC_InitStructure.RTC_AsynchPrediv = 0x7D; // 7f:23768 , 7d:32k
        RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
        RTC_InitStructure.RTC_HourFormat   = RTC_24HOUR_FORMAT;
        /* Check on RTC init */
        if (RTC_Init(&RTC_InitStructure) == ERROR)
        {
            log_info(">> !! RTC Prescaler Config failed. !! <<\r\n");
        }
        else{
            log_info(">> !! RTC Prescaler Config success. !! <<\r\n");
        }
        /* Config date and time by default value */
        RTC_DateTimeRegulate();
    }
    else{
        log_info(">> !! RTC has been inited. !! <<\r\n");
    }
    /* Show RTC date and time */
    RTC_DateShow();
    RTC_TimeShow();
    
    /* Low power timer has used RTC wakeup. */
    #ifndef SLEEP_LP_TIMER_ENABLE
    RTC_EnableWakeUp(DISABLE);
    RTC_ConfigInt(RTC_INT_WUT, DISABLE);   
    /* wake up clock select */
    WakeUpClockSelect(1);
    /* wake up timer value */
    RTC_SetWakeUpCounter( 1000 );//400->200ms ,1000->500ms
    
    EXTI9_RTCWKUP_Configuration(ENABLE);
    /* Enable the RTC Wakeup Interrupt */
    RTC_ConfigInt(RTC_INT_WUT, ENABLE);    
    RTC_EnableWakeUp(ENABLE);
    #endif
    
    RTC_AlarmRegulate();
    EXTI8_RTCAlarm_Configuration(ENABLE);
    RTC_AlarmShow();
}


/**
 * @brief  Wake up clock config.
 */
void WakeUpClockSelect(uint8_t WKUPClkSrcSel)
{
    /* Configure the RTC WakeUp Clock source: CK_SPRE (1Hz) */
    if (WKUPClkSrcSel == 0x01)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV16);
    else if (WKUPClkSrcSel == 0x02)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV8);
    else if (WKUPClkSrcSel == 0x03)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV4);
    else if (WKUPClkSrcSel == 0x04)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV2);
    else if (WKUPClkSrcSel == 0x05)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_16BITS);
    else if (WKUPClkSrcSel == 0x06)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_17BITS);
}
/**
 * @brief  Config RTC wake up Interrupt.
 */
void EXTI9_RTCWKUP_Configuration(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;

    EXTI_ClrITPendBit(EXTI_LINE9);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE9;
#ifdef __TEST_SEVONPEND_WFE_NVIC_DIS__
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
#else
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#endif
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /* Enable the RTC WakeUp Interrupt */
    NVIC_EnableIRQ(RTC_IRQn);
}

/**
 * @brief  Config RTC Alarm Interrupt.
 */
void EXTI8_RTCAlarm_Configuration(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;

    EXTI_ClrITPendBit(EXTI_LINE8);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE8;
#ifdef __TEST_SEVONPEND_WFE_NVIC_DIS__
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
#else
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#endif
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /* Enable the RTC Interrupt */
    NVIC_EnableIRQ(RTC_IRQn);
}

/**
 * @brief  Display the current Date on the Hyperterminal.
 */
void RTC_DateShow(void)
{
    RTC_DateType RTC_DateStructure;

    /* Get the current Date */
    RTC_GetDate(RTC_FORMAT_BIN, &RTC_DateStructure);
    log_info("The current date (WeekDay-Date-Month-Year) is : %0.2d-%0.2d-%0.2d-%0.2d\r\n",
             RTC_DateStructure.WeekDay,
             RTC_DateStructure.Date,
             RTC_DateStructure.Month,
             RTC_DateStructure.Year);
}

/**
 * @brief  Display the current time on the Hyperterminal.
 */
void RTC_TimeShow(void)
{
    RTC_TimeType RTC_TimeStructure;

    /* Get the current Time and Date */
    RTC_GetTime(RTC_FORMAT_BIN, &RTC_TimeStructure);
    log_info("The current time (Hour-Minute-Second) is : %0.2d:%0.2d:%0.2d \r\n",
             RTC_TimeStructure.Hours,
             RTC_TimeStructure.Minutes,
             RTC_TimeStructure.Seconds);
    /* Unfreeze the RTC DAT Register */
    (void)RTC->DATE;
}

/**
 * @brief  Display the current alarm time on the Hyperterminal.
 */
void RTC_AlarmShow(void)
{
    RTC_AlarmType RTC_AlarmStructure;
    /* Get the current Alarm */
    RTC_GetAlarm(RTC_FORMAT_BIN, RTC_A_ALARM, &RTC_AlarmStructure);
    printf("The current alarm is :  %0.2d:%0.2d:%0.2d \r\n",
           RTC_AlarmStructure.AlarmTime.Hours,
           RTC_AlarmStructure.AlarmTime.Minutes,
           RTC_AlarmStructure.AlarmTime.Seconds);
}
/**
 * @brief  RTC date and time regulate with the default value.
 */
ErrorStatus RTC_DateTimeRegulate(void)
{
    RTC_TimeType RTC_TimeDefault;
    RTC_DateType RTC_DateDefault;
    // Date
    RTC_DateDefault.WeekDay = 3;
    RTC_DateDefault.Date    = 20;
    RTC_DateDefault.Month   = 11;
    RTC_DateDefault.Year    = 19;
    // Time
    RTC_TimeDefault.H12     = RTC_AM_H12;
    RTC_TimeDefault.Hours   = 4;
    RTC_TimeDefault.Minutes = 5;
    RTC_TimeDefault.Seconds = 1;    
    
    
    /* Configure the RTC date register */
    if (RTC_SetDate(RTC_FORMAT_BIN, &RTC_DateDefault) == ERROR)
    {
        log_info(">> !! RTC Set Date failed. !! <<\r\n");
        return ERROR;
    }

    /* Configure the RTC time register */
    if (RTC_ConfigTime(RTC_FORMAT_BIN, &RTC_TimeDefault) == ERROR)
    {
        log_info(">> !! RTC Set Time failed. !! <<\r\n");
        return ERROR;
    }
    return SUCCESS;
    
}



/**
 * @brief  RTC alarm regulate with the default value.
 */
ErrorStatus RTC_AlarmRegulate(void)
{
    RTC_AlarmType RTC_AlarmDefault;
    /* Disable the AlarmA */
    RTC_EnableAlarm(RTC_A_ALARM, DISABLE);
    /* Set the Alarm A */
    RTC_AlarmDefault.AlarmTime.H12     = RTC_AM_H12;
    RTC_AlarmDefault.AlarmTime.Hours   = 4;
    RTC_AlarmDefault.AlarmTime.Minutes = 23;
    RTC_AlarmDefault.AlarmTime.Seconds = 05;
    RTC_AlarmDefault.DateWeekMode      = RTC_ALARM_SEL_WEEKDAY_DATE;
    RTC_AlarmDefault.DateWeekValue     = 0x31;
    RTC_AlarmDefault.AlarmMask       = RTC_ALARMMASK_WEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;

    /* Configure the RTC Alarm A register */
    RTC_SetAlarm(RTC_FORMAT_BIN, RTC_A_ALARM, &RTC_AlarmDefault);
    
    /* Enable the RTC Alarm A Interrupt */
    RTC_ConfigInt(RTC_INT_ALRA, ENABLE);
    /* Enable the alarm   */
    RTC_EnableAlarm(RTC_A_ALARM, ENABLE);
   
    return SUCCESS;
}

/**
 * @}
 */
