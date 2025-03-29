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
 * @file log.h
 
 * @version v1.0.0
 *
  */
#ifndef __LOG_H__
#define __LOG_H__

#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

#if LOG_ENABLE

#include <stdio.h>

#define LOG_NONE    0
#define LOG_ERROR   10
#define LOG_WARNING 20
#define LOG_INFO    30
#define LOG_DEBUG   40

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_DEBUG
#endif

#if LOG_LEVEL >= LOG_INFO
#define log_info(...) printf(__VA_ARGS__)
#else
#define log_info(...)
#endif

#if LOG_LEVEL >= LOG_ERROR
#define log_error(...) printf(__VA_ARGS__)
#else
#define log_error(...)
#endif

#if LOG_LEVEL >= LOG_WARNING
#define log_warning(...) printf(__VA_ARGS__)
#else
#define log_warning(...)
#endif

#if LOG_LEVEL >= LOG_DEBUG
#define log_debug(...) printf(__VA_ARGS__)
#else
#define log_debug(...)
#endif

void log_init(void);
void Delay(unsigned int count);
void Delay_ms(unsigned int count);
void log_buff(unsigned char *data, int len);
    
#else /* !LOG_ENABLE */

#define log_info(...)
#define log_warning(...)
#define log_error(...)
#define log_debug(...)
#define log_init()

#endif

#define log_func() log_debug("call %s\r\n", __FUNCTION__)

#endif /* __LOG_H__ */
