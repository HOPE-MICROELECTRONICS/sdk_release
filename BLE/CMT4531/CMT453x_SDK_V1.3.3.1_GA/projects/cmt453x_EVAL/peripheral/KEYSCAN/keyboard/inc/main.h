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
 * @file main.h
 
 * @version v1.0.0
 *
  */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmt453x.h"
#include "log.h"
#include "string.h"
#define  KEYSCAN_INFO_CHECK         2
static const uint32_t keyscan_map[][KEYSCAN_INFO_CHECK] = 
{
    0x00,0x00,               //NULL
    0x00000000,0x00000001,
    0x00000000,0x00000002,
    0x00000000,0x00000004,
    0x00000000,0x00000008,
    0x00000000,0x00000010,
    0x00000000,0x00000020,
    0x00000000,0x00000040,
    0x00000000,0x00000080,
    0x00000000,0x00000100,
    0x00000000,0x00000200,
    0x00000000,0x00000400,
    0x00000000,0x00000800,
    0x00000000,0x00001000,
    0x00000000,0x00002000,
    0x00000000,0x00004000,
    0x00000000,0x00010000,
    0x00000000,0x00020000,
    0x00000000,0x00040000,
    0x00000000,0x00080000,
    0x00000000,0x00100000,
    0x00000000,0x00200000,
    0x00000000,0x00400000,
    0x00000000,0x00800000,
    0x00000000,0x01000000,
    0x00000000,0x02000000,
    0x00000000,0x04000000,
    0x00000000,0x08000000,
    0x00000000,0x10000000,
    0x00000001,0x00000000,
    0x00000002,0x00000000,
    0x00000004,0x00000000,
    0x00000008,0x00000000,
    0x00000010,0x00000000,
    0x00000020,0x00000000,
    0x00000040,0x00000000,
    0x00000080,0x00000000,
    0x00010000,0x00000000,
    0x00020000,0x00000000,
    0x00040000,0x00000000,
    0x00080000,0x00000000,
    0x00100000,0x00000000,
    0x00200000,0x00000000,
    0x00400000,0x00000000,
    0x00800000,0x00000000,

    0x00,0x00,              //NULL

};

enum KEY_NUM{
    KEYSCAN_NULL = 0,
    
    KEYSCAN_KEY1,
    KEYSCAN_KEY2,
    KEYSCAN_KEY3,
    KEYSCAN_KEY4,
    KEYSCAN_KEY5,
    KEYSCAN_KEY6,
    KEYSCAN_KEY7,
    KEYSCAN_KEY8,
    KEYSCAN_KEY9, 
    KEYSCAN_KEY10,
    KEYSCAN_KEY11,
    KEYSCAN_KEY12,
    KEYSCAN_KEY13,
    KEYSCAN_KEY14,
    KEYSCAN_KEY15,
    KEYSCAN_KEY16, 
    KEYSCAN_KEY17,
    KEYSCAN_KEY18,
    KEYSCAN_KEY19,
    KEYSCAN_KEY20,
    KEYSCAN_KEY21,
    KEYSCAN_KEY22,
    KEYSCAN_KEY23,
    KEYSCAN_KEY24,
    KEYSCAN_KEY25,
    KEYSCAN_KEY26,
    KEYSCAN_KEY27,
    KEYSCAN_KEY28,
    KEYSCAN_KEY29,
    KEYSCAN_KEY30,
    KEYSCAN_KEY31,
    KEYSCAN_KEY32,
    KEYSCAN_KEY33,
    KEYSCAN_KEY34,
    KEYSCAN_KEY35,
    KEYSCAN_KEY36,
    KEYSCAN_KEY37,
    KEYSCAN_KEY38,
    KEYSCAN_KEY39,
    KEYSCAN_KEY40,
    KEYSCAN_KEY41,
    KEYSCAN_KEY42,
    KEYSCAN_KEY43,
    KEYSCAN_KEY44,

    KEYSCAN_KEY_MAX
}; 


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
