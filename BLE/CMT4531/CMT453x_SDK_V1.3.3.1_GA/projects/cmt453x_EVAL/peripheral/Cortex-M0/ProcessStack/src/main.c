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
 * @version v1.0.0
 *
  */
#include "main.h"
#include <stdio.h>
#include "log.h"

/**
 *  Cortex-M0 ProcessStack
 */

#define DEMO_USART_BAUDRATE ((uint32_t)115200)

#define SP_PROCESS_SIZE          0x200 /* Process stack size */
#define SP_PROCESS               0x02  /* Process stack */
#define SP_MAIN                  0x00  /* Main stack */

/* clang-format off */
#if defined ( __CC_ARM   )
  __ASM void __SVC(void) 
  { 
    SVC 0x01 
    BX R14
  }
#elif defined ( __ICCARM__ )
  static __INLINE  void __SVC()                     { __ASM ("svc 0x01");}
#elif defined   (  __GNUC__  )
  static __INLINE void __SVC()                      { __ASM volatile ("svc 0x01");}
#endif
/* clang-format on */

__IO uint8_t PSPMemAlloc[SP_PROCESS_SIZE];
__IO uint32_t Index = 0, PSPValue = 0, CurrentStack = 0, IsrStack = 0xFF;

/**
 * @brief  Main program.
 */
int main(void)
{
    /* log Init */
    log_init();
    printf("Cortex-M0 ProcessStack \r\n");

    /* Switch Thread mode Stack from Main to Process */
    /* Initialize memory reserved for Process Stack */
    for (Index = 0; Index < SP_PROCESS_SIZE; Index++)
    {
        PSPMemAlloc[Index] = 0x00;
    }

    /* Set Process stack value */
    __set_PSP((uint32_t)PSPMemAlloc + SP_PROCESS_SIZE);

    /* Select Process Stack as Thread mode Stack */
    __set_CONTROL(SP_PROCESS);

    /* Execute ISB instruction to flush pipeline as recommended by Arm */
    __ISB();

    /* Get the Thread mode stack used */
    CurrentStack = (__get_CONTROL() & 0x02);

    /* Get process stack pointer value */
    PSPValue = __get_PSP();
  
    /* Check is mode has been well applied */
    if(CurrentStack != SP_PROCESS)
    {
        printf("Cortex-M0 ProcessStack Test Error 0 \r\n");
        while(1);
    }

    /* Generate a system call exception: Main Stack pointer should be automaticcaly 
    when entering in ISR context */
    __SVC();

    /* Check is Main stack was used under ISR*/
    if(IsrStack != SP_MAIN)
    {
        printf("Cortex-M0 ProcessStack Test Error 1 \r\n");
        while(1);
    }

    /* Get the Thread mode stack used to verify we have switched back automatically 
    to Process Stack */
    
    CurrentStack = (__get_CONTROL() & 0x02);
    
    /* Check is mode has been well applied */
    if(CurrentStack != SP_PROCESS)
    {
        printf("Cortex-M0 ProcessStack Test Error 2 \r\n");
    }
    else
    {
        printf("Cortex-M0 ProcessStack Test Success \r\n");
    }

    while (1)
    {
    }
}

/**
 * @}
 */

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
*          line: assert_param error line source number
 * @return None
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}

/**
 * @}
 */
#endif
