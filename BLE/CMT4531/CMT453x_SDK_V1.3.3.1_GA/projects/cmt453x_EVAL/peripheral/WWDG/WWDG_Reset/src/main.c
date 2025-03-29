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
#include "main.h"
#include "cmt453x.h"
#include "log.h"

/** @addtogroup cmt453x_StdPeriph_Examples
 * @{
 */

/** @addtogroup WWDG_Reset
 * @{
 */

__IO uint32_t TimingDelay = 0;

static void Delay(__IO uint32_t nTime);

/**
 * @brief  Configures LED GPIO.
 * @param Led Specifies the Led to be configured.
 *   This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    
    /* Configure the GPIO pin */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin        = Pin;
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}

/**
 * @brief  Turns selected Led on.
 * @param Led Specifies the Led to be set on.
 */
void LedOn(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBSC = Pin;
}
/**
 * @brief  Turns selected Led Off.
 * @param Led Specifies the Led to be set off.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->PBC = Pin;
}

/**
 * @brief  Toggles the selected Led.
 * @param Led Specifies the Led to be toggled.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_cmt453x.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_cmt453x.c file
       */
	RCC_ClocksType RCC_Clocks;
	
	log_init();
	printf("\r\n This is WWDG Reset demo\r\n");
	
	WWDG_ClrEWINTF();
	
    LedInit(LED1_PORT, LED1_PIN);
	LedInit(LED2_PORT, LED2_PIN);
	
    LedOff(LED1_PORT, LED1_PIN);
	LedOff(LED2_PORT, LED2_PIN);
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	 /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1)
            ;
    }
	
	/* Enable PWR Clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
	DBG_ConfigPeriph(DBG_WWDG_STOP, ENABLE);
	
    /* Check if the system has resumed from WWDG reset */
    if (RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_WWDGRSTF) != RESET)
    {
        /* WWDGRST flag set, Turn on LED2*/
        LedOn(LED1_PORT, LED1_PIN);
		log_info("reset by WWDG\n");
        /* Clear reset flags */
        RCC_ClrFlag();
    }
    else
    {
        /* WWDGRST flag is not set, Turn off LED2*/
        LedOff(LED1_PORT, LED1_PIN);
    }

    /* WWDG configuration */
    /* Enable WWDG clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_WWDG, ENABLE);
	
	RCC_GetClocksFreqValue(&RCC_Clocks);
	printf("Pclk1Freq:%d\r\n", RCC_Clocks.Pclk1Freq);

    /* WWDG clock counter = 4096/(PCLK1(32MHz)/4) = (~512 us)  */
    WWDG_SetPrescalerDiv(WWDG_PRESCALER_DIV4);

    /* Set Window value to 80; WWDG counter should be refreshed only when the counter
      is below 80 (and greater than 64) otherwise a reset will be generated */
    WWDG_SetWValue(80);

    /*
      Enable WWDG and set counter value to 127, WWDG timeout = ~512 us * 64 = 32.768 ms
      In this case the refresh window is: ~512 us * (127-80) = 24.064 ms < refresh window < ~512 us * (127-64-1) = 32.768ms
    */
    WWDG_Enable(127);

    while (1)
    {
        /* Toggle LED2 */
        LedBlink(LED2_PORT, LED2_PIN);
        Delay(28);
        /* Update WWDG counter */
			
        WWDG_SetCnt(127);
    }
}

/**
 * @brief  Inserts a delay time.
 * @param nTime specifies the delay time length, in milliseconds.
 */
static void Delay(__IO uint32_t nTime)
{
    TimingDelay = nTime;
    while (TimingDelay != 0)
    {
    }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

/**
 * @}
 */

/**
 * @}
 */
