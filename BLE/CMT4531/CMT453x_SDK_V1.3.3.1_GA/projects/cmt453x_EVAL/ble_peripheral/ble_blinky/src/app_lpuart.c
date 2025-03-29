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
 * @file app_lpuart.c
 * @version v1.0.0
 *
  */
 
/* NOTE:
 Please use 32768HZ LSI(or LSE) to support 9600 baud. 
 Add "app_handler.lsc_cfg = BLE_LSC_LSI_32768HZ;" when functon call hp_ble_stack_init.
 Define LSI_TRIM_32768HZ as global macro.
 */

#include "cmt453x.h"
#include "app_lpuart.h"
#include "hp_timer.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LPUARTy                 LPUART1
#define LPUARTy_CLK             RCC_LPUART1CLK
#define LPUARTy_GPIO            GPIOB
#define LPUARTy_GPIO_CLK        RCC_APB2_PERIPH_GPIOB
#define LPUARTy_RxPin           GPIO_PIN_11
#define LPUARTy_TxPin           GPIO_PIN_12
#define LPUARTy_Rx_GPIO_AF      GPIO_AF2_LPUART1
#define LPUARTy_Tx_GPIO_AF      GPIO_AF2_LPUART1
#define LPUARTy_IRQn            LPUART1_IRQn
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char Rx_data[256] = {0};
unsigned char Rx_len = 0;
timer_hnd_t lpuart_timer_hdl = 0xff;
/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configuration(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  LPUART demo code
 * @param  
 * @return 
 * @note   
 */
void app_lpuart_output_demo(void)
{
    static unsigned char tx_idx = 0;
    while(tx_idx != Rx_len)
    {
        app_lpuart_send_data(&Rx_data[tx_idx],1);
        tx_idx++;
    }
    lpuart_timer_hdl = 0xff;
}



/**
 * @brief  LPUART initialization
 * @param  
 * @return 
 * @note   
 */
void app_lpuart_init(void)
{
	LPUART_InitType LPUART_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	EXTI_InitType EXTI_InitStructure;
    
    /* Specifies the LPUARTy clock source, LSE selected as LPUARTy clock */
    RCC_ConfigLpuartClk(LPUARTy_CLK, RCC_LPUARTCLK_SRC_LSI_LSE);
    /* Enable LPUARTy Clock */
    RCC_EnableLpuartClk(ENABLE);   
    
    /* Enable the LPUARTx Wakeup Interrupt through EXTI line 10 */
    EXTI_InitStructure.EXTI_Line	= EXTI_LINE10;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
    
	/* Enable the LPUART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = LPUARTy_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable the LPUART Wakeup Interrupt through EXTI line 10 */
	NVIC_InitStructure.NVIC_IRQChannel                  = EXTI4_12_IRQn;
	NVIC_Init(&NVIC_InitStructure);
    
    /* Configure GPIO for LPUART */
    GPIO_Configuration();

    /* USARTx and USARTz configuration ------------------------------------------------------*/
    LPUART_InitStructure.BaudRate            = 9600; 
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
	LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOHF;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    LPUART_InitStructure.Mode                = LPUART_MODE_RX | LPUART_MODE_TX;

    /* Configure USARTx */
    LPUART_Init(LPUARTy, &LPUART_InitStructure);
	/* Flush LPUARTy RX FIFO */
	LPUART_FlushRxFifo(LPUARTy);
    
    LPUART_ConfigInt(LPUARTy, LPUART_INT_FIFO_NE, ENABLE);
    
    /* Set the Wake-Up Event: specify wake-up on start bit */
    LPUART_ConfigWakeUpMethod(LPUARTy, LPUART_WUSTP_STARTBIT );

    /* Enable MCU Wake-up by LPUARTx */
    LPUART_EnableWakeUpStop(LPUARTy, ENABLE);
}


/**
 * @brief  LPUART disable
 * @param  
 * @return 
 * @note   
 */
void app_lpuart_deinit(void)
{
	GPIO_InitType GPIO_InitStructure;

	RCC_EnableLpuartClk(DISABLE); 
	        
    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    
    /* Configure USARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTy_TxPin|LPUARTy_RxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF0;
    GPIO_InitPeripheral(LPUARTy_GPIO, &GPIO_InitStructure); 
}

void app_lpuart_receive_data(uint8_t *p_data,uint16_t len)
{
    while(len--)
    {
        Rx_data[Rx_len++] = *p_data;
        p_data++;
    }
    
    #if 1 //enable output demo
    if(lpuart_timer_hdl == 0xff)
    {
        lpuart_timer_hdl = hp_timer_create(10,app_lpuart_output_demo);
    }
    else{
        hp_timer_modify(lpuart_timer_hdl,10);
    }
    #endif
}
/**
 * @brief  LPUART send data 
 * @param  
 * @return 
 * @note   
 */
void app_lpuart_send_data(const uint8_t *data,uint16_t len)
{
	uint16_t i = 0;
	for(i = 0; i < len;i++)
	{
		LPUART_SendData(LPUARTy,data[i]);
		while (LPUART_GetFlagStatus(LPUARTy, LPUART_FLAG_TXC) == RESET);
		/* must clear the TXC flag */
		LPUART_ClrFlag(LPUARTy, LPUART_FLAG_TXC); 
	}
}

/**
 * @brief  LPUART GPIO configuration
 * @param  
 * @return 
 * @note   
 */
static void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTy_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);    

    /* Configure LPUARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTy_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTy_Tx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTy_GPIO, &GPIO_InitStructure);

    /* Configure LPUARTy Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = LPUARTy_RxPin;  
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTy_Rx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTy_GPIO, &GPIO_InitStructure);
}


/**
 * @}
 */
