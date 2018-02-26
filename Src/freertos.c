/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include <stdbool.h>
#include "spi.h"
#include "usart.h"
#include "mcb/mcb.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId HspTaskHandle;
uint32_t HspBuffer[ 256 ];
osStaticThreadDef_t HspControlBlock;
osThreadId UserTaskHandle;
uint32_t UserTaskBuffer[ 1024 ];
osStaticThreadDef_t UserTaskControlBlock;
osMessageQId HspTxHandle;
uint8_t HspTxBuffer[ 16 * sizeof( frm_t* ) ];
osStaticMessageQDef_t HspTxControlBlock;
osMessageQId HspRxHandle;
uint8_t HspRxBuffer[ 16 * sizeof( frm_t* ) ];
osStaticMessageQDef_t HspRxControlBlock;
osMutexId CommsMuxHandle;
osStaticMutexDef_t CommsMuxControlBlock;

/* USER CODE BEGIN Variables */
extern volatile unsigned long ulHighFrequencyTimerTicks;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void HspFunc(void const * argument);
void StartUserTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of CommsMux */
  osMutexStaticDef(CommsMux, &CommsMuxControlBlock);
  CommsMuxHandle = osMutexCreate(osMutex(CommsMux));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of HspTask */
  osThreadStaticDef(HspTask, HspFunc, osPriorityNormal, 0, 256, HspBuffer, &HspControlBlock);
  HspTaskHandle = osThreadCreate(osThread(HspTask), NULL);

  /* definition and creation of UserTask */
  osThreadStaticDef(UserTask, StartUserTask, osPriorityLow, 0, 1024, UserTaskBuffer, &UserTaskControlBlock);
  UserTaskHandle = osThreadCreate(osThread(UserTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of HspTx */
  osMessageQStaticDef(HspTx, 16, frm_t*, HspTxBuffer, &HspTxControlBlock);
  HspTxHandle = osMessageCreate(osMessageQ(HspTx), NULL);

  /* definition and creation of HspRx */
  osMessageQStaticDef(HspRx, 16, frm_t*, HspRxBuffer, &HspRxControlBlock);
  HspRxHandle = osMessageCreate(osMessageQ(HspRx), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* HspFunc function */
void HspFunc(void const * argument)
{
  /* USER CODE BEGIN HspFunc */
	osEvent MsgOut;
	char cString[16];
	/** SPI initialization */
	McbInst dvr;

	memset(cString, 0, 16);
	mcb_init(&dvr, MCB_OVER_SPI, MCB_BLOCKING);

	/* Infinite loop */
	for(;;)
	{
		/** Task is locked until a request to Mcb interface
		* is requested.
		*/
		MsgOut = osMessageGet(HspTxHandle, osWaitForever);

		if (MsgOut.status == osEventMessage)
		{
			frm_t* pframe = (frm_t*) MsgOut.value.p;
			switch (frame_get_cmd(pframe))
			{
				case HSP_REQ_READ:
					mcb_read(&dvr, frame_get_addr(pframe), &(pframe->buf[1]), pframe->sz);
					break;
				case HSP_REQ_WRITE:
					mcb_write(&dvr, frame_get_addr(pframe),  &(pframe->buf[1]), pframe->sz);
				  break;
				default:
					/** Nothing */
				  break;
			}
			sprintf(cString, "Status: %u\n\r", pframe->buf[1]);
			CDC_Transmit_FS((uint8_t*)cString, strlen(cString));
			osMessagePut(HspRxHandle, (uint32_t)pframe, osWaitForever);
		}
	}
  /* USER CODE END HspFunc */
}

/* StartUserTask function */
void StartUserTask(void const * argument)
{
  /* USER CODE BEGIN StartUserTask */
  uint32_t u32Millis;
  osEvent MsgOut;
  frm_t frame;

  u32Millis = HAL_GetTick();
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
	  MsgOut = osMessageGet(HspRxHandle, 10);

	  if (MsgOut.status == osEventMessage)
	  {
		  HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
	  }


	  if ((HAL_GetTick() - u32Millis) > 1000)
	  {
		  frame_create(&frame, 0x11, HSP_REQ_READ, HSP_FRM_NOTSEG, NULL, NULL, 0);
		  osMessagePut(HspTxHandle, (uint32_t)&frame, osWaitForever);
		  HAL_GPIO_TogglePin(GPIOD, LD3_Pin|LD5_Pin|LD6_Pin);
		  u32Millis = HAL_GetTick();
	  }
	  osDelay(1);
  }
  /* USER CODE END StartUserTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
