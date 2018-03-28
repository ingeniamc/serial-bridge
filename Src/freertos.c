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
osThreadId McbSlaveTaskHandle;
uint32_t McbSlaveTaskBuffer[ 1024 ];
osStaticThreadDef_t McbSlaveTaskControlBlock;
osThreadId BridgeTaskHandle;
uint32_t BridgeTaskBuffer[ 1024 ];
osStaticThreadDef_t BridgeTaskControlBlock;
osMessageQId HspTxHandle;
uint8_t HspTxBuffer[ 16 * sizeof( McbMssg* ) ];
osStaticMessageQDef_t HspTxControlBlock;
osMessageQId HspRxHandle;
uint8_t HspRxBuffer[ 16 * sizeof( McbMssg* ) ];
osStaticMessageQDef_t HspRxControlBlock;
osMessageQId UartSlaveTxHandle;
uint8_t UartSlaveTxBuffer[ 16 * sizeof( McbMssg* ) ];
osStaticMessageQDef_t UartSlaveTxControlBlock;
osMessageQId UartSlaveRxHandle;
uint8_t UartSlaveRxBuffer[ 16 * sizeof( McbMssg* ) ];
osStaticMessageQDef_t UartSlaveRxControlBlock;
osMutexId CommsMuxHandle;
osStaticMutexDef_t CommsMuxControlBlock;

/* USER CODE BEGIN Variables */
extern volatile unsigned long ulHighFrequencyTimerTicks;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void HspFunc(void const * argument);
void StartUserTask(void const * argument);
void StartMcbSlaveTask(void const * argument);
void StartBridgeTask(void const * argument);

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
  osThreadStaticDef(HspTask, HspFunc, osPriorityLow, 0, 256, HspBuffer, &HspControlBlock);
  HspTaskHandle = osThreadCreate(osThread(HspTask), NULL);

  /* definition and creation of UserTask */
  osThreadStaticDef(UserTask, StartUserTask, osPriorityIdle, 0, 1024, UserTaskBuffer, &UserTaskControlBlock);
  UserTaskHandle = osThreadCreate(osThread(UserTask), NULL);

  /* definition and creation of McbSlaveTask */
  osThreadStaticDef(McbSlaveTask, StartMcbSlaveTask, osPriorityBelowNormal, 0, 1024, McbSlaveTaskBuffer, &McbSlaveTaskControlBlock);
  McbSlaveTaskHandle = osThreadCreate(osThread(McbSlaveTask), NULL);

  /* definition and creation of BridgeTask */
  osThreadStaticDef(BridgeTask, StartBridgeTask, osPriorityLow, 0, 1024, BridgeTaskBuffer, &BridgeTaskControlBlock);
  BridgeTaskHandle = osThreadCreate(osThread(BridgeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of HspTx */
  osMessageQStaticDef(HspTx, 16, McbMssg*, HspTxBuffer, &HspTxControlBlock);
  HspTxHandle = osMessageCreate(osMessageQ(HspTx), NULL);

  /* definition and creation of HspRx */
  osMessageQStaticDef(HspRx, 16, McbMssg*, HspRxBuffer, &HspRxControlBlock);
  HspRxHandle = osMessageCreate(osMessageQ(HspRx), NULL);

  /* definition and creation of UartSlaveTx */
  osMessageQStaticDef(UartSlaveTx, 16, McbMssg*, UartSlaveTxBuffer, &UartSlaveTxControlBlock);
  UartSlaveTxHandle = osMessageCreate(osMessageQ(UartSlaveTx), NULL);

  /* definition and creation of UartSlaveRx */
  osMessageQStaticDef(UartSlaveRx, 16, McbMssg*, UartSlaveRxBuffer, &UartSlaveRxControlBlock);
  UartSlaveRxHandle = osMessageCreate(osMessageQ(UartSlaveRx), NULL);

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
	/** SPI initialization */
	McbInst dvrMaster;
	mcb_init(&dvrMaster, MCB_OVER_SPI, MCB_MASTER, MCB_BLOCKING);

	/* Infinite loop */
	for(;;)
	{
		/** Task is locked until a request to Mcb interface
		* is requested.
		*/
		MsgOut = osMessageGet(HspTxHandle, osWaitForever);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
		if (MsgOut.status == osEventMessage)
		{
			McbMssg* pMcbMssg = (McbMssg*)MsgOut.value.p;

			switch (pMcbMssg->cmd)
			{
				case HSP_REQ_READ:
					pMcbMssg->eStatus = mcb_read(&dvrMaster, pMcbMssg);
					break;
				case HSP_REQ_WRITE:
					pMcbMssg->eStatus = mcb_write(&dvrMaster, pMcbMssg);
					pMcbMssg->size = 4;
					break;
				default:
					/** Nothing */
					break;
			}

			if (pMcbMssg->eStatus != MCB_MESSAGE_SUCCESS)
			{
				/* Error */
			}

			osMessagePut(HspRxHandle, (uint32_t)pMcbMssg, osWaitForever);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		}
	}
  /* USER CODE END HspFunc */
}

/* StartUserTask function */
void StartUserTask(void const * argument)
{
  /* USER CODE BEGIN StartUserTask */

  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
  }
  /* USER CODE END StartUserTask */
}

/* StartMcbSlaveTask function */
void StartMcbSlaveTask(void const * argument)
{
  /* USER CODE BEGIN StartMcbSlaveTask */
	osEvent MsgOut;
	uint32_t u32Millis = HAL_GetTick();
	McbInst dvrSlave;
	mcb_init(&dvrSlave, MCB_OVER_SERIAL, MCB_SLAVE, MCB_BLOCKING);

	McbMssg mcbMessg;
	/* Infinite loop */
	for(;;)
	{
		if ((HAL_GetTick() - u32Millis) > 1)
		{
			/* Chek for incoming uart message*/
			if (mcb_read(&dvrSlave, &mcbMessg) == MCB_MESSAGE_SUCCESS)
			{
				HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
				osMessagePut(UartSlaveTxHandle, (uint32_t)&mcbMessg, osWaitForever);

				MsgOut = osMessageGet(UartSlaveRxHandle, osWaitForever);
				if (MsgOut.status == osEventMessage)
				{
					McbMssg* pMcbSlaveMssg = (McbMssg*) MsgOut.value.p;
					if (mcb_write(&dvrSlave, pMcbSlaveMssg) != MCB_MESSAGE_SUCCESS)
					{
						/* ERROR */
					}
					else
					{
						HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
					}
				}
			}
			u32Millis = HAL_GetTick();
		}
		osDelay(1);
	}
  /* USER CODE END StartMcbSlaveTask */
}

/* StartBridgeTask function */
void StartBridgeTask(void const * argument)
{
  /* USER CODE BEGIN StartBridgeTask */
	osEvent MsgSlaveOut, MsgMasterOut;
	char cString[16];
	memset(cString, 0, 16);
	/* Infinite loop */
	for(;;)
	{
		MsgSlaveOut = osMessageGet(UartSlaveTxHandle, osWaitForever);

		if (MsgSlaveOut.status == osEventMessage)
		{
			McbMssg* pMcbSlaveMssg = (McbMssg*) MsgSlaveOut.value.p;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);

//			for(int i = 0; i < pMcbSlaveMssg->size; i+=4)
//			{
//				sprintf(cString, "BRIDGE	Uart readed, addr: %d, cmd: %d, i: %d, size: %d, data[0x%04X][0x%04X][0x%04X][0x%04X]\n\r",
//						pMcbSlaveMssg->addr, pMcbSlaveMssg->cmd, i, pMcbSlaveMssg->size, pMcbSlaveMssg->data[i+0], pMcbSlaveMssg->data[i+1], pMcbSlaveMssg->data[i+2], pMcbSlaveMssg->data[i+3]);
//				CDC_Transmit_FS((uint8_t*)cString, strlen(cString));
//			}

			osMessagePut(HspTxHandle, (uint32_t)pMcbSlaveMssg, osWaitForever);

			MsgMasterOut = osMessageGet(HspRxHandle, osWaitForever);

			if (MsgMasterOut.status == osEventMessage)
			{
				McbMssg* pMcbMasterMssg = (McbMssg*) MsgMasterOut.value.p;

//				for(int i = 0; i < pMcbMasterMssg->size; i+=4)
//				{
//					sprintf(cString, "BRIDGE Response	Uart readed, addr: %d, cmd: %d, i: %d, size: %d, data[0x%04X][0x%04X][0x%04X][0x%04X]\n\r",
//							pMcbMasterMssg->addr, pMcbMasterMssg->cmd, i, pMcbMasterMssg->size, pMcbMasterMssg->data[i+0], pMcbMasterMssg->data[i+1], pMcbMasterMssg->data[i+2], pMcbMasterMssg->data[i+3]);
//					CDC_Transmit_FS((uint8_t*)cString, strlen(cString));
//				}

				osMessagePut(UartSlaveRxHandle, (uint32_t)pMcbMasterMssg, osWaitForever);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
			}
		}

		osDelay(1);
	}
  /* USER CODE END StartBridgeTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
