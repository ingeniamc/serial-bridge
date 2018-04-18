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
#include "mcb.h"
#include "ipb.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId McbTaskHandle;
uint32_t McbBuffer[ 256 ];
osStaticThreadDef_t McbControlBlock;
osThreadId UserTaskHandle;
uint32_t UserTaskBuffer[ 1024 ];
osStaticThreadDef_t UserTaskControlBlock;
osThreadId IpbTaskHandle;
uint32_t IpbTaskBuffer[ 1024 ];
osStaticThreadDef_t IpbTaskControlBlock;
osThreadId BridgeTaskHandle;
uint32_t BridgeTaskBuffer[ 1024 ];
osStaticThreadDef_t BridgeTaskControlBlock;
osMessageQId McbTxHandle;
uint8_t McbTxBuffer[ 16 * sizeof( McbMsg* ) ];
osStaticMessageQDef_t McbTxControlBlock;
osMessageQId McbRxHandle;
uint8_t McbRxBuffer[ 16 * sizeof( McbMsg* ) ];
osStaticMessageQDef_t McbRxControlBlock;
osMessageQId IpbTxHandle;
uint8_t IpbTxBuffer[ 16 * sizeof( IpbMsg* ) ];
osStaticMessageQDef_t IpbTxControlBlock;
osMessageQId IpbRxHandle;
uint8_t IpbRxBuffer[ 16 * sizeof( IpbMsg* ) ];
osStaticMessageQDef_t IpbRxControlBlock;
osMutexId CommsMuxHandle;
osStaticMutexDef_t CommsMuxControlBlock;

/* USER CODE BEGIN Variables */
extern volatile unsigned long ulHighFrequencyTimerTicks;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void McbFunc(void const * argument);
void StartUserTask(void const * argument);
void IpbSlaveTask(void const * argument);
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

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
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

  /* definition and creation of McbTask */
  osThreadStaticDef(McbTask, McbFunc, osPriorityLow, 0, 256, McbBuffer, &McbControlBlock);
  McbTaskHandle = osThreadCreate(osThread(McbTask), NULL);

  /* definition and creation of UserTask */
  osThreadStaticDef(UserTask, StartUserTask, osPriorityIdle, 0, 1024, UserTaskBuffer, &UserTaskControlBlock);
  UserTaskHandle = osThreadCreate(osThread(UserTask), NULL);

  /* definition and creation of IpbTask */
  osThreadStaticDef(IpbTask, IpbSlaveTask, osPriorityBelowNormal, 0, 1024, IpbTaskBuffer, &IpbTaskControlBlock);
  IpbTaskHandle = osThreadCreate(osThread(IpbTask), NULL);

  /* definition and creation of BridgeTask */
  osThreadStaticDef(BridgeTask, StartBridgeTask, osPriorityLow, 0, 1024, BridgeTaskBuffer, &BridgeTaskControlBlock);
  BridgeTaskHandle = osThreadCreate(osThread(BridgeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of McbTx */
  osMessageQStaticDef(McbTx, 16, McbMsg*, McbTxBuffer, &McbTxControlBlock);
  McbTxHandle = osMessageCreate(osMessageQ(McbTx), NULL);

  /* definition and creation of McbRx */
  osMessageQStaticDef(McbRx, 16, McbMsg*, McbRxBuffer, &McbRxControlBlock);
  McbRxHandle = osMessageCreate(osMessageQ(McbRx), NULL);

  /* definition and creation of IpbTx */
  osMessageQStaticDef(IpbTx, 16, IpbMsg*, IpbTxBuffer, &IpbTxControlBlock);
  IpbTxHandle = osMessageCreate(osMessageQ(IpbTx), NULL);

  /* definition and creation of IpbRx */
  osMessageQStaticDef(IpbRx, 16, IpbMsg*, IpbRxBuffer, &IpbRxControlBlock);
  IpbRxHandle = osMessageCreate(osMessageQ(IpbRx), NULL);

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
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* McbFunc function */
void McbFunc(void const * argument)
{
    /* USER CODE BEGIN McbFunc */
    osEvent MsgOut;
    /** SPI initialization */
    McbInst dvrMaster;
    McbInit(&dvrMaster, MCB_BLOCKING);

    /* Infinite loop */
    for (;;)
    {
        MsgOut = osMessageGet(McbTxHandle, osWaitForever);
        if (MsgOut.status == osEventMessage)
        {
            McbMsg* pMcbMsg = (McbMsg*)MsgOut.value.p;

            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
            uint32_t u32NumTry = 0;
            do
            {
                switch (pMcbMsg->u16Cmd)
                {
                    case MCB_REQ_READ:
                        pMcbMsg->eStatus = McbRead(&dvrMaster, pMcbMsg,
                        DFLT_TIMEOUT);
                        break;
                    case MCB_REQ_WRITE:
                        pMcbMsg->eStatus = McbWrite(&dvrMaster, pMcbMsg,
                        DFLT_TIMEOUT);
                        break;
                    default:
                        pMcbMsg->eStatus = MCB_MESSAGE_ERROR;
                        break;
                }

                if (pMcbMsg->eStatus != MCB_MESSAGE_SUCCESS)
                {
                    /* Error */
                }

            } while ((pMcbMsg->eStatus != MCB_MESSAGE_SUCCESS) && ((u32NumTry++) < COMMS_NUM_TRY));

            osMessagePut(McbRxHandle, (uint32_t) pMcbMsg, osWaitForever);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
        }
    }
    /* USER CODE END McbFunc */
}

/* StartUserTask function */
void StartUserTask(void const * argument)
{
  /* USER CODE BEGIN StartUserTask */

    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END StartUserTask */
}

/* IpbSlaveTask function */
void IpbSlaveTask(void const * argument)
{
    /* USER CODE BEGIN IpbSlaveTask */
    osEvent MsgOut;
    IpbMsg ipbMsg;
    IpbMsg *pIpbMsg;
    IpbInst dvrSlave;

    IpbInit(&dvrSlave, UART_BASED, IPB_BLOCKING);

    HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
    /* Infinite loop */
    for (;;)
    {
        /* Chek for incoming uart message*/
        pIpbMsg = &ipbMsg;

        if (IpbRead(&dvrSlave, pIpbMsg, DFLT_TIMEOUT) == IPB_MESSAGE_SUCCESS)
        {
            if (pIpbMsg->u16Node == NODE)
            {
                osMessagePut(IpbTxHandle, (uint32_t) pIpbMsg,
                             osWaitForever);

                MsgOut = osMessageGet(IpbRxHandle, osWaitForever);
                if (MsgOut.status == osEventMessage)
                {
                    pIpbMsg = (IpbMsg*) MsgOut.value.p;
                    if (pIpbMsg->eStatus == IPB_MESSAGE_SUCCESS)
                    {
                        /** Do something */
                    }
                    else
                    {
                        /** Enable Orange led when driver comm fails */
                        HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
                    }
                }
            }
            uint32_t u32NumTry = 0;
            do
            {
                pIpbMsg->eStatus = IpbWrite(&dvrSlave, pIpbMsg, DFLT_TIMEOUT);

                if (pIpbMsg->eStatus != IPB_MESSAGE_SUCCESS)
                {
                    /* Error */
                    osDelay(100);
                }
            } while ((pIpbMsg->eStatus != IPB_MESSAGE_SUCCESS)
                    && ((u32NumTry++) < COMMS_NUM_TRY));
        }
    }
    /* USER CODE END IpbSlaveTask */
}

/* StartBridgeTask function */
void StartBridgeTask(void const * argument)
{
  /* USER CODE BEGIN StartBridgeTask */
    osEvent MsgSlaveOut, MsgMasterOut;
    char cString[16];
    memset(cString, 0, 16);
    /* Infinite loop */
    for (;;)
    {
        MsgSlaveOut = osMessageGet(IpbTxHandle, osWaitForever);

        if (MsgSlaveOut.status == osEventMessage)
        {
            IpbMsg* pIpbMsg = (IpbMsg*) MsgSlaveOut.value.p;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);

            McbMsg msg;

            msg.u16Node = pIpbMsg->u16SubNode;
            msg.u16Addr = pIpbMsg->u16Addr;
            msg.u16Cmd = pIpbMsg->u16Cmd;
            msg.u16Size = pIpbMsg->u16Size;

            memcpy(msg.u16Data, pIpbMsg->u16Data, msg.u16Size);

            osMessagePut(McbTxHandle, (uint32_t) &msg, osWaitForever);

            MsgMasterOut = osMessageGet(McbRxHandle, osWaitForever);

            if (MsgMasterOut.status == osEventMessage)
            {
                McbMsg* pMcbMasterMsg = (McbMsg*)MsgMasterOut.value.p;

                IpbMsg msg2;

                msg2.u16Node = 1;
                msg2.u16SubNode = pMcbMasterMsg->u16Node;
                msg2.u16Addr = pMcbMasterMsg->u16Addr;
                msg2.u16Cmd = pMcbMasterMsg->u16Cmd;
                msg2.u16Size = pMcbMasterMsg->u16Size;

                memcpy(msg2.u16Data, pMcbMasterMsg->u16Data, msg.u16Size);

                osMessagePut(IpbRxHandle, (uint32_t) &msg2, osWaitForever);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
            }
        }
    }
  /* USER CODE END StartBridgeTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
