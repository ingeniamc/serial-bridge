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
#include "gpio.h"
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
uint32_t McbBuffer[ 1024 ];
osStaticThreadDef_t McbControlBlock;
osThreadId UserTaskHandle;
uint32_t UserTaskBuffer[ 128 ];
osStaticThreadDef_t UserTaskControlBlock;
osThreadId IpbTaskHandle;
uint32_t IpbTaskBuffer[ 1024 ];
osStaticThreadDef_t IpbTaskControlBlock;
osThreadId BridgeTaskHandle;
uint32_t BridgeTaskBuffer[ 2048 ];
osStaticThreadDef_t BridgeTaskControlBlock;
osMessageQId McbTxHandle;
uint8_t McbTxBuffer[ 16 * sizeof( Mcb_TMsg* ) ];
osStaticMessageQDef_t McbTxControlBlock;
osMessageQId McbRxHandle;
uint8_t McbRxBuffer[ 16 * sizeof( Mcb_TMsg* ) ];
osStaticMessageQDef_t McbRxControlBlock;
osMessageQId IpbTxHandle;
uint8_t IpbTxBuffer[ 16 * sizeof( Ipb_TMsg* ) ];
osStaticMessageQDef_t IpbTxControlBlock;
osMessageQId IpbRxHandle;
uint8_t IpbRxBuffer[ 16 * sizeof( Ipb_TMsg* ) ];
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
void IrqEvent(void* pArg);
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
  osThreadStaticDef(McbTask, McbFunc, osPriorityNormal, 0, 1024, McbBuffer, &McbControlBlock);
  McbTaskHandle = osThreadCreate(osThread(McbTask), NULL);

  /* definition and creation of UserTask */
  osThreadStaticDef(UserTask, StartUserTask, osPriorityIdle, 0, 128, UserTaskBuffer, &UserTaskControlBlock);
  UserTaskHandle = osThreadCreate(osThread(UserTask), NULL);

  /* definition and creation of IpbTask */
  osThreadStaticDef(IpbTask, IpbSlaveTask, osPriorityLow, 0, 1024, IpbTaskBuffer, &IpbTaskControlBlock);
  IpbTaskHandle = osThreadCreate(osThread(IpbTask), NULL);

  /* definition and creation of BridgeTask */
  osThreadStaticDef(BridgeTask, StartBridgeTask, osPriorityBelowNormal, 0, 2048, BridgeTaskBuffer, &BridgeTaskControlBlock);
  BridgeTaskHandle = osThreadCreate(osThread(BridgeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of McbTx */
  osMessageQStaticDef(McbTx, 16, Mcb_TMsg*, McbTxBuffer, &McbTxControlBlock);
  McbTxHandle = osMessageCreate(osMessageQ(McbTx), NULL);

  /* definition and creation of McbRx */
  osMessageQStaticDef(McbRx, 16, Mcb_TMsg*, McbRxBuffer, &McbRxControlBlock);
  McbRxHandle = osMessageCreate(osMessageQ(McbRx), NULL);

  /* definition and creation of IpbTx */
  osMessageQStaticDef(IpbTx, 16, Ipb_TMsg*, IpbTxBuffer, &IpbTxControlBlock);
  IpbTxHandle = osMessageCreate(osMessageQ(IpbTx), NULL);

  /* definition and creation of IpbRx */
  osMessageQStaticDef(IpbRx, 16, Ipb_TMsg*, IpbRxBuffer, &IpbRxControlBlock);
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
    Mcb_TMsg* pMcbMsg;
    /** SPI initialization */
    Mcb_TInst dvrMaster;

    Mcb_Init(&dvrMaster, MCB_BLOCKING, 0, false, MCB_DFLT_TIMEOUT);
    AttachExtiEvent(IrqEvent, &dvrMaster.tIntf);

    /* Infinite loop */
    for (;;)
    {
        MsgOut = osMessageGet(McbTxHandle, osWaitForever);

        if (MsgOut.status == osEventMessage)
        {
            pMcbMsg = (Mcb_TMsg*) MsgOut.value.p;

            switch (pMcbMsg->u16Cmd)
            {
                case MCB_REQ_READ:
                    pMcbMsg->eStatus = Mcb_Read(&dvrMaster, pMcbMsg);
                    break;
                case MCB_REQ_WRITE:
                    pMcbMsg->eStatus = Mcb_Write(&dvrMaster, pMcbMsg);
                    break;
                default:
                    pMcbMsg->eStatus = MCB_ERROR;
                    break;
            }

            if (pMcbMsg->eStatus != MCB_SUCCESS)
            {
                /* Error */

            }
            osMessagePut(McbRxHandle, (uint32_t) pMcbMsg, osWaitForever);
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
    Ipb_TMsg ipbMsg;
    Ipb_TMsg* pAnswer;
    Ipb_TInst dvrSlave;

    Ipb_Init(&dvrSlave, USB_BASED, IPB_BLOCKING);

    /* Infinite loop */
    for (;;)
    {
        /* Chek for incoming uart message*/
        if (Ipb_Read(&dvrSlave, &ipbMsg, IPB_DFLT_TIMEOUT) == IPB_SUCCESS)
        {
            if (ipbMsg.u16Node == NODE)
            {
                osMessagePut(IpbTxHandle, (uint32_t) &ipbMsg,
                             osWaitForever);

                MsgOut = osMessageGet(IpbRxHandle, osWaitForever);

                if (MsgOut.status == osEventMessage)
                {
                    pAnswer = (Ipb_TMsg*) MsgOut.value.p;

                    if (pAnswer->eStatus == IPB_SUCCESS)
                    {
                        /** Do something */
                    }
                }
                else
                {
                    pAnswer = NULL;
                }
            }
            else
            {
                pAnswer = NULL;
            }

            if (pAnswer == NULL)
            {
                ipbMsg.u16Node = 10;
                ipbMsg.u16SubNode = 2;
                ipbMsg.u16Addr = 0;
                ipbMsg.u16Cmd = 4;
                ipbMsg.u16Size = 4;
                ipbMsg.eStatus = IPB_ERROR;

                memset(ipbMsg.u16Data, 0, ipbMsg.u16Size);

                pAnswer = &ipbMsg;
            }
            Ipb_Write(&dvrSlave, pAnswer, IPB_DFLT_TIMEOUT);
        }
    }
  /* USER CODE END IpbSlaveTask */
}

/* StartBridgeTask function */
void StartBridgeTask(void const * argument)
{
  /* USER CODE BEGIN StartBridgeTask */
    Mcb_TMsg msg;
    Ipb_TMsg msg2;
    osEvent MsgSlaveOut, MsgMasterOut;
    Mcb_TMsg* pMcbMasterMsg;

    /* Infinite loop */
    for (;;)
    {
        MsgSlaveOut = osMessageGet(IpbTxHandle, osWaitForever);

        if (MsgSlaveOut.status == osEventMessage)
        {
            Ipb_TMsg* pIpbMsg = (Ipb_TMsg*) MsgSlaveOut.value.p;

            /** If data belongs to a subnode and data is ok, send it to linked slave */
            if ((pIpbMsg->u16SubNode != 0) && (pIpbMsg->eStatus == IPB_SUCCESS))
            {
                msg.u16Node = pIpbMsg->u16SubNode;
                msg.u16Addr = pIpbMsg->u16Addr;
                msg.u16Cmd = pIpbMsg->u16Cmd;
                msg.u16Size = pIpbMsg->u16Size;

                memcpy(msg.u16Data, pIpbMsg->u16Data, msg.u16Size * sizeof(uint16_t));

                osMessagePut(McbTxHandle, (uint32_t) &msg, osWaitForever);
                MsgMasterOut = osMessageGet(McbRxHandle, osWaitForever);

                if (MsgMasterOut.status == osEventMessage)
                {
                    pMcbMasterMsg = (Mcb_TMsg*) MsgMasterOut.value.p;
                }
                else
                {
                    pMcbMasterMsg = NULL;
                }
            }
            else
            {
                pMcbMasterMsg = NULL;
            }

            if (pMcbMasterMsg != NULL)
            {
                if (pMcbMasterMsg->eStatus == MCB_SUCCESS)
                {
                    msg2.u16Node = 1;
                    msg2.u16SubNode = pMcbMasterMsg->u16Node;
                    msg2.u16Addr = pMcbMasterMsg->u16Addr;
                    msg2.u16Size = pMcbMasterMsg->u16Size;
                    msg2.u16Cmd = 3;
                    msg2.eStatus = IPB_SUCCESS;
                    memcpy(msg2.u16Data, pMcbMasterMsg->u16Data, msg2.u16Size * sizeof(uint16_t));
                }
                else
                {
                    msg2.u16Node = 1;
                    msg2.u16SubNode = 10;
                    msg2.u16Addr = 0;
                    msg2.u16Size = 4;
                    msg2.u16Cmd = 4;
                    msg2.eStatus = IPB_SUCCESS;
                    memset(msg2.u16Data, 0, msg2.u16Size * sizeof(uint16_t));
                }
            }
            else
            {
                msg2.u16Node = 1;
                msg2.u16SubNode = 10;
                msg2.u16Addr = 0;
                msg2.u16Size = 4;
                msg2.u16Cmd = 4;
                msg2.eStatus = IPB_SUCCESS;
                memset(msg2.u16Data, 0, msg2.u16Size * sizeof(uint16_t));
            }
            osMessagePut(IpbRxHandle, (uint32_t) &msg2, osWaitForever);
        }
    }
  /* USER CODE END StartBridgeTask */
}

/* USER CODE BEGIN Application */
void IrqEvent(void* pArg)
{
    Mcb_TIntf* ptInst = (Mcb_TIntf*) pArg;
    Mcb_IntfIRQEvent(ptInst);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
