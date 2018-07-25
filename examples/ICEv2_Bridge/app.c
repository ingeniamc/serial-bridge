/*
 * Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 *
 *  \brief  Template application tasks file:
 *          This template application exercises multiple tasks and
 *          peripherals. The different task functions are run under
 *          separate Tasks in TI BIOS.
 *          The appTasksCreate function creates the different tasks.
 *          More tasks can be added in this function as required.
 */

/* Standard header files */
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* Local template app header file */
#include "app.h"

/**********************************************************************
 ************************** Function prototypes ***********************
 **********************************************************************/
void TaskCreate(ti_sysbios_knl_Task_FuncPtr taskFunctionPtr,
                char *taskName, int taskPriority, int stackSize);

/* Task functions */
void IpbTask(UArg arg0, UArg arg1);
void McbTask(UArg arg0, UArg arg1);
void BridgeTask(UArg arg0, UArg arg1);
void CocoTask(UArg arg0, UArg arg1);
void i2c_eeprom_read_and_display_task(UArg arg0, UArg arg1);

QueueMsg* BlockingDequeue(Queue_Handle queueHandle);

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

UART_Handle tUartHnd;
SPI_Handle tSpiHnd;

Mcb_TInst tMcbInst;

Queue_Handle IpbTxHdl, IpbRxHdl;
Queue_Handle McbTxHdl, McbRxHdl;
Queue_Handle CocoTxHdl, CocoRxHdl;

void BuffsCreate(void)
{
    /* Ipb task queues */
    IpbTxHdl = Queue_create(NULL, NULL);
    IpbRxHdl = Queue_create(NULL, NULL);

    /* Mcb task queues */
    McbTxHdl = Queue_create(NULL, NULL);
    McbRxHdl = Queue_create(NULL, NULL);

    /* Coco task queues */
    CocoTxHdl = Queue_create(NULL, NULL);
    CocoRxHdl = Queue_create(NULL, NULL);
}

void TasksCreate(void)
{

    /* Create multiple tasks with different task priority & stack size */

    /* Create slave UART task */
    TaskCreate(IpbTask, "IpbTask",
               6, 8192);

    /* Create master spi task */
    TaskCreate(McbTask, "McbTask",
               8, 8192);

    /* Create bridge task, to connect tasks */
    TaskCreate(BridgeTask, "BridgeTask",
               7, 8192);

    /* Create bridge task, to connect tasks */
//    TaskCreate(CocoTask, "CocoTask",
//               8, 4096);

    /* Create task to test spi interface */
//       TaskCreate(i2c_eeprom_read_and_display_task,
//                  "i2c_eeprom_read_and_display_task",
//                  5, 4096);

}

void IpbTask(UArg arg0, UArg arg1)
{
    Ipb_TInst dvrSlave;

    QueueMsg *pQueueMsg;
    Ipb_TMsg tIpbMsg;
    Ipb_TMsg *pIpbMsg;

    Ipb_Init(&dvrSlave, UART_BASED, IPB_BLOCKING);
    
    /* Wait for other tasks to settle */
    Task_sleep(100);

    while (1) {
        /* Chek for incoming uart message*/
        if (Ipb_Read(&dvrSlave, &tIpbMsg, IPB_DFLT_TIMEOUT) == IPB_SUCCESS)
        {
            QueueMsg tQueueMsg;
            tQueueMsg.pData = (void*)&tIpbMsg;
            Queue_enqueue(IpbTxHdl, &(tQueueMsg.queueElem));

            pQueueMsg = BlockingDequeue(IpbRxHdl);
            if (pQueueMsg != NULL)
            {
                pIpbMsg = (Ipb_TMsg*)pQueueMsg->pData;
                Ipb_Write(&dvrSlave, pIpbMsg, IPB_DFLT_TIMEOUT);
            }
        }
        /* Sleep to yield */
        //Task_sleep(10);
    }

    BIOS_exit(0);
    Task_exit();
    return;
}

void McbTask(UArg arg0, UArg arg1)
{
    QueueMsg *pQueueMsg;
    Mcb_TMsg *pMcbMsg;

    Mcb_Init(&tMcbInst, MCB_BLOCKING, 0, true, MCB_DFLT_TIMEOUT);
    while (1)
    {
        pQueueMsg = BlockingDequeue(McbRxHdl);

        if (pQueueMsg != NULL)
        {
            pMcbMsg = (Mcb_TMsg*)pQueueMsg->pData;

            switch (pMcbMsg->u16Cmd)
            {
                case MCB_REQ_READ:
                    pMcbMsg->eStatus = Mcb_Read(&tMcbInst, pMcbMsg);
                    break;
                case MCB_REQ_WRITE:
                    pMcbMsg->eStatus = Mcb_Write(&tMcbInst, pMcbMsg);
                    break;
                default:
                    pMcbMsg->eStatus = MCB_ERROR;
                    break;
            }

            if (pMcbMsg->eStatus != MCB_SUCCESS)
            {
                /* Error */
            }

            QueueMsg tQueueMsg;
            tQueueMsg.pData = (void*)pMcbMsg;
            Queue_enqueue(McbTxHdl, &(tQueueMsg.queueElem));
        }
    }

    Task_sleep(10);
    Task_exit();
}

void BridgeTask(UArg arg0, UArg arg1)
{
    QueueMsg *pQueueMsg;

    Ipb_TMsg tIpbMsg;
    Ipb_TMsg *pIpbMsg;

    Mcb_TMsg tMcbMsg;
    Mcb_TMsg *pMcbMsg;

    while (1)
    {
        pQueueMsg = BlockingDequeue(IpbTxHdl);

        if (pQueueMsg != NULL)
        {
            pIpbMsg = (Ipb_TMsg*)pQueueMsg->pData;

            /** If data belongs to a subnode and data is ok, send it to linked slave */
            if (pIpbMsg->eStatus == IPB_SUCCESS)
            {
                tMcbMsg.u16Node = pIpbMsg->u16SubNode;
                tMcbMsg.u16Addr = pIpbMsg->u16Addr;
                tMcbMsg.u16Cmd = pIpbMsg->u16Cmd;
                tMcbMsg.u16Size = pIpbMsg->u16Size;

                memcpy(tMcbMsg.u16Data, pIpbMsg->u16Data, (tMcbMsg.u16Size * sizeof(uint16_t)));

                QueueMsg tQueueMsg;
                switch (pIpbMsg->u16SubNode)
                {
                    case COCO_NODE:
                        tQueueMsg.pData = (void*)&tMcbMsg;
                        Queue_enqueue(CocoRxHdl, &(tQueueMsg.queueElem));
                        pQueueMsg = BlockingDequeue(CocoTxHdl);
                        break;

                    case MOCO_NODE:
                        tQueueMsg.pData = (void*)&tMcbMsg;
                        Queue_enqueue(McbRxHdl, &(tQueueMsg.queueElem));
                        pQueueMsg = BlockingDequeue(McbTxHdl);
                        break;

                    default:
                        /* Respond with node error mssg */
                        pQueueMsg = NULL;
                        break;
                }

                if (pQueueMsg != NULL)
                {
                    pMcbMsg = (Mcb_TMsg*)pQueueMsg->pData;
                }
                else
                {
                    pMcbMsg = NULL;
                }
            }
            else
            {
                pMcbMsg = NULL;
            }

            if (pMcbMsg != NULL)
            {
                if (pMcbMsg->eStatus == MCB_SUCCESS)
                {
                    tIpbMsg.u16Node = NODE;
                    tIpbMsg.u16SubNode = pMcbMsg->u16Node;
                    tIpbMsg.u16Addr = pMcbMsg->u16Addr;
                    tIpbMsg.u16Size = pMcbMsg->u16Size;
                    tIpbMsg.u16Cmd = 3;
                    tIpbMsg.eStatus = IPB_SUCCESS;
                    memcpy(tIpbMsg.u16Data, pMcbMsg->u16Data, (tIpbMsg.u16Size * sizeof(uint16_t)));
                }
                else
                {
                    tIpbMsg.u16Node = NODE;
                    tIpbMsg.u16SubNode = COCO_NODE;
                    tIpbMsg.u16Addr = 0;
                    tIpbMsg.u16Size = 4;
                    tIpbMsg.u16Cmd = 4;
                    tIpbMsg.eStatus = IPB_SUCCESS;
                    memset(tIpbMsg.u16Data, 0, (tIpbMsg.u16Size * sizeof(uint16_t)));
                }
            }
            else
            {
                tIpbMsg.u16Node = NODE;
                tIpbMsg.u16SubNode = COCO_NODE;
                tIpbMsg.u16Addr = 0;
                tIpbMsg.u16Size = 4;
                tIpbMsg.u16Cmd = 4;
                tIpbMsg.eStatus = IPB_SUCCESS;
                memset(tIpbMsg.u16Data, 0, (tIpbMsg.u16Size * sizeof(uint16_t)));
            }

            QueueMsg tQueueMsg;
            tQueueMsg.pData = (void*)&tIpbMsg;
            Queue_enqueue(IpbRxHdl, &(tQueueMsg.queueElem));
        }
    }
    Task_exit();
    return;
}

void CocoTask(UArg arg0, UArg arg1)
{
    GPIODirModeSet(GPIO_INTR_LED_BASE_ADDR, GPIO_LED_PIN_NUM, GPIO_DIR_OUTPUT);
    while (1)
    {
        Task_sleep(1000);
        GPIOPinWrite(GPIO_INTR_LED_BASE_ADDR, GPIO_LED_PIN_NUM, GPIO_PIN_HIGH);
        Task_sleep(1000);
        GPIOPinWrite(GPIO_INTR_LED_BASE_ADDR, GPIO_LED_PIN_NUM, GPIO_PIN_LOW);
    }
    Task_exit();
    return;
}

void i2c_eeprom_read_and_display_task(UArg arg0, UArg arg1)
{
    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Transaction i2cTransaction;
    bool status;
    char txBuf[2] = {0x00, 0x00};
    char boardName[20];
    char boardVersion[20];


    /* Initialize parameters */
    I2C_Params_init(&i2cParams);

    /* Open I2C instance */
    handle = I2C_open(BOARD_I2C_EEPROM_INSTANCE, &i2cParams);

    /* Configure common parameters with I2C transaction */
    i2cTransaction.slaveAddress = BOARD_I2C_EEPROM_ADDR;
    i2cTransaction.writeBuf = (uint8_t *)&txBuf[0];
    i2cTransaction.writeCount = 2;

     /* Get board name */
     txBuf[0] = (char)(((uint32_t) 0xFF00 & BOARD_EEPROM_BOARD_NAME_ADDR)>>8);
     txBuf[1] = (char)((uint32_t) 0xFF & BOARD_EEPROM_BOARD_NAME_ADDR);
     i2cTransaction.readBuf = boardName;
     i2cTransaction.readCount = BOARD_EEPROM_BOARD_NAME_LENGTH;
     status = I2C_transfer(handle, &i2cTransaction);
     if (status == false)
     {
         I2C_close(handle);
         goto I2C_TEST_EXIT;
     }
     boardName[BOARD_EEPROM_BOARD_NAME_LENGTH] = '\0';

     /* Get board version */
     txBuf[0] = (char)(((uint32_t) 0xFF00 & BOARD_EEPROM_VERSION_ADDR)>>8);
     txBuf[1] = (char)((uint32_t) 0xFF & BOARD_EEPROM_VERSION_ADDR);
     i2cTransaction.readBuf = boardVersion;
     i2cTransaction.readCount = BOARD_EEPROM_VERSION_LENGTH;
     status = I2C_transfer(handle, &i2cTransaction);
     if (status == false)
     {
         I2C_close(handle);
         goto I2C_TEST_EXIT;
     }
     boardVersion[BOARD_EEPROM_VERSION_LENGTH] = '\0';
    Task_sleep(10);

I2C_TEST_EXIT:
    Task_exit();
}

void TaskCreate(ti_sysbios_knl_Task_FuncPtr taskFunctionPtr,
                char *taskName, int taskPriority, int stackSize)
{
    Task_Params taskParams;
    Error_Block eb;
    Task_Handle task;

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.instance->name = taskName;
    taskParams.priority = taskPriority;
    taskParams.stackSize = stackSize;
    task = Task_create(taskFunctionPtr, &taskParams, &eb);
    if (task == NULL) {
       BIOS_exit(0);
    }

    return;
}

QueueMsg* BlockingDequeue(Queue_Handle queueHandle)
{
    QueueMsg *pQueueMsg;
    while (Queue_empty(queueHandle) != false)
    {
        Task_sleep(10);
    }
    pQueueMsg = Queue_dequeue(queueHandle);
    return pQueueMsg;
}


void MCBIrqDetection(void)
{
    Mcb_IntfIRQEvent(&tMcbInst.tIntf);
}
