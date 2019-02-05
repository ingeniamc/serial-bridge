/* Standard header files */
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* Local template app header file */
#include "app.h"

void TaskCreate(ti_sysbios_knl_Task_FuncPtr taskFunctionPtr,
                char *taskName, int taskPriority, int stackSize);

/* Task functions */
void IpbTask(UArg arg0, UArg arg1);
void McbTask(UArg arg0, UArg arg1);
void BridgeTask(UArg arg0, UArg arg1);
void CocoTask(UArg arg0, UArg arg1);

QueueMsg* BlockingDequeue(Queue_Handle queueHandle);

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
    TaskCreate(CocoTask, "CocoTask",
               8, 4096);
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
}

void CocoTask(UArg arg0, UArg arg1)
{
    GPIODirModeSet(GPIO_INTR_LED_RED_BASE_ADDR, GPIO_LED_RED_PIN_NUM, GPIO_DIR_OUTPUT);
    while (1)
    {
        Task_sleep(1000);
        GPIOPinWrite(GPIO_INTR_LED_RED_BASE_ADDR, GPIO_LED_RED_PIN_NUM, GPIO_PIN_HIGH);
        Task_sleep(1000);
        GPIOPinWrite(GPIO_INTR_LED_RED_BASE_ADDR, GPIO_LED_RED_PIN_NUM, GPIO_PIN_LOW);
    }
    Task_exit();
    return;
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
