/* High Speed Protocol */
#include "mcb.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void McbInit(McbInst* ptInst, EMcbIntf eIntf, EMcbMode eMode,
             EMcbRequestMode eReqMode)
{
    ptInst->eIntf = eIntf;
    ptInst->isCyclic = false;
    ptInst->eMode = eMode;
    ptInst->eReqMode = eReqMode;

    EHspIntf eHspIntf;
    if (eIntf == MCB_OVER_SPI)
    {
        eHspIntf = SPI_BASED;
    }
    else
    {
        eHspIntf = UART_BASED;
    }

    EHspMode eHspMode;
    if (eMode == MCB_MASTER)
    {
        eHspMode = MASTER_MODE;
    }
    else
    {
        eHspMode = SLAVE_MODE;
    }

    HspInit(&ptInst->Hsp, eHspIntf, eHspMode);
}

void McbDeinit(McbInst* ptInst)
{
    ptInst->eIntf = SPI_BASED;
    ptInst->isCyclic = false;
    ptInst->eMode = MCB_SLAVE;
    ptInst->eReqMode = MCB_BLOCKING;
    HspDeinit(&ptInst->Hsp);
}

EMcbReqStatus McbWrite(McbInst* ptInst, McbMsg* mcbMsg, uint32_t u32Timeout)
{
    EMcbReqStatus eResult = MCB_MESSAGE_ERROR;
    EHspStatus eStatus;
    size_t sz;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eReqMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = HAL_GetTick();
            do
            {
                eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg->u16Node,
                                            &mcbMsg->u16SubNode,
                                            &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                                            &mcbMsg->data[0], &mcbMsg->size);

            }while ((eStatus != HSP_ERROR) && (eStatus != HSP_SUCCESS)
                    && ((HAL_GetTick() - u32Millis) < u32Timeout));
        }
        else
        {
            /** No blocking mode */
            eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg->u16Node,
                                        &mcbMsg->u16SubNode, &mcbMsg->u16Addr,
                                        &mcbMsg->u16Cmd, &mcbMsg->data[0], &sz);
        }
        if (eStatus == HSP_SUCCESS)
        {
            eResult = MCB_MESSAGE_SUCCESS;
        }
        else
        {
            eResult = MCB_MESSAGE_ERROR;
        }
    }
    else
    {
        /* Cyclic mode */
    }

    return eResult;
}

EMcbReqStatus McbRead(McbInst* ptInst, McbMsg* mcbMsg, uint32_t u32Timeout)
{
    EMcbReqStatus eResult = 0;
    EHspStatus eStatus = HSP_ERROR;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eReqMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = HAL_GetTick();
            do
            {
                eStatus = ptInst->Hsp.read(&ptInst->Hsp, &mcbMsg->u16Node,
                                           &mcbMsg->u16SubNode,
                                           &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                                           &mcbMsg->data[0]);

            }while ((eStatus != HSP_ERROR) && (eStatus != HSP_SUCCESS)
                    && ((HAL_GetTick() - u32Millis) < u32Timeout));
            mcbMsg->size = ptInst->Hsp.sz;
        }
        else
        {
            /** No blocking mode */
            eStatus = ptInst->Hsp.read(&ptInst->Hsp, &mcbMsg->u16Node,
                                       &mcbMsg->u16SubNode, &mcbMsg->u16Addr,
                                       &mcbMsg->u16Cmd, &mcbMsg->data[0]);
        }

        if (eStatus == HSP_SUCCESS)
        {
            eResult = MCB_MESSAGE_SUCCESS;
        }
        else
        {
            eResult = MCB_MESSAGE_ERROR;
        }
    }
    else
    {
        /* Cyclic mode */
    }

    if (eStatus == HSP_SUCCESS)
    {
        eResult = MCB_MESSAGE_SUCCESS;
    }
    else
    {
        eResult = MCB_MESSAGE_ERROR;
    }

    return eResult;
}

void* mcb_tx_map(McbInst* ptInst, uint16_t addr, size_t sz)
{
    return NULL;
}

void* mcb_rx_map(McbInst* ptInst, uint16_t addr, size_t sz)
{
    return NULL;
}

int32_t mcb_enable_cyclic(McbInst* ptInst)
{
    int32_t i32Result = 0;
    EHspStatus eStatus;

    if ((ptInst->isCyclic == false) && (ptInst->eMode == MCB_MASTER))
    {
        size_t sz;
        McbMsg mcbMsg;
        mcbMsg.u16Addr = 0x640;
        mcbMsg.u16Cmd = 1;
        mcbMsg.data[0] = 2;
        mcbMsg.data[1] = 0;
        mcbMsg.data[2] = 0;
        mcbMsg.data[3] = 0;
        sz = 4;
        do
        {
            eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg.u16Node,
                                        &mcbMsg.u16SubNode, &mcbMsg.u16Addr,
                                        &mcbMsg.u16Cmd, &mcbMsg.data[0], &sz);

        }while ((eStatus != HSP_ERROR) && (eStatus != HSP_SUCCESS));
    }

    return i32Result;
}

int32_t mcb_disable_cyclic(McbInst* ptInst)
{
    if (ptInst->isCyclic != false)
    {

    }

    return 0;
}

