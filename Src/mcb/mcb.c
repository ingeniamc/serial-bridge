/* High Speed Protocol */
#include "mcb.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void
mcb_init(McbInst* ptInst, EMcbIntf eIntf,
		EMcbMode eMode, EMcbRequestMode eReqMode)
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

	hsp_init(&ptInst->Hsp, eHspIntf, eHspMode);
}

void mcb_deinit(McbInst* ptInst)
{
	ptInst->eIntf = SPI_BASED;
	ptInst->isCyclic = false;
	ptInst->eMode = MCB_SLAVE;
	ptInst->eReqMode = MCB_BLOCKING;
	hsp_deinit(&ptInst->Hsp);
}

EMcbMsgStatus
mcb_write(McbInst* ptInst, McbMsg *mcbMsg)
{
	EMcbMsgStatus eResult = MCB_MESSAGE_ERROR;
	EHspStatus eStatus;
	size_t sz;

	if (ptInst->isCyclic == false)
	{
		if (ptInst->eReqMode == MCB_BLOCKING)
		{
			do
			{
				eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0], &mcbMsg->size);
			}while((eStatus != HSP_ERROR) && (eStatus != HSP_SUCCESS));
		}
		else
		{
			eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0], &sz);
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

EMcbMsgStatus
mcb_read(McbInst* ptInst, McbMsg *mcbMsg)
{
	EMcbMsgStatus eResult = 0;
	EHspStatus eStatus = HSP_ERROR;

	if (ptInst->isCyclic == false)
	{
		if (ptInst->eReqMode == MCB_BLOCKING)
		{
			do
			{
				eStatus = ptInst->Hsp.read(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0]);
			}while((eStatus != HSP_ERROR) && (eStatus != HSP_SUCCESS) && (eStatus != HSP_CRC_ERROR));
			mcbMsg->size = ptInst->Hsp.sz;
		}
		else
		{
			eStatus = ptInst->Hsp.read(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0]);
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
		mcbMsg.addr = 0x640;
		mcbMsg.cmd = 1;
		// TODO memcpy
		mcbMsg.data[0] = 2;
		mcbMsg.data[1] = 0;
		mcbMsg.data[2] = 0;
		mcbMsg.data[3] = 0;
		sz = 4;
		do
		{
			eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg.addr, &mcbMsg.cmd, &mcbMsg.data[0], &sz);
		}while((eStatus != HSP_ERROR) && (eStatus != HSP_SUCCESS));
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


