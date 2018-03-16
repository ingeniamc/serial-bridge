/* High Speed Protocol */
#include "mcb_slave.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void
mcb_slave_init(McbInst* ptInst, EMcbIntf eIntf, EMcbMode eMode)
{
	switch(eIntf)
	  {
	  case MCB_OVER_SPI:
		  ptInst->eIntf = eIntf;
		  ptInst->isCyclic = false;
		  ptInst->eMode = eMode;
		  hsp_init(&ptInst->Hsp, SPI_BASED, SLAVE_MODE);
		  break;
	  case MCB_OVER_SERIAL:
		  ptInst->eIntf = eIntf;
		  ptInst->isCyclic = false;
		  ptInst->eMode = eMode;
		  hsp_init(&ptInst->Hsp, UART_BASED, SLAVE_MODE);
		  break;
	  default:
		  /* Nothing */
		  break;
	  }
}

void mcb_slave_deinit(McbInst* ptInst)
{
	ptInst->eIntf = SPI_BASED;
	ptInst->isCyclic = false;
	ptInst->eMode = MCB_BLOCKING;
	hsp_deinit(&ptInst->Hsp);
}

EMcbMssgStatus
mcb_slave_write(McbInst* ptInst, McbMssg *mcbMsg)
{
	EMcbMssgStatus eResult = MCB_MESSAGE_ERROR;
	EHspStatus eStatus;

	if (ptInst->isCyclic == false)
	{
		if (ptInst->eMode == MCB_BLOCKING)
		{
			do
			{
	//			eStatus = ptInst->Hsp.write(&ptInst->Hsp, &addr, buf);
			}while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);
		}
		else {
			size_t sWrite;
			ptInst->Hsp.eState = HSP_WRITE_ANSWER;
			eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0], &sWrite);
		}
	}
	else
	{

	}

	return eResult;
}

EMcbMssgStatus
mcb_slave_read(McbInst* ptInst, McbMssg *mcbMsg)
{
	EMcbMssgStatus eResult = MCB_MESSAGE_ERROR;
	EHspStatus eStatus;

	if (ptInst->isCyclic == false)
	{

		if (ptInst->eMode == MCB_BLOCKING)
		{
			do
			{
//				eStatus = ptInst->Hsp.read(&ptInst->Hsp, 0, &mcbMsg, &sz);
			} while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);

		}
		else
		{
			size_t sRead;
			ptInst->Hsp.eState = HSP_READ_REQUEST;
			eStatus = ptInst->Hsp.read(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0], &sRead);
		}

		if (eStatus == HSP_SUCCESS) eResult = MCB_MESSAGE_SUCCESS;
		else eResult = MCB_MESSAGE_NOT_READY;
	}
	else
	{

	}
	return eResult;
}

void* mcb_slave_tx_map(McbInst* ptInst, uint16_t addr, size_t sz)
{
	return NULL;
}

void* mcb_slave_rx_map(McbInst* ptInst, uint16_t addr, size_t sz)
{
	return NULL;
}

int32_t mcb_slave_enable_cyclic(McbInst* ptInst)
{
	int32_t i32Result = 0;
	EHspStatus eStatus;
	uint16_t u16Cyclic = 2;

	if (ptInst->isCyclic == false)
	{
		do
		{
//			eStatus = ptInst->Hsp.write(&ptInst->Hsp, 0x640, &u16Cyclic, 1);
		}while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);
	}

	return i32Result;
}

int32_t mcb_slave_disable_cyclic(McbInst* ptInst)
{
	if (ptInst->isCyclic != false)
	{

	}

	return 0;
}


