/* High Speed Protocol */
#include "mcb_master.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void
mcb_master_init(McbInst* ptInst, EMcbIntf eIntf, EMcbMode eMode)
{
  switch(eIntf)
  {
  case MCB_OVER_SPI:
	  ptInst->eIntf = eIntf;
	  ptInst->isCyclic = false;
	  ptInst->eMode = MCB_BLOCKING;
	  hsp_init(&ptInst->Hsp, SPI_BASED, MASTER_MODE);
	  break;
  case MCB_OVER_SERIAL:
	  ptInst->eIntf = eIntf;
	  ptInst->isCyclic = false;
	  ptInst->eMode = MCB_BLOCKING;
	  hsp_init(&ptInst->Hsp, UART_BASED, MASTER_MODE);
	  break;
  default:
	  /* Nothing */
	  break;
  }
}

void mcb_master_deinit(McbInst* ptInst)
{
	ptInst->eIntf = SPI_BASED;
	ptInst->isCyclic = false;
	ptInst->eMode = MCB_BLOCKING;
	hsp_deinit(&ptInst->Hsp);
}

EMcbMssgStatus
mcb_master_write(McbInst* ptInst, McbMssg *mcbMsg)
{
	EMcbMssgStatus eResult = MCB_MESSAGE_ERROR;
	EHspStatus eStatus;

	if (ptInst->isCyclic == false)
	{
		size_t sz = 4;
		do
		{
			eStatus = ptInst->Hsp.write(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0], &sz);
		}while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);

		if (eStatus == HSP_SUCCESS) eResult = MCB_MESSAGE_SUCCESS;
		else eResult = MCB_MESSAGE_ERROR;
	}
	else
	{

	}

	return eResult;
}

EMcbMssgStatus
mcb_master_read(McbInst* ptInst, McbMssg *mcbMsg)
{
	EMcbMssgStatus eResult = 0;
	EHspStatus eStatus = HSP_ERROR;

	if (ptInst->isCyclic == false)
	{
		do
		{
			size_t sRead;
			eStatus = ptInst->Hsp.read(&ptInst->Hsp, &mcbMsg->addr, &mcbMsg->cmd, &mcbMsg->data[0], &sRead);
		}while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);
	}
	else
	{

	}

	if (eStatus == HSP_SUCCESS) eResult = MCB_MESSAGE_SUCCESS;
	else eResult = MCB_MESSAGE_ERROR;

	return eResult;
}

void* mcb_master_tx_map(McbInst* ptInst, uint16_t addr, size_t sz)
{
	return NULL;
}

void* mcb_master_rx_map(McbInst* ptInst, uint16_t addr, size_t sz)
{
	return NULL;
}

int32_t mcb_master_enable_cyclic(McbInst* ptInst)
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

int32_t mcb_master_disable_cyclic(McbInst* ptInst)
{
	if (ptInst->isCyclic != false)
	{

	}

	return 0;
}


