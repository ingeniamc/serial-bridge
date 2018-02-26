/* High Speed Protocol */
#include "mcb.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void
mcb_init(McbInst* ptInst, EMcbIntf eIntf, EMcbMode eMode)
{
  switch(eIntf)
  {
  case MCB_OVER_SPI:
	  ptInst->eIntf = eIntf;
	  ptInst->isCyclic = false;
	  ptInst->eMode = MCB_BLOCKING;
	  hsp_init(&ptInst->Hsp, SPI_BASED);
	  break;
  case MCB_OVER_SERIAL:
	  ptInst->eIntf = eIntf;
	  ptInst->isCyclic = false;
	  ptInst->eMode = MCB_BLOCKING;
	  hsp_init(&ptInst->Hsp, UART_BASED);
	  break;
  default:
	  /* Nothing */
	  break;
  }
}

void mcb_deinit(McbInst* ptInst)
{
	ptInst->eIntf = SPI_BASED;
	ptInst->isCyclic = false;
	ptInst->eMode = MCB_BLOCKING;
	hsp_deinit(&ptInst->Hsp);
}

int32_t
mcb_write(McbInst* ptInst, uint16_t addr, uint16_t *buf, size_t sz)
{
	int32_t i32Result = 0;
	EHspStatus eStatus;

	if (ptInst->isCyclic == false)
	{
		do
		{
			eStatus = ptInst->Hsp.write(&ptInst->Hsp, addr, buf, sz);
		}while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);
	}
	else
	{

	}

	return i32Result;
}

int32_t
mcb_read(McbInst* ptInst, uint16_t addr, uint16_t *buf, size_t sz)
{
	int32_t i32Result = 0;
	EHspStatus eStatus;
	size_t rxsz;

	if (ptInst->isCyclic == false)
	{
		do
		{
			eStatus = ptInst->Hsp.read(&ptInst->Hsp, addr, buf, &rxsz);
		}while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);
	}
	else
	{

	}

	return i32Result;
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
	uint16_t u16Cyclic = 2;

	if (ptInst->isCyclic == false)
	{
		do
		{
			eStatus = ptInst->Hsp.write(&ptInst->Hsp, 0x640, &u16Cyclic, 1);
		}while(eStatus != HSP_ERROR && eStatus != HSP_SUCCESS);
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


