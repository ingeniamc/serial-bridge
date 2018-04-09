/* High Speed Protocol */
#include "hsp.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "error.h"
#include "gpio.h"
#include "cmsis_os.h"

static uint16_t u16Irq;

static void
HspSPITransfer(const HspInst* ptInst, TFrame *tInFrame, TFrame *tOutFrame);

/** Read a static data */
static EHspStatus
hsp_read_spi_master(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data);

static EHspStatus
HspReadUartSlave(HspInst* ptInst, uint16_t *u16Addr, uint16_t *u16Cmd, uint16_t *u16Data);

/** Write a static data */
static EHspStatus
hsp_write_spi_master(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz);

static EHspStatus
HSPWriteUartSlave(HspInst* ptInst, uint16_t *ptAddr, uint16_t *ptCmd, uint16_t *ptData, size_t *ptSz);

static EHspStatus
hsp_cyclic_spi_tranfer(HspInst* ptInst, uint16_t *in_buf, uint16_t *out_buf);

static void HspSPITransfer(const HspInst* ptInst, TFrame *tInFrame, TFrame *tOutFrame)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(ptInst->phSpi, (uint8_t*)tInFrame->buf, (uint8_t*)tOutFrame->buf, tInFrame->sz);
}

void
hsp_init(HspInst* ptInst, EHspIntf eIntf, EHspMode eMode)
{
	ptInst->eState = HSP_STANDBY;

	switch(eIntf)
	{
	case SPI_BASED:
		/** This must depend on the selected SPI interface */
		ptInst->phSpi = &hspi1;
		if (eMode == MASTER_MODE)
		{
			/* SPI Master mode */
			ptInst->write = &hsp_write_spi_master;
			ptInst->read = &hsp_read_spi_master;
		}
		else
		{
			/* SPI Slave mode */
			ptInst->write = NULL;
			ptInst->read = NULL;
		}
		break;
	case UART_BASED:
		ptInst->phUsart = &huart2;
		if (eMode == MASTER_MODE)
		{
			/* UART Master mode */
			ptInst->write = NULL;
			ptInst->read = NULL;
		}
		else
		{
			/* UART Slave mode */
			ptInst->write = &HSPWriteUartSlave;
			ptInst->read = &HspReadUartSlave;
		}
		break;
	default:
		/* Nothing */
		break;
	}
	ptInst->pu16Irq = &u16Irq;
	*ptInst->pu16Irq = 0;
}

void hsp_deinit(HspInst* ptInst)
{
	ptInst->phSpi = NULL;
	ptInst->phUsart = NULL;
	ptInst->write = NULL;
	ptInst->read = NULL;
	ptInst->pu16Irq = NULL;
}

EHspStatus
hsp_write_spi_master(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz)
{
	switch (ptInst->eState)
	{
		case HSP_STANDBY:
			*ptInst->pu16Irq = 0;
			ptInst->sz = *sz;
			ptInst->u16Pending = 0;
			ptInst->eState = HSP_WRITE_REQUEST;
			break;
		case HSP_WRITE_REQUEST:
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
			{
				/* Check if static transmission should be segmented */
				if (ptInst->sz > HSP_FRM_STA_SZ)
				{
					frame_create(&(ptInst->Txfrm), *addr, *cmd, HSP_FRM_SEG, &data[*sz-ptInst->sz], NULL, 0, false);
					ptInst->u16Pending = 1;
				}
				else
				{
					frame_create(&(ptInst->Txfrm), *addr, *cmd, HSP_FRM_NOTSEG, &data[*sz-ptInst->sz], NULL, 0, false);
				}
				if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY))
				{
					if (ptInst->sz > 0)
					{
						ptInst->sz -= 4;
					}
					*ptInst->pu16Irq = 0;
					HspSPITransfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
					if (ptInst->sz >= HSP_FRM_STA_SZ)
					{
						frame_create(&(ptInst->Txfrm), *addr, *cmd, HSP_FRM_SEG, &data[*sz-ptInst->sz], NULL, 0, false);
					}
					else
					{
						/* Note: We prepare the next frame before the activation of the IRQ to
						 * improve the timing of the system */
						frame_create(&(ptInst->Txfrm), *addr, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0, false);
					}
					ptInst->eState = HSP_WRITE_REQUEST_ACK;
				}
			}
			break;
		case HSP_WRITE_REQUEST_ACK:
			/** Check if data is already available (IRQ) */
			if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY) && (*ptInst->pu16Irq == 1))
			{
				ptInst->eState = HSP_WRITE_ANSWER;
				if (ptInst->sz > 0)
				{
					ptInst->sz -= 4;
				}
				*ptInst->pu16Irq = 0;
				/* Now we just need to send the already built frame */
				HspSPITransfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
			}
			break;
		case HSP_WRITE_ANSWER:
			/** Wait until data is received */
			if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY) && (*ptInst->pu16Irq == 1))
			{
				/** Check reception */
				if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true) &&
						(frame_get_addr(&(ptInst->Rxfrm)) == *addr))
				{
					if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_WRITE_ERROR)
					{
						*cmd = frame_get_cmd(&(ptInst->Rxfrm));
						ptInst->eState = HSP_CANCEL;
					}
					else if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
					{
						if (ptInst->u16Pending)
						{
							/* Prepare next frame */
							if (ptInst->sz > HSP_FRM_STA_SZ)
							{
								frame_create(&(ptInst->Txfrm), *addr, *cmd, HSP_FRM_SEG, &data[*sz-ptInst->sz], NULL, 0, false);
							}
							else if (ptInst->sz == HSP_FRM_STA_SZ)
							{
								frame_create(&(ptInst->Txfrm), *addr, *cmd, HSP_FRM_NOTSEG, &data[*sz-ptInst->sz], NULL, 0, false);
							}
							else
							{
								/* Dummy message, allow reception of last frame CRC */
								ptInst->u16Pending = 0;
								frame_create(&(ptInst->Txfrm), *addr, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0, false);
							}
							ptInst->eState = HSP_WRITE_REQUEST_ACK;
						}
						else
						{
							*cmd = frame_get_cmd(&(ptInst->Rxfrm));
							ptInst->eState = HSP_SUCCESS;
							*sz = 4;
						}
				   }
				   else
				   {
					   ptInst->eState = HSP_CANCEL;
				   }
			   }
			   else
			   {
				   ptInst->eState = HSP_CANCEL;
			   }
			}
			break;
		case HSP_CANCEL:
			/* Cancel init transaction */
			if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
			{
				frame_create(&(ptInst->Txfrm), 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0, false);
				HspSPITransfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
				ptInst->eState = HSP_ERROR;
			}
			break;
		default:
			ptInst->eState = HSP_STANDBY;
			break;
    }

    return ptInst->eState;
}

EHspStatus
hsp_read_spi_master(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data)
{
	switch(ptInst->eState)
	{
		case HSP_STANDBY:
			*ptInst->pu16Irq = 0;
			ptInst->eState = HSP_READ_REQUEST;
			break;
		case HSP_READ_REQUEST:
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
			{
				/* Send read request */
				frame_create(&(ptInst->Txfrm), *addr, HSP_REQ_READ, HSP_FRM_NOTSEG, data, NULL, 0, false);
				*ptInst->pu16Irq = 0;
				HspSPITransfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
				ptInst->sz = 0;
				/* Note: We prepare the next frame before checking the IRQ to improve
				* the timing of the system */
				frame_create(&(ptInst->Txfrm), 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0, false);
				ptInst->eState = HSP_READ_REQUEST_ACK;
			}
			break;
		case HSP_READ_REQUEST_ACK:
			/* Check if data is already available (IRQ) */
			if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY) &&
					(*ptInst->pu16Irq == 1))
			{
				/* Now we just need to send the already built frame */
				*ptInst->pu16Irq = 0;
				HspSPITransfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
				ptInst->eState = HSP_READ_ANSWER;
			}
			break;
		case HSP_READ_ANSWER:
			/** Wait until data is received */
			if (*ptInst->pu16Irq == 1)
			{
				/* Check reception */
				if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true) && (frame_get_addr(&(ptInst->Rxfrm)) == *addr))
				{
					/* Copy read data to buffer - Also copy it in case of error msg */
					size_t szReaded = frame_get_static_data(&(ptInst->Rxfrm), data);
					data += szReaded;
					ptInst->sz += szReaded;
					*cmd = frame_get_cmd(&(ptInst->Rxfrm));
					if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_READ_ERROR)
					{
						ptInst->eState = HSP_CANCEL;
					}
					else if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
					{
						if(frame_get_segmented(&(ptInst->Rxfrm)) != false)
						{
							ptInst->eState = HSP_READ_REQUEST_ACK;
						}
						else
						{
							ptInst->eState = HSP_SUCCESS;
						}
					}
					else
					{
						ptInst->eState = HSP_CANCEL;
					}
				}
				else
				{
					ptInst->eState = HSP_CANCEL;
				}
			}
			break;
		case HSP_CANCEL:
			/* Cancel init transaction */
			if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
			{
				frame_create(&(ptInst->Txfrm), 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0, false);
				HspSPITransfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
				ptInst->eState = HSP_ERROR;
			}
			break;
		default:
			ptInst->eState = HSP_STANDBY;
			break;
    }

    return ptInst->eState;
}

EHspStatus
HspReadUartSlave(HspInst* ptInst, uint16_t *ptAddr, uint16_t *ptCmd, uint16_t *ptData)
{
    switch(ptInst->eState)
    {
    	case HSP_STANDBY:
    		ptInst->eState = HSP_READ_REQUEST;
    		ptInst->sz = 0;
    		break;
    	case HSP_READ_REQUEST:
			if (HAL_UART_GetState(ptInst->phUsart) == HAL_UART_STATE_READY)
			{
				if (HAL_UART_Receive(ptInst->phUsart, (uint8_t*)ptInst->Rxfrm.buf,
						HSP_FRM_STATIC_SIZE_BYTES, 100) == HAL_OK)
				{
					ptInst->Rxfrm.sz = HSP_FRM_STATIC_SIZE_BYTES/2;
					uint16_t u16Tmp;
					for (int i = 0; i < ptInst->Rxfrm.sz; i++)
					{
						u16Tmp = ptInst->Rxfrm.buf[i];
						ptInst->Rxfrm.buf[i] = ((u16Tmp & 0x00ff) << 8) |
								((u16Tmp & 0xff00) >> 8);
					}
					if (FrameCheckCRC(&ptInst->Rxfrm))
					{
						*ptAddr = frame_get_addr(&ptInst->Rxfrm);
						*ptCmd = frame_get_cmd(&ptInst->Rxfrm);
						ptInst->sz += frame_get_static_data(&ptInst->Rxfrm, &ptData[ptInst->sz]);

						/** If request is segmented type */
						if (ptInst->sz > (HSP_FRM_STATIC_SIZE_BYTES/2) ||
								(frame_get_segmented(&ptInst->Rxfrm) == 1))
						{
							if (ptInst->sz > (size_t)HSP_FRM_MAX_DATA_SZ)
							{
								ptInst->eState = HSP_ERROR;
							}
							ptInst->eState = HSP_READ_REQUEST_ACK;
						}
						else
						{
							ptInst->eState = HSP_SUCCESS;
						}
					}
					else
					{
						/** CRC Error */
						ptInst->eState = HSP_ERROR;
					}
				}
				else
				{
					ptInst->eState = HSP_ERROR;
				}
			}
			else
			{
				ptInst->eState = HSP_ERROR;
			}
			break;
    	case HSP_READ_REQUEST_ACK:
    		if (HAL_UART_GetState(ptInst->phUsart) == HAL_UART_STATE_READY)
    		{
				frame_create(&(ptInst->Txfrm), 0, HSP_READ_REQUEST_ACK, HSP_FRM_NOTSEG, NULL, NULL, 0, true);

				uint16_t u16Tmp;
				for (int i = 0; i < ptInst->Txfrm.sz; i++)
				{
					u16Tmp = ptInst->Txfrm.buf[i];
					ptInst->Txfrm.buf[i] = ((u16Tmp & 0x00ff) << 8) |
							((u16Tmp & 0xff00) >> 8);
				}

				HAL_UART_Transmit(ptInst->phUsart, (uint8_t*)ptInst->Txfrm.buf, (ptInst->Txfrm.sz * 2), 100);

				if (frame_get_segmented(&ptInst->Rxfrm) == 0)
				{
					ptInst->eState = HSP_SUCCESS;
				}
				else
				{
					ptInst->eState = HSP_READ_REQUEST;
				}
    		}
    		break;
		default:
			ptInst->eState = HSP_STANDBY;
			break;
    }
	return ptInst->eState;
}

EHspStatus
HSPWriteUartSlave(HspInst* ptInst, uint16_t *ptAddr, uint16_t *ptCmd, uint16_t *ptData, size_t *ptSz)
{
	switch(ptInst->eState)
	{
		case HSP_STANDBY:
			ptInst->eState = HSP_WRITE_ANSWER;
			ptInst->sz = *ptSz;
			break;
		case HSP_WRITE_ANSWER:
		case HSP_WRITE_ANSWER_PENDING:
		{
			if (*ptSz <= 4)
			{
				frame_create(&ptInst->Txfrm, *ptAddr, *ptCmd, HSP_FRM_NOTSEG, &ptData[(ptInst->sz - *ptSz)], NULL, 0, true);
			}
			else
			{
				frame_create(&ptInst->Txfrm, *ptAddr, *ptCmd, HSP_FRM_SEG, &ptData[(ptInst->sz - *ptSz)], NULL, 0, true);
			}
			uint16_t u16Tmp;
			for (int i = 0; i < ptInst->Txfrm.sz; i++)
			{
				u16Tmp = ptInst->Txfrm.buf[i];
				ptInst->Txfrm.buf[i] = ((u16Tmp & 0x00ff) << 8) |
						((u16Tmp & 0xff00) >> 8);
			}
			if (HAL_UART_Transmit(ptInst->phUsart, (uint8_t*)&ptInst->Txfrm.buf,
					HSP_FRM_STATIC_SIZE_BYTES, 100) == HAL_OK)
			{
				ptInst->eState = HSP_SUCCESS;
			}
			else
			{
				ptInst->eState = HSP_ERROR;
			}
		}
			break;
		default:
			ptInst->eState = HSP_STANDBY;
			break;
	}

    return ptInst->eState;
}

EHspStatus hsp_cyclic_spi_tranfer(HspInst* ptInst, uint16_t *in_buf, uint16_t *out_buf)
{
    switch(ptInst->eState)
    {
    	case HSP_STANDBY:
		/* Wait fall IRQ indicating received data */
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
		{
			frame_create(&(ptInst->Rxfrm), 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, in_buf, out_buf, 3, false);

     	    ptInst->eState = HSP_CYCLIC_ANSWER;
            /* Now we just need to send the already built frame */
     	    HspSPITransfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
		}
		break;
    	case HSP_CYCLIC_ANSWER:
			/** Wait until data is received */
			if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
			{
				ptInst->eState = HSP_STANDBY;
			}
    		break;
    	default:
    		break;
	}

    return ptInst->eState;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		default:
			u16Irq = 1;
			break;
	}
}
