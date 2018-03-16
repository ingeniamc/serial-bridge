/* High Speed Protocol */
#include "hsp.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "error.h"
#include "gpio.h"

static uint16_t u16Irq;

static void
hsp_spi_transfer(const HspInst* ptInst, frm_t *in, frm_t *out);

/** Read a static data */
static EHspStatus
hsp_read_spi_master(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz);

static EHspStatus
hsp_read_uart_slave(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz);

/** Write a static data */
static EHspStatus
hsp_write_spi_master(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz);

static EHspStatus
hsp_write_uart_slave(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz);

static EHspStatus
hsp_cyclic_spi_tranfer(HspInst* ptInst, uint16_t *in_buf, uint16_t *out_buf);

static void
hsp_spi_transfer(const HspInst* ptInst, frm_t *in, frm_t *out)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(ptInst->phSpi, (uint8_t*)in->buf, (uint8_t*)out->buf, in->sz);
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
			  ptInst->write = &hsp_write_spi_master;
			  ptInst->read = &hsp_read_spi_master;
		  }
		  else
		  {
//			  ptInst->write = &hsp_write_spi_slave;
//			  ptInst->read = &hsp_read_spi_slave;
		  }
		  ptInst->pu16Irq = &u16Irq;
		  *ptInst->pu16Irq = 0;
		  break;
	  case UART_BASED:
		  ptInst->phUsart = &huart2;
		  if (eMode == MASTER_MODE)
		  {
//			  ptInst->write = &hsp_write_uart_master;
//			  ptInst->read = &hsp_read_uart_master;
		  }
		  else
		  {
			  ptInst->write = &hsp_write_uart_slave;
			  ptInst->read = &hsp_read_uart_slave;
		  }
		  ptInst->pu16Irq = &u16Irq;
		  *ptInst->pu16Irq = 0;
		  break;
	  default:
		  /* Nothing */
		  break;
	  }

}

void hsp_deinit(HspInst* ptInst)
{
	ptInst->phSpi = NULL;
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
    	ptInst->eState = HSP_WRITE_REQUEST;
    	break;
    case HSP_WRITE_REQUEST:
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
        {
			/* Check if static transmission should be segmented */
			if (ptInst->sz > HSP_FRM_STA_SZ)
			{
				frame_create(&(ptInst->Txfrm), *addr, HSP_REQ_WRITE, HSP_FRM_SEG, data, NULL, 0);
			}
			else
			{
				frame_create(&(ptInst->Txfrm), *addr, HSP_REQ_WRITE, HSP_FRM_NOTSEG, data, NULL, 0);
			}
			ptInst->sz = ptInst->Txfrm.sz;

			hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));

        	ptInst->eState = HSP_WRITE_REQUEST_ACK;
        }
    	break;
    case HSP_WRITE_REQUEST_ACK:
        /* Check if data is already available (IRQ) */
        if (*ptInst->pu16Irq == 1)
        {
        	*ptInst->pu16Irq = 0;
        	ptInst->eState = HSP_WRITE_ANSWER;
            /* Now we just need to send the already built frame */
        	hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
        }
    	break;
    case HSP_WRITE_ANSWER:
    	/** Wait until data is received */
    	if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY) && (*ptInst->pu16Irq == 1))
    	{
    	   /* Check reception */
		   if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true) &&
			   (frame_get_addr(&(ptInst->Rxfrm)) == *addr))
		   {

			   *cmd = frame_get_cmd(&(ptInst->Rxfrm));
			   if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_WRITE_ERROR)
			   {
				   ptInst->eState = HSP_ERROR;
			   }
			   else if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
			   {
				   if(ptInst->sz > (HSP_FRM_STA_SZ + HSP_FRM_HEAD_SZ))
				   {
					   ptInst->sz -= HSP_FRM_STA_SZ;
//					   data += HSP_FRM_STA_SZ;
					   frame_get_static_data(&(ptInst->Rxfrm), data);
					   ptInst->eState = HSP_WRITE_REQUEST;
				   }
				   else
				   {
					   ptInst->eState = HSP_SUCCESS;
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
    	}
    	break;
    default:
    	ptInst->eState = HSP_STANDBY;
    	break;
    }

    return ptInst->eState;
}

EHspStatus
hsp_read_spi_master(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz)
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
				frame_create(&(ptInst->Txfrm), *addr, HSP_REQ_READ, HSP_FRM_NOTSEG, NULL, NULL, 0);
				hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
				*sz = 0;
				/* Note: We prepare the next frame before checking the IRQ to improve
				* the timing of the system */
				frame_create(&(ptInst->Txfrm), 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0);
				ptInst->eState = HSP_READ_REQUEST_ACK;
			}
       	break;
       case HSP_READ_REQUEST_ACK:
           /* Check if data is already available (IRQ) */
    	   if (*ptInst->pu16Irq == 1)
           {
               /* Now we just need to send the already built frame */
        	   hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
        	   ptInst->eState = HSP_READ_ANSWER;
           }
       	break;
       case HSP_READ_ANSWER:
			/** Wait until data is received */
			if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY) && (*ptInst->pu16Irq == 1))
			{
			   /* Check reception */
			   if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true) &&
				   (frame_get_addr(&(ptInst->Rxfrm)) == *addr))
			   {
				   /* Copy read data to buffer - Also copy it in case of error msg */
//				   size_t sz_tmp = frame_get_static_data(&(ptInst->Rxfrm), data);
//				   data += sz_tmp;
//				   *sz += sz_tmp;
				   *cmd = frame_get_cmd(&(ptInst->Rxfrm));
				   if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_READ_ERROR)
				   {
					   ptInst->eState = HSP_ERROR;
				   }
				   else if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
				   {
					   if(frame_get_segmented(&(ptInst->Rxfrm)) != false)
					   {
						   ptInst->eState = HSP_READ_REQUEST_ACK;
					   }
					   else
					   {
						   *sz = frame_get_static_data(&(ptInst->Rxfrm), data);
						   ptInst->eState = HSP_SUCCESS;
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
    	   }
       	break;
    default:
    	ptInst->eState = HSP_STANDBY;
    	break;
    }

    return ptInst->eState;
}

EHspStatus
hsp_read_uart_slave(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz)
{
    switch(ptInst->eState)
    {
    	case HSP_STANDBY:
    		*ptInst->pu16Irq = 0;
			ptInst->eState = HSP_READ_REQUEST;
			break;
       case HSP_READ_REQUEST:
//    	   if(HAL_UART_GetState(ptInst->phUsart) == HAL_UART_STATE_BUSY_TX)
//		   {
//			   HAL_UART_AbortTransmit(ptInst->phUsart);
//		   }
			if (HAL_UART_GetState(ptInst->phUsart) == HAL_UART_STATE_READY)
			{
				frm_t frame;
				if (HAL_UART_Receive_DMA(ptInst->phUsart, (uint8_t*)frame.buf, SIZEOF_STATIC_FRAME_BYTES) == HAL_OK)
				{
					uint8_t u8Tmp;
					for (int i = 0; i < SIZEOF_STATIC_FRAME_BYTES; i+=2)
					{
						u8Tmp = ((uint8_t*)frame.buf)[i];
						((uint8_t*)frame.buf)[i] = ((uint8_t*)frame.buf)[i+1];
						((uint8_t*)frame.buf)[i+1] = u8Tmp;
					}
					*addr = frame_get_addr(&frame);
					*cmd = frame_get_cmd(&frame);
					*sz = frame_get_static_data(&frame, data);
					ptInst->eState = HSP_SUCCESS;
				}
			}
			else
			{
				HAL_UART_StateTypeDef state = HAL_UART_GetState(ptInst->phUsart);
				HAL_UART_AbortTransmit(ptInst->phUsart);
				if(state == HAL_UART_ERROR_DMA)
				{
				   ptInst->eState = HSP_ERROR;
				}
				else ptInst->eState = HSP_STANDBY;
			}
			break;
		default:
			ptInst->eState = HSP_STANDBY;
			break;
    }

    return ptInst->eState;
}

EHspStatus
hsp_write_uart_slave(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz)
{
    switch(ptInst->eState)
    {
    	case HSP_STANDBY:
    		*ptInst->pu16Irq = 0;
    		ptInst->sz = *sz;
			ptInst->eState = HSP_WRITE_ANSWER;
			break;
       case HSP_WRITE_ANSWER:
    	   *ptInst->pu16Irq = 0;
    	   ptInst->sz = *sz;
//    	   if(HAL_UART_GetState(ptInst->phUsart) == HAL_UART_STATE_BUSY_RX)
//    	   {
//    		   HAL_UART_AbortReceive(ptInst->phUsart);
//    	   }
//    	   if (HAL_UART_GetState(ptInst->phUsart) == HAL_UART_STATE_READY)
//    	   {
				frm_t frame;
				frame_create(&frame, *addr, *cmd, HSP_FRM_NOTSEG, data, NULL, 0);
				uint8_t u8Tmp;
				for (int i = 0; i < SIZEOF_STATIC_FRAME_BYTES; i+=2)
				{
					u8Tmp = ((uint8_t*)frame.buf)[i];
					((uint8_t*)frame.buf)[i] = ((uint8_t*)frame.buf)[i+1];
					((uint8_t*)frame.buf)[i+1] = u8Tmp;
				}
				if (HAL_UART_Transmit_DMA(ptInst->phUsart, (uint8_t*)frame.buf, SIZEOF_STATIC_FRAME_BYTES) == HAL_OK)
				{
					ptInst->eState = HSP_SUCCESS;
				}
				else
				{
					ptInst->eState = HSP_STANDBY;
				}
//    	   }
//    	   else
//    	   {
//    		   HAL_UART_StateTypeDef state = HAL_UART_GetState(ptInst->phUsart);
//    		   if(state == HAL_UART_ERROR_DMA)
//    		   {
//    			   ptInst->eState = HSP_ERROR;
//    		   }
//    		   else ptInst->eState = HSP_STANDBY;
//    	   }
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
			frame_create(&(ptInst->Rxfrm), 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, in_buf, out_buf, 3);

     	    ptInst->eState = HSP_CYCLIC_ANSWER;
            /* Now we just need to send the already built frame */
     	   hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
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
