/* High Speed Protocol */
#include "hsp.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "error.h"
#include "gpio.h"

static void
hsp_spi_transfer(const HspInst* ptInst, frm_t *in, frm_t *out);

/** Read a static data */
static EHspStatus
hsp_read_spi_config(HspInst* ptInst, uint16_t addr, uint16_t *buff, size_t *sz);

/** Write a static data */
static EHspStatus
hsp_write_spi_config(HspInst* ptInst, uint16_t addr, uint16_t *buff, size_t sz );

static EHspStatus
hsp_cyclic_spi_tranfer(HspInst* ptInst, uint16_t *in_buf, uint16_t *out_buf);

static void
hsp_spi_transfer(const HspInst* ptInst, frm_t *in, frm_t *out)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(ptInst->phSpi, (uint8_t*)in->buf, (uint8_t*)out->buf, in->sz);
}

void
hsp_init(HspInst* ptInst, EHspIntf eIntf)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  ptInst->eState = HSP_STANDBY;

	  switch(eIntf)
	  {
	  case SPI_BASED:
		  /** This must depend on the selected SPI interface */
		  GPIO_InitStruct.Pin = GPIO_PIN_4;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		  GPIO_InitStruct.Pin = GPIO_PIN_15;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  ptInst->phSpi = &hspi1;
		  ptInst->write = &hsp_write_spi_config;
		  ptInst->read = &hsp_read_spi_config;
		  break;
	  case UART_BASED:
		  ptInst->phUsart = &huart2;
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
}

EHspStatus
hsp_write_spi_config(HspInst* ptInst, uint16_t addr, uint16_t *buf, size_t sz)
{
    switch (ptInst->eState)
    {
    case HSP_STANDBY:
    	ptInst->sz = sz;
    	ptInst->eState = HSP_WRITE_REQUEST;
    	break;
    case HSP_WRITE_REQUEST:
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
        {
			/* Check if static transmission should be segmented */
			if (ptInst->sz > HSP_FRM_STA_SZ)
			{
				frame_create(&(ptInst->Txfrm), addr, HSP_REQ_WRITE, HSP_FRM_SEG, buf, NULL, 0);
			}
			else
			{
				frame_create(&(ptInst->Txfrm), addr, HSP_REQ_WRITE, HSP_FRM_NOTSEG, buf, NULL, 0);
			}

			hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
			ptInst->eState = HSP_WRITE_REQUEST_ACK;
        }
    	break;
    case HSP_WRITE_REQUEST_ACK:
        /* Wait fall IRQ indicating received data */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)
        {
            /* Note: We prepare the next frame before the activation of the IRQ to
             * improve the timing of the system */
            frame_create(&(ptInst->Txfrm), addr, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0);
        	ptInst->eState = HSP_WRITE_REQUEST_WAIT;
        }
    	break;
    case HSP_WRITE_REQUEST_WAIT:
        /* Check if data is already available (IRQ) */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
        {
        	ptInst->eState = HSP_WRITE_ANSWER;
            /* Now we just need to send the already built frame */
        	hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
        }
    	break;
    case HSP_WRITE_ANSWER:
    	/** Wait until data is received */
    	if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
    	{
    	   /* Check reception */
		   if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true) &&
			   (frame_get_addr(&(ptInst->Rxfrm)) == addr))
		   {
			   if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_WRITE_ERROR)
			   {
				   ptInst->eState = HSP_ERROR;
			   }
			   else if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
			   {
				   if(ptInst->sz > HSP_FRM_STA_SZ)
				   {
					   ptInst->sz -= HSP_FRM_STA_SZ;
					   buf += HSP_FRM_STA_SZ;
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
hsp_read_spi_config(HspInst* ptInst, uint16_t addr, uint16_t *buff, size_t *out_sz)
{
    switch(ptInst->eState)
    {
    	case HSP_STANDBY:
			ptInst->eState = HSP_READ_REQUEST;
			break;
       case HSP_READ_REQUEST:
           if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
           {
				/* Send read request */
				frame_create(&(ptInst->Txfrm), addr, HSP_REQ_READ, HSP_FRM_NOTSEG, NULL, NULL, 0);
				hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
				*out_sz = 0;
				ptInst->eState = HSP_READ_REQUEST_ACK;
           }
       	break;
       case HSP_READ_REQUEST_ACK:
           /* Wait fall IRQ indicating received data */
           if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)
           {
               /* Note: We prepare the next frame before checking the IRQ to improve
                * the timing of the system */
               frame_create(&(ptInst->Txfrm), 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0);
               ptInst->eState = HSP_READ_REQUEST_WAIT;
           }
       	break;
       case HSP_READ_REQUEST_WAIT:
           /* Check if data is already available (IRQ) */
           if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
           {
        	   ptInst->eState = HSP_READ_ANSWER;
               /* Now we just need to send the already built frame */
        	   hsp_spi_transfer(ptInst, &(ptInst->Txfrm),  &(ptInst->Rxfrm));
           }
       	break;
       case HSP_READ_ANSWER:
			/** Wait until data is received */
			if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
			{
			   /* Check reception */
			   if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true) &&
				   (frame_get_addr(&(ptInst->Rxfrm)) == addr))
			   {
				   /* Copy read data to buffer - Also copy it in case of error msg */
				   size_t sz = frame_get_static_data(&(ptInst->Rxfrm), buff);
				   buff += sz;
				   *out_sz += sz;
				   if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_READ_ERROR)
				   {
					   ptInst->eState = HSP_ERROR;
				   }
				   else if (frame_get_cmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
				   {
					   if(frame_get_segmented(&(ptInst->Rxfrm)) != false)
					   {
						   ptInst->eState = HSP_READ_REQUEST_WAIT;
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


