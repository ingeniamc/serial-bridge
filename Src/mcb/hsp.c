/* High Speed Protocol */
#include "hsp.h"
#include "frame.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "error.h"
#include "gpio.h"

static void
hsp_transfer(const HspInst* ptInst, frm_t *frm);

static void
hsp_transfer(const HspInst* ptInst, frm_t *fr)
{
	uint16_t *pu16RxBuf = fr->buf;
	uint16_t *pu16TxBuf =  fr->buf;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(ptInst->phSpi, (uint8_t*)pu16TxBuf, (uint8_t*)pu16RxBuf, fr->sz);
}

void
hsp_init(HspInst* ptInst, SPI_HandleTypeDef* pspi)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  /** This must depend on the selected SPI interface */
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  ptInst->eState = HSP_STANDBY;
	  ptInst->phSpi = pspi;
}

eHspStatus
hsp_write_async(HspInst* ptInst, uint16_t addr, uint16_t *buf, size_t sz)
{
    frm_t fr;

    switch (ptInst->eState)
    {
    case HSP_STANDBY:
    	ptInst->pTxBuf = buf;
    	ptInst->sz = sz;
    	ptInst->eState = HSP_WRITE_REQUEST;
    	break;
    case HSP_WRITE_REQUEST:
        /* Check if static transmission should be segmented */
        if (ptInst->sz > HSP_FRM_STA_SZ)
        {
            frame_init(&fr, addr, HSP_REQ_WRITE, HSP_FRM_SEG, ptInst->pTxBuf, NULL, 0);
        }
        else
        {
            frame_init(&fr, addr, HSP_REQ_WRITE, HSP_FRM_NOTSEG, ptInst->pTxBuf, NULL, 0);
        }

        hsp_transfer(ptInst, &fr);
        ptInst->eState = HSP_WRITE_REQUEST_ACK;
    	break;
    case HSP_WRITE_REQUEST_ACK:
        /* Wait fall IRQ indicating received data */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)
        {
            /* Note: We prepare the next frame before the activation of the IRQ to
             * improve the timing of the system */
            memset((void *)ptInst->pTxBuf, 0, sizeof(*ptInst->pTxBuf) * HSP_FRM_STA_SZ);
            frame_init(&fr, addr, HSP_REQ_IDLE, HSP_FRM_NOTSEG, ptInst->pTxBuf, NULL, 0);
        	ptInst->eState = HSP_WRITE_REQUEST_WAIT;
        }
    	break;
    case HSP_WRITE_REQUEST_WAIT:
        /* Check if data is already available (IRQ) */
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
        {
        	ptInst->eState = HSP_WRITE_ANSWER;
            /* Now we just need to send the already built frame */
            hsp_transfer(ptInst, &fr);
        }
    	break;
    case HSP_WRITE_ANSWER:
    	/** Wait until data is received */
    	if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
    	{
            if ((MX_SPI1_CheckCrc(ptInst->phSpi) != true) ||
                (frame_get_addr(&fr) != addr) ||
                (frame_get_cmd(&fr) != HSP_REP_ACK))
            {
            	ptInst->eState = HSP_STANDBY;
            }
            else
            {
				if (ptInst->sz >= HSP_FRM_STA_SZ)
				{
					ptInst->eState = HSP_WRITE_REQUEST;
					ptInst->pTxBuf += HSP_FRM_STA_SZ;
					ptInst->sz -= HSP_FRM_STA_SZ;
				}
				else
				{
					ptInst->eState = HSP_STANDBY;
					sz = 0;
				}
            }
    	}
    	break;
    default:
    	ptInst->eState = HSP_STANDBY;
    	break;
    }

    return ptInst->eState;
}

eHspStatus
hsp_read_async(HspInst* ptInst, uint16_t addr, uint16_t *in_buf,
        uint16_t *out_buf, size_t *out_sz)
{
    static frm_t fr;
    
    switch(ptInst->eState)
    {
    case HSP_STANDBY:
       	ptInst->pTxBuf = in_buf;
       	ptInst->pRxBuf = out_buf;
       	ptInst->eState = HSP_READ_REQUEST;
       	break;
       case HSP_READ_REQUEST:
    	    /* Send read request */
    	    frame_init(&fr, addr, HSP_REQ_READ, HSP_FRM_NOTSEG, ptInst->pTxBuf, NULL, 0);
    	    hsp_transfer(ptInst, &fr);
    	    *out_sz = 0;
    	    ptInst->eState = HSP_READ_REQUEST_ACK;
       	break;
       case HSP_READ_REQUEST_ACK:
           /* Wait fall IRQ indicating received data */
           if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET)
           {
               /* Note: We prepare the next frame before checking the IRQ to improve
                * the timing of the system */
               memset((void *)ptInst->pTxBuf, 0, sizeof(*ptInst->pTxBuf) * HSP_FRM_STA_SZ);
               frame_init(&fr, 0, HSP_REQ_IDLE, HSP_FRM_NOTSEG, ptInst->pTxBuf, NULL, 0);
               ptInst->eState = HSP_READ_REQUEST_WAIT;
           }
       	break;
       case HSP_READ_REQUEST_WAIT:
           /* Check if data is already available (IRQ) */
           if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
           {
        	   ptInst->eState = HSP_READ_ANSWER;
               /* Now we just need to send the already built frame */
               hsp_transfer(ptInst, &fr);
           }
       	break;
       case HSP_READ_ANSWER:
			/** Wait until data is received */
			if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
			{
			   /* Check reception */
			   if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true) && (frame_get_addr(&fr) == addr))
			   {
				   /* Copy read data to buffer - Also copy it in case of error msg */
				   size_t sz = frame_get_static_data(&fr, ptInst->pRxBuf);
				   ptInst->pRxBuf += sz;
				   *out_sz += sz;
				   if (frame_get_cmd(&fr) == HSP_REP_READ_ERROR)
				   {
					   ptInst->eState = HSP_ERROR;
				   }
				   else if (frame_get_cmd(&fr) == HSP_REP_ACK)
				   {
					   if(frame_get_segmented(&fr) != false)
					   {
						   ptInst->eState = HSP_READ_REQUEST;
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
