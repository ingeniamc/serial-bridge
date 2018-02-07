#ifndef HSP_H
#define HSP_H

#include <stdint.h>
#include <stdio.h>
#include "spi.h"


typedef enum
{
	HSP_SUCCESS,
	HSP_STANDBY,
	HSP_WRITE_REQUEST,
	HSP_WRITE_REQUEST_ACK,
	HSP_WRITE_REQUEST_WAIT,
	HSP_WRITE_ANSWER,
	HSP_READ_REQUEST,
	HSP_READ_REQUEST_ACK,
	HSP_READ_REQUEST_WAIT,
	HSP_READ_ANSWER,
	HSP_CYCLIC_ANSWER,
	HSP_ERROR
}eHspStatus;

typedef struct
{
	/** Indicates the state of the communication bus */
	eHspStatus eState;
	/** Pointer to spi struct */
	SPI_HandleTypeDef* phSpi;
	/** Pointer to Tx buffer */
	uint16_t *pTxBuf;
	/** Pointer to Rx buffer */
	uint16_t *pRxBuf;
	/** Pending data size to be transmitted/received */
	size_t sz;
}HspInst;

void
hsp_init(HspInst* ptInst, SPI_HandleTypeDef* pspi);

/** Read a static data */
eHspStatus
hsp_read_async(HspInst* ptInst, uint16_t addr, uint16_t *in_buf, uint16_t *out_buf, size_t *sz);

/** Write a static data */
eHspStatus
hsp_write_async(HspInst* ptInst, uint16_t addr, uint16_t *buff, size_t sz );

eHspStatus
hsp_cyclic_tranfer(HspInst* ptInst, uint16_t *in_buf, uint16_t *out_buf);
#endif
