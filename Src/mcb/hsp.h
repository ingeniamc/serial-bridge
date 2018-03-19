#ifndef HSP_H
#define HSP_H

#include <stdint.h>
#include <stdio.h>
#include "spi.h"
#include "usart.h"
#include "frame.h"

/** Hsp communication states */
typedef enum
{
	/** Transmission successful */
	HSP_SUCCESS,
	/** Bus in stand by */
	HSP_STANDBY,
	/** Sending a write request */
	HSP_WRITE_REQUEST,
	/** Waiting for write request ack */
	HSP_WRITE_REQUEST_ACK,
	/** Processing answer from write request */
	HSP_WRITE_ANSWER,
	/** Sending a read request */
	HSP_READ_REQUEST,
	/** Waiting for read request ack */
	HSP_READ_REQUEST_ACK,
	/** Processing answer from read request */
	HSP_READ_ANSWER,
	/** Waiting and processing slave cyclic frame */
	HSP_CYCLIC_ANSWER,
	/** Transmission error */
	HSP_ERROR
}EHspStatus;

/** Hsp interfaces options */
typedef enum
{
	/** Spi interface */
	SPI_BASED,
	/** Uart interdace */
	UART_BASED
}EHspIntf;

/** Hsp modes options */
typedef enum
{
	/** Master mode */
	MASTER_MODE,
	/** Slave mode */
	SLAVE_MODE
}EHspMode;
typedef struct EHspMode HspMode;

typedef struct HspInst HspInst;
struct HspInst
{
	/** Indicates the state of the communication bus */
	EHspStatus eState;
	/** Indicates the interface type */
	EHspIntf eIntf;
	/** Indicates the interface mode */
	EHspMode eMode;
	/** Pointer to spi struct */
	SPI_HandleTypeDef* phSpi;
	/** Pointer to IRQ signal */
	uint16_t* pu16Irq;
	/** Pointer to uart struct */
	UART_HandleTypeDef* phUsart;
	/** Frame pool for holding tx data */
	frm_t Txfrm;
	/** Frame pool for holding rx data */
    frm_t Rxfrm;
	/** Pending data size to be transmitted/received */
	size_t sz;
	/** Write frame */
	EHspStatus (*write)(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz);
	/** Read frame */
	EHspStatus (*read)(HspInst* ptInst, uint16_t *addr, uint16_t *cmd, uint16_t *data, size_t *sz);
};

/** Initialize a High speed protocol interface */
void
hsp_init(HspInst* ptInst, EHspIntf eIntf, EHspMode eMode);

/** Deinitialize a high speed protocol interface */
void
hsp_deinit(HspInst* ptInst);
#endif
