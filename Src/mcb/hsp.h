#ifndef HSP_H
#define HSP_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "spi.h"
#include "usart.h"
#include "frame.h"
#include "tim.h"

/** Hsp communication states */
typedef enum
{
    /** Transmission successful */
    HSP_SUCCESS = 0,
    /** Bus in stand by */
    HSP_STANDBY,
    /** Sending a write request */
    HSP_WRITE_REQUEST,
    /** Waiting for write request ack */
    HSP_WRITE_REQUEST_ACK,
    /** Processing answer from write request */
    HSP_WRITE_ANSWER,
    /** Processing write */
    HSP_WRITE_ANSWER_PENDING,
    /** Sending a read request */
    HSP_READ_REQUEST,
    /** Waiting for read request ack */
    HSP_READ_REQUEST_ACK,
    /** Processing answer from read request */
    HSP_READ_ANSWER,
    /** Processing request */
    HSP_READ_REQUEST_PENDING,
    /** Waiting and processing slave cyclic frame */
    HSP_CYCLIC_ANSWER,
    /** Cancel transaction */
    HSP_CANCEL,
    /** Transaction error */
    HSP_ERROR
} EHspStatus;

/** Hsp interfaces options */
typedef enum
{
    /** Spi interface */
    SPI_BASED,
    /** Uart interdace */
    UART_BASED
} EHspIntf;

/** Hsp modes options */
typedef enum
{
    /** Master mode */
    MASTER_MODE,
    /** Slave mode */
    SLAVE_MODE
} EHspMode;
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
    TFrame Txfrm;
    /** Frame pool for holding rx data */
    TFrame Rxfrm;
    /** Pending data size to be transmitted/received */
    size_t sz;
    /** Pending bits flag */
    bool bPending;
    /** Write frame */
    EHspStatus (*write)(HspInst* ptInst, uint16_t* ptNode, uint16_t* ptSubNode,
                        uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData,
                        size_t* ptSz);
    /** Read frame */
    EHspStatus (*read)(HspInst* ptInst, uint16_t* ptNode, uint16_t* ptSubNode,
                       uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData);
};

/** Initialize a High speed protocol interface */
void
HspInit(HspInst* ptInst, EHspIntf eIntf, EHspMode eMode);

/** Deinitialize a high speed protocol interface */
void
HspDeinit(HspInst* ptInst);
#endif
