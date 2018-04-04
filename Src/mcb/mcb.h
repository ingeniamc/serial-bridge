#ifndef MCB_H
#define MCB_H

#include <stdint.h>
#include "hsp.h"

#define HSP_MAX_DATA_SZ 128

/** Available interfaces */
typedef enum
{
	MCB_OVER_SPI = 0,
	MCB_OVER_SERIAL
}EMcbIntf;

/** Available interfaces */
typedef enum
{
	MCB_MASTER = 0,
	MCB_SLAVE
}EMcbMode;

typedef enum
{
	/* Blocking mode, each request block until response */
	MCB_BLOCKING = 0,
	/* Non Blocking mode, if not ready, return state */
	MCB_NON_BLOCKING
}EMcbRequestMode;

typedef enum
{
	/* Message not ready */
	MCB_MESSAGE_NOT_READY = 0,
	/* Success request */
	MCB_MESSAGE_SUCCESS,
	/* Request error */
	MCB_MESSAGE_ERROR
}EMcbMsgStatus;

/** Motion control but instance */
typedef struct
{
	/** Specific interface */
	EMcbIntf eIntf;
	/** Indicates if mcb is cyclic */
	bool isCyclic;
	/** Linked Hsp module */
	HspInst Hsp;
	/** Transmission mode */
	EMcbMode eMode;
	/** Request mode */
	EMcbRequestMode eReqMode;
}McbInst;

/** Frame data struct */
typedef struct
{
	/* Address data */
	uint16_t addr;
	/* Command data */
	uint16_t cmd;
	/* Message total size (bytes) */
	size_t 	 size;
	/* Static data */
	uint16_t data[HSP_MAX_DATA_SZ];
	/* Message status */
	EMcbMsgStatus eStatus;
}McbMsg;

/** Initialization functions */
void mcb_init(McbInst* ptInst, EMcbIntf eIntf,
		EMcbMode eMode, EMcbRequestMode eReqMode);
void mcb_deinit(McbInst* ptInst);

/** Generic read write functions */
EMcbMsgStatus mcb_write(McbInst* ptInst, McbMsg *mcbMsg);
EMcbMsgStatus mcb_read(McbInst* ptInst, McbMsg *mcbMsg);

/** Motion read/write functions */

/** Mapping functions */
void* mcb_tx_map(McbInst* ptInst, uint16_t addr, size_t sz);
void* mcb_rx_map(McbInst* ptInst, uint16_t addr, size_t sz);

/** Enabling cyclic mode.
 * Blocking function, while the config is written into driver. */
int32_t mcb_enable_cyclic(McbInst* ptInst);
/** Disable cyclic mode. */
int32_t mcb_disable_cyclic(McbInst* ptInst);

#endif
