#ifndef MCB_H
#define MCB_H

#include <stdint.h>
#include "hsp.h"

#define HSP_MAX_DATA_SZ 128

/** Available interfaces */
typedef enum
{
	MCB_OVER_SPI,
	MCB_OVER_SERIAL
}EMcbIntf;

/** Available interfaces */
typedef enum
{
	MCB_MASTER,
	MCB_SLAVE
}EMcbMode;

typedef enum
{
	MCB_BLOCKING,
	MCB_NON_BLOCKING
}EMcbRequestMode;

typedef enum
{
	MCB_MESSAGE_NOT_READY,
	MCB_MESSAGE_SUCCESS,
	MCB_MESSAGE_ERROR
}EMcbMssgStatus;

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

/** Motion control but instance */
typedef struct
{
	uint16_t addr;
	uint16_t cmd;
	uint16_t data[HSP_MAX_DATA_SZ];
	EMcbMssgStatus eStatus;
}McbMssg;

/** Initialization functions */
void mcb_init(McbInst* ptInst, EMcbIntf eIntf,
		EMcbMode eMode, EMcbRequestMode eReqMode);
void mcb_deinit(McbInst* ptInst);

/** Generic read write functions */
EMcbMssgStatus mcb_write(McbInst* ptInst,  McbMssg *mcbMsg);
EMcbMssgStatus mcb_read(McbInst* ptInst, McbMssg *mcbMsg);

/** Motion read/write functions */

/** Mapping functions */
void* mcb_tx_map(McbInst* ptInst, uint16_t addr, size_t sz);
void* mcb_rx_map(McbInst* ptInst, uint16_t addr, size_t sz);

/** Enabling cyclic mode.
 * Blocking function, while the config is writted into driver. */
int32_t mcb_enable_cyclic(McbInst* ptInst);
/** Disable cyclic mode. */
int32_t mcb_disable_cyclic(McbInst* ptInst);

#endif
