#ifndef MCB_H
#define MCB_H

#include <stdint.h>
#include "hsp.h"

/** Available interfaces */
typedef enum
{
	MCB_OVER_SPI,
	MCB_OVER_SERIAL
}EMcbIntf;

/** Available interfaces */
typedef enum
{
	MCB_BLOCKING,
	MCB_NON_BLOCKING
}EMcbMode;

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
}McbInst;

/** Initialization functions */
void mcb_init(McbInst* ptInst, EMcbIntf eIntf, EMcbMode eMode);
void mcb_deinit(McbInst* ptInst);

/** Generic read write functions */
int32_t mcb_write(McbInst* ptInst, uint16_t addr, uint16_t *buf, size_t sz);
int32_t mcb_read(McbInst* ptInst, uint16_t addr, uint16_t *buf, size_t sz);

/** Motion read/write functions */

/** Mapping functions */
void* mcb_tx_map(McbInst* ptInst, uint16_t addr, size_t sz);
void* mcb_rx_map(McbInst* ptInst, uint16_t addr, size_t sz);

/** Enabling / disabling cyclic mode */
int32_t mcb_enable_cyclic(McbInst* ptInst);
int32_t mcb_disable_cyclic(McbInst* ptInst);
#endif
