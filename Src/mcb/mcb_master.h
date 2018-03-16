#ifndef MCB_MASTER_H
#define MCB_MASTER_H

#include <stdint.h>
#include "hsp.h"
#include "mcb.h"

/** Initialization functions */
void mcb_master_init(McbInst* ptInst, EMcbIntf eIntf, EMcbMode eMode);
void mcb_master_deinit(McbInst* ptInst);

/** Generic read write functions */
EMcbMssgStatus mcb_master_write(McbInst* ptInst,  McbMssg *mcbMsg);
EMcbMssgStatus mcb_master_read(McbInst* ptInst, McbMssg *mcbMsg);

/** Motion read/write functions */

/** Mapping functions */
void* mcb_master_tx_map(McbInst* ptInst, uint16_t addr, size_t sz);
void* mcb_master_rx_map(McbInst* ptInst, uint16_t addr, size_t sz);

/** Enabling / disabling cyclic mode */
int32_t mcb_master_enable_cyclic(McbInst* ptInst);
int32_t mcb_master_disable_cyclic(McbInst* ptInst);
#endif
