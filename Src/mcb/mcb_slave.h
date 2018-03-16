#ifndef MCB_SLAVE_H
#define MCB_SLAVE_H

#include <stdint.h>
#include "hsp.h"
#include "mcb.h"

/** Initialization functions */
void mcb_slave_init(McbInst* ptInst, EMcbIntf eIntf, EMcbMode eMode);
void mcb_slave_deinit(McbInst* ptInst);

/** Generic read write functions */
EMcbMssgStatus mcb_slave_write(McbInst* ptInst, McbMssg *mcbMsg);
EMcbMssgStatus mcb_slave_read(McbInst* ptInst, McbMssg *mcbMsg);

/** Motion read/write functions */

/** Mapping functions */
void* mcb_slave_tx_map(McbInst* ptInst, uint16_t addr, size_t sz);
void* mcb_slave_rx_map(McbInst* ptInst, uint16_t addr, size_t sz);

/** Enabling / disabling cyclic mode */
int32_t mcb_slave_enable_cyclic(McbInst* ptInst);
int32_t mcb_slave_disable_cyclic(McbInst* ptInst);
#endif
