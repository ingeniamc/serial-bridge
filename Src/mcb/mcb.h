#ifndef MCB_H
#define MCB_H

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
	MCB_BLOCKING,
	MCB_NON_BLOCKING
}EMcbMode;

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
}McbInst;

/** Motion control but instance */
typedef struct
{
	uint16_t addr;
	uint16_t cmd;
	uint16_t data[HSP_MAX_DATA_SZ];
	EMcbMssgStatus eStatus;
}McbMssg;

#endif
