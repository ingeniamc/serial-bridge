/**
 * @brief
 * @copyright Ingenia Motion Control (c) 2017. All rights reserved.
 */

#ifndef HSP_FRAME_H
#define HSP_FRAME_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "error.h"

/** Ingenia protocol frame maximum buffer size */
#define HSP_FRM_MAX_DATA_SZ     128U

/** Ingenia protocol uart frame statatic buffer node header size */
#define HSP_NODE_FRM_HDR_SZ		1U
/** Ingenia protocol frame static buffer header size */
#define HSP_FRM_HEAD_SZ			1U
/** Ingenia protocol frame static buffer size */
#define HSP_FRM_STA_SZ          4U
/** Ingenia protocol frame CRC size */
#define HSP_FRM_CRC_SZ          1U

#define HSP_FRM_STATIC_SIZE_BYTES 		((HSP_FRM_HEAD_SZ + HSP_FRM_STA_SZ + HSP_FRM_CRC_SZ) * 2)
#define HSP_UART_FRM_STATIC_SIZE_BYTES 	((HSP_NODE_FRM_HDR_SZ + HSP_FRM_HEAD_SZ + \
										  HSP_FRM_STA_SZ + HSP_FRM_CRC_SZ) * 2)

/** Ingenia protocol static function requests/replies */
/** Read request */
#define HSP_REQ_READ            1U
/** Write request */
#define HSP_REQ_WRITE           2U
/** Idle request  */
#define HSP_REQ_IDLE            7U

/** Acknowledge */
#define HSP_REP_ACK             3U
/** Error detected during read */
#define HSP_REP_READ_ERROR      5U
/** Error detected during write */
#define HSP_REP_WRITE_ERROR     6U

/** Ingenia protocol segmentation definitions */
#define HSP_FRM_NOTSEG          0U
#define HSP_FRM_SEG             1U

/** Type of frames */
typedef enum
{
	UART_FRAME = 0,
	FRAME
}TFrameType;

/** High speed Ingenia protocol frame */
typedef struct {
	TFrameType 	tFrameType;
    uint16_t    buf[HSP_FRM_MAX_DATA_SZ];
    uint16_t    sz;
}TFrame;

/**
 * Initialises an Ingenia High Speed Protocol frame.
 *
 * @param [out] tFrame
 *      Destination frame
 * @param [in] tFrameType
 * 		Frame type
 * @param [in] u16Node
 *      Destination Node.
 * @param [in] u16SubNode
 *      Destination internal network node.
 * @param [in] u16Addr
 *      Destination address.
 * @param [in] u16Cmd
 *      Frame command (request or reply)
 * @param [in] u8Pending
 *      Indicates if the static data will be segmented.
 * @param [in] pStaBuf
 *      Buffer with Static data.
 * @param [in] pDynBuf
 *      Buffer with Dynamic data.
 * @param [in] szDyn
 *      Size of the dynamic data.
 * @param [in] calcCRC
 * 		Indicates if CRC field will be calculated.
 * @return 0 success, error code otherwise
 */
IER_RET
FrameCreate(TFrame* tFrame, TFrameType tFrameType, uint16_t u16Node, uint16_t u16SubNode,
            uint16_t u16Addr, uint8_t u8Cmd, uint8_t u8Pending, const void* pStaBuf,
            const void* pDynBuf, size_t szDyn, bool calcCRC);

/**
 * Returns the node of the header.
 *
 * @param [in] tFrame
 *      Input frame.
 * @return Node.
 */
uint16_t
FrameGetNode(const TFrame *tFrame);

/**
 * Returns the SubNode of the header.
 *
 * @param [in] tFrame
 *      Input frame.
 * @return SubNode.
 */
uint16_t
FrameGetSubNode(const TFrame *tFrame);

/**
 * Returns the address of the header.
 *
 * @param [in] tFrame
 *      Input frame.
 * @return Address.
 */
uint16_t
FrameGetAddr(const TFrame* tFrame);

/**
 * Returns the command (request or reply) of the static data.
 *
 * @param [in] tFrame
 *      Input frame.
 * @return Command.
 */
uint8_t
FrameGetCmd(const TFrame* tFrame);

/**
 * Checks if the static data is segmented and requires further data.
 *
 * @param [in] tFrame
 *      Input frame.
 * @return true if static data is segmented.
 */
bool
FrameGetSegmented(const TFrame* tFrame);

/**
 * Returns the static data of a frame.
 *
 * @param [in] tFrame
 *      Input frame.
 * @return Static data
 */
uint16_t
FrameGetStaticData(const TFrame* tFrame, uint16_t* buf);

/**
 * Indicates if the crc for the input frame is correct
 *
 * @param[in] tFrame
 *  Input frame
 *
 * @return true if crc is correct
 *         false if crc is wrong
 */
bool
FrameCheckCRC(const TFrame* tFrame);

#endif
