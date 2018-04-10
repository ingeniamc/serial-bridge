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

/** Ingenia protocol frame static buffer header size */
#define HSP_FRM_HEAD_SZ			1U
/** Ingenia protocol frame static buffer size */
#define HSP_FRM_STA_SZ          4U
/** Ingenia protocol frame CRC size */
#define HSP_FRM_CRC_SZ          1U

#define HSP_FRM_STATIC_SIZE_BYTES ((HSP_FRM_HEAD_SZ + HSP_FRM_STA_SZ + HSP_FRM_CRC_SZ) * 2)

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

/** High speed Ingenia protocol frame */
typedef struct {
    uint16_t       buf[HSP_FRM_MAX_DATA_SZ];
    uint16_t       sz;
}TFrame;

/**
 * Initialises an Ingenia High Speed Protocol frame.
 *
 * @param [in/out] TFrame
 *      Destination frame
 * @param [in] addr
 *      Destination address.
 * @param [in] cmd
 *      Frame command (request or reply)
 * @param [in] segmented
 *      Indicates if the static data will be segmented.
 * @param [in] sta_buf
 *      Buffer with Static data.
 * @param [in] dyn_buff
 *      Buffer with Dynamic data.
 * @param [in] dyn_sz
 *      Size of the dynamic data.
 * @return 0 success, error code otherwise
 */
IER_RET
frame_create(TFrame *tFrame, uint16_t u16Addr, uint8_t u8Cmd, uint8_t u8Pending,
           	 const void *pStaBuf, const void *pDynBuf, size_t szDyn, bool calcCRC);

/**
 * Returns the address of the static data.
 *
 * @param [in] TFrame
 *      Input frame.
 * @return Address.
 */
uint16_t
frame_get_addr(const TFrame *frm);

/**
 * Returns the command (request or reply) of the static data.
 *
 * @param [in] TFrame
 *      Input frame.
 * @return Command.
 */
uint8_t
frame_get_cmd(const TFrame *frm);

/**
 * Checks if the static data is segmented and requires further data.
 *
 * @param [in] TFrame
 *      Input frame.
 * @return true if static data is segmented.
 */
bool
frame_get_segmented(const TFrame *frm);

/**
 * Returns the static data of a frame.
 *
 * @param [in] TFrame
 *      Input frame.
 * @return Static data
 */
uint16_t
frame_get_static_data(const TFrame *frm, uint16_t *buf);

/**
 * Indicates if the crc for the input frame is correct
 *
 * @param[in] TFrame
 *  Input frame
 *
 * @return true if crc is correct
 *         false if crc is wrong
 */
bool
FrameCheckCRC(const TFrame *frm);

#endif
