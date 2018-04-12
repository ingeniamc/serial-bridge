#include "frame.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "checksum.h"
#include "error.h"

/** Frame description
 * Word 0      - Header
 * Word 1..4   - Static data
 * Word 5..N-1 - Dynamic data (optional)
 * Word N      - CRC
 */

/** Uart frame description
 * Word 0      - Node Header
 * Word 1	   - Header
 * Word 2..5   - Static data
 * Word 6..N-1 - Dynamic data (optional)
 * Word N      - CRC
 */

typedef union
{
    struct
    {
        /** Segmented message */
        unsigned int   pending:1;
        /** Frame command identification */
        unsigned int   cmd:3;
        /** Address of the Static Data */
        unsigned int   addr:12;
    };
    uint16_t       all;
}THeader;

typedef union
{
    struct
    {
        /** Node address */
    	uint16_t 	u16Node:12;
        /** Internal network node */
    	uint16_t 	u16SubNode:4;
    };
    uint16_t       all;
}TNodeHeader;

/** Ingenia protocol frame header size */
#define HSP_FRM_HDR_SZ         1U
/** Ingenia protocol frame CRC size */
#define HSP_FRM_CRC_SZ         1U
/** Ingenia protocol frame dynamic buffer size */
#define HSP_FRM_MAX_DYN_SZ     (HSP_FRM_MAX_DATA_SZ - HSP_FRM_HDR_SZ - \
                                HSP_FRM_STA_SZ - HSP_FRM_CRC_SZ)

uint16_t
FrameCRC(const TFrame *tFrame);

IER_RET
FrameCreate(TFrame *tFrame, TFrameType tFrameType, uint16_t u16Node, uint16_t u16SubNode,
		uint16_t u16Addr, uint8_t u8Cmd, uint8_t u8Pending,	const void *pStaBuf,
		const void *pDynBuf, size_t szDyn, bool calcCRC)
{
    IER_RET err = IER_SUCCESS;

    while (1)
    {
        THeader tHeader;
        TNodeHeader tNodeHeader;
        uint16_t u16HeadSz = 0;
        if (tFrame == NULL)
        {
            err = IER_INVAL;
            break;
        }
        
        /* Check dynamic buffer size */
        if (szDyn > HSP_FRM_MAX_DYN_SZ)
        {
            err = IER_PARAM;
            break;
        }
        
        tFrame->tFrameType = tFrameType;
        /* Build header and assign it to buffer */
        if (tFrameType == UART_FRAME)
        {
        	tNodeHeader.u16Node = u16Node;
        	tNodeHeader.u16SubNode = u16SubNode;
        	tFrame->buf[u16HeadSz] = tNodeHeader.all;
        	u16HeadSz += HSP_NODE_FRM_HDR_SZ;
        }
        tHeader.addr = u16Addr;
        tHeader.cmd = u8Cmd;
        tHeader.pending = u8Pending;
        tFrame->buf[u16HeadSz] = tHeader.all;
        u16HeadSz += HSP_FRM_HEAD_SZ;

        /* Copy static & dynamic buffer (if any) */
        if (pStaBuf != NULL)
        {
			memcpy(&tFrame->buf[u16HeadSz], pStaBuf, (sizeof(tFrame->buf[0]) *
					HSP_FRM_STA_SZ));
        }
        else
        {
			memset(&tFrame->buf[u16HeadSz], 0, (sizeof(tFrame->buf[0]) *
					HSP_FRM_STA_SZ ));
        }

		memcpy(&tFrame->buf[(u16HeadSz + HSP_FRM_STA_SZ)], pDynBuf,
				(sizeof(tFrame->buf[0]) * szDyn));

		tFrame->sz = u16HeadSz + HSP_FRM_STA_SZ + szDyn;
		if (calcCRC != false)
		{
			/* Compute CRC and add it to buffer */
			tFrame->buf[(u16HeadSz + HSP_FRM_STA_SZ) + szDyn] = FrameCRC(tFrame);
			tFrame->sz += 1;
		}
        break;
    }

    return err;
}

uint16_t
FrameGetNode(const TFrame *tFrame)
{
	TNodeHeader tNodeHeader;
	if (tFrame->tFrameType == UART_FRAME)
	{
		tNodeHeader.all = tFrame->buf[0];
	}

    return (uint16_t)tNodeHeader.u16Node;
}

uint16_t
FrameGetSubNode(const TFrame *tFrame)
{
	TNodeHeader tNodeHeader;
	if (tFrame->tFrameType == UART_FRAME)
	{
		tNodeHeader.all = tFrame->buf[0];
	}

    return (uint16_t)tNodeHeader.u16SubNode;
}

bool
FrameGetSegmented(const TFrame *tFrame)
{
	THeader tHeader;
	uint16_t u16HeadField = 0;
	if (tFrame->tFrameType == UART_FRAME)
	{
		u16HeadField = HSP_NODE_FRM_HDR_SZ;
	}

    tHeader.all = tFrame->buf[u16HeadField];
    return (bool)tHeader.pending;
}

uint16_t
FrameGetAddr(const TFrame *tFrame)
{
    THeader tHeader;
    uint16_t u16HeadField = 0;
	if (tFrame->tFrameType == UART_FRAME)
	{
		u16HeadField = HSP_NODE_FRM_HDR_SZ;
	}

    tHeader.all = tFrame->buf[u16HeadField];
    return (uint16_t)tHeader.addr;
}

uint8_t
FrameGetCmd(const TFrame *tFrame)
{
    THeader tHeader;
    uint16_t u16HeadField = 0;
	if (tFrame->tFrameType == UART_FRAME)
	{
		u16HeadField = HSP_NODE_FRM_HDR_SZ;
	}

    tHeader.all = tFrame->buf[u16HeadField];
    return (uint8_t)tHeader.cmd;
}

uint16_t
FrameGetStaticData(const TFrame *tFrame, uint16_t *buf)
{
	uint16_t u16HeadField = 0;
	if (tFrame->tFrameType == UART_FRAME)
	{
		u16HeadField = HSP_NODE_FRM_HDR_SZ;
	}

    memcpy(buf, &tFrame->buf[(u16HeadField + HSP_FRM_HEAD_SZ)],
            sizeof(tFrame->buf[0]) * HSP_FRM_STA_SZ);
    return HSP_FRM_STA_SZ;
}

bool FrameCheckCRC(const TFrame *tFrame)
{
    bool bCRC = true;

    if (FrameCRC(tFrame) != 0)
    {
        bCRC = false;
    }

    return bCRC;
}

uint16_t FrameCRC(const TFrame *tFrame)
{
    uint16_t crc = CRC_START_XMODEM;

    for (uint16_t i = 0; i < tFrame->sz; i++)
    {
        /* TODO: Implement swap word and then just pointer */
        crc = update_crc_ccitt(crc, (tFrame->buf[i] >> 8) & 0xFF);
        crc = update_crc_ccitt(crc, (tFrame->buf[i] & 0xFF));
    }
    return crc;
}

