#include "frame.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "checksum.h"
#include "error.h"

/** frm description
 * Word 0      - Header
 * Word 1..4   - Static data
 * Word 5..N-1 - Dynamic data (optional)
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
}hdr_t;

/** Ingenia protocol frame header size */
#define HSP_FRM_HDR_SZ         1U
/** Ingenia protocol frame CRC size */
#define HSP_FRM_CRC_SZ         1U
/** Ingenia protocol frame dynamic buffer size */
#define HSP_FRM_MAX_DYN_SZ     (HSP_FRM_MAX_DATA_SZ - HSP_FRM_HDR_SZ - \
                                HSP_FRM_STA_SZ - HSP_FRM_CRC_SZ)

#define HSP_FRM_HDR_FLD        0U
#define HSP_FRM_STA_FLD        HSP_FRM_HDR_FLD + HSP_FRM_HDR_SZ
#define HSP_FRM_DYN_FLD        HSP_FRM_STA_FLD + HSP_FRM_STA_SZ

uint16_t
frameCRC(const TFrame *tFrame);

IER_RET
frame_create(TFrame *tFrame, uint16_t u16Addr, uint8_t u8Cmd, uint8_t u8Pending,
           	 const void *pStaBuf, const void *pDynBuf, size_t szDyn, bool calcCRC)
{
    IER_RET err = IER_SUCCESS;

    while (1)
    {
        hdr_t header;
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
        
        /* Build header and assign it to buffer */
        header.addr = u16Addr;
        header.cmd = u8Cmd;
        header.pending = u8Pending;
        tFrame->buf[HSP_FRM_HDR_FLD] = header.all;

        /* Copy static & dynamic buffer (if any) */
        if (pStaBuf != NULL)
        {
			memcpy(&tFrame->buf[HSP_FRM_STA_FLD], pStaBuf, (sizeof(tFrame->buf[0]) *
					HSP_FRM_STA_SZ));
        }
        else
        {
			memset(&tFrame->buf[HSP_FRM_STA_FLD], 0, (sizeof(tFrame->buf[0]) *
					HSP_FRM_STA_SZ ));
        }

		memcpy(&tFrame->buf[HSP_FRM_DYN_FLD], pDynBuf, (sizeof(tFrame->buf[0]) *
				szDyn));

		tFrame->sz = HSP_FRM_HDR_SZ + HSP_FRM_STA_SZ + szDyn;
		if (calcCRC != false)
		{
			/* Compute CRC and add it to buffer */
			tFrame->buf[HSP_FRM_DYN_FLD + szDyn] = frameCRC(tFrame);
			tFrame->sz += 1;
		}
        break;
    }

    return err;
}

bool
frame_get_segmented(const TFrame *frm)
{
    hdr_t hdr;
    hdr.all = frm->buf[HSP_FRM_HDR_FLD]; 
    return (bool)hdr.pending;
}

uint16_t
frame_get_addr(const TFrame *frm)
{
    hdr_t hdr;
    hdr.all = frm->buf[HSP_FRM_HDR_FLD]; 
    return (uint16_t)hdr.addr;
}

uint8_t
frame_get_cmd(const TFrame *frm)
{
    hdr_t hdr;
    hdr.all = frm->buf[HSP_FRM_HDR_FLD]; 
    return (uint8_t)hdr.cmd;
}

uint16_t
frame_get_static_data(const TFrame *frm, uint16_t *buf)
{
    memcpy(buf, &frm->buf[HSP_FRM_STA_FLD],
            sizeof(frm->buf[0]) * HSP_FRM_STA_SZ);
    return HSP_FRM_STA_SZ;
}

bool frameCheckCRC(const TFrame *tFrame)
{
    bool bCRC = true;

    if (frameCRC(tFrame) != 0)
    {
        bCRC = false;
    }

    return bCRC;
}

uint16_t frameCRC(const TFrame *tFrame)
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

