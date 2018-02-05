#include "frame.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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


IER_RET
frame_init(frm_t *frm, uint16_t addr, uint8_t cmd, uint8_t pending,
           const void *sta_buf, const void *dyn_buf, size_t dyn_sz)
{
    IER_RET err = IER_SUCCESS;

    while (1)
    {
        hdr_t header;
        if (frm == NULL)
        {
            err = IER_INVAL;
            break;
        }
        
        /* Check dynamic buffer size */
        if (dyn_sz > HSP_FRM_MAX_DYN_SZ)
        {
            err = IER_PARAM;
            break;
        }
        
        /* Build header and assign it to buffer */
        header.addr = addr;
        header.cmd = cmd;
        header.pending = pending;
        frm->buf[HSP_FRM_HDR_FLD] = header.all;

        /* Copy static & dynamic buffer (if any) */
        memcpy(&frm->buf[HSP_FRM_STA_FLD], sta_buf, sizeof(frm->buf[0]) *
               HSP_FRM_STA_SZ);
        memcpy(&frm->buf[HSP_FRM_DYN_FLD], dyn_buf, sizeof(frm->buf[0]) *
               dyn_sz);

        /* Compute CRC and add it to buffer */
        frm->sz = HSP_FRM_HDR_SZ + HSP_FRM_STA_SZ + dyn_sz;
        break;
    }

    return err;
}

bool
frame_get_segmented(const frm_t *frm)
{
    hdr_t hdr;
    hdr.all = frm->buf[HSP_FRM_HDR_FLD]; 
    return (bool)hdr.pending;
}

uint16_t
frame_get_addr(const frm_t *frm)
{
    hdr_t hdr;
    hdr.all = frm->buf[HSP_FRM_HDR_FLD]; 
    return (uint16_t)hdr.addr;
}

uint8_t
frame_get_cmd(const frm_t *frm)
{
    hdr_t hdr;
    hdr.all = frm->buf[HSP_FRM_HDR_FLD]; 
    return (uint8_t)hdr.cmd;
}

uint16_t
frame_get_static_data(const frm_t *frm, uint16_t *buf)
{
    memcpy(buf, &frm->buf[HSP_FRM_STA_FLD],
            sizeof(frm->buf[0]) * HSP_FRM_STA_SZ);
    return HSP_FRM_STA_SZ;
}

