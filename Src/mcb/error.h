#ifndef INGENIA_ERROR_H
#define INGENIA_ERROR_H

typedef int16_t IER_RET;

#define IER_SUCCESS     ((IER_RET)0)
/** General failure. */
#define IER_FAIL        ((IER_RET)-1)
/** Invalid parameters */
#define IER_PARAM       ((IER_RET)-2)
/** Invalid state */
#define IER_INVAL       ((IER_RET)-3)
/** Write fail */
#define IER_WRITE       ((IER_RET)-4)
/** Read fail */
#define IER_READ        ((IER_RET)-5)

#endif
