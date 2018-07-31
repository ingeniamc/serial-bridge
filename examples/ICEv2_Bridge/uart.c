/*
 * uart.c
 *
 *  Created on: Jul 18, 2018
 */
#include <stdint.h>

#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/src/UART_utils_defs.h>
#include <ti/drv/uart/src/UART_osal.h>

extern UART_Handle tUartHnd;

uint16_t Ipb_IntfUartReception(uint16_t u16Id, uint8_t *pu8Buf, uint16_t u16Size)
{
    uint16_t u16ReadBytes = 0;

    switch (u16Id)
    {
        case 0:
            u16ReadBytes = UART_readPolling(tUartHnd, (char*)(pu8Buf), u16Size);
            break;
        default:
            /* Nothing */
            UART_printf();
            break;
    }
    return u16ReadBytes;
}

uint16_t Ipb_IntfUartTransmission(uint16_t u16Id, const uint8_t *pu8Buf, uint16_t u16Size)
{
    uint16_t u16WrittenBytes = 0;

    switch (u16Id)
    {
        case 0:
            u16WrittenBytes = UART_write(tUartHnd, (const char*)(pu8Buf), u16Size);
            break;
        default:
            /* Nothing */
            break;
    }

    return u16WrittenBytes;
}

void Ipb_IntfUartDiscardData(uint16_t u16Id)
{
    switch (u16Id)
    {
        case 0:
            /* Discard buffered UART data */
            break;
        default:
            /* Nothing */
            break;
    }
}
