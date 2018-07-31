/*
 * spi.c
 *
 *  Created on: Jul 19, 2018
 */

#include <stdint.h>

#include <ti/drv/spi/MCSPI.h>

extern SPI_Handle tSpiHnd;

bool Mcb_IntfIsReady(uint16_t u16Id)
{
    bool IsReady = false;

    switch (u16Id)
    {
        case 0:
            if (tSpiHnd != NULL)
            {
                IsReady = true;
            }
            break;
        default:
            /* Nothing */
            break;
    }

    return IsReady;
}

void Mcb_IntfSPITransfer(uint16_t u16Id, uint16_t* pu16In, uint16_t* pu16Out, uint16_t u16Sz)
{
    SPI_Transaction tSpiTransaction;

    switch (u16Id)
    {
        case 0:
            tSpiTransaction.txBuf = (void*)pu16In;
            tSpiTransaction.rxBuf = (void*)pu16Out;
            /* Size to be transmited, from words to bytes */
            tSpiTransaction.count = (size_t)(u16Sz);
            SPI_transfer(tSpiHnd, &tSpiTransaction);

            break;
        default:
            /* Nothing */
            break;
    }
}
