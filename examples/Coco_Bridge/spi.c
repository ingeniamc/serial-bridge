/*
 * spi.c
 *
 *  Created on: Jul 19, 2018
 */

#include "gpio.h"
#include <ti/drv/gpio/GPIO.h>

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
            GPIO_write(GPIO_INDEX_MCB_SPI_CS1, GPIO_PIN_VAL_LOW);
            SPI_transfer(tSpiHnd, &tSpiTransaction);
            GPIO_write(GPIO_INDEX_MCB_SPI_CS1, GPIO_PIN_VAL_HIGH);

            break;
        default:
            /* Nothing */
            break;
    }
}
