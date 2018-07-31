/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 *
 *  \brief  Template application main file:
 *          The main code initializes the platform , calls function
 *          to create application tasks and then starts BIOS.
 *          The initialization include Board specific
 *          initialization and initialization of used peripherals.
 *          The application specific task create function used here are
 *          defined in a separate file: app.c.
 */

/* Standard header files */
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Board header file */
#include <ti/board/board.h>

/* Local template app header file */
#include "app.h"

/**********************************************************************
 ************************** Function prototypes ***********************
 **********************************************************************/
void appExitFunction(Int argument);
void PeripheralInit(void);
void PheripheralStart(void);

extern UART_Handle tUartHnd;
extern SPI_Handle tSpiHnd;

/**
 *  @brief Function main : Main application function
 *  @retval              : 0: success ; -1: fail
 */
int main(void)
{
    Board_initCfg boardCfg;
    int status;

    /* Add exit function */
    System_atexit(appExitFunction);

    /* First step here is board specific initialization
     * Note that the Board_init is specific to the
     * platform. If running on a newly custom platform
     * the Board library need to be ported and customized
     * for the new platform.
     * See Details of customizing the board library in the
     * PDK/Processor SDK User guide */
    /* Set Board initialization flags: */
    boardCfg =
         /* Enabling Board Pinmux */
         BOARD_INIT_PINMUX_CONFIG |
         BOARD_INIT_MODULE_CLOCK;

    /* Initialize Board */
    status = Board_init(boardCfg);
    if (status != BOARD_SOK)
    {
        return (false);
    }

    /* Second step to Initialize peripherals and start it*/
    PeripheralInit();

    PheripheralStart();

    /* Third step is to create Application tasks */
    BuffsCreate();
    TasksCreate();

    /* Fourth step is to Start BIOS */
    BIOS_start();
    return (0);
}

/**
 *  @brief Function appExitFunction : Just prints end of application
 *             This function is plugged into BIOS to run on exit.
 *  @retval              : 0: success ; -1: fail
 */
void appExitFunction(Int argument)
{

}

/**
 *  @brief Function peripheralInit : Initializes peripherals needed by
 *             application
 *  @retval              : None
 */
void PeripheralInit(void)
{
    /* GPIO initialization */
    GPIO_init();

    /* UART initialization */
    UART_init();

    /* I2C initialization */
    I2C_init();

    /* SPI initialization */
    SPI_init();
}

void PheripheralStart(void)
{
    GPIO_setCallback(GPIO_INDEX_MCB_IRQ, MCBIrqDetection);
    GPIO_enableInt(GPIO_INDEX_MCB_IRQ);

    UART_Params uartParams;
    UART_Params_init(&uartParams);
    tUartHnd = UART_open(UART_INSTANCE, &uartParams);
    if(tUartHnd == NULL)
    {
        /* Error opening UART interface */
        while (1);
    }

    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.transferMode = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 2000000000;
    spiParams.mode = SPI_MASTER;
    spiParams.bitRate = 10000000;
    spiParams.dataSize = 16;
    spiParams.frameFormat = SPI_POL1_PHA0;
    tSpiHnd = SPI_open(SPI_INSTANCE, &spiParams);
    if (tSpiHnd == NULL)
    {
        /* Error opening SPI interface */
        while (1);
    }
}

