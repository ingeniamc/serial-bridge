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

void appExitFunction(Int argument);
void PeripheralInit(void);
void PeripheralStart(void);

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

    HWREG(SOC_CONTROL_REGS + 0x95C) = 7; /** spi0_cs0 pin set to gpio0[5] */

    /* Second step to Initialize peripherals and start it*/
    PeripheralInit();

    PeripheralStart();

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

void PeripheralStart(void)
{
    GPIOModuleEnable(SOC_GPIO_0_REGS);
    GPIODirModeSet(SOC_GPIO_0_REGS, 5, 0);
    GPIOPinWrite(SOC_GPIO_0_REGS, 5, 1);

    GPIO_setCallback(GPIO_INDEX_MCB_IRQ, MCBIrqDetection);
    GPIO_enableInt(GPIO_INDEX_MCB_IRQ);

    GPIO_write(GPIO_INDEX_MCB_RESET, GPIO_PIN_VAL_HIGH);
    GPIO_write(GPIO_INDEX_MCB_SYNC0, GPIO_PIN_VAL_LOW);

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

