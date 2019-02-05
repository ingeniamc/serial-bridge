/**
 * \file   am335x_cocov2.c
 *
 * \brief  This file contains the implementation of board information for
 *         AM335x based COCOV2 board.
 *
 * \copyright Copyright (C) 2013-2017 Texas Instruments Incorporated -
 *             http://www.ti.com/
 */

/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "types.h"
#include "board_priv.h"
#include "device.h"
#include "error.h"
#include "board_am335x.h"
#include "am335x_pinmux.h"
#include "am335x_cocov2.h"
#include "gpio.h"
#include "delay_utils.h"
#include "prcm.h"
#include "console_utils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   This API configures the Phy Reset, Phy Mux select and Other Phy
 *          related configurations.
 *
 * \param   devId       PHY device ID.
 * \param   devInst     PHY device instance.
 *
 * \status  S_PASS      PHY is successfully reset
 * \status  E_FAIL      Failed to reset
 */
int32_t BoardAm335xCocoV2PhySetupAndReset(uint32_t devId, uint32_t devInstNum);



/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Variable for Phy Reset status intended for reset to happen only once */
static int32_t gBoardAm335xCocoV2PhyResetStatus;

/** \brief Information for all devices on the board is provided in this array */
const boardDeviceData_t gBoardAm335xCocoV2DevData[] =
{
#if defined(BUILDCFG_MOD_UART)
    { /* CONSOLE */
        DEVICE_ID_CONSOLE,                  /* devId */
        0U,                                 /* devInstNum */
        CHIPDB_MOD_ID_UART,                 /* ctrlModId */
        1U,                                 /* ctrlModInstNum */
        73U, /* Interrupt Number */         /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
#endif /* if defined(BUILDCFG_MOD_UART) */
#if defined(BUILDCFG_MOD_PRUSS)
    { /* PRUSS */
        DEVICE_ID_PRUSS,            		/* devId */
        0U,                                 /* devInstNum */
        CHIPDB_MOD_ID_PRU_ICSS_FW,          /* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        0U, /* Resvd */           			/* ctrlInfo */
        CHIPDB_MOD_ID_INVALID, 		        /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
#endif /* if defined(BUILDCFG_MOD_PRU_ETH) */
#if defined(BUILDCFG_MOD_PRU_ETH)
    { /* Ethernet PHY device */
        DEVICE_ID_ENET_PHY_MII,            	/* devId */
        0U,                                 /* devInstNum */
        CHIPDB_MOD_ID_ICSS_MDIO,          	/* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        0U, /* PRU port number */           /* ctrlInfo */
        1U, /* PHY device address */        /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
#endif /* if defined(BUILDCFG_MOD_PRU_ETH) */
#if defined(BUILDCFG_MOD_PRU_ETH)
    { /* Ethernet PHY device */
        DEVICE_ID_ENET_PHY_MII,            	/* devId */
        1U,                                 /* devInstNum */
        CHIPDB_MOD_ID_ICSS_MDIO,          	/* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        1U, /* PRU port number */           /* ctrlInfo */
        3U, /* PHY device address */        /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
#endif /* if defined(BUILDCFG_MOD_PRU_ETH) */

#if defined(BUILDCFG_MOD_GPIO)
    { /* TRICOLOR LED */
        DEVICE_ID_LED,                      /* devId */
        0U,                                 /* devInstNum */
        CHIPDB_MOD_ID_GPIO,                 /* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        17, /* GPIO pin number */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
    { /* TRICOLOR LED */
        DEVICE_ID_LED,                      /* devId */
        1U,                                 /* devInstNum */
        CHIPDB_MOD_ID_GPIO,                 /* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        16, /* GPIO pin number */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
    { /* TRICOLOR LED */
        DEVICE_ID_LED,                      /* devId */
        2U,                                 /* devInstNum */
        CHIPDB_MOD_ID_GPIO,                 /* ctrlModId */
        3U,                                 /* ctrlModInstNum */
        9, /* GPIO pin number */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
    { /* TRICOLOR LED */
        DEVICE_ID_LED,                      /* devId */
        3U,                                 /* devInstNum */
        CHIPDB_MOD_ID_GPIO,                 /* ctrlModId */
        1U,                                 /* ctrlModInstNum */
        30, /* GPIO pin number */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
    { /* TRICOLOR LED */
        DEVICE_ID_LED,                      /* devId */
        4U,                                 /* devInstNum */
        CHIPDB_MOD_ID_GPIO,                 /* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        20, /* GPIO pin number */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
    { /* TRICOLOR LED */
        DEVICE_ID_LED,                      /* devId */
        5U,                                 /* devInstNum */
        CHIPDB_MOD_ID_GPIO,                 /* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        19, /* GPIO pin number */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
    { /* PHY RESET GPIO */
        DEVICE_ID_RESET_PHY,                      /* devId */
        6U,                                 /* devInstNum */
        CHIPDB_MOD_ID_GPIO,                 /* ctrlModId */
        2U,                                 /* ctrlModInstNum */
        5, /* GPIO pin number */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    },
#endif /* if defined(BUILDCFG_MOD_GPIO) */
#if defined(BUILDCFG_MOD_MCSPI)
   { /* M29W160EB SPI Flash */
        DEVICE_ID_W25Q64BV,                 /* devId */
        0U,                                 /* devInstNum */
        CHIPDB_MOD_ID_MCSPI,                /* ctrlModId */
        0U,                                 /* ctrlModInstNum */
        0U, /* Chip Select */           /* ctrlInfo */
        CHIPDB_MOD_ID_INVALID,              /* dataModId */
        INVALID_INST_NUM,                   /* dataModInstNum */
        NULL,                               /* pFnSelectDev */
        NULL,                               /* pFnResetDev */
        NULL                                /* pFnPowerOnDev */
    }
#endif /* if defined(BUILDCFG_MOD_MCSPI) */
};

/** \brief Strings configured for board revision information in the EEPROM. */
const char * gpBoardAm335xCocoV2RevStrTable[] =
{
    "UNKNOWN",   /* Corresponds to UNKNOWN board. */
    "00A1",      /* COCOV2 version board */
};

/** \brief This object contains details of all devices on the board. */
const boardData_t   gBoardAm335xCocoV2Data =
{
    (sizeof (gBoardAm335xCocoV2DevData) / sizeof (boardDeviceData_t)),
    /* numDev */
    gBoardAm335xCocoV2DevData,
    /* pDevData */
    gCocoV2PinmuxData,
    /* pinmux data */
    (BOARD_REV_COCOV2_MAX + 1),
    /* numRevStrings */
    gpBoardAm335xCocoV2RevStrTable,
    /* pRevStringTable */
    NULL,
    /* pDCards */
    NULL,
    /* pFnBoardInit */
    NULL
    /* pFnBoardLcdDevData */
};


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t BoardAm335xCocoV2PhySetupAndReset(uint32_t devId, uint32_t devInstNum)
{
    return S_PASS;
}
