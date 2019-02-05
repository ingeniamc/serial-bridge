/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BOARD_CFG_H_
#define BOARD_CFG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/starterware/include/hw/soc_am335x.h>

/* External oscilator frequency */
#define EXT_OSC_FREQ                    (uint32_t)24000000UL

/* UART LLD instance number */
#define BOARD_UART_INSTANCE             (uint16_t)1U

/* McSPI serializer instance */
#define BOARD_MCSPI_SERIALIZER_INSTANCE 0

/* EEPROM Data read length */
#define I2C_EEPROM_RX_LENGTH            10U

/* LEDs defines */
#define GPIO_INTR_LED_RED_BASE_ADDR     SOC_GPIO_1_REGS
#define GPIO_LED_RED_PIN_NUM            (uint16_t)30U

#define GPIO_INTR_LED_GRN_BASE_ADDR     SOC_GPIO_0_REGS
#define GPIO_LED_GRN_PIN_NUM            (uint16_t)20U

/* No of LEDs connected to GPIOs. */
#define BOARD_GPIO_LED_NUM              (uint16_t)4U

/* Phy defines */
#define GPIO_PHY_RESET_BASE_ADDR 		SOC_GPIO_2_REGS
#define GPIO_PHY_RESET_PIN_NUM	 		(uint16_t)5U

#define GPIO_PHY1_DV_BASE_ADDR 			SOC_GPIO_2_REGS
#define GPIO_PHY1_DV_PIN_NUM 			(uint16_t)23U

/* Monitor */
#define BOARD_CCP_TIMER_INSTANCE        (uint16_t)5U
#define BOARD_CCP_TIMER_INSTANCE_INDEX  (uint16_t)3U

/* Port and pin number mask for MMCSD Card Detect pin.
   Bits 7-0: Pin number  and Bits 15-8: (Port number + 1) */
// #define GPIO_MMC_SDCD_PIN_NUM           (0x6)
// #define GPIO_MMC_SDCD_PORT_NUM          (0x1)
// #define GPIO_PIN_MMC_SDCD_ACTIVE_STATE  (0x0)

/* McSPI instance to support Slave mode is 1.
 * To make the example generic accross AM57x
 * platforms, added 1 to the instance number.
 */
// #define BOARD_MCSPI_MASTER_INSTANCE     (0x2U)
// #define BOARD_MCSPI_SLAVE_INSTANCE      (0x2U)

/* Board ID information */
#define BOARD_INFO_CPU_NAME             "AM335x"
#define BOARD_INFO_BOARD_NAME           "COCOV2AM335x"

/* Mmeory Sections */
#define BOARD_DDR3_START_ADDR           0x80010000
#define BOARD_DDR3_SIZE                 ((256 * 1024 * 1024UL) - 0x00010000)
#define BOARD_DDR3_END_ADDR             (BOARD_DDR3_START_ADDR + BOARD_DDR3_SIZE - 1)

/* I2C instance connected to EEPROM */
// #define BOARD_I2C_EEPROM_INSTANCE       0

/* I2C address for EEPROM */
// #define BOARD_I2C_EEPROM_ADDR           (0x50)

// /* EEPROM board ID information */
#define BOARD_EEPROM_HEADER_LENGTH      0
// #define BOARD_EEPROM_BOARD_NAME_LENGTH  8
// #define BOARD_EEPROM_VERSION_LENGTH     4
// #define BOARD_EEPROM_SERIAL_NO_LENGTH   12
// #define BOARD_EEPROM_HEADER_ADDR        0
// #define BOARD_EEPROM_BOARD_NAME_ADDR    (BOARD_EEPROM_HEADER_ADDR + BOARD_EEPROM_HEADER_LENGTH)
// #define BOARD_EEPROM_VERSION_ADDR       (BOARD_EEPROM_BOARD_NAME_ADDR + BOARD_EEPROM_BOARD_NAME_LENGTH)
// #define BOARD_EEPROM_SERIAL_NO_ADDR     (BOARD_EEPROM_VERSION_ADDR + BOARD_EEPROM_VERSION_LENGTH)

/* Enable NOR flash driver */
#define BOARD_NOR_FLASH_IN

/* GPMC instance connected to the flash */
// #define BOARD_GPMC_INSTANCE             (0U)

/* ICSS EMAC PHY address definitions */
#define BOARD_ICSS_EMAC_PORT0_PHY_ADDR  (1U)
#define BOARD_ICSS_EMAC_PORT1_PHY_ADDR  (3U)

typedef enum _BoardIcssClkOut
{
    BOARD_ICSS_PLL_CLK_200MHZ,
    BOARD_ICSS_PLL_CLK_225MHZ
} BoardIcssClkOut;

#ifdef __cplusplus
}
#endif

#endif
