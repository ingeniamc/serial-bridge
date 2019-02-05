/*
 * gpio.h
 *
 *  Created on: Jul 20, 2018
 *      Author: jcastro
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

/** @def BOARD_TRICOLOR0_RED
 *       Macro for configuring Tri color0 Red
 */
#define BOARD_TRICOLOR0_RED                     (1U << (0U))
/** @def BOARD_TRICOLOR0_GREEN
 *       Macro for configuring Tri color0 green
 */
#define BOARD_TRICOLOR0_GREEN                   (1U << (1U))
/** @def BOARD_TRICOLOR0_YELLOW
 *       Macro for configuring Tri color0 yellow
 */
#define BOARD_TRICOLOR0_YELLOW                  (1U << (2U))
/** @def BOARD_TRICOLOR1_RED
 *       Macro for configuring Tri color1 red
 */
#define BOARD_TRICOLOR1_RED                     (1U << (3U))
/** @def BOARD_TRICOLOR1_GREEN
 *       Macro for configuring Tri color1 green
 */
#define BOARD_TRICOLOR1_GREEN                   (1U << (4U))
/** @def BOARD_TRICOLOR1_YELLOW
 *       Macro for configuring Tri color1 yellow
 */
#define BOARD_TRICOLOR1_YELLOW                  (1U << (5U))

/** Indexs needed by the GPIO API  */
#define GPIO_INDEX_MCB_IRQ                      6
#define GPIO_INDEX_MCB_SYNC0                    7
#define GPIO_INDEX_MCB_SYNC1                    8
#define GPIO_INDEX_MCB_RESET                    (uint16_t)9U
#define GPIO_INDEX_MCB_SPI_CS1                  (uint16_t)10U

#define GPIO_PIN_VAL_LOW     (0U)
#define GPIO_PIN_VAL_HIGH    (1U)

void  Board_setTriColorLED(uint32_t gpioLeds, uint8_t value);

#endif /* GPIO_H_ */
