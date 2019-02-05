#include "gpio.h"

#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_v1.h>
#include <ti/csl/src/ip/gpio/V1/gpio_v2.h>

/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] =
{
    /* Output pin : AM335X_ICE_RED0_LED, index 0 */
    GPIO_DEVICE_CONFIG(0 + 1, 17) |
    GPIO_CFG_OUTPUT,

    /* Output pin :AM335X_ICE_GRN0_LED, index 1 */
    GPIO_DEVICE_CONFIG(0 + 1, 16) |
    GPIO_CFG_OUTPUT,

    /* Output pin : AM335X_ICE_YEL0_LED, index 2 */
    GPIO_DEVICE_CONFIG(3 + 1, 9) |
    GPIO_CFG_OUTPUT,

    /* Output pin : AM335X_ICE_RED1_LED, index 3 */
    GPIO_DEVICE_CONFIG(1 + 1, 30) |
    GPIO_CFG_OUTPUT,

    /* Output pin : AM335X_ICE_GRN1_LED, index 4 */
    GPIO_DEVICE_CONFIG(0 + 1, 20) |
    GPIO_CFG_OUTPUT,

    /* Output pin : AM335X_ICE_YEL1_LED, index 5 */
    GPIO_DEVICE_CONFIG(0 + 1, 19) |
    GPIO_CFG_OUTPUT,

    /* Output pin : MCB_IRQ, index 6 */
    GPIO_DEVICE_CONFIG(3 + 1, 20) |
    GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING,

    /* Output pin : AM335X_ICE_HVS_PIN / MCB_SYNC0, index 7 */
    GPIO_DEVICE_CONFIG(3 + 1, 18) |
    GPIO_CFG_OUTPUT,

    /* Output pin : MCB_SYNC1, index 8 */
    GPIO_DEVICE_CONFIG(3 + 1, 19) |
    GPIO_CFG_OUTPUT,

    /* Output pin : MCB_RESET, index 9 */
    GPIO_DEVICE_CONFIG(1 + 1, 8) |
    GPIO_CFG_OUT_OD_NOPULL,

    /* Output pin : MCB_SPI0_CS1, index 10 */
    GPIO_DEVICE_CONFIG(0 + 1, 6) |
    GPIO_CFG_OUT_OD_PU,
};

/* GPIO Driver call back functions */
GPIO_CallbackFxn gpioCallbackFunctions[] =
{
    NULL,                   /* Index 0 */
    NULL,                   /* Index 1 */
    NULL,                   /* Index 2 */
    NULL,                   /* Index 3 */
    NULL,                   /* Index 4 */
    NULL,                   /* Index 5 */
    NULL,                   /* Index 6 */
    NULL,                   /* Index 7 */
    NULL,                   /* Index 8 */
};

GPIO_v1_Config GPIO_v1_config =
{
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
    0x2U,
};

void  Board_setTriColorLED(uint32_t gpioLeds, uint8_t value)
{
    if(gpioLeds & BOARD_TRICOLOR0_RED)
    {
        GPIO_write(0, value);
    }

    if(gpioLeds & BOARD_TRICOLOR0_GREEN)
    {
        GPIO_write(1, value);
    }

    if(gpioLeds & BOARD_TRICOLOR0_YELLOW)
    {
        GPIO_write(2, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_RED)
    {
        GPIO_write(3, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_GREEN)
    {
        GPIO_write(4, value);
    }

    if(gpioLeds & BOARD_TRICOLOR1_YELLOW)
    {
        GPIO_write(5, value);
    }
}
