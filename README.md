# saab-coco

## Highlights ##

The HAL source code is generated with "SM32CubeMX" software, there are a few issues with that code generation tool:
* DMA UART operations do not recover the interface state
* Configuration of NSS (Chip Select) signal of SPI

This configurations are located:
* Src/spi.c
* Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c 

When the "STM32CubeMX" generate the code, the listed files have to be recovered.
