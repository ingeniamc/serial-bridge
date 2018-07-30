# COCO Bridge

## Overview ##
This repository contains different examples of bridge implementation using the mcb library. The examples are based on different platforms including low MCU performance without OS and high MPU performance such as the sitara family available on development board like beaglebone.

All the examples implements a bridge between a serial communication based on UART or USB (CDC) protocols and the extended SPI used for motion control bus.

## ST ##

### Highlights ###
* Developed for STM32 devices (STM32F042 & STM32F407)
* Developed with [Atollic TrueStudio](https://atollic.com/truestudio/)
* Compiled using ANSI/ISO C89

### STM32CubeMx HAL code generator ###
The HAL source code is generated with "SM32CubeMX" software, there are a few issues with that code generation tool:
* DMA UART operations do not recover the interface state
* Configuration of NSS (Chip Select) signal of SPI

This configurations are located:
* Src/spi.c
* Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c 

When the "STM32CubeMX" generate the code, the listed files have to be recovered.

## Sitara ##
### Highlights ###

* Developed for AM335x devices
* Developed with [Code Compose Studio](http://www.ti.com/tool/ccstudio-C2000) and the GNU compiler from TI
* Compiled using ANSI/ISO C89 & ISO C++03 language

### PDK installation
The next packages must be installed in order to build the repository correctly:

#### sitara - [SDK RTOS 04.03.00.05](http://software-dl.ti.com/processor-sdk-rtos/esd/AM335X/latest/index_FDS.html)
- AM335x PDK 1.0.10
- XDC tools 3.50.3.33
- SYSBIOS 6.52.0.12
- NDK 2.26.0.08
- EDMA 2.12.05

### Patch the SDK
The TI SDK contains a Uart driver configured to work as text mode, the driver needs some modifications to work in binary mode.
The SDK need to be patched and rebuild, it can be done manually or with the script setupenv.bat

First of all download the tools [patch](http://gnuwin32.sourceforge.net/packages/patch.htm) and [make](http://gnuwin32.sourceforge.net/packages/make.htm), and extract the content.
Call the setupenv.bat script:

` # setupenv.bat <PATH TO PATCH FOLDER> <PATH TO MAKE FOLDER> <PATH TO PDK FOLDER> `

Example:

` # utils\setupenv.bat C:\Users\MyUser\Downloads\patch-2.5.9-7-bin\ C:\Users\MyUser\Downloads\make-3.81-bin C:\ti\pdk_am335x_1_0_10\ `


### Texas instruments projects
1. Clone the repository in your desktop.
2. Install [Code Compose Studio](http://www.ti.com/tool/ccstudio-C2000 "Code Compose Studio") and check *Sitara AMx Processors* package.
3. Open CCS and select the workspace, for example the default workspace. Important: any folder is valid except the repository.
4. Import existing projects using the import CCS Project wizard (Tool bar -> Project/Import CCS Projects...).
5. Select the Release configuration and click on the build tool.

#### Debugging
1. Repeat the steps above but select the Debug configuration.
2. Make sure that you have a debugger tool connected to the PC and to the target board.
3. Edit or create the target configuration (the first time):
    1. Create/open a .ccxml file.
    2. (eval board) Select *TI XDS100v2 USB Debug Probe* and select the device *ICE_AM3359*.
    3. Save the configuration and click on *Test Connection* to verify it.
4. If desired, open a terminal (e.g. TeraTerm) to see the UART (baud to 115200).
5. Click on the *bug* icon arrow and select the desired target configuration.
6. Right click on *CortxA8* and select *Connect Target*. The GEL initialization script will execute.
7. Click Run -> Load -> Load Program and browse to the project executable (.out) file.
8. Enjoy your debugging!

### Who do I talk to? ###

* This repository is maintained by Ingenia FW team.
