# Serial Bridge

## Overview ##
This repository contains different examples of bridge implementation using the MCB library. The examples are based on different platforms including low MCU performance without OS and high MPU performance such as the sitara family available on development board like beaglebone.

All the examples implements a bridge between a serial communication based on UART or USB (CDC) protocols and the extended SPI used for motion control bus.

## Init Repository ##
To clone the repository:

` # git clone https://github.com/ingeniamc/serial-bridge.git `

` # cd serial-bridge `

` # git submodule init `

` # git submodule update `

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

#### Sitara [Processor SDK RTOS 04.03.00](http://software-dl.ti.com/processor-sdk-rtos/esd/docs/latest/rtos/Release_Specific.html#release-04-03-00)
- EDMA3 LLD 2.12.05.30C
- GNU GCC ARM 6.1
- NDK 2.26.00.08
- PDK AM335x 1.0.10
- SYS/BIOS 6.52.00.12
- XDC tools 3.50.03.33

[Download](http://software-dl.ti.com/processor-sdk-rtos/esd/AM335X/latest/index_FDS.html)

It is highly recommended to install the SDK into the default path. In case of installing the SDK in another path, follow the instructions from this [link](http://processors.wiki.ti.com/index.php/Rebuilding_The_PDK) to set up correctly the environment variables.

### Patch the SDK
The TI SDK contains a UART driver configured to work with text data, the driver needs some modifications to work in binary mode.
Coco bridge example needs the coco board to be added to the TI SDK.
The SDK has to be patched and rebuild, it can be done manually or with the script *setupenv*.

#### Windows
Download and install the tools [patch](http://gnuwin32.sourceforge.net/packages/patch.htm)

Note: the default installation location of the Gnu utilities is C:\Program Files (x86)\GnuWin32

Call the *setupenv.bat* script with administrator permissions, this script is located in utils folder of the repository:

` # <PATH TO SCRIPT>\setupenv.bat <PATH TO PATCH FOLDER> <PATH TO PDK FOLDER> `


Example:

` # C:\Users\MyUser\Downloads\serial-bridge\utils\setupenv.bat C:\Users\MyUser\Downloads\patch-2.5.9-7-bin\bin\ C:\ti\pdk_am335x_1_0_10\ `

Note: if the Gnu utilities were downloaded to the default location on the C: drive, the arguments to the setupenv.bat function will need to be enclosed in quotes in order for DOS to correctly parse the path.

Example:

` # C:\serial-bridge\utils>setupenv.bat "C:\Program Files (x86)\GnuWin32\bin" "c:\ti\pdk_am335x_1_0_10" `


If the patch utility ask about reverse previous path, overwrite it with *y* option.

#### Linux
Download and add to path the *patch* and *make* tools. Make the script executable and execute:

` # chmod +x <PATH TO REPO>/utils/setupenv.sh `

` # <PATH TO REPO>/utils/setupenv.sh <PATH TO PDK FOLDER> `

Example

` # /home/MyUser/Documents/serial-bridge/utils/setupenv.sh /home/MyUser/ti/pdk_am335x_1_0_10 `

If the patch utility ask about reverse previous path, overwrite it with *y* option.

### Texas instruments projects
1. Clone the repository and patch the SDK.
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
