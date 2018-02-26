# saab-coco

### Highlights ###

* Develop for [STM32F4](http://www.st.com/en/microcontrollers/stm32f4-series.html?querycriteria=productId=SS1577) devices
* Developed with [TrueSTUDIO](https://atollic.com/truestudio/) from atollic and [STM32CubeMx](http://www.st.com/en/development-tools/stm32cubemx.html) tool from STmicroelectronics.
* Compiled using ANSI/ISO C89 language and the GNU Tools for ARM
Embedded Processors (Build 17.03).


## Getting started
### Install Atollic TrueStudio (from release 9.0.0)
1. Download [atollic truestudio](https://atollic.com/resources/download/).
2. Execute the downloaded file and just keep clicking on Next (windows style).

## Install STM32CubeMX
1. Download [STM32CubeMX Eclipse plug-in](http://www.st.com/en/development-tools/stsw-stm32095.html)
2. Open trueStudio and click on *Help>Install New Software...* tab.
3. Click on *Add...* button.
4. On the new window, click on *Local...* and look for the directory where STM32CubeMX plug-in has been downloaded.
5. Click on *Archive...* and select the downloaded file.
6. Click on *OK* and select the added site from the list.
7. A STM32CubeMX software will appear.
8. Select it and install it.
9. Leave Eclipse do its job.
10. 
### Import atollic project projects
1. Clone the repository in your desktop.
4. Open TrueStudio (if it is the first time, locate the workspace wherever you want).
5. Click on *File>Open Projects From File System...*
6. Select the Release configuration and click on the build tool.
7. Look for the folder where the repository has been downloaded and choose ccore-lite project from the project list.
8. Use the *Build* buttons to compile the code.

#### Debugging
1. Once the project is imported, Click on *Debug* button
 

### Contribution guidelines ###
1. STM32CubeMX is a code generator tool that abstract the users from the MCU registers and configuration. If New HW modules are needed to be implemented, this tool must be used.


### Who do I talk to? ###

* This repository is maintained by Ingenia FW team.
