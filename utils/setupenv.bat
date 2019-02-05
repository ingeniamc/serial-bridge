@echo off

if [%2] == [] goto usage

@echo %~dp0

:: coco board
xcopy /s %~dp0\src\common\cocoV1\am335x_coco.c %2\packages\ti\starterware\board\am335x\
xcopy /s %~dp0\src\common\cocoV1\am335x_coco.h %2\packages\ti\starterware\board\am335x\
xcopy /s %~dp0\src\common\cocoV1\am335x_coco_pinmux_data.c %2\packages\ti\starterware\board\am335x\
xcopy /s %~dp0\src\common\cocoV1\sorte_cocoam335x_app.cfg %2\packages\ti\drv\pruss\example\apps\sorte\src\
mkdir %2\packages\ti\board\src\cocoAM335x
xcopy /s %~dp0\src\common\cocoV1\cocoAM335x %2\packages\ti\board\src\cocoAM335x

xcopy /s %~dp0\src\common\cocoV2\am335x_cocov2.c %2\packages\ti\starterware\board\am335x\
xcopy /s %~dp0\src\common\cocoV2\am335x_cocov2.h %2\packages\ti\starterware\board\am335x\
xcopy /s %~dp0\src\common\cocoV2\am335x_cocov2_pinmux_data.c %2\packages\ti\starterware\board\am335x\
xcopy /s %~dp0\src\common\cocoV2\sorte_cocoV2am335x_app.cfg %2\packages\ti\drv\pruss\example\apps\sorte\src\
mkdir %2\packages\ti\board\src\cocoV2AM335x
xcopy /s %~dp0\src\common\cocoV2\cocoV2AM335x %2\packages\ti\board\src\cocoV2AM335x
@echo Files copy done, gonna patch SDK files

%1\patch.exe %2\packages\ti\drv\uart\src\v1\UART_v1.c %~dp0\UART_v1.patch
%1\patch.exe %2\packages\ti\drv\uart\src\UART_drv.c %~dp0\UART_drv.patch
@echo UART patch applied, gonna apply board patch

%1\patch.exe -p2 --directory=%2 < %~dp0\pdk_am335x_1_0_10_coco_board.patch
@echo Board patch applied, gonna rebuild SDK

PUSHD %2\packages
pdksetupenv.bat & gmake.exe -k LIMIT_SOCS="am335x" clean starterware_libs board_lib spi csl gpio uart i2c osal
POPD
@echo Build succeed.

goto :eof

:usage
@echo Usage: %0 ^<PATH TO PATCH FOLDER^> ^<PATH TO PDK FOLDER^> 
exit /B 1