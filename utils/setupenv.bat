@echo off

if [%3]==[] goto usage

@echo %~dp0

%1\bin\patch.exe %3\packages\ti\drv\uart\src\v1\UART_v1.c %~dp0\UART_v1.patch
%1\bin\patch.exe %3\packages\ti\drv\uart\src\UART_drv.c %~dp0\UART_drv.patch
@echo Patch applied, gonna rebuild SDK
PUSHD %3\packages
%2\bin\make.exe uart
POPD
@echo Build succeed.
goto :eof

:usage
@echo Usage: %0 ^<PATH TO PATCH FOLDER^> ^<PATH TO MAKE FOLDER^> ^<PATH TO PDK FOLDER^> 
exit /B 1