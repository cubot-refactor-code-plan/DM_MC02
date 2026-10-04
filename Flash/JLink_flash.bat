@echo off
setlocal enabledelayedexpansion

for %%i in ("%~dp0\..") do (
    set "PROJECT_DIR=%%~fi"
    set "PROJECT_NAME=%%~nxi"
)

set "BUILD_TYPE=%~1"
if "%BUILD_TYPE%"=="" set "BUILD_TYPE=Debug"

cd /d "%PROJECT_DIR%" || exit /b 1

set "ELF_FILE=build\%BUILD_TYPE%\%PROJECT_NAME%.elf"
if not exist "!ELF_FILE!" (
    for /f "delims=" %%f in ('dir /b /s build\*.elf 2^>nul') do (
        set "ELF_FILE=%%f"
        goto :found
    )
)
:found

if not exist "!ELF_FILE!" (
    echo Error: ELF file not found under build\
    exit /b 1
)

where openocd >nul 2>nul
if errorlevel 1 (
    echo Error: openocd not found; this script drives J-Link through OpenOCD's jlink adapter.
    exit /b 1
)

echo Project: !PROJECT_NAME! (!BUILD_TYPE!)
echo ELF    : !ELF_FILE!
openocd -f Flash/jlink.cfg ^
    -c "gdb_port disabled" ^
    -c "tcl_port disabled" ^
    -c "telnet_port disabled" ^
    -c "program \"!ELF_FILE!\" verify reset exit"
if errorlevel 1 exit /b %errorlevel%

echo Done.
exit /b 0
