@echo off
rem 对 User\ 与 QSPI_Flash\ 做 clang-format 检查或格式化。
rem
rem 用法：
rem   Tools\clang_format.bat check  [文件...]   只检查，不符合规范则退出码 1
rem   Tools\clang_format.bat format [文件...]   原地格式化
rem
rem 不传文件时默认覆盖 User\ 与 QSPI_Flash\ 下的 .c/.h/.cpp/.hpp。
rem Core\ 由 CubeMX 生成、第三方目录为上游代码，两者都不处理。

setlocal enabledelayedexpansion
set "ROOT=%~dp0.."
set "MODE=%~1"
if "%MODE%"=="" set "MODE=check"

rem clang-format 优先取 VS Code C/C++ 扩展自带的版本，取不到再退回 PATH。
set "CLANG_FORMAT="
for /d %%D in ("%USERPROFILE%\.vscode\extensions\ms-vscode.cpptools-*") do (
    if exist "%%D\LLVM\bin\clang-format.exe" set "CLANG_FORMAT=%%D\LLVM\bin\clang-format.exe"
)
if "%CLANG_FORMAT%"=="" set "CLANG_FORMAT=clang-format"

set "FILES="
if not "%~2"=="" (
    for %%F in (%*) do (
        if not "%%F"=="%MODE%" set "FILES=!FILES! "%%F""
    )
) else (
    for %%E in (c h cpp hpp) do (
        for /r "%ROOT%\User" %%F in (*.%%E) do set "FILES=!FILES! "%%F""
        for /r "%ROOT%\QSPI_Flash" %%F in (*.%%E) do set "FILES=!FILES! "%%F""
    )
)

if "%MODE%"=="format" (
    call %CLANG_FORMAT% -i %FILES%
    if errorlevel 1 exit /b 1
    echo 格式化完成（%CLANG_FORMAT%）
    exit /b 0
)

if not "%MODE%"=="check" (
    echo 用法：%0 [check^|format] [文件...] 1>&2
    exit /b 2
)

set "FAILED=0"
for %%F in (%FILES%) do (
    call %CLANG_FORMAT% --dry-run --Werror "%%~F" >nul 2>&1
    if errorlevel 1 (
        echo 格式不符合 .clang-format: %%~F
        set "FAILED=1"
    )
)
if "%FAILED%"=="1" (
    echo 检查失败：运行 Tools\clang_format.bat format 统一格式后再提交
    exit /b 1
)
echo 格式检查通过
exit /b 0
