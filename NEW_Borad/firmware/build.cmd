@echo off
rem =========================================================
rem  STM32F103C8T6 小车固件 一键构建（双击即可）
rem  流程：CubeMX 无头生成 MDK-ARM 工程 -> 挂载 mycode
rem        -> 注入 main.c 入口 -> UV4 编译
rem  工程落地在 D:\tmp_my\STM32F103C8_Car
rem  参数会原样转给 build.ps1，例如：
rem    build.cmd -NoGenerate
rem    build.cmd -MxToolchain "MDK-ARM V5"
rem    build.cmd -CubemxRoot "E:\STM32CUBEMX" -Uv4Path "C:\Keil\UV4\UV4.exe"
rem =========================================================
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*
exit /b %ERRORLEVEL%
