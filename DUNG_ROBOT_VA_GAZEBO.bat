@echo off
setlocal
title [UEH CRC 2026] Dung Mo phong
cd /d "%~dp0"

for /f "delims=" %%i in ('wsl wslpath -a "%cd%"') do set "WSL_PATH=%%i"

echo ==============================================================
echo   Dang dung toan bo Gazebo va Robot...
echo ==============================================================

wsl -d Ubuntu-22.04 bash -c "cd '%WSL_PATH%/scripts' && bash run_docker.sh down"

echo.
echo Da tat hoan toan mo phong.
ping 127.0.0.1 -n 4 >nul
