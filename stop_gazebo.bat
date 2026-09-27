@echo off
setlocal
title [UEH CRC 2026] Dung Mo phong Gazebo
cd /d "%~dp0"

for /f "delims=" %%i in ('wsl wslpath -a "%cd%"') do set "WSL_PATH=%%i"

echo ==============================================================
echo   UEH CRC 2026 - Dang dung Gazebo va container...
echo ==============================================================

wsl -d Ubuntu-22.04 bash -c "cd '%WSL_PATH%/scripts' && bash run_docker.sh down"

echo Mo phong da duoc dung hoan toan.
echo ==============================================================
timeout /t 3
