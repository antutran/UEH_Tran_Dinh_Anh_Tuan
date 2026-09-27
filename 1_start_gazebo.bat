@echo off
setlocal
title [UEH CRC 2026] 1. Khoi dong Gazebo Simulation
cd /d "%~dp0"

for /f "delims=" %%i in ('wsl wslpath -a "%cd%"') do set "WSL_PATH=%%i"

echo ==============================================================
echo   UEH CRC 2026 - Khoi dong mo phong Gazebo tren WSL2 / Docker
echo ==============================================================
echo [1/2] Dang kiem tra va khoi chay container Gazebo...
echo Thu muc WSL: %WSL_PATH%
echo.

wsl -d Ubuntu-22.04 bash -c "cd '%WSL_PATH%/scripts' && bash run_docker.sh up"

echo.
echo [2/2] Cua so Gazebo dang duoc mo len man hinh.
echo (Ban co the thu nho cua so nay, de Gazebo tiep tuc chay ngam).
echo.
echo De chay robot: Hay mo file '2_run_robot.bat' hoac '3_teleop_keyboard.bat'
echo De dung mo phong: Hay chay file 'stop_gazebo.bat'
echo ==============================================================
pause
