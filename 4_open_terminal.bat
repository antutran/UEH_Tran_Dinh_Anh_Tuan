@echo off
setlocal
title [UEH CRC 2026] 4. Terminal trong Container
cd /d "%~dp0"

for /f "delims=" %%i in ('wsl wslpath -a "%cd%"') do set "WSL_PATH=%%i"

echo ==============================================================
echo   UEH CRC 2026 - Mo Bash Terminal ben trong Container ROS 2
echo ==============================================================
echo Go 'exit' de thoat terminal.
echo.

wsl -d Ubuntu-22.04 bash -c "cd '%WSL_PATH%/scripts' && bash run_docker.sh sh"

pause
