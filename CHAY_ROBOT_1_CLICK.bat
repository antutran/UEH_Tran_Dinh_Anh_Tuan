@echo off
setlocal
title [UEH CRC 2026] 1-Click: Mo Gazebo va Chay Robot
cd /d "%~dp0"

for /f "delims=" %%i in ('wsl wslpath -a "%cd%"') do set "WSL_PATH=%%i"

echo ==============================================================
echo   UEH CRC 2026 - KHOI DONG 1-CLICK (KHONG CAN GO LENH)
echo ==============================================================
echo 1. Dang mo sa ban va hien thi Gazebo...
echo 2. Dang tu dong kich hoat robot chay tren sa ban...
echo 3. Dang bat cua so Camera Do Lan len man hinh...
echo.

wsl -d Ubuntu-22.04 bash -c "cd '%WSL_PATH%/scripts' && bash start_all.sh"

echo.
echo ==============================================================
echo   DA MO GAZEBO, ROBOT DANG CHAY VA CAMERA DA DUOC BAT!
echo   Ban co the xem sa ban 3D va camera truc tiep tren man hinh.
echo   (Cua so nay se tu dong dong sau 5 giay...)
echo ==============================================================
ping 127.0.0.1 -n 6 >nul
