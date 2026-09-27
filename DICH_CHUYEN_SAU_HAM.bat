@echo off
setlocal
title [UEH CRC 2026] Dich chuyen xe ra giua vong cung sau ham
cd /d "%~dp0"

echo ==============================================================
echo   UEH CRC 2026 - DICH CHUYEN XE RA GIUA VONG CUNG SAU HAM
echo ==============================================================
echo Dang kiem tra container mo phong Gazebo...

REM Kiem tra xem container crc co dang chay khong
docker ps | findstr /i "crc" >nul 2>&1
if %errorlevel% neq 0 (
    echo [CANH BAO] Container "crc" chua chay!
    echo Ban can khoi dong Gazebo bang file "1_start_gazebo.bat" truoc.
    echo.
    pause
    exit /b 1
)

echo Dang gui lenh dich chuyen xe toi Gazebo...
echo.

wsl -d Ubuntu-22.04 bash -c "docker exec crc bash /ws/scripts/teleport_after_tunnel.sh %*"

echo.
echo ==============================================================
echo Hoan tat! Nhan phim bat ky de dong cua so nay.
pause >nul
