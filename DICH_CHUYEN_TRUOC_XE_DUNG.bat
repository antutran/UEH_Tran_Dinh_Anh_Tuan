@echo off
setlocal
title [UEH CRC 2026] Dich chuyen xe ra truoc xe dung tren cao toc
cd /d "%~dp0"

echo ==============================================================
echo   UEH CRC 2026 - DICH CHUYEN XE RA TRUOC XE DUNG (CAO TOC)
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

echo Dang gui lenh dich chuyen xe toi vi tri x = -0.50m, y = 1.68m, yaw = 180 do...
echo.

wsl -d Ubuntu-22.04 bash -c "docker exec crc bash /ws/scripts/teleport_after_tunnel.sh -0.50 1.68 180.0"

echo.
echo ==============================================================
echo Hoan tat! Xe da san sang truoc xe dung de test vuot xe.
echo Nhan phim bat ky de dong cua so nay.
pause >nul
