@echo off
setlocal
title [UEH CRC 2026] 5. Kiem tra Cảm bien Robot (sensor_check)
cd /d "%~dp0"

echo ==============================================================
echo   UEH CRC 2026 - Kiem tra trang thai Camera, LiDAR, Odom, IMU
echo ==============================================================
echo Nhan Ctrl+C de thoat.
echo.

wsl -d Ubuntu-22.04 bash -c "docker exec -it crc bash -lc 'source /ws_build/install/setup.bash && ros2 run crc_sim sensor_check'"

echo.
pause
