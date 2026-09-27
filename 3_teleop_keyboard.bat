@echo off
setlocal
title [UEH CRC 2026] 3. Lai Robot bang Ban phim (teleop_twist_keyboard)
cd /d "%~dp0"

echo ==============================================================
echo   UEH CRC 2026 - Lai Robot bang Ban phim
echo ==============================================================
echo Phim dieu khien:
echo    u    i    o      (Tien trai / Tien thang / Tien phai)
echo    j    k    l      (Quay trai / Dung lai / Quay phai)
echo    m    ,    .      (Lui trai  / Lui thang / Lui phai)
echo.
echo    w / x : Tang / giam toc do di chuyen
echo    e / c : Tang / giam toc do quay
echo.
echo Nhan Ctrl+C de thoat.
echo ==============================================================
echo.

wsl -d Ubuntu-22.04 bash -c "docker exec -it crc bash -lc 'source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard'"

echo.
pause
