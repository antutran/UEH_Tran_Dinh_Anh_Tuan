@echo off
setlocal
title [UEH CRC 2026] 2. Chay Node Dieu khien Robot (starter_node)
cd /d "%~dp0"

echo ==============================================================
echo   UEH CRC 2026 - Chay Robot (starter_node: tien va tranh vat can)
echo ==============================================================
echo Dang ket noi vao container crc dang chay...
echo Nhan Ctrl+C de dung robot.
echo.

wsl -d Ubuntu-22.04 bash -c "docker exec -it crc bash -lc 'source /ws_build/install/setup.bash && ros2 run crc_sim starter'"

echo.
echo Robot da dung.
pause
