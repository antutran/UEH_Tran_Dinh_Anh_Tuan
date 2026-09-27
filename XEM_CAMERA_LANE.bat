@echo off
setlocal
title [UEH CRC 2026] Xem Camera Do Lan (rqt_image_view)
cd /d "%~dp0"

for /f "delims=" %%i in ('wsl wslpath -a "%cd%"') do set "WSL_PATH=%%i"

echo ==============================================================
echo   UEH CRC 2026 - Mo Cua so Xem Camera Do Lan va Toan Canh
echo ==============================================================
echo Dang mo rqt_image_view...
echo Ban co the dung menu tha xuong (dropdown) de chon:
echo   - /camera/lane_debug : Xem camera phan tich do lan (kem HUD)
echo   - /camera/image_raw  : Xem camera goc cua xe
echo   - /sky_cam/image_raw : Xem toan canh sa ban tu tren cao
echo.
echo Nhan Ctrl+C hoac dong cua so de thoat.
echo ==============================================================
echo.

wsl -d Ubuntu-22.04 bash -c "docker exec -it crc bash /ws/scripts/open_camera.sh"

pause
