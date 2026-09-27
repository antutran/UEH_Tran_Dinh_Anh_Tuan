#!/usr/bin/env bash
# 1-Click Startup: Launches Gazebo + Robot + Camera View
set -e
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Start Gazebo
bash "$SELF/run_docker.sh" up

# 2. Wait for Gazebo master & robot model to finish spawning
echo ">> Cho Gazebo va sa ban khoi tao (khoang 6 giay)..."
sleep 6

# 3. Start autonomous robot node
echo ">> Dang kich hoat robot tu dong chay..."
docker exec -d crc bash /ws/scripts/auto_drive.sh
sleep 1

# 4. Open Camera View directly on screen!
echo ">> Dang mo cua so Camera Do Lan len man hinh..."
docker exec -d crc bash /ws/scripts/open_camera.sh

echo ">> Thanh cong! Robot dang chay va Cua so Camera da duoc bat len man hinh."
