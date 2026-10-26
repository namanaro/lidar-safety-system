#!/bin/bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

DEBUG=0
for arg in "$@"; do
    if [ "$arg" == "--debug" ] || [ "$arg" == "-d" ] || [ "$arg" == "debug" ]; then
        DEBUG=1
    fi
done

# Build binaries
make -s

# Check serial port
PORT=$(grep -o '"port": *"[^"]*"' config.json | head -1 | cut -d'"' -f4)
if [ -z "$PORT" ]; then
    PORT="/dev/ttyUSB0"
fi

if [ ! -e "$PORT" ]; then
    echo "[WARNING] Serial port $PORT not found!"
    echo "Waiting for LiDAR device on $PORT..."
    while [ ! -e "$PORT" ]; do
        sleep 1
    done
    echo "LiDAR detected at $PORT!"
fi

# In debug mode, start the Web UI
if [ "$DEBUG" -eq 1 ]; then
    PORT_UI=8080
    if ! pgrep -f "./lidar-ui config.json" >/dev/null; then
        ./lidar-ui config.json $PORT_UI > ui.log 2>&1 &
        UI_PID=$!
        trap "kill $UI_PID 2>/dev/null" EXIT
        IP=$(hostname -I | awk '{print $1}')
        echo "=========================================================="
        echo "  [DEBUG MODE] Web UI Active"
        echo "  URL: http://$IP:$PORT_UI"
        echo "=========================================================="
    else
        echo "[DEBUG MODE] Web UI already running on port $PORT_UI"
    fi
fi

echo "Starting lidar-guard on $PORT..."
exec ./lidar-guard config.json
