#!/usr/bin/env sh
# run_demo.sh — One-command Hardware-Free Demo Launcher
set -e

make all

PORT="${PORT:-8080}"
URL="http://127.0.0.1:${PORT}/"

echo "================================================================"
echo " Starting Hardware-Free Auto-Demo on ${URL}"
echo " Virtual Hardware: MPU6050(250Hz), LM35, 4 Relays, SIM7670C, LoRa"
echo "================================================================"

# Open browser in background if a desktop environment is available
if command -v open >/dev/null 2>&1; then
  (sleep 1 && open "${URL}") &
elif command -v xdg-open >/dev/null 2>&1; then
  (sleep 1 && xdg-open "${URL}") &
fi

exec ./build/engine --auto-demo --seed 42 --port "${PORT}"
