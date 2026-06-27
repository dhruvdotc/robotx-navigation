#!/bin/bash
# Run ON the Jetson (SSH or local terminal) to prove the board is alive.
# Usage:
#   bash scripts/jetson_heartbeat.sh          # one-shot
#   bash scripts/jetson_heartbeat.sh --loop   # print every 5s (Ctrl+C to stop)

set -euo pipefail

LOOP=0
if [[ "${1:-}" == "--loop" ]]; then
  LOOP=1
fi

stamp() {
  date '+%Y-%m-%d %H:%M:%S'
}

print_status() {
  echo "=============================================="
  echo " JETSON ALIVE — $(stamp)"
  echo "=============================================="
  echo "hostname:   $(hostname 2>/dev/null || echo unknown)"
  echo "uptime:     $(uptime 2>/dev/null || echo n/a)"
  if [[ -r /proc/device-tree/model ]]; then
    echo "model:      $(tr -d '\0' </proc/device-tree/model)"
  fi
  if command -v nvidia-smi >/dev/null 2>&1; then
    echo "gpu:        $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || echo n/a)"
  fi
  echo "ip addrs:"
  ip -4 addr show 2>/dev/null | awk '/inet / {print "  " $2 " on " $NF}' || ifconfig 2>/dev/null | grep 'inet ' || true
  echo "disk:       $(df -h / 2>/dev/null | awk 'NR==2 {print $3 " used / " $2 " total (" $5 ")"}')"
  MARKER="${HOME}/jetson_alive.txt"
  echo "$(stamp) ok" >> "${MARKER}"
  echo "marker:     ${MARKER} (appended)"
  echo "=============================================="
}

if [[ "${LOOP}" -eq 1 ]]; then
  while true; do
    print_status
    sleep 5
  done
else
  print_status
fi
