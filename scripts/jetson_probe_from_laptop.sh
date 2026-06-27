#!/bin/bash
# Run ON your Mac/laptop to see if the Jetson is reachable (LED may be dead).
#
# Setup USB-C link first (partner guide):
#   sudo ifconfig en10 192.168.55.100 netmask 255.255.255.0
#   (use your real interface name: en9, en10, en11, ...)
#
# Usage:
#   bash scripts/jetson_probe_from_laptop.sh
#   bash scripts/jetson_probe_from_laptop.sh 192.168.4.50   # WiFi IP instead

JETSON_IP="${1:-192.168.55.1}"
JETSON_USER="${JETSON_USER:-babydragon}"

echo "=== Probing Jetson at ${JETSON_IP} ==="

echo -n "ping ... "
if ping -c 1 -W 2 "${JETSON_IP}" >/dev/null 2>&1; then
  echo "OK"
else
  echo "FAIL (no reply — board off, wrong IP, or USB network not set up)"
  echo "Tip: ifconfig | grep -E '^en|192.168.55'"
  exit 1
fi

echo -n "ssh port 22 ... "
if nc -z -G 2 "${JETSON_IP}" 22 2>/dev/null; then
  echo "open"
else
  echo "closed (booting or ssh not running)"
  exit 1
fi

echo "ssh quick test (may prompt for password: companion) ..."
ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${JETSON_USER}@${JETSON_IP}" \
  'echo SSH_OK; hostname; uptime; test -f ~/jetson_alive.txt && tail -1 ~/jetson_alive.txt || echo no_marker_yet'

echo "=== If you see SSH_OK above, the Jetson is working (LED not required). ==="
