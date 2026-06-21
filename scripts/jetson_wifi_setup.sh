#!/bin/bash
# Run ON the Jetson (first time via USB-C SSH: babydragon@192.168.55.1)
# Connects TP-Link / USB WiFi to your router or phone hotspot for SSH over WiFi.
#
# Usage:
#   bash scripts/jetson_wifi_setup.sh                    # scan + status only
#   bash scripts/jetson_wifi_setup.sh "MyWifi" "password"  # connect once
#
# After connect: note the wlan IP and from laptop:
#   ssh babydragon@<wlan-ip>

set -euo pipefail

SSID="${1:-}"
PASS="${2:-}"

echo "=== Jetson WiFi setup ==="
echo "hostname: $(hostname)"
echo

echo "--- USB WiFi hardware ---"
lsusb | grep -iE 'realtek|tp-link|802.11|wireless' || echo "(no obvious WiFi in lsusb — is dongle plugged in?)"
echo

echo "--- kernel / driver (last 15 lines) ---"
dmesg 2>/dev/null | tail -20 | grep -iE 'wlan|8188|rtl|wifi|usb' || dmesg 2>/dev/null | tail -8
echo

echo "--- network interfaces ---"
ip -br link | grep -E 'wlan|wl' || ip -br link
echo

IFACE="$(ip -br link | awk '/^wl/ {print $1; exit}')"
if [[ -z "${IFACE}" ]]; then
  echo "ERROR: No wlan interface (wlan0). Common fixes for TP-Link TL-WN725N on JetPack 6:"
  echo "  sudo apt update && sudo apt install -y git dkms build-essential linux-headers-\$(uname -r)"
  echo "  git clone https://github.com/aircrack-ng/rtl8188eus.git && cd rtl8188eus"
  echo "  sudo make && sudo make install && sudo modprobe 8188eu"
  echo "  (Use a USB 2.0 port; replug dongle; rerun this script)"
  exit 1
fi

echo "Using WiFi interface: ${IFACE}"
echo

if ! command -v nmcli >/dev/null 2>&1; then
  echo "Installing NetworkManager CLI..."
  sudo apt update && sudo apt install -y network-manager
fi

sudo nmcli radio wifi on 2>/dev/null || true

echo "--- nearby networks ---"
nmcli dev wifi list ifname "${IFACE}" 2>/dev/null | head -15
echo

if [[ -n "${SSID}" ]]; then
  echo "Connecting to SSID: ${SSID}"
  if [[ -n "${PASS}" ]]; then
    sudo nmcli dev wifi connect "${SSID}" password "${PASS}" ifname "${IFACE}"
  else
    sudo nmcli dev wifi connect "${SSID}" ifname "${IFACE}"
  fi
  sleep 2
fi

echo "--- connection status ---"
nmcli -f GENERAL.STATE,IP4.ADDRESS dev show "${IFACE}" 2>/dev/null || true
echo
echo "All IPv4 addresses on this Jetson:"
hostname -I
echo

WIP="$(ip -4 addr show "${IFACE}" 2>/dev/null | awk '/inet / {print $2}' | cut -d/ -f1 | head -1)"
if [[ -n "${WIP}" ]]; then
  echo "SSH from laptop (same WiFi network):"
  echo "  ssh babydragon@${WIP}"
  echo
  echo "Buoy pipeline on Jetson — use your laptop WiFi IP for --gcs-ip:"
  echo "  python3 camera_live_feed.py ... --gcs-ip <laptop-wifi-ip>"
else
  echo "Not connected yet. Run:"
  echo "  bash scripts/jetson_wifi_setup.sh \"YourNetworkName\" \"YourPassword\""
  echo "Or interactive: sudo nmtui"
fi

# Ensure SSH server starts on boot
if systemctl is-enabled ssh >/dev/null 2>&1 || systemctl is-enabled sshd >/dev/null 2>&1; then
  echo "SSH service: enabled"
else
  echo "Enabling SSH on boot..."
  sudo systemctl enable --now ssh 2>/dev/null || sudo systemctl enable --now sshd 2>/dev/null || true
fi
