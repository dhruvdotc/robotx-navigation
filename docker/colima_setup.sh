#!/bin/bash
set -e

# Fix Docker daemon DNS so it can reach Docker Hub
sudo tee /etc/docker/daemon.json > /dev/null << 'EOF'
{
  "exec-opts": ["native.cgroupdriver=cgroupfs"],
  "features": {"buildkit": true},
  "dns": ["8.8.8.8", "1.1.1.1"]
}
EOF

sudo systemctl restart docker
sleep 4
echo "[INFO] Docker restarted, pulling base image..."
docker pull ros:humble-ros-base
echo "[DONE] Pull complete"
