#!/bin/bash
REPO=/Users/xurui/Downloads/ROBOTX/robotx-navigation
docker rm -f robotx-sim 2>/dev/null || true
docker run -d \
  --name robotx-sim \
  -v "${REPO}:/ws/robotx-navigation" \
  --network host \
  robotx-sim \
  bash -c "sleep infinity"
echo "Container started: $(docker ps --filter name=robotx-sim --format '{{.ID}}')"
