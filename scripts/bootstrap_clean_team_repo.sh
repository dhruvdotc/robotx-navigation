#!/usr/bin/env bash
# Option B: fresh clean repo on dhruvdotc (no Cursor co-author — run in Terminal.app)
set -euo pipefail

REPO_NAME="${REPO_NAME:-robotx-navigation}"
SRC="$(cd "$(dirname "$0")/.." && pwd)"
DST="${HOME}/${REPO_NAME}"

echo "Source: $SRC"
echo "Target: $DST"

if gh repo view "dhruvdotc/${REPO_NAME}" >/dev/null 2>&1; then
  echo "Repo dhruvdotc/${REPO_NAME} already exists on GitHub."
else
  gh repo create "$REPO_NAME" --public \
    --description "RobotX aerial buoy detection + MAVLink GCS — clean team repo"
fi

if [[ -d "$DST/.git" ]]; then
  echo "Using existing clone: $DST"
else
  if [[ -d "$DST" ]]; then
    echo "Removing non-git folder $DST ..."
    rm -rf "$DST"
  fi
  gh config set git_protocol https
  git clone "https://github.com/dhruvdotc/${REPO_NAME}.git" "$DST"
fi

cd "$DST"

# Strip Cursor co-author if any tool adds it
mkdir -p .git/hooks
cat > .git/hooks/prepare-commit-msg <<'HOOK'
#!/bin/bash
sed -i '' '/cursoragent@cursor\.com/d' "$1" 2>/dev/null || sed -i '/cursoragent@cursor\.com/d' "$1"
sed -i '' '/Made-with: Cursor/d' "$1" 2>/dev/null || sed -i '/Made-with: Cursor/d' "$1"
HOOK
chmod +x .git/hooks/prepare-commit-msg

mkdir -p fulldemo mavlink_comms scripts \
  yolo_comparison_test/path2_switch_proposal/demo_preserved/weights \
  jetson_flight_logs/repo_detection_logs/successful_flight_6_2

cp "$SRC/camera_live_feed.py" "$SRC/camera_capture_spacebar.py" "$SRC/jetson_setup.sh" .
cp -R "$SRC/fulldemo/." fulldemo/
cp -R "$SRC/mavlink_comms/." mavlink_comms/
cp "$SRC/scripts/"*.sh scripts/
cp "$SRC/yolo_comparison_test/path2_switch_proposal/demo_preserved/weights/buoy_balloon_roboflow_best.pt" \
   "$SRC/yolo_comparison_test/path2_switch_proposal/demo_preserved/weights/buoy_balloon_roboflow_best.onnx" \
   yolo_comparison_test/path2_switch_proposal/demo_preserved/weights/ 2>/dev/null || true

if [[ -d "$SRC/jetson_flight_logs/repo_detection_logs/successful_flight_6_2" ]]; then
  cp "$SRC/jetson_flight_logs/repo_detection_logs/successful_flight_6_2/"*.log \
     "$SRC/jetson_flight_logs/repo_detection_logs/successful_flight_6_2/README.md" \
     "$SRC/jetson_flight_logs/repo_detection_logs/successful_flight_6_2/RESTORE.md" \
     "$SRC/jetson_flight_logs/repo_detection_logs/successful_flight_6_2/SHA256SUMS" \
     jetson_flight_logs/repo_detection_logs/successful_flight_6_2/ 2>/dev/null || true
fi

find . -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
rm -f fulldemo/detections.jsonl

cat > .gitignore <<'EOF'
.DS_Store
.cursor/
.cursorignore
.cursorindexingignore
__pycache__/
*.py[cod]
.venv*/
detection_logs/
fulldemo/detections.jsonl
*.avi
*.mov
*.part-*
captures/
archives/*
yolo_comparison_test/**/runs/
yolo_comparison_test/**/captures/
yolo11n.pt
yolo_comparison_test/path2_switch_proposal/demo_preserved/weights/buoy_roi_best.pt
jetson_flight_logs/**/*
!jetson_flight_logs/repo_detection_logs/successful_flight_6_2/
!jetson_flight_logs/repo_detection_logs/successful_flight_6_2/**
jetson_flight_logs/repo_detection_logs/successful_flight_6_2/recording_demo_flight_mac.mp4
jetson_flight_logs/repo_detection_logs/successful_flight_6_2/recording_demo_flight_mac_screen.mov
EOF

cat > README.md <<'EOF'
# robotx-navigation

Clean team repo for **live buoy detection** (Jetson) → **MAVLink UDP** → **Mac ground station**.

Forked from [saxysteph/145-237D-robotx-navigation](https://github.com/saxysteph/145-237D-robotx-navigation) (UCSD CSE 237D / RobotX 2026).

## Quick start

**Mac (Terminal 1):**
```bash
bash fulldemo/run_gcs_mac.sh
```

**Jetson (Terminal 2):**
```bash
ssh babydragon@<JETSON_IP>
cd ~/robotx-navigation
GCS_IP=<MAC_IP> bash fulldemo/run_detection_jetson.sh
```

See `fulldemo/README.md` and `fulldemo/PARTNER_INSTRUCTIONS.md` for WiFi router setup, tuning, and troubleshooting.

## Layout

| Path | Purpose |
|------|---------|
| `camera_live_feed.py` | YOLO + HSV + GPS + MAVLink pipeline |
| `fulldemo/` | One-command Mac/Jetson demo scripts |
| `mavlink_comms/` | UDP buoy protocol + ground station |
| `scripts/` | Jetson WiFi helpers |
| `yolo_comparison_test/.../buoy_balloon_roboflow_best.pt` | Roboflow-trained detector |
| `jetson_setup.sh` | Jetson dependency bootstrap |

## Weights

Runtime prefers (in order): `buoy_best.onnx`, Roboflow `.onnx`, Roboflow `.pt` — see `fulldemo/run_detection_jetson.sh`.
EOF

cat > TEAM_SETUP.md <<'EOF'
# Team setup checklist

1. **Mac:** clone this repo, run `bash mavlink_comms/scripts/setup_mavlink_env.sh` once (or let `run_gcs_mac.sh` create `.venv-mavlink`).
2. **Jetson:** clone to `~/robotx-navigation`, run `bash jetson_setup.sh`.
3. **Network:** both devices on same router WiFi (not UCSD-GUEST). See `scripts/jetson_wifi_setup.sh`.
4. **Demo:** Mac `run_gcs_mac.sh` first, then Jetson `GCS_IP=<mac-ip> run_detection_jetson.sh`.
5. **Success:** Jetson `[TX]` lines match Mac `[GCS]` JSON on UDP port 14555.

Do not commit router passwords, Jetson SSH passwords, or local `detection_logs/` video dumps.
EOF

git add -A
git status

if git diff --cached --quiet; then
  echo "Nothing new to commit (repo may already be bootstrapped)."
else
  git commit -m "$(cat <<'EOF'
Initial clean import for team drone iterations.

YOLO+HSV live pipeline, fulldemo scripts, MAVLink GCS, Roboflow weights,
and successful 6/2 flight TX/RX logs (no large video blobs).
EOF
)"
fi

git push -u origin main

echo ""
echo "Done: https://github.com/dhruvdotc/${REPO_NAME}"
echo "Author check:"
git log -1 --format='Author: %an <%ae>%nBody:%n%B'
