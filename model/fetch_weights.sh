#!/usr/bin/env bash
# Fetch the last validated buoy-detector weights from a GitHub Release,
# instead of running the full ~50 min model/run_pipeline.sh retrain.
#
# Usage: model/fetch_weights.sh [--tag <release-tag>]
#
# Places files at the exact paths the rest of the pipeline expects:
#   yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/{best.pt,best.onnx}
# and copies the onnx to repo-root buoy_best.onnx (what fulldemo/run_detection_jetson.sh looks for).
set -euo pipefail

REPO="dhruvdotc/robotx-navigation"
TAG="model-2026-07-31"   # bump this when a new validated model is released
while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag) TAG="$2"; shift 2 ;;
    -h|--help) sed -n '1,10p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/.." && pwd)"
WEIGHTS_DIR="$REPO_ROOT/yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights"
mkdir -p "$WEIGHTS_DIR"

echo "Fetching weights from $REPO release '$TAG'..."

if command -v gh >/dev/null 2>&1; then
  if ! gh release download "$TAG" --repo "$REPO" \
      -p 'best.pt' -p 'best.onnx' -D "$WEIGHTS_DIR" --clobber; then
    echo "FATAL: gh release download failed. Does release '$TAG' exist on $REPO?" >&2
    echo "       List releases: gh release list --repo $REPO" >&2
    exit 1
  fi
else
  echo "[INFO] gh CLI not found; falling back to curl against the release asset URLs."
  BASE="https://github.com/$REPO/releases/download/$TAG"
  for f in best.pt best.onnx; do
    if ! curl -fSL "$BASE/$f" -o "$WEIGHTS_DIR/$f"; then
      echo "FATAL: failed to download $f from $BASE. Does release '$TAG' exist?" >&2
      exit 1
    fi
  done
fi

[[ -f "$WEIGHTS_DIR/best.pt" && -f "$WEIGHTS_DIR/best.onnx" ]] || {
  echo "FATAL: download reported success but best.pt/best.onnx are missing at $WEIGHTS_DIR" >&2
  exit 1
}

cp "$WEIGHTS_DIR/best.onnx" "$REPO_ROOT/buoy_best.onnx"

echo ""
echo "Done."
echo "  $WEIGHTS_DIR/best.pt"
echo "  $WEIGHTS_DIR/best.onnx"
echo "  $REPO_ROOT/buoy_best.onnx  (picked up automatically by fulldemo/run_detection_jetson.sh)"
