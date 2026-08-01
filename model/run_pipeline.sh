#!/usr/bin/env bash
# One-command retraining for the buoy detector.
#
# Thin orchestrator: it does NOT reimplement any pipeline logic - it just chains
# the real scripts in yolo_comparison_test/path2_switch_proposal/scripts/ in the
# documented order (see docs/08_annotation_and_training.md), fails fast and loud
# at the validation gate, and prints a stage-by-stage summary.
#
#   captures/ (+ classes/red|green|blue.jpg)
#     -> 00_preprocess -> 01_autolabel -> [02_finetune] -> validation_step1..5
#     -> training/balloon_proper/weights/best.pt  (+ honest_results.txt)
#
# Usage:
#   model/run_pipeline.sh [--skip-finetune] [--onnx] [--min-map50 X] [-h]
#
#   --skip-finetune  Skip 02_finetune (the quick in-sample preview run). The
#                    deployed weights come from validation_step2 either way, so
#                    this just saves one training cycle.
#   --onnx           Export the final best.pt to ONNX after validation.
#   --min-map50 X    Fail the run if held-out mAP50 < X (default 0.80, the
#                    legitimacy threshold from docs/08).
set -euo pipefail

MIN_MAP50="0.80"
RUN_FINETUNE=1
DO_ONNX=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-finetune) RUN_FINETUNE=0; shift ;;
    --onnx) DO_ONNX=1; shift ;;
    --min-map50) MIN_MAP50="$2"; shift 2 ;;
    -h|--help) sed -n '1,24p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/.." && pwd)"
SCRIPTS="$REPO_ROOT/yolo_comparison_test/path2_switch_proposal/scripts"
CAPTURES="$REPO_ROOT/yolo_comparison_test/path2_switch_proposal/captures"
WEIGHTS="$SCRIPTS/training/balloon_proper/weights/best.pt"

[[ -d "$SCRIPTS" ]]  || { echo "FATAL: scripts dir not found: $SCRIPTS" >&2; exit 1; }
for c in red green blue; do
  [[ -f "$CAPTURES/classes/$c.jpg" ]] || {
    echo "FATAL: missing reference crop $CAPTURES/classes/$c.jpg" >&2
    echo "       Place one tight crop per colour (exact stems red/green/blue) - see docs/08." >&2
    exit 1; }
done

cd "$SCRIPTS"
STAGE_N=0
stage() {  # stage "Name" cmd...
  STAGE_N=$((STAGE_N + 1))
  local name="$1"; shift
  echo ""
  echo "============================================================"
  echo ">> STAGE $STAGE_N: $name"
  echo "   \$ $*"
  echo "============================================================"
  local t0 t1
  t0=$(date +%s)
  if ! "$@"; then
    echo "!! STAGE $STAGE_N FAILED ($name). Stopping." >&2
    exit 1
  fi
  t1=$(date +%s)
  echo "   [ok] $name  (${t1}-${t0}=$((t1 - t0))s)"
}

echo "Retraining buoy detector from: $CAPTURES"
echo "Deployed weights target:       $WEIGHTS"

stage "Preprocess (split + augment + normalize)" python3 00_preprocess_training_data.py
stage "Auto-label (HSV -> YOLO labels)"          python3 01_autolabel.py --captures-dir ../preprocessed_captures
if [[ "$RUN_FINETUNE" == "1" ]]; then
  stage "Fine-tune preview (02_finetune, in-sample)" python3 02_finetune.py
else
  echo ">> skipping 02_finetune (--skip-finetune)"
fi
stage "Proper split (stage leak-proof train/val)" python3 validation_step1_proper_split.py
stage "Retrain on train split -> deployed best.pt" python3 validation_step2_retrain.py

# ---- Validation gate: fail loudly if held-out mAP50 is too low ------------
MAP50="$(cat honest_map50.txt 2>/dev/null | tr -d '[:space:]' || true)"
echo ""
echo ">> Validation gate: held-out mAP50 = ${MAP50:-<none>} (threshold ${MIN_MAP50})"
if [[ -z "$MAP50" ]]; then
  echo "!! No honest_map50.txt produced - retrain did not report a metric. Stopping." >&2
  exit 1
fi
if awk "BEGIN{exit !($MAP50 < $MIN_MAP50)}"; then
  echo "!! GATE FAILED: mAP50 $MAP50 < $MIN_MAP50. Not a usable model. Stopping." >&2
  exit 1
fi
echo "   [ok] gate passed"

stage "Held-out inference metrics"        python3 validation_step3_val_inference.py
stage "Overfit check (train vs val loss)" python3 validation_step4_overfit_check.py
stage "Stress test (UAV noise)"           python3 validation_step5_stress_test.py

if [[ "$DO_ONNX" == "1" ]]; then
  stage "Export ONNX" python3 -c "from ultralytics import YOLO; YOLO('$WEIGHTS').export(format='onnx', imgsz=640)"
  # Put it where fulldemo/run_detection_jetson.sh looks first (repo-root buoy_best.onnx).
  cp "${WEIGHTS%.pt}.onnx" "$REPO_ROOT/buoy_best.onnx"
  echo "   ONNX exported: ${WEIGHTS%.pt}.onnx"
  echo "   Copied to $REPO_ROOT/buoy_best.onnx (fulldemo picks this up locally)"
  echo "   Jetson deploy: scp '$REPO_ROOT/buoy_best.onnx' <jetson>:~/robotx-navigation/buoy_best.onnx"
fi

echo ""
echo "############################################################"
echo "# PIPELINE COMPLETE"
echo "#   weights : $WEIGHTS"
echo "############################################################"
echo "--- honest_results.txt ---";     cat honest_results.txt      2>/dev/null || echo "(missing)"
echo "--- honest_map50.txt ---";       cat honest_map50.txt        2>/dev/null || echo "(missing)"
echo "--- stress_test_results.txt ---"; cat stress_test_results.txt 2>/dev/null || echo "(missing)"
