# Annotation & YOLO Training Pipeline

This is the pipeline to run when you have new images and need to produce a new `best.pt` for the Jetson.

All scripts live in `yolo_comparison_test/path2_switch_proposal/scripts/`.

---

## Overview

```
captures/ (raw images)
    │
    ▼
01_autolabel.py          ← HSV auto-labels → YOLO .txt format
    │
    ▼
path2_dataset/           ← images/ + labels/ + dataset.yaml
    │
    ▼
02_finetune.py           ← fine-tune YOLOv11n → best.pt
    │
    ├── validation_step1_proper_split.py   ← 80/20 train/val split
    ├── validation_step2_retrain.py        ← retrain on train-only
    ├── validation_step3_val_inference.py  ← held-out metrics
    ├── validation_step4_overfit_check.py  ← loss gap check
    └── validation_step5_stress_test.py    ← UAV noise robustness
    │
    ▼
best.pt → copy to Jetson
```

---

## Step 0 — Collect images

Capture 30–50 images per buoy color at actual venue conditions. See [02_data_pipeline.md](02_data_pipeline.md) for how to use `camera_capture_spacebar.py`.

Place all raw captures flat in `yolo_comparison_test/path2_switch_proposal/captures/` (not in subdirectories).

Then place **one representative reference crop** per color into:
```
yolo_comparison_test/path2_switch_proposal/captures/classes/
├── red.jpg      ← tight crop of a red buoy
├── green.jpg    ← tight crop of a green buoy
└── blue.jpg     ← tight crop of a blue buoy
```

**Important:** filename stems must be exactly `red`, `green`, `blue`. The auto-label script reads the color name from the filename — any other names are silently ignored. One crop per color is sufficient; pick the most representative, well-lit shot.

---

## Step 1 — Auto-label

Auto-labeling uses the HSV detector to generate YOLO bounding box labels. No manual drawing needed as a first pass.

```bash
cd yolo_comparison_test/path2_switch_proposal/scripts
python 01_autolabel.py
```

**What it does:**
1. Reads reference crops from `captures/classes/` → derives per-class HSV ranges
2. Runs HSV detection on every `.jpg` in `captures/` (ignoring `classes/` subdirectory)
3. Writes one `.txt` label file per image in YOLO format: `class cx cy w h` (normalized)
4. Copies images and labels into `path2_dataset/images/` and `path2_dataset/labels/`
5. Writes `path2_dataset/dataset.yaml` and `path2_dataset/autolabel_summary.txt`

**YOLO label format:**
```
0 0.512 0.431 0.087 0.112    ← red buoy
1 0.234 0.667 0.091 0.098    ← green buoy
2 0.789 0.312 0.083 0.104    ← blue buoy
```
Class IDs: `0=red, 1=green, 2=blue`

**Check the summary:**
```bash
cat path2_dataset/autolabel_summary.txt
# Expected: hundreds of boxes across all images, reasonably balanced per class
```

### Manual correction (optional but recommended)

Auto-labels can miss buoys or mislabel colors in tricky lighting. You can spot-check and fix labels using [LabelImg](https://github.com/HumanSignal/labelImg) or [Roboflow](https://roboflow.com):

```bash
pip install labelImg
labelImg path2_dataset/images path2_dataset/labels
```

In LabelImg: switch to YOLO format, open image folder, open label folder. Check bounding boxes, fix any wrong colors, delete false positives. Save.

Even fixing 10–20% of the worst labels makes a meaningful difference in training quality.

---

## Step 2 — Fine-tune (quick full run)

```bash
cd yolo_comparison_test/path2_switch_proposal/scripts
python 02_finetune.py
```

**What it does:**
- Loads base weights `yolo11n.pt` (YOLO Nano, pre-trained on COCO)
- Trains on the full `path2_dataset/` (train and val = same images in this script — use validation steps below for honest eval)
- Saves weights to `scripts/path2_training/balloon_finetune/weights/best.pt`
- Runs inference on all captures → annotated frames in `scripts/path2_results/annotated/`
- Writes `scripts/path2_results/detections.csv`

Training runs for 100 epochs by default on CPU or CUDA if available. On a MacBook this takes ~10–20 minutes for ~100 images. On Jetson it takes longer — prefer training on the Mac and copying weights over.

**Check annotated output:**
```
scripts/path2_results/annotated/   ← open any JPG to visually verify boxes
```

---

## Step 3 — Honest validation (run all 5 steps)

The full-dataset run in Step 2 trains and validates on the same images (not honest). Run the 5 validation scripts to get real held-out metrics.

```bash
cd yolo_comparison_test/path2_switch_proposal/scripts

python validation_step1_proper_split.py   # 80/20 split, seed=42 → scripts/dataset/
python validation_step2_retrain.py        # retrain on train split → training/balloon_proper/weights/best.pt
python validation_step3_val_inference.py  # held-out inference → val_annotated/ + honest_results.txt
python validation_step4_overfit_check.py  # train vs val loss gap → overfit verdict
python validation_step5_stress_test.py    # UAV noise stress test → stress_test_results.txt
```

**Read the results** (you are already inside `scripts/`):
```bash
cat honest_results.txt      # precision / recall / F1 / mAP50 per class
cat honest_map50.txt        # single mAP50 number
```

**Current baseline (from 6/26 training run):**

| Metric | Value |
|--------|-------|
| Val set | 22 images (never seen during training) |
| Precision | 0.944 |
| Recall | 0.977 |
| F1 | 0.960 |
| mAP50 | 0.967 |

Per-class:
- Red: P=1.000 / R=1.000 / F1=1.000
- Green: P=0.988 / R=1.000 / F1=0.994
- Blue: P=0.816 / R=0.909 / F1=0.860 ← weakest, watch this one

**Acceptance thresholds:**
- mAP50 > 0.80 → legitimate
- mAP50 drop vs training mAP50 < 0.15 → not overfit
- If blue precision/recall is much lower than red/green → collect more blue images

**Regenerate validation grids (visual sanity check):**
```bash
python regenerate_val_grids.py
# Saves to: balloon_ultralytics_runs/val/val_batch0_labels.jpg (ground truth)
# Saves to: balloon_ultralytics_runs/val/val_batch0_pred.jpg   (predictions)
# Script prints the exact path on exit
```

---

## Step 4 — Export to ONNX and copy to Jetson

`run_detection_jetson.sh` searches for `buoy_best.onnx` (not `.pt`). Export first:

```bash
# From repo root (Mac or Ubuntu, with ultralytics installed)
python3 -c "
from ultralytics import YOLO
m = YOLO('yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.pt')
m.export(format='onnx', imgsz=640)
"
# This creates best.onnx next to best.pt
```

Then copy to Jetson:
```bash
# USB-C option
scp yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.onnx \
    babydragon@192.168.55.1:~/robotx-navigation/buoy_best.onnx

# Router WiFi option
scp yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.onnx \
    babydragon@<JETSON_IP>:~/robotx-navigation/buoy_best.onnx
```

> **Why ONNX?** `run_detection_jetson.sh` looks for `buoy_best.onnx` first. ONNX also runs faster on Jetson via TensorRT than raw `.pt`.

> **Note — YOLO integration pending (TODO #2):** `run_detection_jetson.sh` passes `--yolo-model` and other flags to `camera_live_feed.py` that don't exist yet. Until TODO #2 is done, the `.onnx` file can be placed on the Jetson but won't be called automatically.

---

## Pre-existing model weights

| File | Notes |
|------|-------|
| `yolo11n.pt` (repo root) | Base YOLOv11 Nano — COCO pretrained, no balloon fine-tuning |
| `yolo_comparison_test/path2_switch_proposal/demo_preserved/weights/buoy_balloon_roboflow_best.pt` | Previous best model from Roboflow-labeled training run |
| `buoy_best.pt` (Jetson, `~/robotx-navigation/`) | Currently deployed model |

---

## What each team member can do in parallel

When you have lots of images to label and train on, split the work:

| Role | Task |
|------|------|
| **Collector** | Capture images with `camera_capture_spacebar.py`, sort into `captures/` |
| **Annotator** | Run auto-label, spot-check in LabelImg, fix worst errors |
| **Trainer** | Run `01_autolabel.py` + `02_finetune.py` + validation steps on Mac |
| **Field op** | Set up WiFi, ground station, Jetson connection while training runs |

---

## Install requirements (first time only)

```bash
cd yolo_comparison_test/path2_switch_proposal
pip install -r requirements.txt
# Installs: ultralytics, opencv-python, numpy, etc.
```
