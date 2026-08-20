# `model/` - retrain the buoy detector on new images

This is the front door for **retraining**. When new real venue photos arrive,
start here. It does not hold any model code - it wraps the real pipeline in
`yolo_comparison_test/path2_switch_proposal/scripts/` (kept there so nothing's
relative paths break) behind one command.

For the deep, per-step reference see
[`docs/08_annotation_and_training.md`](../docs/08_annotation_and_training.md);
this file is the quickstart + gotchas, not a replacement for it.

**Don't want to retrain?** Trained weights aren't committed to keep the repo
small (`.pt`/`.onnx` are gitignored). Run `model/fetch_weights.sh` to pull the
last validated model from a GitHub Release in seconds - no GPU, no wait.
`run_pipeline.sh` below is for producing a *new* model from new captures.

---

## 1. Drop in new images

Two things go into `yolo_comparison_test/path2_switch_proposal/captures/`:

```
captures/
├── course1_frame_*.jpg      ← raw captures, flat (NOT in subfolders)
├── ...  (30-50 per colour at real venue lighting)
└── classes/
    ├── red.jpg              ← ONE tight crop of a red buoy
    ├── green.jpg            ← ONE tight crop of a green buoy
    └── blue.jpg             ← ONE tight crop of a blue buoy
```

> **The reference-crop filenames matter.** The stems must be exactly
> `red`, `green`, `blue` (`.jpg`). The auto-labeler reads the class name from the
> filename; anything else is silently ignored and that colour gets no labels.
> One well-lit, representative crop per colour is enough. Capture the frames with
> `camera_capture_spacebar.py` (see `docs/02_data_pipeline.md`).

The same `red.jpg/green.jpg/blue.jpg` crops also drive the **live** HSV detector
via `captures/classes/` at the repo root - keep the two in sync if you want the
HSV fallback path tuned to the same buoys.

---

## 2. One-command retrain

```bash
model/run_pipeline.sh                 # full pipeline
model/run_pipeline.sh --skip-finetune # skip the redundant preview training
model/run_pipeline.sh --onnx          # also export ONNX for the Jetson
model/run_pipeline.sh --min-map50 0.85  # stricter validation gate (default 0.80)
```

It chains, failing fast and loud at the validation gate:

| # | Stage (script) | Produces | "Worked" looks like |
|---|----------------|----------|---------------------|
| 1 | `00_preprocess_training_data.py` | `preprocessed_captures/{train,val,classes}` + `split_manifest.txt` | raw split 80/20 (seed 42), each train image augmented 4x |
| 2 | `01_autolabel.py` | `path2_dataset/{images,labels}/{train,val}` + `autolabel_summary.txt` | "hundreds of boxes across all images, balanced per class" |
| 3 | `02_finetune.py` *(skippable)* | `path2_training/balloon_finetune/…` + `path2_results/` | quick in-sample preview, not the honest number |
| 4 | `validation_step1_proper_split.py` | `dataset/{images,labels}/{train,val}` | "Val set: N images (split at RAW level …)" |
| 5 | `validation_step2_retrain.py` | **`training/balloon_proper/weights/best.pt`** + `honest_map50.txt` | "Best mAP50 on HELD-OUT val set: 0.9xx" |
| - | **validation gate** | - | run aborts if held-out mAP50 < `--min-map50` |
| 6 | `validation_step3_val_inference.py` | `honest_results.txt` + `val_annotated/` | per-class P/R/F1, mAP50 |
| 7 | `validation_step4_overfit_check.py` | `loss_curves.png` | VERDICT: HEALTHY (val-train loss gap < 0.05) |
| 8 | `validation_step5_stress_test.py` | `stress_test_results.txt` + `stress_strip.png` | retention rate under UAV noise |

**Observed runtime** on the dev box (WSL2 + RTX 5070, 325 raw captures) for a
full `--onnx` run: preprocess ~7 min (disk-bound on `/mnt/c`), autolabel ~1.5 min,
`02_finetune` ~20 min, split ~1 min, retrain ~20 min, val+overfit+stress ~1 min,
ONNX ~15 s - about **50 min total**, dominated by the two trainings. Use
`--skip-finetune` to drop the redundant preview and roughly halve it. Last
validated run: held-out **mAP50 0.994, P 0.930, R 0.989**, overfit check
**HEALTHY**, stress-test retention **97%**.

The deployed weights land at
`yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.pt`.
All the reports above are written next to the scripts (paths in the table are
relative to that `scripts/` folder). Read `honest_results.txt` first - it is the
real held-out number.

Everything after `00_preprocess` is deterministic (seed 42), so a rerun on the
same `captures/` reproduces the same split and a near-identical model.

### Running a single stage by hand

Every stage is just its script; to re-run one in isolation:

```bash
cd yolo_comparison_test/path2_switch_proposal/scripts
python3 validation_step3_val_inference.py     # e.g. re-measure metrics only
```

Same order as the table. `docs/08_annotation_and_training.md` documents each
script's flags and outputs in full.

---

## 3. Deploy the trained model

The Jetson demo (`fulldemo/run_detection_jetson.sh`) looks for `buoy_best.onnx`
at the repo root first, then falls back to preserved `.pt` demo weights. Export
and copy:

```bash
model/run_pipeline.sh --onnx     # writes best.onnx next to best.pt
scp yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/best.onnx \
    <jetson>:~/robotx-navigation/buoy_best.onnx
```

The live detector loads it via `camera_live_feed.py --yolo-model <path>`
(`--yolo-conf` to tune the threshold). ONNX runs faster than `.pt` on the Jetson
via TensorRT.

**Publishing a new release** (after a retrain that beats the current one):
```bash
gh release create model-<date> \
    yolo_comparison_test/path2_switch_proposal/scripts/training/balloon_proper/weights/{best.pt,best.onnx} \
    --repo dhruvdotc/robotx-navigation \
    --title "Buoy detector weights (<date>)" \
    --notes "mAP50 <x>, P <x>, R <x>, held-out val n=<n>."
```
Then bump the `TAG` variable at the top of `model/fetch_weights.sh` to `model-<date>`.

---

## 4. Troubleshooting (real gotchas in this checkout)

**"Failed to derive class HSV ranges from captures/classes"** - the reference
crops are missing or misnamed. Stems must be exactly `red/green/blue.jpg` (see
§1). This blocks `01_autolabel`.

**A `validation_step1` "path2_dataset has no images/train" refusal** - it will
NOT silently re-split a flat dataset (doing so leaked augmented twins across the
split and inflated mAP). Re-run from `00_preprocess` so the raw-level split
exists; don't hand it a pre-split-fix dataset.

**Calibration file** - retraining does not touch calibration. The *live* detector
resolves intrinsics CLI override > `--calibration-file`
(`calibration/camera_intrinsics_latest.json`) > legacy fallback; keep passing
`--calibration-file` in the field. It has no effect on the training pipeline.

**GPU / `ultralytics` / `torch` (WSL2 + RTX 5070 dev box)** - training uses CUDA
automatically when `torch.cuda.is_available()`. If it silently trains on CPU,
your torch wheel is CPU-only - install a CUDA build matching your driver
(this box: torch 2.13.0+cu130, ultralytics 8.4.92). `pip install -r
yolo_comparison_test/path2_switch_proposal/requirements.txt` covers
ultralytics/opencv/numpy/albumentations. On the slow `/mnt/c` Windows mount the
augment + I/O stages are disk-bound; the GPU training itself is fast.

**Numpy 2.x ABI** - `cv_bridge`/ROS is not needed for retraining; the training
scripts don't import it, so the NumPy-2 segfault that bites the ROS input path
(handled in `camera_live_feed.py`) does not apply here.

---

## See also

- `docs/08_annotation_and_training.md` - full per-step reference + manual LabelImg correction.
- `docs/07_roadmap.md` - current state, known bugs, real-photo fine-tune TODO.
- `docs/10_safe_passage.md` - P4 stretch: a 4th `black` class for OFF buoys would be trained here.
