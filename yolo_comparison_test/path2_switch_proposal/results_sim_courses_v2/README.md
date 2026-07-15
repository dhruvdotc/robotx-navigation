# Sim-Courses v2 Training Run (leak-proof split, includes blue)

YOLOv11n fine-tuned on frames captured from the Gazebo sim (courses 1-3 plus a
Scan-the-Code loiter over the light buoy), auto-labeled by HSV, trained 100
epochs on an RTX 5070. First run ever with a blue class (the light buoy's
color cycling was built this session) and the first with a leak-proof
train/val split (raw images split BEFORE augmentation; see split_manifest.txt).

## Headline metrics (held-out val: 65 raw images, 82 boxes, never augmented)

| Class | Precision | Recall | F1 | mAP50 |
|-------|-----------|--------|-----|-------|
| red   | 0.892     | 0.943  | 0.917 | 0.984 |
| green | 0.973     | 1.000  | 0.986 | 0.995 |
| blue  | 1.000     | 1.000  | 1.000 | 0.995 |
| all   | 0.941     | 0.976  | 0.958 | 0.991 |

- Overfit check: HEALTHY (val loss 0.51 BELOW train loss; train is heavily
  augmented, val is pristine). The v1 run's verdict was SEVERE OVERFIT because
  its split leaked augmented variants across train/val.
- Stress test (UAV noise): 89.4% detection retention, conf 0.635 -> 0.555.
- Balloon baseline honest mAP50 was 0.968. v1 sim run (0.957) is INVALID
  (leaky split), do not cite it.

## Files

| File | What it is |
|------|-----------|
| weights/full_dataset_best.pt | best.pt from the 100-epoch full run (02_finetune) |
| weights/honest_split_best.pt | best.pt from the independent step-2 retrain |
| honest_results.txt / honest_map50.txt | step-3 held-out TP/FP/FN metrics |
| overfit_check.txt | step-4 train/val loss gap verdict |
| stress_test_results.txt | step-5 noise robustness |
| autolabel_summary.txt | per-class box counts (train and val) |
| split_manifest.txt | which raw frame went to which side of the split |
| full_dataset_results.csv / honest_split_results.csv | per-epoch training curves |
| plots/ | results.png (curves), confusion_matrix.png, PR/F1 curves, val grids |
| val_annotated/ | all 65 val images with the model's boxes drawn |

## How to look at the results visually

Open `plots/results.png` for training curves, `plots/confusion_matrix.png`
for per-class confusion, `plots/val_batch*_pred.jpg` vs
`plots/val_batch*_labels.jpg` to compare predictions against ground truth
tile by tile, and browse `val_annotated/` for every held-out image with
boxes drawn.

## Honest caveats (read before quoting the 0.99)

1. **The ground truth is weak HSV autolabels, and it has real errors.**
   Example found during review: in val image
   `course1_frame_1783758533843.jpg` the autolabeler put overlapping red AND
   green boxes on one object and completely missed a fully saturated red
   buoy mid-frame (hue 1, sat 255, ~4200 px, passes every documented
   threshold). The model learned to agree with its teacher, so the metrics
   measure label agreement, not true detection quality. Fix path: audit
   autolabel output (docs/08 describes the LabelImg spot-fix flow) or label a
   small human-verified test set for final numbers.
2. Everything is Gazebo-rendered imagery. Clean sim frames are an easy
   domain; a real-photo fine-tune is still required before trusting this on
   the physical drone.
3. Blue is learned from a single object type (the light buoy's glow panel)
   and is ~16% of boxes; red/green come from gate buoys plus the panel.
4. Training images went through Phase-2 color normalization (CLAHE,
   gray-world, unsharp) but camera_live_feed.py's YOLO path feeds RAW
   frames. Apply the same normalization at inference or retrain without it
   before deploying these weights.
