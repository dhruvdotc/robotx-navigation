# Sim-Courses v3 (red-detection root-cause fix)

v2's checked-in `honest_split_best.pt` never actually detected real red
buoys. Its own `honest_results.txt` claimed red precision=0.892
recall=0.943, but direct, independent reproduction of the same TP/FP/FN
methodology against the real val set found red TP=0 FP=38 FN=36 -
precision 0.000, recall 0.000. Every "red" prediction that model ever made
(confirmed across all 325 raw captures) was a phantom duplicate on a green
or other object, never a real detection. Ground truth was independently
confirmed clean (all 141 red labels genuinely red-hued). This was a real
model training failure, not a labeling or pipeline bug.

## What changed

Re-ran the full honest pipeline from scratch with identical settings
(same raw split reproduced exactly - 260 train/65 val, seed 42; YOLOv11n;
100 epoch ceiling; patience 15) and independently re-verified the result
before trusting it.

## Verified result

Direct TP/FP/FN reproduction against the real held-out val set (65 images,
82 boxes), not the training script's self-report:

| Class | TP | FP | FN | Precision | Recall | F1 |
|-------|----|----|----|-----------|--------|-----|
| red   | 35 | 4  | 0  | 0.897     | 1.000  | 0.945 |
| green | 36 | 1  | 0  | 0.973     | 1.000  | 0.986 |
| blue  | 11 | 0  | 0  | 1.000     | 1.000  | 1.000 |
| all   | 82 | 5  | 0  | 0.943     | 1.000  | 0.971 |

Red went from a confirmed, complete failure (0/36 matched) to working
(35/35 matched) on the same images and ground truth. See `honest_results.txt`
for full detail, including why this is trustworthy this time.

## Files

| File | What it is |
|------|-----------|
| weights/honest_split_best.pt | The retrained, independently-verified model |
| honest_results.txt | Full verification detail and the failure/fix story |
| split_manifest.txt | Raw-level train/val split (reproduced identically to v2) |
| autolabel_summary.txt | Per-class box counts |
| honest_split_results.csv | Per-epoch training curve |

## Also reverted

`camera_live_feed.py`'s `find_detections_yolo()` briefly carried an
agnostic-NMS + HSV color-verification mitigation for the OLD model's
hallucination. Measured against this new model, that mitigation made things
worse and was reverted; plain raw model output is what's verified above.
