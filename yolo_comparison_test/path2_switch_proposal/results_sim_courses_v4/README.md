# Sim-Courses v4 (the real fix, verified two ways)

Short version: v3 looked fixed but wasn't. The real bug was in
`00_preprocess_training_data.py`'s reference-crop handling, not the model.
This run fixes that at the root and is verified in a way that would have
caught v2 and v3's mistakes.

## The chain of bugs

1. **v2**: red never worked. Every "red" prediction was a phantom duplicate
   on a green object (confirmed: 0/36 real red boxes ever matched).
2. **v3**: looked fixed (TP=35/FN=0) but the ground truth itself was
   corrupted. `copy_and_normalise_classes()` ran Phase-2 colour
   normalisation (built for full scenes) on the tiny `classes/red.jpg`
   reference crop. Gray-world white balance overcorrects a near-solid-colour
   crop; red's derived hue center shifted from ~0 to ~90 (green's
   territory). Every "red" ground-truth box in that dataset was actually
   green. v3's model correctly learned to reproduce that mistake.
3. **Fixing #2 exposed a second bug**: blue's autolabel count collapsed
   174 -> 6, because `derive_class_hsv_ranges()` used the reference crop's
   own 15th-percentile saturation/value as a hard floor - fragile for a
   near-solid crop with an unusually high percentile.

## What v4 does differently

- `copy_and_normalise_classes()` keeps reference crops raw (no more Phase-2
  normalisation applied to them).
- `derive_class_hsv_ranges()` uses the caller's sat/val floor directly.
- Ground truth was independently verified clean (all 47 red/36 green/11
  blue val boxes genuinely hue-correct, zero cross-class co-location)
  *before* spending GPU time retraining.
- After training, verified two ways: TP/FP/FN against ground truth, AND a
  direct hue check of every single predicted box's own pixels (the check
  that would have caught v3 immediately).

## Verified result

| Class | TP | FP | FN | Precision | Recall | Predicted boxes genuinely correct-hued |
|-------|----|----|----|-----------|--------|------|
| red   | 47 | 6  | 0  | 0.887     | 1.000  | 53/53 |
| green | 36 | 2  | 0  | 0.947     | 1.000  | 38/38 |
| blue  | 11 | 0  | 0  | 1.000     | 1.000  | 11/11 |

Zero hallucination anywhere this time. See `honest_results.txt` for the
full story and why this verification is trustworthy where v3's wasn't.

## Files

| File | What it is |
|------|-----------|
| weights/honest_split_best.pt | The retrained, doubly-verified model (best checkpoint, epoch 57) |
| honest_results.txt | Full verification detail and the three-bug story |
| split_manifest.txt | Raw-level train/val split (reproduced identically across v2/v3/v4) |
| autolabel_summary.txt | Per-class box counts on the corrected dataset |
| honest_split_results.csv | Per-epoch training curve |
