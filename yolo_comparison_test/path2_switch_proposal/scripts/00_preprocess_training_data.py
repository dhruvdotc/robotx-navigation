#!/usr/bin/env python3
"""
00_preprocess_training_data.py - Split-then-augment training data pre-processor.

=== Pipeline overview ===

  raw captures/
       │
       ▼  Phase 0 - Raw-level train/val split  (leak-proof by construction)
       │     • The RAW images are split train/val FIRST (--val-fraction, seeded,
       │       recorded in split_manifest.txt) so an original and its own
       │       augmented variants can never straddle the split. The old
       │       behaviour - augment everything, split later - silently put
       │       near-duplicates of training images into the "held-out" set,
       │       which inflated validation numbers while the train/val loss gap
       │       screamed overfit. Never re-split downstream of this script.
       │
       ▼  Phase 1 - Geometric / environmental augmentation  (each side separately)
       │     • Keep every original image unchanged.
       │     • train/: --aug-per-image variants per image (default 4).
       │     • val/:   --val-aug-per-image variants (default 0 - a pristine
       │       val set is the defensible choice; robustness-under-noise is
       │       measured separately by validation_step5_stress_test.py).
       │     • Augmentations simulate real UAV flight conditions:
       │         HorizontalFlip · Perspective (bank/pitch) · Affine (roll/zoom)
       │         HueSaturationValue (lighting shifts) · CoarseDropout (wave occlusion)
       │         RandomFog · GaussNoise · Spatter (weather / lens droplets)
       │
       ▼  Phase 2 - Color normalisation  (applied to ALL images, both sides)
       │     • CLAHE on YUV luma channel → suppress glare and water reflections
       │     • Gray-World white balance  → correct blue/green water cast
       │     • Unsharp masking           → restore buoy edge sharpness
       │
       ▼
  preprocessed_captures/
      ├── train/               ← originals + augmented variants
      ├── val/                 ← held-out originals (never augmented by default)
      ├── classes/             ← normalised HSV reference crops
      └── split_manifest.txt   ← which raw file went where (defensibility)

=== Typical usage ===

  cd yolo_comparison_test/path2_switch_proposal/scripts

  # Full pipeline (split, augment train, normalise everything)
  python 00_preprocess_training_data.py

  # More augmentation variants on the train side
  python 00_preprocess_training_data.py --aug-per-image 8

  # Custom dirs
  python 00_preprocess_training_data.py \\
      --input-dir  ../captures \\
      --output-dir ../preprocessed_captures

  # Normalise only (skip Phase 1), e.g. after manual augmentation
  python 00_preprocess_training_data.py --skip-augment

Then run the rest of the pipeline on the output directory:
  python 01_autolabel.py --captures-dir ../preprocessed_captures
"""

from __future__ import annotations

import argparse
import os
import random
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Albumentations - optional; Phase 1 requires it
# ---------------------------------------------------------------------------
try:
    import albumentations as A

    _ALB_OK = True
    _ALB_VER = tuple(int(x) for x in A.__version__.split(".")[:2])
    _ALB_2X = _ALB_VER >= (2, 0)
except ImportError:
    _ALB_OK = False
    _ALB_2X = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
IMAGE_EXTS: set[str] = {".jpg", ".jpeg", ".png", ".bmp"}
CLASSES_SUBDIR = "classes"

# CLAHE parameters
CLAHE_CLIP = 2.0
CLAHE_TILE = (8, 8)

# Unsharp masking parameters
UNSHARP_SIGMA = 1.0          # Gaussian blur sigma for detail extraction
UNSHARP_STRENGTH = 0.8       # weight of the recovered detail layer


# ===========================================================================
# Phase 2 - Colour normalisation
# ===========================================================================

def clahe_yuv(img_bgr: np.ndarray) -> np.ndarray:
    """
    CLAHE on the Y (luma) channel in YUV colour space.

    Processes small, localised tiles rather than the whole image, so it
    suppresses strong water reflections and sun glare without washing out
    the surrounding scene.  Pairing with YUV (rather than BGR directly)
    leaves the colour channels untouched.
    """
    yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_TILE)
    yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)


def gray_world_wb(img_bgr: np.ndarray) -> np.ndarray:
    """
    Gray-World white balance.

    Assumes the mean colour of the full scene is neutral grey, then scales
    each channel so all three means match the overall luminance mean.  This
    corrects the blue/green water cast caused by differential light absorption
    at depth and by atmospheric haze.
    """
    img_f = img_bgr.astype(np.float64)
    b_mean = img_f[:, :, 0].mean()
    g_mean = img_f[:, :, 1].mean()
    r_mean = img_f[:, :, 2].mean()
    gray_mean = (b_mean + g_mean + r_mean) / 3.0

    if b_mean > 1e-6:
        img_f[:, :, 0] *= gray_mean / b_mean
    if g_mean > 1e-6:
        img_f[:, :, 1] *= gray_mean / g_mean
    if r_mean > 1e-6:
        img_f[:, :, 2] *= gray_mean / r_mean

    return np.clip(img_f, 0, 255).astype(np.uint8)


def unsharp_mask(img_bgr: np.ndarray,
                 sigma: float = UNSHARP_SIGMA,
                 strength: float = UNSHARP_STRENGTH) -> np.ndarray:
    """
    Unsharp masking.

    Extracts a high-frequency detail residual and blends it back into the
    image at `strength`.  This recovers the distinct conical / spherical edges
    of buoys that are softened by atmospheric moisture, fog, or drone motion
    blur - especially after CLAHE smoothing.
    """
    blurred = cv2.GaussianBlur(img_bgr, (0, 0), sigma)
    sharpened = cv2.addWeighted(img_bgr, 1.0 + strength, blurred, -strength, 0)
    return np.clip(sharpened, 0, 255).astype(np.uint8)


def color_normalize(img_bgr: np.ndarray) -> np.ndarray:
    """Full normalisation pipeline: CLAHE → Gray-World WB → Unsharp Masking."""
    img = clahe_yuv(img_bgr)
    img = gray_world_wb(img)
    img = unsharp_mask(img)
    return img


# ===========================================================================
# Phase 1 - Geometric / environmental augmentation
# ===========================================================================

def _coarse_dropout(p: float) -> object:
    """Build CoarseDropout with API compatible across albumentations 1.x and 2.x."""
    if _ALB_2X:
        # albumentations 2.x: fill_value → fill
        return A.CoarseDropout(
            num_holes_range=(2, 8),
            hole_height_range=(20, 60),
            hole_width_range=(20, 60),
            fill=0,
            p=p,
        )
    # albumentations 1.x keyword names
    return A.CoarseDropout(
        max_holes=8,
        min_holes=2,
        max_height=60,
        max_width=60,
        min_height=20,
        min_width=20,
        fill_value=0,
        p=p,
    )


def _gauss_noise(p: float) -> object:
    """Build GaussNoise with API compatible across albumentations 1.x and 2.x."""
    if _ALB_2X:
        # 2.x std_range is normalised: pixel_std / 255.  8–40 px ≈ 0.031–0.157
        return A.GaussNoise(std_range=(8.0 / 255.0, 40.0 / 255.0), p=p)
    return A.GaussNoise(var_limit=(64.0, 1600.0), per_channel=True, p=p)


def _random_fog(p: float) -> object:
    """Build RandomFog with API compatible across albumentations 1.x and 2.x."""
    if _ALB_2X:
        return A.RandomFog(fog_coef_range=(0.05, 0.25), alpha_coef=0.08, p=p)
    return A.RandomFog(fog_coef_lower=0.05, fog_coef_upper=0.25, alpha_coef=0.08, p=p)


def build_aug_pipeline() -> "A.Compose":
    """
    Assemble the full augmentation pipeline.

    Every transform is chosen to represent a physically realistic degradation
    a drone camera would experience during a maritime competition run:

      HorizontalFlip       - free label-preserving doubling (buoys are radially
                             symmetric from nadir; vertical flip excluded)
      Perspective          - simulates UAV banking / off-nadir viewing angles
      Affine               - roll, zoom, and small translations from wind gusts
      HueSaturationValue   - noon vs overcast vs golden-hour lighting variation
      CoarseDropout        - whitecap / sea-spray occlusion of the buoy body
      GridDropout          - partial structural occlusion for robustness
      RandomFog            - coastal fog and atmospheric haze
      GaussNoise           - sensor noise at long distances
      Spatter              - physical water droplets hitting the camera lens
      MotionBlur           - camera shake + sun-glint streaks (roadmap TODO #1
                             "streak glare": a directional blur kernel)
      GaussianBlur         - defocus / altitude-dependent softening at range
    """
    if not _ALB_OK:
        raise RuntimeError(
            "albumentations is not installed.\n"
            "  pip install albumentations>=1.3\n"
            "Or run with --skip-augment to only apply colour normalisation."
        )

    weather_candidates: list = [_random_fog(p=1.0), _gauss_noise(p=1.0)]
    try:
        # Spatter was added in albumentations 1.3; constructor args vary
        weather_candidates.append(
            A.Spatter(
                mean=0.65,
                std=0.3,
                gauss_sigma=2.0,
                cutout_threshold=0.68,
                intensity=0.6,
                p=1.0,
            )
        )
    except (AttributeError, TypeError):
        pass  # gracefully skip on unsupported versions

    return A.Compose(
        [
            A.HorizontalFlip(p=0.5),
            # UAV bank/pitch → foreshortened buoy geometry
            A.Perspective(scale=(0.03, 0.08), keep_size=True, p=0.5),
            # Wind gusts → roll, zoom, translation
            A.Affine(
                scale=(0.85, 1.10),
                translate_percent={"x": (-0.05, 0.05), "y": (-0.05, 0.05)},
                rotate=(-15.0, 15.0),
                shear=(-5.0, 5.0),
                p=0.6,
            ),
            # Noon vs overcast vs golden-hour lighting
            A.HueSaturationValue(
                hue_shift_limit=20,
                sat_shift_limit=35,
                val_shift_limit=35,
                p=0.7,
            ),
            # Wave / sea-spray occlusion - partial buoy visibility
            A.OneOf(
                [
                    _coarse_dropout(p=1.0),
                    A.GridDropout(ratio=0.12, p=1.0),
                ],
                p=0.35,
            ),
            # Weather: fog, sensor noise, lens droplets
            A.OneOf(weather_candidates, p=0.35),
            # Altitude-dependent defocus + motion / streak-glare blur. Kept mild
            # (small kernels) and infrequent so buoy edges survive; addresses
            # roadmap TODO #1 (streak glare + altitude-aware blur).
            A.OneOf(
                [
                    A.MotionBlur(blur_limit=(3, 11), p=1.0),
                    A.GaussianBlur(blur_limit=(3, 7), p=1.0),
                ],
                p=0.3,
            ),
        ]
    )


def augment_image(img_bgr: np.ndarray, pipeline: "A.Compose") -> np.ndarray:
    """Apply pipeline to a single BGR image; returns augmented BGR array."""
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    result = pipeline(image=rgb)["image"]
    return cv2.cvtColor(result, cv2.COLOR_RGB2BGR)


# ===========================================================================
# File helpers
# ===========================================================================

def list_images(directory: str) -> list[Path]:
    """Return sorted list of image paths directly inside *directory* (non-recursive)."""
    return sorted(
        p for p in Path(directory).iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS
    )


def save_jpg(img_bgr: np.ndarray, path: Path, quality: int = 95) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])


# ===========================================================================
# Main pipeline
# ===========================================================================

def split_raw_images(
    images: list[Path],
    val_fraction: float,
    seed: int,
) -> tuple[list[Path], list[Path]]:
    """
    Phase 0: split the RAW image list into (train, val) - before any
    augmentation exists, so an original and its variants cannot straddle the
    split. Deterministic for a given seed (sorted names, seeded shuffle).
    """
    ordered = sorted(images, key=lambda p: p.name)
    rng = random.Random(seed)
    rng.shuffle(ordered)
    n_val = max(1, round(len(ordered) * val_fraction)) if ordered else 0
    val = sorted(ordered[:n_val], key=lambda p: p.name)
    train = sorted(ordered[n_val:], key=lambda p: p.name)
    return train, val


def write_split_manifest(
    manifest_path: Path,
    train: list[Path],
    val: list[Path],
    val_fraction: float,
    seed: int,
) -> None:
    """Record exactly which raw file went to which side (defensibility: anyone
    auditing the numbers can confirm no val original ever fed augmentation on
    the train side)."""
    with open(manifest_path, "w", encoding="utf-8") as f:
        f.write(f"# Raw-level train/val split (val_fraction={val_fraction}, seed={seed})\n")
        f.write(f"# {len(train)} train / {len(val)} val raw images\n")
        for p in train:
            f.write(f"train\t{p.name}\n")
        for p in val:
            f.write(f"val\t{p.name}\n")


def run_phase1_augment(
    images: list[Path],
    output_dir: Path,
    aug_per_image: int,
    seed: int,
    verbose: bool,
    subset: str = "",
) -> list[Path]:
    """
    Phase 1: geometric / environmental augmentation over an explicit image
    list (a train or val subset from Phase 0 - never a whole directory, so
    the leak-proof split is preserved by construction).

    For every image:
      • Copies the original unchanged into *output_dir*.
      • Writes *aug_per_image* augmented variants named
        ``<stem>_aug<N><ext>``.

    Returns the list of all written output paths.
    """
    random.seed(seed)
    np.random.seed(seed)

    tag = f" [{subset}]" if subset else ""
    written: list[Path] = []
    n = len(images)
    if not images:
        print(f"[Phase 1]{tag} No images. Nothing to augment.")
        return written

    output_dir.mkdir(parents=True, exist_ok=True)
    if aug_per_image > 0:
        pipeline = build_aug_pipeline()
        print(f"[Phase 1]{tag} Augmenting {n} image(s) × {aug_per_image} variants each …")
    else:
        pipeline = None
        print(f"[Phase 1]{tag} Copying {n} original(s), no augmented variants …")

    for i, src in enumerate(images, 1):
        img = cv2.imread(str(src))
        if img is None:
            print(f"  [WARN] Cannot read {src.name} - skipped.")
            continue

        # --- Keep original ---
        orig_dst = output_dir / src.name
        shutil.copy2(src, orig_dst)
        written.append(orig_dst)

        # --- Augmented variants ---
        for aug_idx in range(1, aug_per_image + 1):
            aug_img = augment_image(img, pipeline)
            stem = src.stem
            out_name = f"{stem}_aug{aug_idx:02d}{src.suffix}"
            dst = output_dir / out_name
            save_jpg(aug_img, dst)
            written.append(dst)

        if verbose:
            print(f"  [{i}/{n}] {src.name} → +{aug_per_image} variants")

    orig_count = n
    aug_count = len(written) - orig_count
    print(
        f"[Phase 1]{tag} Done. {orig_count} originals + {aug_count} augmented "
        f"= {len(written)} total images."
    )
    return written


def run_phase2_normalize(
    image_paths: list[Path],
    verbose: bool,
) -> None:
    """
    Phase 2: colour normalisation applied in-place to all *image_paths*.

    Pipeline: CLAHE (YUV) → Gray-World WB → Unsharp Masking.
    """
    n = len(image_paths)
    print(f"[Phase 2] Normalising {n} image(s) …")

    for i, path in enumerate(image_paths, 1):
        img = cv2.imread(str(path))
        if img is None:
            print(f"  [WARN] Cannot read {path.name} - skipped.")
            continue
        normalised = color_normalize(img)
        cv2.imwrite(str(path), normalised, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if verbose:
            print(f"  [{i}/{n}] {path.name}")

    print("[Phase 2] Done. All images colour-normalised.")


def copy_and_normalise_classes(
    src_classes: Path,
    dst_classes: Path,
    verbose: bool,
) -> None:
    """
    Copy the HSV reference crops from *src_classes* to *dst_classes* AS-IS,
    deliberately WITHOUT colour normalisation.

    This used to also run Phase 2 normalisation on the crops, on the theory
    that 01_autolabel.py's derived HSV ranges should come from the same
    colour space as the (normalised) training images. That reasoning breaks
    down for these crops specifically: gray_world_wb() assumes a scene
    averages to neutral gray, which is true-ish for a full capture but false
    by construction for a reference crop that is mostly one saturated colour
    -- normalising red.jpg measurably shifted its derived hue center from
    red (~0) into green's range (~90), so autolabel's "red" mask silently
    became a second green mask. Every "red" box in a dataset built this way
    was confirmed (by direct HSV check) to sit on a real green object, and a
    model trained on it faithfully learned to reproduce that mistake instead
    of ever detecting real red. Reference crops must stay raw so the derived
    hue centers reflect the object's real colour, not an artifact of a
    normalisation step that only makes sense for full scenes.
    """
    if not src_classes.is_dir():
        return
    dst_classes.mkdir(parents=True, exist_ok=True)
    n = 0
    for p in src_classes.iterdir():
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS:
            shutil.copy2(p, dst_classes / p.name)
            n += 1
    if n:
        print(f"[Classes] Copied {n} reference crop(s), kept raw (not colour-normalised).")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Pre-process raw training images: augment then colour-normalise.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    _scripts_dir = Path(__file__).resolve().parent
    _repo_dir = _scripts_dir.parent

    parser.add_argument(
        "--input-dir",
        type=Path,
        default=_repo_dir / "captures",
        metavar="DIR",
        help="Directory of raw training images (default: ../captures)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=_repo_dir / "preprocessed_captures",
        metavar="DIR",
        help="Output directory for processed images (default: ../preprocessed_captures)",
    )
    parser.add_argument(
        "--aug-per-image",
        type=int,
        default=4,
        metavar="N",
        help="Augmented variants per original TRAIN image (default: 4)",
    )
    parser.add_argument(
        "--val-aug-per-image",
        type=int,
        default=0,
        metavar="N",
        help="Augmented variants per original VAL image (default: 0 - a "
             "pristine val set is the defensible choice; noise robustness is "
             "measured separately by validation_step5_stress_test.py)",
    )
    parser.add_argument(
        "--val-fraction",
        type=float,
        default=0.2,
        metavar="F",
        help="Fraction of RAW images held out as val before augmentation "
             "(default: 0.2). The split happens at the raw level so an "
             "original and its augmented variants can never straddle it.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        metavar="INT",
        help="Random seed for the split and augmentation (default: 42)",
    )
    parser.add_argument(
        "--skip-augment",
        action="store_true",
        help="Skip Phase 1 (only copy originals and apply colour normalisation)",
    )
    parser.add_argument(
        "--skip-normalize",
        action="store_true",
        help="Skip Phase 2 (only run geometric augmentation, no colour normalisation)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Print per-image progress",
    )

    args = parser.parse_args(argv)

    input_dir: Path = args.input_dir
    output_dir: Path = args.output_dir

    if not input_dir.is_dir():
        print(f"[ERROR] Input directory not found: {input_dir}", file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    # -----------------------------------------------------------------------
    # Handle classes/ subdirectory (HSV reference crops)
    # -----------------------------------------------------------------------
    src_classes = input_dir / CLASSES_SUBDIR
    dst_classes = output_dir / CLASSES_SUBDIR
    if src_classes.is_dir():
        copy_and_normalise_classes(src_classes, dst_classes, verbose=args.verbose)

    # -----------------------------------------------------------------------
    # Phase 0 - Raw-level train/val split (must precede ALL augmentation)
    # -----------------------------------------------------------------------
    raw_images = list_images(str(input_dir))
    if not raw_images:
        print(f"[ERROR] No images found in {input_dir}.", file=sys.stderr)
        return 1
    train_raw, val_raw = split_raw_images(raw_images, args.val_fraction, args.seed)
    write_split_manifest(output_dir / "split_manifest.txt",
                         train_raw, val_raw, args.val_fraction, args.seed)
    print(f"[Phase 0] Raw split: {len(train_raw)} train / {len(val_raw)} val "
          f"(val_fraction={args.val_fraction}, seed={args.seed}; "
          f"see split_manifest.txt)")

    train_dir = output_dir / "train"
    val_dir = output_dir / "val"

    # -----------------------------------------------------------------------
    # Phase 1 - Geometric / environmental augmentation (per side, no crossing)
    # -----------------------------------------------------------------------
    if args.skip_augment:
        print("[Phase 1] Skipped (--skip-augment). Copying originals …")
        train_dir.mkdir(parents=True, exist_ok=True)
        val_dir.mkdir(parents=True, exist_ok=True)
        for src in train_raw:
            shutil.copy2(src, train_dir / src.name)
        for src in val_raw:
            shutil.copy2(src, val_dir / src.name)
        train_paths = [train_dir / p.name for p in train_raw]
        val_paths = [val_dir / p.name for p in val_raw]
    else:
        needs_alb = args.aug_per_image > 0 or args.val_aug_per_image > 0
        if needs_alb and not _ALB_OK:
            print(
                "[ERROR] albumentations is required for Phase 1 but is not installed.\n"
                "        Run:  pip install albumentations>=1.3\n"
                "        Or use --skip-augment to only apply colour normalisation.",
                file=sys.stderr,
            )
            return 1
        train_paths = run_phase1_augment(
            images=train_raw,
            output_dir=train_dir,
            aug_per_image=args.aug_per_image,
            seed=args.seed,
            verbose=args.verbose,
            subset="train",
        )
        val_paths = run_phase1_augment(
            images=val_raw,
            output_dir=val_dir,
            aug_per_image=args.val_aug_per_image,
            seed=args.seed + 1,  # decorrelate from train-side augmentation
            verbose=args.verbose,
            subset="val",
        )

    # -----------------------------------------------------------------------
    # Phase 2 - Colour normalisation (both sides: val must look like train
    # because the SAME deterministic normalisation runs at inference time)
    # -----------------------------------------------------------------------
    if args.skip_normalize:
        print("[Phase 2] Skipped (--skip-normalize).")
    else:
        run_phase2_normalize(train_paths + val_paths, verbose=args.verbose)

    # -----------------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------------
    print()
    print("=" * 60)
    print("Pre-processing complete.")
    print(f"  Input:   {input_dir}")
    print(f"  Output:  {output_dir}")
    print(f"  Train:   {len(train_paths)} images ({len(train_raw)} originals)")
    print(f"  Val:     {len(val_paths)} images ({len(val_raw)} originals, held out at raw level)")
    print()
    print("Next step:")
    print(f"  python 01_autolabel.py --captures-dir {output_dir}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
