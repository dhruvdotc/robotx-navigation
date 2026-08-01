#!/usr/bin/env python3
"""Stage the honest train/val split into scripts/dataset/ for steps 2-5.

This script used to CREATE the split here, by shuffling path2_dataset/images
80/20. That was quietly dishonest once 00_preprocess_training_data.py entered
the pipeline: the pool being shuffled contained augmented variants, so an
original could land in train while its own _augNN twins landed in "val" -
near-duplicate leakage that inflated held-out mAP while the train/val loss
gap (step 4) correctly screamed overfit.

The split now happens ONCE, at the raw-capture level, inside
00_preprocess_training_data.py (see its split_manifest.txt), and 01_autolabel
preserves it as path2_dataset/images/{train,val}. This script's only job is
to copy that structure verbatim into scripts/dataset/ - the layout steps 2-5
consume - and to refuse loudly if handed a flat (pre-split-fix) dataset
rather than silently re-splitting it.
"""

from __future__ import annotations

import os
import shutil
import sys


def _copy_subset(src_images: str, src_labels: str, dst_images: str, dst_labels: str) -> int:
    os.makedirs(dst_images, exist_ok=True)
    os.makedirs(dst_labels, exist_ok=True)
    count = 0
    for name in sorted(os.listdir(src_images)):
        if not name.lower().endswith(".jpg"):
            continue
        stem = os.path.splitext(name)[0]
        shutil.copy2(os.path.join(src_images, name), os.path.join(dst_images, name))
        src_l = os.path.join(src_labels, stem + ".txt")
        dst_l = os.path.join(dst_labels, stem + ".txt")
        if os.path.exists(src_l):
            shutil.copy2(src_l, dst_l)
        else:
            open(dst_l, "w", encoding="utf-8").close()
        count += 1
    return count


def main() -> int:
    root = os.path.dirname(__file__)
    src = os.path.join(root, "path2_dataset")
    dst_root = os.path.join(root, "dataset")

    if not os.path.isdir(os.path.join(src, "images", "train")):
        print(
            "[ERROR] path2_dataset has no images/train + images/val structure.\n"
            "        Re-splitting a flat dataset here would leak augmented\n"
            "        variants across the split (the bug this script used to\n"
            "        have). Re-run the pipeline from the raw captures:\n"
            "          python 00_preprocess_training_data.py\n"
            "          python 01_autolabel.py --captures-dir ../preprocessed_captures",
            file=sys.stderr,
        )
        return 1

    n_train = _copy_subset(
        os.path.join(src, "images", "train"), os.path.join(src, "labels", "train"),
        os.path.join(dst_root, "images", "train"), os.path.join(dst_root, "labels", "train"),
    )
    n_val = _copy_subset(
        os.path.join(src, "images", "val"), os.path.join(src, "labels", "val"),
        os.path.join(dst_root, "images", "val"), os.path.join(dst_root, "labels", "val"),
    )

    yaml_path = os.path.join(dst_root, "dataset.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("path: ./dataset\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("nc: 3\n")
        f.write("names: ['red', 'green', 'blue']\n")

    print(f"Train set: {n_train} images")
    print(f"Val set:   {n_val} images (split at RAW level by 00_preprocess - "
          f"no augmented variant of any val original exists in train)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
