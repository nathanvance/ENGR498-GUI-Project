#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from yolo_inference_common import resolve_pose_recovery_inputs, write_json  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Import Colab-generated masks/meta outputs back into a local Fusion inference directory."
    )
    parser.add_argument("--pose-recovery-run-dir", required=True, help="Local rosbag_preprocessing run directory containing images/ and image_timestamps.csv")
    parser.add_argument("--colab-run-root", required=True, help="Local path to a completed colab_bundle directory")
    parser.add_argument("--output-dir", help="Destination directory for imported inference outputs. Defaults to <pose_run>/yolo_inference")
    return parser.parse_args()


def copy_tree(src: Path, dst: Path) -> int:
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True, exist_ok=True)
    count = 0
    for item in sorted(src.rglob("*")):
        if item.is_dir():
            continue
        rel = item.relative_to(src)
        target = dst / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(item, target)
        count += 1
    return count


def main() -> int:
    args = parse_args()
    inputs = resolve_pose_recovery_inputs(Path(args.pose_recovery_run_dir))
    colab_root = Path(args.colab_run_root).resolve()
    output_dir = Path(args.output_dir).resolve() if args.output_dir else (inputs.run_dir / "yolo_inference").resolve()

    masks_dir = colab_root / "masks_npz"
    meta_dir = colab_root / "meta_json"
    pred_dir = colab_root / "pred_images"
    combined_dir = colab_root / "combined_class_masks"

    if not masks_dir.is_dir():
        raise FileNotFoundError(f"Could not find Colab masks directory: {masks_dir}")
    if not meta_dir.is_dir():
        raise FileNotFoundError(f"Could not find Colab metadata directory: {meta_dir}")

    stem_set = {Path(name).stem for name in inputs.image_filenames}
    missing_masks = [stem for stem in sorted(stem_set) if not (masks_dir / f"{stem}_masks.npz").is_file()]
    missing_meta = [stem for stem in sorted(stem_set) if not (meta_dir / f"{stem}_meta.json").is_file()]
    if missing_masks or missing_meta:
        raise FileNotFoundError(
            f"Colab results are incomplete. Missing masks: {len(missing_masks)}, missing metadata: {len(missing_meta)}"
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    copied_masks = copy_tree(masks_dir, output_dir / "masks_npz")
    copied_meta = copy_tree(meta_dir, output_dir / "meta_json")
    copied_pred = copy_tree(pred_dir, output_dir / "pred_images") if pred_dir.is_dir() else 0
    copied_combined = copy_tree(combined_dir, output_dir / "combined_class_masks") if combined_dir.is_dir() else 0

    payload = {
        "runtime_mode": "colab_import",
        "pose_recovery_run_dir": str(inputs.run_dir),
        "colab_run_root": str(colab_root),
        "output_dir": str(output_dir),
        "copied_masks": copied_masks,
        "copied_meta": copied_meta,
        "copied_pred_images": copied_pred,
        "copied_combined_masks": copied_combined,
    }
    write_json(output_dir / "inference_manifest.json", payload)
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
