#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
for candidate in (THIS_DIR, THIS_DIR.parent):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from yolo_inference_common import export_ultralytics_results, resolve_pose_recovery_inputs, write_json  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run YOLO segmentation inside Colab on a prepared inference bundle.")
    parser.add_argument("--run-root", required=True, help="Path to the prepared colab_bundle directory")
    parser.add_argument("--weights", help="Optional explicit weights path. Overrides config and bundled weights")
    parser.add_argument("--local-device", default="0", help="CUDA device selector, e.g. 0 or cuda:0")
    parser.add_argument("--preferred-gpu", help="Preferred GPU name to report, e.g. A100")
    parser.add_argument("--force-remount-drive", action="store_true", help="Force remount Google Drive in Colab")
    return parser.parse_args()


def maybe_mount_drive(run_root: Path, force_remount: bool) -> None:
    if "google.colab" not in sys.modules:
        return
    if not str(run_root).startswith("/content/drive"):
        return

    mount_root = Path("/content/drive")
    if not force_remount and run_root.exists():
        return

    from google.colab import drive

    drive.mount(str(mount_root), force_remount=force_remount)


def parse_device_index(device: str) -> tuple[int, str]:
    raw = str(device).strip()
    if raw.isdigit():
        index = int(raw)
        return index, f"cuda:{index}"
    lowered = raw.lower()
    if lowered.startswith("cuda:") and lowered[5:].isdigit():
        index = int(lowered[5:])
        return index, f"cuda:{index}"
    raise ValueError(f"Unsupported CUDA device selector '{device}'. Use forms like '0' or 'cuda:0'.")


def load_config(run_root: Path) -> dict:
    config_path = run_root / "colab_inference_config.json"
    if not config_path.is_file():
        raise FileNotFoundError(f"Could not find Colab inference config: {config_path}")
    return json.loads(config_path.read_text(encoding="utf-8"))


def resolve_weights(run_root: Path, config: dict, explicit: str | None) -> Path:
    if explicit:
        weights_path = Path(explicit)
        if weights_path.is_file():
            return weights_path
        raise FileNotFoundError(f"Explicit weights path not found: {weights_path}")

    bundled_name = config.get("weights_filename")
    if bundled_name:
        bundled_path = run_root / bundled_name
        if bundled_path.is_file():
            return bundled_path

    drive_path = config.get("drive_weights_path")
    if drive_path:
        drive_weights = Path(drive_path)
        if drive_weights.is_file():
            return drive_weights
        raise FileNotFoundError(f"Configured Drive weights path not found: {drive_weights}")

    raise FileNotFoundError("No weights were found. Supply --weights or include a bundled/configured weights path.")


def main() -> int:
    args = parse_args()
    run_root = Path(args.run_root).resolve()
    maybe_mount_drive(run_root, args.force_remount_drive)

    import torch
    from ultralytics import YOLO

    if not torch.cuda.is_available():
        raise RuntimeError("Colab inference requires a CUDA GPU runtime. Switch the Colab runtime to GPU.")

    config = load_config(run_root)
    preferred_gpu = args.preferred_gpu or config.get("preferred_gpu") or "A100"

    device_index, device_name = parse_device_index(args.local_device)
    if device_index >= torch.cuda.device_count():
        raise RuntimeError(
            f"Requested CUDA device {device_index} does not exist. Device count: {torch.cuda.device_count()}"
        )

    actual_gpu = torch.cuda.get_device_name(device_index)
    if preferred_gpu.lower() in actual_gpu.lower():
        print(f"[info] preferred Colab GPU matched: {actual_gpu}")
    else:
        print(f"[info] preferred Colab GPU '{preferred_gpu}' not assigned; using '{actual_gpu}' instead")

    weights_path = resolve_weights(run_root, config, args.weights)
    inputs = resolve_pose_recovery_inputs(run_root)
    inference = config.get("inference", {})
    output_root = run_root

    model = YOLO(str(weights_path))
    results = list(
        model.predict(
            source=str(inputs.images_dir),
            imgsz=int(inference.get("imgsz", 640)),
            conf=float(inference.get("conf", 0.25)),
            device=device_name,
            save=False,
            verbose=False,
        )
    )

    export_summary = export_ultralytics_results(
        results,
        output_root,
        save_annotated=bool(inference.get("save_annotated", True)),
        save_combined_masks=bool(inference.get("save_combined_masks", True)),
        clean_root=False,
    )
    payload = {
        "runtime_mode": "colab",
        "run_root": str(run_root),
        "weights_path": str(weights_path),
        "preferred_gpu": preferred_gpu,
        "actual_gpu": actual_gpu,
        "device": device_name,
        "export_summary": export_summary,
    }
    write_json(run_root / "colab_runtime_summary.json", payload)
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
