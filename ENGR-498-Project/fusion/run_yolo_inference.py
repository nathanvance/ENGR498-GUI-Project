#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import sys
import textwrap
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from yolo_inference_common import (  # noqa: E402
    PoseRecoveryInputs,
    copy_file,
    copy_pose_recovery_inputs,
    export_ultralytics_results,
    find_default_weight_candidates,
    resolve_pose_recovery_inputs,
    write_json,
    write_text,
    zip_directory,
)


@dataclass
class RuntimeProbe:
    torch_available: bool
    ultralytics_available: bool
    cuda_available: bool
    selected_device: str | None
    selected_device_name: str | None
    capability: tuple[int, int] | None
    modern_cuda: bool
    usable_local_gpu: bool
    reason: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run local YOLO segmentation on rosbag-preprocessing JPG frames, or prepare a Colab fallback bundle."
    )
    parser.add_argument("--pose-recovery-run-dir", required=True, help="Path to a rosbag_preprocessing pose recovery run directory")
    parser.add_argument(
        "--runtime",
        choices=("auto", "local", "colab"),
        default="auto",
        help="Inference runtime: auto uses a modern local NVIDIA CUDA GPU when available, local forces the local GPU path, colab prepares a Colab bundle",
    )
    parser.add_argument("--weights", help="Path to YOLO segmentation weights (.pt). If omitted, the script searches common relative locations")
    parser.add_argument("--output-dir", help="Output directory for masks/meta or Colab bundle. Defaults to <pose_run>/yolo_inference")
    parser.add_argument("--local-device", default="0", help="Local CUDA device index or torch device string, e.g. 0 or cuda:0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--preferred-colab-gpu", default="A100", help="Preferred GPU name to request/expect in Colab")
    parser.add_argument(
        "--colab-drive-weights-path",
        help="Optional Drive path to weights for Colab if the local weights file should not be bundled",
    )
    parser.add_argument(
        "--no-include-weights-in-colab-bundle",
        action="store_true",
        help="Do not copy the local weights file into the prepared Colab bundle",
    )
    parser.add_argument("--no-save-annotated", action="store_true", help="Do not write annotated prediction JPGs")
    parser.add_argument("--no-save-combined-masks", action="store_true", help="Do not write optional combined per-class masks")
    return parser.parse_args()


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


def module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def probe_local_runtime(local_device: str) -> RuntimeProbe:
    if not module_available("torch"):
        return RuntimeProbe(False, module_available("ultralytics"), False, None, None, None, False, False, "torch is not installed")
    if not module_available("ultralytics"):
        return RuntimeProbe(True, False, False, None, None, None, False, False, "ultralytics is not installed")

    import torch

    if not torch.cuda.is_available():
        return RuntimeProbe(True, True, False, None, None, None, False, False, "CUDA is not available in torch")

    index, device_name = parse_device_index(local_device)
    if index >= torch.cuda.device_count():
        return RuntimeProbe(True, True, True, None, None, None, False, False, f"Requested CUDA device {index} does not exist")

    gpu_name = torch.cuda.get_device_name(index)
    capability = tuple(int(v) for v in torch.cuda.get_device_capability(index))
    modern_cuda = capability[0] >= 7
    return RuntimeProbe(
        torch_available=True,
        ultralytics_available=True,
        cuda_available=True,
        selected_device=device_name,
        selected_device_name=gpu_name,
        capability=capability,
        modern_cuda=modern_cuda,
        usable_local_gpu=True,
        reason="ok",
    )


def resolve_weights_path(explicit_path: str | None) -> Path | None:
    candidates: list[Path] = []
    if explicit_path:
        candidates.append(Path(explicit_path))

    env_path = os.environ.get("FUSION_YOLO_WEIGHTS")
    if env_path:
        candidates.append(Path(env_path))

    candidates.extend(find_default_weight_candidates(Path(__file__)))

    seen: set[Path] = set()
    for candidate in candidates:
        try:
            resolved = candidate.expanduser().resolve()
        except FileNotFoundError:
            resolved = candidate.expanduser().absolute()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.is_file():
            return resolved
    return None


def run_local_inference(
    *,
    inputs: PoseRecoveryInputs,
    output_dir: Path,
    weights_path: Path,
    probe: RuntimeProbe,
    args: argparse.Namespace,
) -> dict[str, Any]:
    from ultralytics import YOLO

    print(f"[info] running local inference on {probe.selected_device_name} ({probe.selected_device})")
    print(f"[info] using weights: {weights_path}")
    print(f"[info] consuming rosbag JPGs from: {inputs.images_dir}")

    model = YOLO(str(weights_path))
    results = list(
        model.predict(
            source=str(inputs.images_dir),
            imgsz=int(args.imgsz),
            conf=float(args.conf),
            device=probe.selected_device,
            save=False,
            verbose=False,
        )
    )

    export_summary = export_ultralytics_results(
        results,
        output_dir,
        save_annotated=not args.no_save_annotated,
        save_combined_masks=not args.no_save_combined_masks,
        clean_root=True,
    )
    payload = {
        "runtime_mode": "local",
        "pose_recovery_run_dir": str(inputs.run_dir),
        "images_dir": str(inputs.images_dir),
        "image_timestamps_csv": str(inputs.image_timestamps_csv),
        "weights_path": str(weights_path),
        "imgsz": int(args.imgsz),
        "conf": float(args.conf),
        "local_probe": asdict(probe),
        "export_summary": export_summary,
    }
    write_json(output_dir / "inference_manifest.json", payload)
    return payload


def prepare_colab_bundle(
    *,
    inputs: PoseRecoveryInputs,
    output_dir: Path,
    weights_path: Path | None,
    args: argparse.Namespace,
    probe: RuntimeProbe,
    auto_reason: str,
) -> dict[str, Any]:
    bundle_root = output_dir / "colab_bundle"
    print(f"[info] preparing Colab fallback bundle from rosbag JPGs in: {inputs.images_dir}")
    copied = copy_pose_recovery_inputs(inputs, bundle_root, clean=True)

    colab_script_src = THIS_DIR / "colab" / "run_inference_colab.py"
    common_script_src = THIS_DIR / "yolo_inference_common.py"
    copy_file(colab_script_src, bundle_root / "run_inference_colab.py")
    copy_file(common_script_src, bundle_root / "yolo_inference_common.py")

    bundled_weight_name: str | None = None
    if weights_path is not None and not args.no_include_weights_in_colab_bundle:
        copy_file(weights_path, bundle_root / weights_path.name)
        bundled_weight_name = weights_path.name

    config = {
        "run_name": inputs.run_dir.name,
        "preferred_gpu": args.preferred_colab_gpu,
        "fallback_gpu": "any_cuda",
        "weights_filename": bundled_weight_name,
        "drive_weights_path": args.colab_drive_weights_path,
        "input": {
            "images_dir": "images",
            "image_timestamps_csv": "image_timestamps.csv",
        },
        "output": {
            "pred_images_dir": "pred_images",
            "masks_npz_dir": "masks_npz",
            "meta_json_dir": "meta_json",
            "combined_class_masks_dir": "combined_class_masks",
        },
        "inference": {
            "imgsz": int(args.imgsz),
            "conf": float(args.conf),
            "save_annotated": not args.no_save_annotated,
            "save_combined_masks": not args.no_save_combined_masks,
            "rotate_mode": "none",
        },
    }
    write_json(bundle_root / "colab_inference_config.json", config)

    next_steps = textwrap.dedent(
        f"""\
        Colab fallback bundle prepared successfully.

        This bundle was created because runtime mode '{args.runtime}' resolved to the Colab path.
        Reason: {auto_reason}

        Local rosbag-preprocessing inputs:
        - images: {inputs.images_dir}
        - timestamps: {inputs.image_timestamps_csv}

        Bundle folder:
        - {bundle_root}

        Bundle zip:
        - {output_dir / 'colab_bundle.zip'}

        To run in Colab:
        1. Put the entire 'colab_bundle' folder somewhere under Google Drive.
        2. Open a Colab notebook with GPU runtime enabled.
        3. Prefer an {args.preferred_colab_gpu} if Colab offers it; otherwise use the assigned CUDA GPU.
        4. Run:

             %run /content/drive/MyDrive/.../colab_bundle/run_inference_colab.py \\
               --run-root /content/drive/MyDrive/.../colab_bundle

        5. After Colab finishes, the bundle will contain:
           - pred_images/
           - masks_npz/
           - meta_json/
           - combined_class_masks/ (optional)

        6. Copy or sync those folders back locally and point Fusion at:
           - --image-timestamps-csv {inputs.image_timestamps_csv}
           - --mask-dir <bundle>/masks_npz
           - --meta-dir <bundle>/meta_json
        """
    )
    write_text(bundle_root / "COLAB_NEXT_STEPS.txt", next_steps)

    bundle_zip = zip_directory(bundle_root, output_dir / "colab_bundle.zip")
    payload = {
        "runtime_mode": "colab_bundle",
        "pose_recovery_run_dir": str(inputs.run_dir),
        "images_dir": str(inputs.images_dir),
        "image_timestamps_csv": str(inputs.image_timestamps_csv),
        "weights_path": str(weights_path) if weights_path is not None else None,
        "bundled_weight_name": bundled_weight_name,
        "drive_weights_path": args.colab_drive_weights_path,
        "preferred_colab_gpu": args.preferred_colab_gpu,
        "bundle_root": str(bundle_root),
        "bundle_zip": str(bundle_zip),
        "local_probe": asdict(probe),
        "reason": auto_reason,
    }
    write_json(output_dir / "inference_manifest.json", payload)
    return payload


def main() -> int:
    args = parse_args()
    inputs = resolve_pose_recovery_inputs(Path(args.pose_recovery_run_dir))
    output_dir = Path(args.output_dir).resolve() if args.output_dir else (inputs.run_dir / "yolo_inference").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    weights_path = resolve_weights_path(args.weights)
    probe = probe_local_runtime(args.local_device)

    if args.runtime == "local":
        if weights_path is None:
            raise FileNotFoundError("Local runtime requires YOLO weights. Pass --weights or set FUSION_YOLO_WEIGHTS.")
        if not probe.usable_local_gpu:
            raise RuntimeError(f"Local runtime requested, but no usable local CUDA GPU was found: {probe.reason}")
        payload = run_local_inference(inputs=inputs, output_dir=output_dir, weights_path=weights_path, probe=probe, args=args)
    elif args.runtime == "auto":
        if weights_path is not None and probe.usable_local_gpu and probe.modern_cuda:
            payload = run_local_inference(inputs=inputs, output_dir=output_dir, weights_path=weights_path, probe=probe, args=args)
        else:
            if weights_path is None and not args.colab_drive_weights_path:
                raise FileNotFoundError(
                    "Auto mode could not find local weights and no --colab-drive-weights-path was provided for fallback."
                )
            reason_parts = []
            if weights_path is None:
                reason_parts.append("local weights were not found")
            if not probe.usable_local_gpu:
                reason_parts.append(f"local GPU runtime unavailable: {probe.reason}")
            elif not probe.modern_cuda:
                reason_parts.append(
                    f"local GPU {probe.selected_device_name} has capability {probe.capability}, below the modern-GPU threshold"
                )
            reason = "; ".join(reason_parts) if reason_parts else "colab fallback requested"
            payload = prepare_colab_bundle(
                inputs=inputs,
                output_dir=output_dir,
                weights_path=weights_path,
                args=args,
                probe=probe,
                auto_reason=reason,
            )
    else:
        if weights_path is None and not args.colab_drive_weights_path:
            raise FileNotFoundError(
                "Colab bundle mode requires either a local weights file (--weights) or --colab-drive-weights-path."
            )
        payload = prepare_colab_bundle(
            inputs=inputs,
            output_dir=output_dir,
            weights_path=weights_path,
            args=args,
            probe=probe,
            auto_reason="colab mode requested explicitly",
        )

    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
