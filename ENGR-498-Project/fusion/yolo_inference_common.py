from __future__ import annotations

import csv
import json
import shutil
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any


IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


@dataclass(frozen=True)
class PoseRecoveryInputs:
    run_dir: Path
    images_dir: Path
    image_timestamps_csv: Path
    image_filenames: tuple[str, ...]


def load_image_timestamps(path: Path, filename_column: str = "filename", time_column: str = "t_query_sec") -> dict[str, float]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        if filename_column not in fieldnames:
            raise ValueError(f"Missing image filename column '{filename_column}' in {path}")
        if time_column not in fieldnames:
            raise ValueError(f"Missing image timestamp column '{time_column}' in {path}")

        mapping: dict[str, float] = {}
        for row in reader:
            image_name = Path(row[filename_column]).name
            mapping[image_name] = float(row[time_column])

    if not mapping:
        raise ValueError(f"No image timestamps were loaded from {path}")
    return mapping


def resolve_pose_recovery_inputs(run_dir: Path) -> PoseRecoveryInputs:
    run_dir = Path(run_dir).resolve()
    images_dir = run_dir / "images"
    image_timestamps_csv = run_dir / "image_timestamps.csv"

    if not run_dir.exists():
        raise FileNotFoundError(f"Pose-recovery run directory not found: {run_dir}")
    if not images_dir.is_dir():
        raise FileNotFoundError(f"Pose-recovery images directory not found: {images_dir}")
    if not image_timestamps_csv.is_file():
        raise FileNotFoundError(f"Pose-recovery image timestamp CSV not found: {image_timestamps_csv}")

    image_timestamps = load_image_timestamps(image_timestamps_csv)
    missing_images = [name for name in sorted(image_timestamps) if not (images_dir / name).is_file()]
    if missing_images:
        preview = ", ".join(missing_images[:5])
        suffix = "..." if len(missing_images) > 5 else ""
        raise FileNotFoundError(
            f"Missing {len(missing_images)} images referenced by {image_timestamps_csv}: {preview}{suffix}"
        )

    return PoseRecoveryInputs(
        run_dir=run_dir,
        images_dir=images_dir,
        image_timestamps_csv=image_timestamps_csv,
        image_filenames=tuple(sorted(image_timestamps)),
    )


def ensure_directory(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def reset_directory(path: Path) -> Path:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def make_inference_output_dirs(output_root: Path, clean: bool = True, clean_root: bool = False) -> dict[str, Path]:
    output_root = Path(output_root)
    if clean and clean_root:
        reset_directory(output_root)
    else:
        ensure_directory(output_root)

    paths = {"root": output_root}
    for key, dirname in (
        ("pred_images", "pred_images"),
        ("masks_npz", "masks_npz"),
        ("meta_json", "meta_json"),
        ("combined_class_masks", "combined_class_masks"),
    ):
        target = output_root / dirname
        if clean and target.exists():
            shutil.rmtree(target)
        target.mkdir(parents=True, exist_ok=True)
        paths[key] = target
    return paths


def copy_pose_recovery_inputs(inputs: PoseRecoveryInputs, bundle_root: Path, clean: bool = True) -> dict[str, Path]:
    bundle_root = Path(bundle_root)
    if clean:
        reset_directory(bundle_root)
    else:
        ensure_directory(bundle_root)

    images_out = ensure_directory(bundle_root / "images")
    for image_name in inputs.image_filenames:
        shutil.copy2(inputs.images_dir / image_name, images_out / image_name)

    shutil.copy2(inputs.image_timestamps_csv, bundle_root / "image_timestamps.csv")
    return {
        "root": bundle_root,
        "images": images_out,
        "image_timestamps_csv": bundle_root / "image_timestamps.csv",
    }


def copy_file(src: Path, dst: Path) -> Path:
    src = Path(src)
    dst = Path(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return dst


def zip_directory(source_dir: Path, zip_path: Path) -> Path:
    source_dir = Path(source_dir).resolve()
    zip_path = Path(zip_path).resolve()
    zip_path.parent.mkdir(parents=True, exist_ok=True)
    if zip_path.exists():
        zip_path.unlink()

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source_dir.rglob("*")):
            if path.is_file():
                archive.write(path, path.relative_to(source_dir))
    return zip_path


def find_default_weight_candidates(script_path: Path) -> list[Path]:
    script_path = Path(script_path).resolve()
    candidates: list[Path] = []
    seen: set[Path] = set()

    def maybe_add(path: Path) -> None:
        resolved = path.resolve()
        if resolved not in seen:
            seen.add(resolved)
            candidates.append(resolved)

    for parent in (script_path.parent, *script_path.parents):
        maybe_add(parent / "best.pt")
        maybe_add(parent / "last.pt")
        maybe_add(parent / "colab" / "best.pt")
        maybe_add(parent / "colab" / "last.pt")
        maybe_add(parent / "Colab" / "best.pt")
        maybe_add(parent / "Colab" / "last.pt")

    return candidates


def export_ultralytics_results(
    results: list[Any],
    output_root: Path,
    *,
    save_annotated: bool = True,
    save_combined_masks: bool = True,
    clean_root: bool = False,
) -> dict[str, Any]:
    import cv2
    import numpy as np

    paths = make_inference_output_dirs(output_root, clean=True, clean_root=clean_root)
    exported: list[str] = []

    for index, result in enumerate(results, start=1):
        input_path = Path(str(result.path))
        stem = input_path.stem
        exported.append(stem)

        meta = {
            "source_image": str(input_path),
            "orig_shape": list(result.orig_shape) if result.orig_shape is not None else None,
            "names": result.names,
            "detections": [],
        }

        h0 = int(result.orig_shape[0]) if result.orig_shape is not None else 1
        w0 = int(result.orig_shape[1]) if result.orig_shape is not None else 1

        if save_annotated:
            annotated = result.plot()
            cv2.imwrite(str(paths["pred_images"] / f"{stem}.jpg"), annotated)

        if result.masks is None or result.boxes is None or len(result.boxes) == 0:
            np.savez_compressed(
                paths["masks_npz"] / f"{stem}_masks.npz",
                masks=np.zeros((0, h0, w0), dtype=np.uint8),
                cls=np.zeros((0,), dtype=np.int32),
                conf=np.zeros((0,), dtype=np.float32),
            )
            write_json(paths["meta_json"] / f"{stem}_meta.json", meta)
            continue

        masks = result.masks.data.cpu().numpy()
        cls = result.boxes.cls.cpu().numpy().astype(np.int32)
        conf = result.boxes.conf.cpu().numpy().astype(np.float32)

        masks_resized = np.stack(
            [cv2.resize(mask, (w0, h0), interpolation=cv2.INTER_NEAREST) for mask in masks],
            axis=0,
        )
        masks_u8 = (masks_resized > 0.5).astype(np.uint8)

        boxes_xyxy = result.boxes.xyxy.cpu().numpy().tolist()
        for detection_index in range(len(cls)):
            class_id = int(cls[detection_index])
            meta["detections"].append(
                {
                    "instance_index": detection_index,
                    "class_id": class_id,
                    "class_name": result.names[class_id],
                    "confidence": float(conf[detection_index]),
                    "box_xyxy": boxes_xyxy[detection_index],
                }
            )

        np.savez_compressed(
            paths["masks_npz"] / f"{stem}_masks.npz",
            masks=masks_u8,
            cls=cls,
            conf=conf,
        )
        write_json(paths["meta_json"] / f"{stem}_meta.json", meta)

        if save_combined_masks:
            unique_classes = sorted(set(cls.tolist()))
            for class_id in unique_classes:
                combined = np.zeros((h0, w0), dtype=np.uint8)
                for mask, mask_class in zip(masks_u8, cls):
                    if int(mask_class) == class_id:
                        combined |= (mask * 255)
                class_name = result.names[int(class_id)].replace(" ", "_")
                cv2.imwrite(str(paths["combined_class_masks"] / f"{stem}__{class_name}.png"), combined)

    summary = {
        "num_images_processed": len(results),
        "exported_stems": exported,
        "output_root": str(paths["root"]),
        "pred_images_dir": str(paths["pred_images"]),
        "masks_npz_dir": str(paths["masks_npz"]),
        "meta_json_dir": str(paths["meta_json"]),
        "combined_class_masks_dir": str(paths["combined_class_masks"]),
        "save_annotated": bool(save_annotated),
        "save_combined_masks": bool(save_combined_masks),
    }
    return summary
