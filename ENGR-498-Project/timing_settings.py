from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

from project_paths import PROJECT_ROOT


DEFAULT_GUI_SETTINGS = {
    "timing": {
        "fusion_time_offset_sec": 0.0,
    },
    "fusion": {
        "visualize": False,
    },
    "gps": {
        "allow_missing": False,
    },
    "pose_recovery": {
        "blur_filter_enabled": True,
    },
}


def timing_settings_path() -> Path:
    return PROJECT_ROOT / "gui_settings.json"


def _deep_merge_dict(base: dict[str, Any], incoming: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in incoming.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge_dict(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_global_gui_settings() -> dict[str, Any]:
    path = timing_settings_path()
    if path.is_file():
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
    else:
        payload = {}
    merged = _deep_merge_dict(DEFAULT_GUI_SETTINGS, payload if isinstance(payload, dict) else {})
    return merged


def save_global_gui_settings(settings: dict[str, Any]) -> None:
    merged = _deep_merge_dict(DEFAULT_GUI_SETTINGS, settings if isinstance(settings, dict) else {})
    timing_settings_path().write_text(json.dumps(merged, indent=2), encoding="utf-8")


def load_global_timing_settings() -> dict[str, Any]:
    return load_global_gui_settings()


def save_global_timing_settings(settings: dict[str, Any]) -> None:
    save_global_gui_settings(settings)


def resolve_effective_timing(metadata: dict[str, Any]) -> dict[str, Any]:
    global_cfg = load_global_gui_settings().get("timing", {})
    overrides = (
        metadata.get("config", {})
        .get("timing_overrides", {})
    )
    if not isinstance(overrides, dict):
        overrides = {}
    effective = {
        "fusion_time_offset_sec": float(global_cfg.get("fusion_time_offset_sec", 0.0)),
    }
    if "fusion_time_offset_sec" in overrides:
        effective["fusion_time_offset_sec"] = float(overrides.get("fusion_time_offset_sec") or 0.0)
    return effective


def resolve_global_fusion_settings() -> dict[str, Any]:
    global_cfg = load_global_gui_settings().get("fusion", {})
    return {
        "visualize": bool(global_cfg.get("visualize", False)),
    }


def resolve_global_gps_settings() -> dict[str, Any]:
    global_cfg = load_global_gui_settings().get("gps", {})
    return {
        "allow_missing": bool(global_cfg.get("allow_missing", False)),
    }


def resolve_global_pose_recovery_settings() -> dict[str, Any]:
    global_cfg = load_global_gui_settings().get("pose_recovery", {})
    return {
        "blur_filter_enabled": bool(global_cfg.get("blur_filter_enabled", True)),
    }
