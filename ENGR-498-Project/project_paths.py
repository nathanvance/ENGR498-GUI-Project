from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
ASSETS_DIR = PROJECT_ROOT / "assets"
MATLAB_EXTRACT_DIR = PROJECT_ROOT / "Matlab_ExtractPowerLine"
FUSION_DIR = PROJECT_ROOT / "fusion"
ROSBAG_PREPROCESSING_DIR = PROJECT_ROOT / "rosbag_preprocessing"
OUTPUTS_DIR = PROJECT_ROOT / "outputs"
MATLAB_OUTPUT_DIR = OUTPUTS_DIR / "matlab_extract"

DEFAULT_LAS_PATH = PROJECT_ROOT / "powerlineAerialLidarData.las"
DEFAULT_WIRES_NPZ_PATH = PROJECT_ROOT / "wires_points.npz"
DEFAULT_WIRE_INFO_JSON_PATH = PROJECT_ROOT / "wire_info.json"
DEFAULT_GROUND_POINTS_NPZ_PATH = PROJECT_ROOT / "ground_points.npz"


def project_path(*parts: str) -> Path:
    return PROJECT_ROOT.joinpath(*parts)
