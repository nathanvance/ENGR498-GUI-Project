from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


RUN_PIPELINE = Path(__file__).with_name("run_pipeline.py")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the calibration workflow inside the portable ROS container.",
        epilog=(
            "Example:\n"
            "  python run_calibration_workflow.py <dataset_path> --run-name test1_manual\n\n"
            "Outputs default to the mounted project folder:\n"
            "  rosbag_preprocessing\\outputs\\calibration"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _, workflow_args = parser.parse_known_args()

    command = [sys.executable, str(RUN_PIPELINE), "calibration", *workflow_args]
    completed = subprocess.run(command, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
