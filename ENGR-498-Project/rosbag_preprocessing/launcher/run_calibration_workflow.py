from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from calibration_preflight import run_preflight

RUN_PIPELINE = Path(__file__).with_name("run_pipeline.py")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the calibration workflow inside the portable ROS container.",
        epilog=(
            "Example:\n"
            "  python run_calibration_workflow.py <dataset_path> --run-name test1_manual\n\n"
            "Requirements check only:\n"
            "  python run_calibration_workflow.py --check-only\n\n"
            "Outputs default to the mounted project folder:\n"
            "  rosbag_preprocessing\\outputs\\calibration"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Run the calibration requirements check and exit without starting calibration.",
    )
    args, workflow_args = parser.parse_known_args()

    result = run_preflight()
    prefix = "PASS" if result.ok else "FAIL"
    print(f"[preflight] {prefix}: {result.summary}")
    for detail in result.details:
        print(f"[preflight] {detail}")

    if not result.ok:
        print(
            "[preflight] Calibration requires hardware-accelerated OpenGL through WSLg. "
            "Software rendering is unsupported on Windows."
        )
        return 2

    if args.check_only:
        return 0

    command = [sys.executable, str(RUN_PIPELINE), "calibration", *workflow_args]
    completed = subprocess.run(command, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
